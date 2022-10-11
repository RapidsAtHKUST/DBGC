#include <string.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>

#ifdef ENABLE_TBB
#include <tbb/parallel_for.h>
#include <tbb/tbb.h>
#endif

#include "common/config.h"
#include "common/types.h"

void get_connected_nbrs(
    const PointCloud<float>& cloud,
    float grid_size,
    float max_dis,
    std::vector<std::vector<int>>& nearest_nbrs_and_dis
) {
    // initialize grids
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float max_y = std::numeric_limits<float>::min();
    float max_z = std::numeric_limits<float>::min();
    int *ids = new int[cloud.size() + 1];

    // set the bbox
    for (auto point: cloud) {
        if (point.x < min_x) { min_x = point.x; }
        if (point.x > max_x) { max_x = point.x; }
        if (point.y < min_y) { min_y = point.y; }
        if (point.y > max_y) { max_y = point.y; }
        if (point.z < min_z) { min_z = point.z; }
        if (point.z > max_z) { max_z = point.z; }
    }

    int ngrids_x, ngrids_y, ngrids_z;
    long ngrids_xy, ngrids;
    grid_size /= 2;

    do {
        grid_size *= 2;
        ngrids_x = std::floor((max_x - min_x) / grid_size) + 1;
        ngrids_y = std::floor((max_y - min_y) / grid_size) + 1;
        ngrids_z = std::floor((max_z - min_z) / grid_size) + 1;

        ngrids_xy = ngrids_x * ngrids_y;
        ngrids = ngrids_xy * ngrids_z;
    } while (
        ngrids_xy > std::numeric_limits<int>::max() || 
        ngrids > std::numeric_limits<int>::max()
    );

    std::unordered_map<int, int> cell_start;
    std::unordered_map<int, int> cell_end;

    auto idx {
        [&ngrids_x, &ngrids_y, &ngrids_z](int i, int j, int k)
        {
            return i * ngrids_y * ngrids_z + 
                   j * ngrids_z + k;
        }
    };

    // put points into grids
    std::unordered_map<int, std::vector<int>> hist;
    for (size_t i = 0; i < cloud.size (); i++) {
        auto& point = cloud.at(i);
        int x_idx = (point.x - min_x) / grid_size;
        int y_idx = (point.y - min_y) / grid_size;
        int z_idx = (point.z - min_z) / grid_size;
        hist[x_idx * ngrids_y * ngrids_z + 
             y_idx * ngrids_z + z_idx].push_back(i);
    }

    int write_pos = 0;
    for (auto & [i, vs]: hist)
    {
        cell_start[i] = write_pos;
        memcpy(&ids[write_pos], vs.data(), vs.size() * sizeof(int));
        write_pos += vs.size();
        cell_end[i] = write_pos;
    }

    int span_grids = std::ceil(max_dis / static_cast<float>(grid_size));

#ifdef ENABLE_TBB
    tbb::parallel_for(0, static_cast<int>(cloud.size()), [&](int cur_id) {
#else
    for (int cur_id = 0; cur_id < static_cast<int>(cloud.size()); cur_id++) {
#endif
        auto& cur_point = cloud.at(cur_id);
        int x_idx = (cur_point.x - min_x) / grid_size;
        int y_idx = (cur_point.y - min_y) / grid_size;
        int z_idx = (cur_point.z - min_z) / grid_size;

        for (int i = x_idx - span_grids; i <= x_idx + span_grids; i++)
        for (int j = y_idx - span_grids; j <= y_idx + span_grids; j++)
        for (int k = z_idx - span_grids; k <= z_idx + span_grids; k++)
        {
            if (cell_start.find(idx(i, j, k)) == cell_start.end())
                continue;

            for (int m = cell_start.at(idx(i, j, k));
                    m < cell_end.at(idx(i, j, k)); m++)
            {
                int id = ids[m];
                if (id == cur_id) continue;
                Point<float> diff = Point<float>::diff(cur_point, cloud.at(id));
                float dis = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
                
                if (dis < max_dis)
                    nearest_nbrs_and_dis[cur_id].push_back(id);
            }
        }
    
#ifdef ENABLE_TBB
    });
#else
    }
#endif
}


void expansion (
    int cur_id, int cur_cluster_id, 
    const std::vector<std::vector<int>>& nearest_nbrs_and_dis, 
    std::vector<int>& cluster_results,
    std::vector<bool>& visited,
    const size_t min_num_points,
    const int level
) {
    for (auto& nbr_id: nearest_nbrs_and_dis[cur_id])
    {
        if (visited[nbr_id]) continue;
        visited[nbr_id] = true;

        cluster_results[nbr_id] = cur_cluster_id;
        if (nearest_nbrs_and_dis[nbr_id].size() >= min_num_points)
        {
            if (level < conf::max_recru_levels)
                expansion(nbr_id, cur_cluster_id, nearest_nbrs_and_dis, 
                cluster_results, visited, min_num_points, level + 1);
        }
    }
}

void dbscan (
    const PointCloud<float>& cloud, 
    float resolution,
    std::vector<int>& cluster_results
) {
    using clock = std::chrono::high_resolution_clock;
    using duration = std::chrono::duration<double>;

    const int scale = 5;
    const float epsilon = resolution * scale;
    const size_t min_num_points = scale * scale * conf::pi;

    clock::time_point start = clock::now();

    std::vector<std::vector<int>> nearest_nbrs_and_dis (cloud.size());

    get_connected_nbrs(cloud, epsilon, epsilon, nearest_nbrs_and_dis);

    cluster_results.clear();
    cluster_results.resize(cloud.size(), -1);
    
    clock::time_point end = clock::now();
    duration diff = end - start;
    printf("stage 1: %.9lf s\n", diff.count());

    start = clock::now();
    std::vector<bool> visited (cloud.size(), false);
    int cur_cluster_id = 0;
    for (size_t i = 0; i < cloud.size(); i++)
    {
        if (nearest_nbrs_and_dis[i].size() < min_num_points || visited[i])
            continue;
        visited[i] = true;
        cluster_results[i] = cur_cluster_id;
        expansion(
            i, cur_cluster_id, nearest_nbrs_and_dis, 
            cluster_results, visited, min_num_points, 0);
        cur_cluster_id++;
    }
    end = clock::now();
    diff = end - start;
    printf("stage 2: %.9lf s\n", diff.count());
    std::cout << "num clusters: " << cur_cluster_id << std::endl;
}

void dbscan_approximate (
    const PointCloud<float>& cloud, 
    float resolution,
    std::vector<int>& cluster_results
) {
    using clock = std::chrono::high_resolution_clock;
    using duration = std::chrono::duration<double>;

    const int scale = 5;
    const float epsilon = resolution * scale;

    clock::time_point start = clock::now();

    // initialize grids
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float max_y = std::numeric_limits<float>::min();
    float max_z = std::numeric_limits<float>::min();
    int *ids = new int[cloud.size() + 1];

    // set the bbox
    for (auto point: cloud) {
        if (point.x < min_x) { min_x = point.x; }
        if (point.x > max_x) { max_x = point.x; }
        if (point.y < min_y) { min_y = point.y; }
        if (point.y > max_y) { max_y = point.y; }
        if (point.z < min_z) { min_z = point.z; }
        if (point.z > max_z) { max_z = point.z; }
    }

    int ngrids_x, ngrids_y, ngrids_z;
    long ngrids_xy, ngrids;
    int grid_size = epsilon / 2;

    do {
        grid_size *= 2;
        ngrids_x = std::floor((max_x - min_x) / grid_size) + 1;
        ngrids_y = std::floor((max_y - min_y) / grid_size) + 1;
        ngrids_z = std::floor((max_z - min_z) / grid_size) + 1;

        ngrids_xy = ngrids_x * ngrids_y;
        ngrids = ngrids_xy * ngrids_z;
    } while (
        ngrids_xy > std::numeric_limits<int>::max() || 
        ngrids > std::numeric_limits<int>::max()
    );

    std::unordered_map<int, int> cell_start;
    std::unordered_map<int, int> cell_end;

    auto idx {
        [&ngrids_x, &ngrids_y, &ngrids_z](int i, int j, int k)
        {
            return i * ngrids_y * ngrids_z + 
                   j * ngrids_z + k;
        }
    };

    // put points into grids
    std::unordered_map<int, std::vector<int>> hist;
    std::unordered_map<int, int> num_ps_in_grid;
    std::unordered_map<int, bool> grid_is_core;
    tbb::concurrent_vector<int> cores;
    for (size_t i = 0; i < cloud.size (); i++) {
        auto& point = cloud.at(i);
        int x_idx = (point.x - min_x) / grid_size;
        int y_idx = (point.y - min_y) / grid_size;
        int z_idx = (point.z - min_z) / grid_size;
        hist[x_idx * ngrids_y * ngrids_z + 
             y_idx * ngrids_z + z_idx].push_back(i);
        if (num_ps_in_grid.find(x_idx * ngrids_y * ngrids_z + 
             y_idx * ngrids_z + z_idx) == num_ps_in_grid.end())
        {
            num_ps_in_grid[x_idx * ngrids_y * ngrids_z + 
             y_idx * ngrids_z + z_idx] = 0;
        }
        num_ps_in_grid[x_idx * ngrids_y * ngrids_z + 
             y_idx * ngrids_z + z_idx] += 1;
        grid_is_core[x_idx * ngrids_y * ngrids_z + 
             y_idx * ngrids_z + z_idx] = false;
    }

    int write_pos = 0;
    for (auto & [i, vs]: hist)
    {
        cell_start[i] = write_pos;
        memcpy(&ids[write_pos], vs.data(), vs.size() * sizeof(int));
        write_pos += vs.size();
        cell_end[i] = write_pos;
    }

    int max_dis = epsilon * 5;
    int span_grids = std::ceil(max_dis / static_cast<float>(grid_size));

#ifdef ENABLE_TBB
    tbb::parallel_for(0, static_cast<int>(cloud.size()), [&](int cur_id) {
#else
    for (int cur_id = 0; cur_id < static_cast<int>(cloud.size()); cur_id++) {
#endif
        auto& cur_point = cloud.at(cur_id);
        int x_idx = (cur_point.x - min_x) / grid_size;
        int y_idx = (cur_point.y - min_y) / grid_size;
        int z_idx = (cur_point.z - min_z) / grid_size;
        if (grid_is_core[idx(x_idx, y_idx, z_idx)])
        {
            return;
        }

        int num_nbrs = 0;
        for (int i = x_idx - span_grids; i <= x_idx + span_grids; i++)
        for (int j = y_idx - span_grids; j <= y_idx + span_grids; j++)
        for (int k = z_idx - span_grids; k <= z_idx + span_grids; k++)
        {
            if (cell_start.find(idx(i, j, k)) == cell_start.end())
                continue;

            num_nbrs += num_ps_in_grid[idx(i, j, k)];
        }
    
#ifdef ENABLE_TBB
    });
#else
    }
#endif

    cluster_results.clear();
    cluster_results.resize(cloud.size(), -1);
    
    clock::time_point end = clock::now();
    duration diff = end - start;
    printf("stage 1: %.9lf s\n", diff.count());

    start = clock::now();
    std::vector<bool> visited (cloud.size(), false);

    for (auto& [g, is_core]: grid_is_core)
    {
        if (is_core)
        {
            for (auto p: hist[g])
            cluster_results.push_back(p);
        }
    }
    end = clock::now();
    diff = end - start;
    printf("stage 2: %.9lf s\n", diff.count());
    //std::cout << "num clusters: " << cur_cluster_id << std::endl;
}
