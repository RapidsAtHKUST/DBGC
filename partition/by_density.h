#ifndef PARTITION_BY_DENSITY
#define PARTITION_BY_DENSITY

#include <unordered_map>
#include <vector>

#include "common/types.h"


void dbscan (
    const PointCloud<float>& cloud, 
    float resolution,
    std::vector<int>& cluster_results
);

inline void partition_by_dbscan (
    const PointCloud<float>& total_cloud,
    std::vector<int>& cluster_results,
    PointCloud<float>& dense_cloud,
    PointCloud<float>& sparse_cloud,
    const float threshold = std::numeric_limits<float>::max(),
    const int num_num_points_in_a_cluster = 300
) {
    std::unordered_map<int, int> num_points_in_cluster;

    for (size_t i = 0; i < total_cloud.size(); i++)
    {
        int cluster_id = cluster_results[i];
        if (cluster_id == -1) continue;
        if (num_points_in_cluster.find(cluster_id) == num_points_in_cluster.end())
            num_points_in_cluster[cluster_id] = 1;
        else
            num_points_in_cluster[cluster_id] ++;
    }
    
    for (size_t i = 0; i < total_cloud.size(); i++)
    {
        int cluster_id = cluster_results[i];
        const auto& p = total_cloud[i];
        float depth = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (num_points_in_cluster[cluster_id] > num_num_points_in_a_cluster && 
            depth < threshold)
        {
            dense_cloud.push_back(p);
        }
        else
        {
            sparse_cloud.push_back(p);
        }
    }
}

#endif