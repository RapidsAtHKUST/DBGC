#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>

#include "common/types.h"
#include "common/file_io.h"
#include "compressors/adaptive_compressor.h"
#include "compressors/base.h"
#include "compressors/general.h"
#include "compressors/sline_compressor.h"
#include "compressors/trees/octree.h"
#include "compressors/trees/quadtree.h"
#include "partition/by_lambda.h"
#include "partition/by_density.h"


size_t AdaptiveCompressor::compress(
    const PointCloud<float>& input, 
    std::vector<unsigned char>& output
) {
    using clock = std::chrono::high_resolution_clock;
    using duration = std::chrono::duration<double>;
    clock::time_point start, end;
    duration diff;

    output.clear();
    if (input.empty()) return 0UL;

    PointCloud<float> dense_cloud, sparse_cloud, outliers;
    const size_t sline_levels = segs_.size() - 1;
    
    std::vector<unsigned char> octree_compressed, sline_compressed[sline_levels], quadtree_compressed;
    size_t ori_octree_size, ori_sline_size[sline_levels], ori_quadtree_size;
    size_t com_octree_size, com_sline_size[sline_levels], com_quadtree_size;

    if (rate_ < 0.0f || rate_ > 1.0f)
    {
        std::cout << "***** DBSCAN *****" << std::endl;
        std::vector<int> cluster_results;
        TIME_START();
        dbscan(input, resolution_, cluster_results);

        partition_by_dbscan(input, cluster_results, 
            dense_cloud, sparse_cloud, 100.0f, 300);
        TIME_END("partitioning");
    }
    else
    {
        PointCloud<float> temp = const_cast<PointCloud<float>&>(input);
        std::sort(
            temp.begin(), temp.end(), 
            [](const Point<float> &p1, const Point<float> &p2){
                return p1.x * p1.x + p1.y * p1.y + p1.z * p1.z <
                p2.x * p2.x + p2.y * p2.y + p2.z * p2.z;
        });
        size_t i = 0;
        while (i < temp.size())
        {
            if (i < temp.size() * rate_)
                dense_cloud.push_back(temp[i]);
            else
                sparse_cloud.push_back(temp[i]);
            i++;
        }
    }

    std::cout << "\n***** OctreeCompressor Compression *****\n";
    PointCloud<int> dense_cloud_int;
    TIME_START();
    io::point_cloud_to_int(dense_cloud, dense_cloud_int, 2 / resolution_);
    ori_octree_size = dense_cloud.size () * 3 * sizeof(float);
    
    OctreeCompressor octree (2);
    com_octree_size = octree.compress(dense_cloud_int, octree_compressed);
    TIME_END("octree");

    PRINT_LOCAL_COMPRESSION_RATIO(ori_octree_size, com_octree_size)


    std::cout << "\n***** Sperical Line Compression *****\n";
    std::vector<int> remaining_points;

    PointCloud<float> pre_cloud, cur_cloud;
    for (size_t i = 0; i < sline_levels; i++)
    {
        std::cout << '\n';

        cur_cloud.clear();
        filter<Point<float>>(sparse_cloud, cur_cloud, [this, &i](const Point<float>& p){
            float depth = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (depth > segs_[i] && depth <= segs_[i + 1]) return true;
            else return false;
        });

        for (auto j: remaining_points)
        {
            cur_cloud.push_back(pre_cloud[j]);
        }

        SPointCloud<float> s_cur_cloud;
        io::cartesian_to_spherical(cur_cloud, s_cur_cloud);
        
        if (intra_line_coding_only_)
        {
            SLineCompressor compressor(0.01f, resolution_, 0.01f, conf::sensors[KITTI]);
            com_sline_size[i] = compressor.compress(s_cur_cloud, sline_compressed[i]);

            std::swap(remaining_points, compressor.get_outliers());
        }
        else
        {
            TIME_START();
            SLineCompressorInterLine compressor(0.01f, resolution_, 0.01f, conf::sensors[KITTI], off_the_shelf_);
            com_sline_size[i] = compressor.compress(s_cur_cloud, sline_compressed[i]);
            TIME_END("inter-line coding");

            std::swap(remaining_points, compressor.get_outliers());
        }
        ori_sline_size[i] = (cur_cloud.size() - remaining_points.size()) * 3 * sizeof(float);

        if (i != segs_.size() - 2)
            std::swap(pre_cloud, cur_cloud);
        else
            std::swap(sparse_cloud, cur_cloud);

        PRINT_LOCAL_COMPRESSION_RATIO(ori_sline_size[i], com_sline_size[i])

        if (i == segs_.size() - 2)
            std::cout << "remaining_points: " << remaining_points.size() << std::endl;
    }

    std::cout << "\n***** Quadtree Compression *****\n";
    
    TIME_START();
    for (auto j: remaining_points)
    {
        outliers.push_back(sparse_cloud[j]);
    }
    ori_quadtree_size = outliers.size() * 3 * sizeof(float);

    PointCloud<int> outliers_int;
    io::point_cloud_to_int(outliers, outliers_int, 
        2 / resolution_, 2 / resolution_, 1 / resolution_);

    QuadtreeCompressor quadtree(2);
    com_quadtree_size = quadtree.compress(outliers_int, quadtree_compressed);
    TIME_END("quadtree");

    PRINT_LOCAL_COMPRESSION_RATIO(ori_quadtree_size, com_quadtree_size)


    // organize the output
    size_t write_pos = 0UL;

    size_t total_size = 
        sizeof(float) +         // resolution
        sizeof(OffTheShelf) +   // OffTheShelf
        sizeof(bool) +          // intra-line coding
        sizeof(size_t) + com_octree_size +
        sizeof(size_t) + com_quadtree_size;
    for (size_t i = 0; i < sline_levels; i++)
        total_size += sizeof(size_t) + com_sline_size[i];
    
    output.resize(total_size);

    memcpy(output.data() + write_pos, &resolution_, sizeof(float));
    write_pos += sizeof(float);
    memcpy(output.data() + write_pos, &off_the_shelf_, sizeof(OffTheShelf));
    write_pos += sizeof(OffTheShelf);
    memcpy(output.data() + write_pos, &intra_line_coding_only_, sizeof(bool));
    write_pos += sizeof(bool);

    memcpy(output.data() + write_pos, &com_octree_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, octree_compressed.data(), com_octree_size);
    write_pos += com_octree_size;

    for (size_t i = 0; i < sline_levels; i++)
    {
        memcpy(output.data() + write_pos, &com_sline_size[i], sizeof(size_t));
        write_pos += sizeof(size_t);
        memcpy(output.data() + write_pos, sline_compressed[i].data(), com_sline_size[i]);
        write_pos += com_sline_size[i];
    }

    memcpy(output.data() + write_pos, &com_quadtree_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, quadtree_compressed.data(), com_quadtree_size);
    write_pos += com_quadtree_size;
    
    return output.size();
}

size_t AdaptiveCompressor::decompress(
    const std::vector<unsigned char>& input,
    PointCloud<float>& output
) {
    using clock = std::chrono::high_resolution_clock;
    using duration = std::chrono::duration<double>;
    clock::time_point start, end;
    duration diff;

    output.clear();
    if (input.empty()) return 0UL;
    
    const size_t sline_levels = segs_.size() - 1;
    
    std::vector<unsigned char> octree_compressed, sline_compressed[sline_levels], quadtree_compressed;
    size_t com_octree_size, com_sline_size[sline_levels], com_quadtree_size;
    PointCloud<float> octree_cloud, sline_cloud[sline_levels], quadtree_cloud;

    size_t read_pos = 0UL;

    memcpy(&resolution_, input.data() + read_pos, sizeof(float));
    read_pos += sizeof(float);
    memcpy(&off_the_shelf_, input.data() + read_pos, sizeof(OffTheShelf));
    read_pos += sizeof(OffTheShelf);
    memcpy(&intra_line_coding_only_, input.data() + read_pos, sizeof(bool));
    read_pos += sizeof(bool);
    if (intra_line_coding_only_)
    {
        std::cout << "intra-line coding decompression not supported!" << std::endl;
        exit(-1);
    }

    memcpy(&com_octree_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    octree_compressed.resize(com_octree_size);
    memcpy(octree_compressed.data(), input.data() + read_pos, com_octree_size);
    read_pos += com_octree_size;

    for (size_t i = 0; i < sline_levels; i++)
    {
        memcpy(&com_sline_size[i], input.data() + read_pos, sizeof(size_t));
        read_pos += sizeof(size_t);
        sline_compressed[i].resize(com_sline_size[i]);
        memcpy(sline_compressed[i].data(), input.data() + read_pos, com_sline_size[i]);
        read_pos += com_sline_size[i];
    }

    memcpy(&com_quadtree_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    quadtree_compressed.resize(com_quadtree_size);
    memcpy(quadtree_compressed.data(), input.data() + read_pos, com_quadtree_size);
    read_pos += com_quadtree_size;

    TIME_START();
    OctreeCompressor octree(2);
    PointCloud<int> octree_cloud_int;
    octree.decompress(octree_compressed, octree_cloud_int);
    TIME_END("octree");
    
    io::point_cloud_to_float(octree_cloud_int, octree_cloud, 2 / resolution_);

    for (size_t i = 0; i < sline_levels; i++)
    {
        SPointCloud<float> s_cur_cloud;

        SLineCompressorInterLine compressor(0.01f, resolution_, 0.01f, conf::sensors[KITTI], off_the_shelf_);
        compressor.decompress(sline_compressed[i], s_cur_cloud);

        io::spherical_to_cartesian(s_cur_cloud, sline_cloud[i]);
    }

    PointCloud<int> quadtree_cloud_int;

    TIME_START();
    QuadtreeCompressor quadtree(2);
    quadtree.decompress(quadtree_compressed, quadtree_cloud_int);
    TIME_END("quadtree");
    
    io::point_cloud_to_float(quadtree_cloud_int, quadtree_cloud, 
        2 / resolution_, 2 / resolution_, 1 / resolution_);
    
    output.insert(output.end(), octree_cloud.begin(), octree_cloud.end());
    for (size_t i = 0; i < sline_levels; i++)
        output.insert(output.end(), sline_cloud[i].begin(), sline_cloud[i].end());
    output.insert(output.end(), quadtree_cloud.begin(), quadtree_cloud.end());

    return output.size();
}