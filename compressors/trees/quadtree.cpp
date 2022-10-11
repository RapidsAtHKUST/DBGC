#include <algorithm>    /* For min_element */
#include <cassert>
#include <cmath>        /* For pow */
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

#include "common/types.h"
#include "compressors/entropy.h"
#include "compressors/trees/quadtree.h"

QuadtreeCompressor::Node::Node ()
: point_ids_()
, children_{}
, x_min_()
, x_max_()
, y_min_()
, y_max_()
{}

QuadtreeCompressor::Node::Node (int x_min, int x_max, int y_min, int y_max)
: point_ids_()
, children_{}
, x_min_(x_min)
, x_max_(x_max)
, y_min_(y_min)
, y_max_(y_max)
{}

void QuadtreeCompressor::Node::build_recursive (
    const PointCloud<int>& cloud, 
    const int min_resolution
) {
    if (x_max_ - x_min_ <= min_resolution) return;

    int x_segs[3] = {x_min_, (x_max_ - x_min_) / 2 + x_min_, x_max_};
    int y_segs[3] = {y_min_, (y_max_ - y_min_) / 2 + y_min_, y_max_};

    std::vector<int> seperated_point_ids_[4];
    for (auto i: point_ids_)
    {
        int index = (cloud.at(i).x >= x_segs[1]) * 2 +
            (cloud.at(i).y >= y_segs[1]);
        seperated_point_ids_[index].push_back(i);
    }
    point_ids_.clear();

    for (int i = 0; i < 4; i++)
    {
        int x_idx = i / 2;
        int y_idx = i % 2;

        if (!seperated_point_ids_[i].empty())
        {
            children_[i] = new Node (
                x_segs[x_idx],
                x_segs[x_idx + 1],
                y_segs[y_idx],
                y_segs[y_idx + 1]
            );
            children_[i]->point_ids_ = std::move(seperated_point_ids_[i]);
            children_[i]->build_recursive(cloud, min_resolution);
        }
        else
        {
            children_[i] = nullptr;
        }
    }
}

QuadtreeCompressor::QuadtreeCompressor(int resolution)
: root_{}
, depth_(0)
, min_depth_(0)
, resolution_(resolution)
, cloud_(nullptr)
{}

size_t QuadtreeCompressor::compress(
    const PointCloud<int>& input, 
    std::vector<unsigned char>& output
) {
    output.clear();
    if (input.empty()) return 0UL;
    
    cloud_= &input;
    
    set_boundary();

    // recursively construct the octree
    root_.build_recursive(input, resolution_);

    EntropyCompressor compressor;

    std::vector<int> blocks;
    serialize_tree(blocks);
    std::vector<unsigned char> tree_output;
    compressor.compress(blocks, tree_output);

    std::vector<int> leaf_blocks;
    serialize_leafs(leaf_blocks);
    std::vector<unsigned char> leaf_output;

    // if any leaf contains more than one point
    // record the number of points of each leaf and
    // the delta from a point to the corner of the leaf
    bool record_npoints_leafs = false;
    for (auto npoints: leaf_blocks)
    {
        if (npoints > 1)
        {
            record_npoints_leafs = true;
            break;
        }
    }
    if (record_npoints_leafs)
    {
        compressor.compress(leaf_blocks, leaf_output); 
    }

    /*if (resolution_ > 2)
    {
        std::vector<int> leaf_delta_blocks;
        serialize_leaf_delta(leaf_delta_blocks);
        compressed_size += compressor.compress(leaf_delta_blocks, output);
    }*/

    // encode the delta of depth
    std::vector<int> depth_blocks;
    serialize_depth(depth_blocks);
    std::vector<unsigned char> depth_output;
    for (size_t i = depth_blocks.size() - 1; i > 0; i--)
    {
        depth_blocks[i] -= depth_blocks[i - 1];
    }
    compressor.compress(depth_blocks, depth_output);

    // organize output
    size_t write_pos = 0UL;
    size_t tree_output_size = tree_output.size();
    size_t leaf_output_size = leaf_output.size();
    size_t depth_output_size = depth_output.size();
    output.resize(
        sizeof(int) * 4 +   // bbox
        sizeof(int) +       // depth
        sizeof(size_t) +    // tree block size
        tree_output_size +  // tree block
        sizeof(size_t) +    // leaf block size
        leaf_output_size +  // leaf block
        sizeof(size_t) +    // depth block size
        depth_output_size   // depth block
    );

    memcpy(output.data() + write_pos, &(root_.x_min_), sizeof(int));
    write_pos += sizeof(int);
    memcpy(output.data() + write_pos, &(root_.x_max_), sizeof(int));
    write_pos += sizeof(int);
    memcpy(output.data() + write_pos, &(root_.y_min_), sizeof(int));
    write_pos += sizeof(int);
    memcpy(output.data() + write_pos, &(root_.y_max_), sizeof(int));
    write_pos += sizeof(int);

    memcpy(output.data() + write_pos, &depth_, sizeof(int));
    write_pos += sizeof(int);

    memcpy(output.data() + write_pos, &(tree_output_size), sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, tree_output.data(), tree_output_size);
    write_pos += tree_output_size;

    memcpy(output.data() + write_pos, &(leaf_output_size), sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, leaf_output.data(), leaf_output_size);
    write_pos += leaf_output_size;

    memcpy(output.data() + write_pos, &(depth_output_size), sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, depth_output.data(), depth_output_size);
    write_pos += depth_output_size;

    return output.size();
}

size_t QuadtreeCompressor::decompress (
    const std::vector<unsigned char>& input, 
    PointCloud<int>& output
) {
    output.clear();
    if (input.empty()) return 0UL;

    size_t tree_compressed_size, leaf_compressed_size, depth_compressed_size;
    std::vector<unsigned char> tree_compressed;
    std::vector<unsigned char> leaf_compressed;
    std::vector<unsigned char> depth_compressed;

    size_t read_pos = 0UL;

    memcpy(&(root_.x_min_), input.data() + read_pos, sizeof(int));
    read_pos += sizeof(int);
    memcpy(&(root_.x_max_), input.data() + read_pos, sizeof(int));
    read_pos += sizeof(int);
    memcpy(&(root_.y_min_), input.data() + read_pos, sizeof(int));
    read_pos += sizeof(int);
    memcpy(&(root_.y_max_), input.data() + read_pos, sizeof(int));
    read_pos += sizeof(int);

    memcpy(&depth_, input.data() + read_pos, sizeof(int));
    read_pos += sizeof(int);

    memcpy(&tree_compressed_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    tree_compressed.resize(tree_compressed_size);
    memcpy(tree_compressed.data(), input.data() + read_pos, tree_compressed_size);
    read_pos += tree_compressed_size;

    memcpy(&leaf_compressed_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    leaf_compressed.resize(leaf_compressed_size);
    memcpy(leaf_compressed.data(), input.data() + read_pos, leaf_compressed_size);
    read_pos += leaf_compressed_size;

    memcpy(&depth_compressed_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    depth_compressed.resize(depth_compressed_size);
    memcpy(depth_compressed.data(), input.data() + read_pos, depth_compressed_size);
    read_pos += depth_compressed_size;
    assert(read_pos == input.size());

    // uncompressed blocks
    EntropyCompressor compressor;
    std::vector<int> tree_block;
    compressor.decompress(tree_compressed, tree_block);

    std::vector<int> leaf_block;
    compressor.decompress(leaf_compressed, leaf_block);
    std::vector<int> depth_block;
    compressor.decompress(depth_compressed, depth_block);

    // recover leaf and tree blocks
    size_t tree_read_pos = 0, leaf_read_pos = 0, depth_read_pos = 0;
    int run_depth = 0;

    std::queue<QuadtreeCompressor::Node*> q;
    q.push(&root_);
    int depth = 0;
    while (tree_read_pos < tree_block.size())
    {
        size_t cur_size = q.size();
        depth += 1;
        for (size_t i = 0; i < cur_size; i++)
        {
            QuadtreeCompressor::Node* node = q.front();

            int x_segs[3] = {node->x_min_, (node->x_max_ - node->x_min_) / 2 + node->x_min_, node->x_max_};
            int y_segs[3] = {node->y_min_, (node->y_max_ - node->y_min_) / 2 + node->y_min_, node->y_max_};

            unsigned char o = tree_block[tree_read_pos++];
            for (int j = 0; j < 4; j++)
            {
                if (!(o & (1 << j))) continue;
                
                if (depth == depth_ - 1)
                {
                    if (!leaf_block.empty())
                    {
                        for (int j = 0; j < leaf_block[leaf_read_pos]; j++)
                        {
                            run_depth += depth_block[depth_read_pos++];
                            output.emplace_back(
                                (node->x_max_ + node->x_min_) / 2,
                                (node->y_max_ + node->y_min_) / 2,
                                run_depth
                            );
                        }
                        leaf_read_pos ++;
                    }
                    else
                    {
                        run_depth += depth_block[depth_read_pos++];
                        output.emplace_back(
                            (node->x_max_ + node->x_min_) / 2,
                            (node->y_max_ + node->y_min_) / 2,
                            run_depth
                        );
                    }
                }
                else
                {
                    int x_idx = j / 2;
                    int y_idx = j % 2;
                    node->children_[j] = new QuadtreeCompressor::Node(
                        x_segs[x_idx], x_segs[x_idx + 1],
                        y_segs[y_idx], y_segs[y_idx + 1]
                    );
                    q.push(node->children_[j]);
                }
            }
            q.pop();
        }
    }
    return output.size();
}

void QuadtreeCompressor::set_boundary()
{
    // get boundary of the point cloud
    int x_min = std::numeric_limits<int>().max();
    int x_max = std::numeric_limits<int>().min();
    int y_min = std::numeric_limits<int>().max();
    int y_max = std::numeric_limits<int>().min();

    for (size_t i = 0; i < cloud_->size(); i++)
    {
        if (cloud_->at(i).x > x_max) { x_max = cloud_->at(i).x; }
        if (cloud_->at(i).x < x_min) { x_min = cloud_->at(i).x; }
        if (cloud_->at(i).y > y_max) { y_max = cloud_->at(i).y; }
        if (cloud_->at(i).y < y_min) { y_min = cloud_->at(i).y; }

        root_.point_ids_.push_back(i);
    }

    // compute the boundary of the quad tree
    root_.x_min_ = x_min - 1;
    root_.y_min_ = y_min - 1;
    do {
        depth_ += 1;
        root_.x_max_ = root_.x_min_ + resolution_ * std::pow(2, depth_ - 1);
        root_.y_max_ = root_.y_min_ + resolution_ * std::pow(2, depth_ - 1);
    }
    while (root_.x_max_ < x_max || root_.y_max_ < y_max);
}

void QuadtreeCompressor::serialize_tree (std::vector<int>& blocks)
{
    
    std::queue<QuadtreeCompressor::Node*> q;
    q.push(&root_);

    while (!q.empty())
    {
        size_t cur_size = q.size();
        for (size_t i = 0; i < cur_size; i++)
        {
            QuadtreeCompressor::Node* node = q.front();

            unsigned char o = 0;
            for (int j = 0; j < 4; j++)
            {
                o |= !!node->children_[j] << j;
            }
            q.pop();
            if (!o) continue;
            blocks.push_back(o);

            for (int j = 0; j < 4; j++)
            {
                if (node->children_[j])
                {
                    q.push(node->children_[j]);
                }
            }
        }
    }
}

void QuadtreeCompressor::serialize_leafs (std::vector<int>& blocks)
{
    std::queue<QuadtreeCompressor::Node*> q;
    q.push(&root_);
    int depth = 0;

    while (!q.empty())
    {
        depth += 1;
        size_t cur_size = q.size();
        for (size_t i = 0; i < cur_size; i++)
        {
            QuadtreeCompressor::Node* node = q.front();

            unsigned char o = 0;
            for (int j = 0; j < 4; j++)
            {
                o |= !!node->children_[j] << j;
            }
            q.pop();
            if (depth_ == depth) 
            {
                blocks.push_back(node->point_ids_.size());
            }

            for (int j = 0; j < 4; j++)
            {
                if (node->children_[j])
                {
                    q.push(node->children_[j]);
                }
            }
        }
    }
}

void QuadtreeCompressor::serialize_depth (std::vector<int>& blocks)
{
    std::queue<QuadtreeCompressor::Node*> q;
    q.push(&root_);
    int depth = 0;

    while (!q.empty())
    {
        depth += 1;
        size_t cur_size = q.size();
        for (size_t i = 0; i < cur_size; i++)
        {
            QuadtreeCompressor::Node* node = q.front();

            unsigned char o = 0;
            for (int j = 0; j < 4; j++)
            {
                o |= !!node->children_[j] << j;
            }
            q.pop();
            if (depth_ == depth) 
            {
                for (auto pid: node->point_ids_)
                {
                    blocks.push_back(cloud_->at(pid).z);
                }
            }

            for (int j = 0; j < 4; j++)
            {
                if (node->children_[j])
                {
                    q.push(node->children_[j]);
                }
            }
        }
    }
}

void QuadtreeCompressor::serialize_leaf_delta (std::vector<int>& blocks)
{
    serialize_leaf_delta_recursive(&root_, blocks);
}

void QuadtreeCompressor::serialize_leaf_delta_recursive (Node* node, std::vector<int>& blocks)
{
    unsigned char o = 0;
    for (int j = 0; j < 4; j++)
    {
        o |= !!node->children_[j] << j;
    }
    if (!o)
    {
        for (auto id: node->point_ids_)
        {
            blocks.push_back(cloud_->at(id).x - node->x_min_);
            blocks.push_back(cloud_->at(id).y - node->y_min_);
        }
    }
    
    for (int j = 0; j < 4; j++)
    {
        if (node->children_[j])
        {
            serialize_leaf_delta_recursive(node->children_[j], blocks);
        }
    }
}
