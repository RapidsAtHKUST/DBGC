#ifndef QUADTREE_COMPRESSOR
#define QUADTREE_COMPRESSOR

#include <vector>

#include "common/types.h"
#include "compressors/base.h"


class QuadtreeCompressor
: public Compressor<PointCloud<int>> {
private:
    struct Node {
        std::vector<int> point_ids_;
        Node *children_[4] = {};
        int x_min_, x_max_, y_min_, y_max_;

        Node ();
        Node (int x_min, int x_max, int y_min, int y_max);

        void build_recursive (
            const PointCloud<int>& cloud, 
            const int min_resolution);
    };

public:
    Node root_;
    int depth_;
    int min_depth_;
    int resolution_;
    const PointCloud<int> *cloud_;

    QuadtreeCompressor(int resolution = 2);
    size_t compress (
        const PointCloud<int>& input, 
        std::vector<unsigned char>& output);
    size_t decompress (
        const std::vector<unsigned char>& input, 
        PointCloud<int>& output);
    
private:
    void set_boundary();

    void serialize_tree (std::vector<int>& blocks);
    void serialize_tree_recursive (Node* node, std::vector<int>& blocks);
    
    void serialize_leafs (std::vector<int>& blocks);
    void serialize_leafs_recursive (Node* node, std::vector<int>& blocks);
    void serialize_depth (std::vector<int>& blocks);

    void serialize_leaf_delta (std::vector<int>& blocks);
    void serialize_leaf_delta_recursive (Node* node, std::vector<int>& blocks);

};

#endif