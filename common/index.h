#ifndef COMMON_INDEX_H
#define COMMON_INDEX_H

#include <string.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/config.h"
#include "common/types.h"


template <typename T>
class GridIndex
{
protected:
    T grid_size_;
    std::unordered_map<int, int> cell_start_;
    std::unordered_map<int, int> cell_end_;
    int *ids_;
    int size_;
    
public:
    GridIndex(const T grid_size)
    : grid_size_(grid_size)
    , cell_start_{}
    , cell_end_{}
    , ids_(nullptr)
    , size_(0u) {}

    virtual int size () const
    {
        return size_;
    }

    virtual int get_nearest_nbr(
        int cur_id, T max_dis, std::vector<bool>& visited, 
        float& min_dis, bool with_dir, bool positive
    ) const = 0;
};

template<typename T>
class PointCloudIndex: public GridIndex<T>
{
private:
    const PointCloud<T> *cloud_;
    
    T min_x_, min_y_, min_z_;
    int ngrids_x_, ngrids_y_, ngrids_z_;

public:
    PointCloudIndex (const T grid_size)
    : GridIndex<T>(grid_size)
    , cloud_(nullptr)
    , min_x_(std::numeric_limits<T>::max())
    , min_y_(std::numeric_limits<T>::max())
    , min_z_(std::numeric_limits<T>::max())
    , ngrids_x_(0u)
    , ngrids_y_(0u)
    , ngrids_z_(0u)
    {}

    int idx(int x_idx, int y_idx, int z_idx) const
    {
        return x_idx * ngrids_y_ * ngrids_z_ + 
               y_idx * ngrids_z_ + z_idx;
    }

    void set_input (const PointCloud<T>& cloud)
    {
        this->size_ = cloud.size();
        cloud_ = &cloud;
        this->ids_ = new int[cloud.size() + 1];

        T max_x = std::numeric_limits<T>::min();
        T max_y = std::numeric_limits<T>::min();
        T max_z = std::numeric_limits<T>::min();
        for (auto point: cloud) {
            if (point.x < min_x_) { min_x_ = point.x; }
            if (point.x > max_x) { max_x = point.x; }
            if (point.y < min_y_) { min_y_ = point.y; }
            if (point.y > max_y) { max_y = point.y; }
            if (point.z < min_z_) { min_z_ = point.z; }
            if (point.z > max_z) { max_z = point.z; }
        }

        ngrids_x_ = std::floor((max_x - min_x_) / this->grid_size_) + 1;
        ngrids_y_ = std::floor((max_y - min_y_) / this->grid_size_) + 1;
        ngrids_z_ = std::floor((max_z - min_z_) / this->grid_size_) + 1;

        long ngrids_xy = ngrids_x_ * ngrids_y_;
        long ngrids = ngrids_xy * ngrids_z_;
        if (ngrids_xy > std::numeric_limits<int>::max() || ngrids > std::numeric_limits<int>::max())
        {
            std::cout << "number of grids greater than INT_MAX!" << std::endl;
            exit(-1);
        }
        
        std::unordered_map<int, std::vector<int>> hist;
        for (size_t i = 0; i < cloud_->size (); i++) {
            auto& point = cloud_->at(i);
            int x_idx = (point.x - min_x_) / this->grid_size_;
            int y_idx = (point.y - min_y_) / this->grid_size_;
            int z_idx = (point.z - min_z_) / this->grid_size_;
            hist[idx(x_idx, y_idx, z_idx)].push_back(i);
        }

        int write_pos = 0;
        for (auto & [i, vs]: hist)
        {
            this->cell_start_[i] = write_pos;
            memcpy(&this->ids_[write_pos], vs.data(), vs.size() * sizeof(int));
            write_pos += vs.size();
            this->cell_end_[i] = write_pos;
        }
    }

    int get_nearest_nbr(
        int cur_id, T max_dis, std::vector<bool>& visited, 
        float& min_dis, bool with_dir = false, bool positive = true
    ) const {
        if (with_dir == true)
        {
            std::cout << "with_dir not supported in PointCloudIndex." << std::endl;
            exit(-1);
        }
        auto& cur_point = cloud_->at(cur_id);
        int x_idx = (cur_point.x - min_x_) / this->grid_size_;
        int y_idx = (cur_point.y - min_y_) / this->grid_size_;
        int z_idx = (cur_point.z - min_z_) / this->grid_size_;

        int span_grids = std::ceil(max_dis / static_cast<float>(this->grid_size_));
        
        min_dis = std::numeric_limits<float>::max();
        int min_dis_idx = -1;

        for (int i = x_idx - span_grids; i <= x_idx + span_grids; i++)
        for (int j = y_idx - span_grids; j <= y_idx + span_grids; j++)
        for (int k = z_idx - span_grids; k <= z_idx + span_grids; k++)
        {
            if (this->cell_start_.find(idx(i, j, k)) == this->cell_start_.end())
                continue;

            for (int m = this->cell_start_.at(idx(i, j, k));
                    m < this->cell_end_.at(idx(i, j, k)); m++)
            {
                int id = this->ids_[m];
                if (visited[id]) continue;

                Point<T> diff = Point<T>::diff(cur_point, cloud_->at(id));
                float dis = std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
                if (dis < min_dis)
                {
                    min_dis = dis;
                    min_dis_idx = id;
                }
            }
        }
        return min_dis_idx;
    }
};


template<typename T>
class SPointCloudIndex: public GridIndex<T>
{
private:
    const SPointCloud<T> *spoint_cloud_;
    
    T min_x_, min_y_;
    int ngrids_x_, ngrids_y_, ngrids_z_;

public:
    SPointCloudIndex (const T grid_size)
    : GridIndex<T>(grid_size)
    , spoint_cloud_(nullptr)
    , min_x_(std::numeric_limits<T>::max())
    , min_y_(std::numeric_limits<T>::max())
    , ngrids_x_(0u)
    , ngrids_y_(0u)
    {}

    int idx(int x_idx, int y_idx) const
    {
        return x_idx * ngrids_y_ + y_idx;
    }

    void set_input (const SPointCloud<T>& cloud)
    {
        this->size_ = cloud.size();
        spoint_cloud_ = &cloud;
        this->ids_ = new int[cloud.size() + 1];

        T max_x = std::numeric_limits<T>::min();
        T max_y = std::numeric_limits<T>::min();
        for (auto point: cloud) {
            if (point.xy_ang < min_x_) { min_x_ = point.xy_ang; }
            if (point.xy_ang > max_x) { max_x = point.xy_ang; }
            if (point.z_ang < min_y_) { min_y_ = point.z_ang; }
            if (point.z_ang > max_y) { max_y = point.z_ang; }
        }
        
        ngrids_x_ = std::floor((max_x - min_x_) / this->grid_size_) + 1;
        ngrids_y_ = std::floor((max_y - min_y_) / this->grid_size_) + 1;

        long ngrids_xy = ngrids_x_ * ngrids_y_;
        if (ngrids_xy > std::numeric_limits<int>::max())
        {
            std::cout << "number of grids greater than INT_MAX!" << std::endl;
            exit(-1);
        }

        std::unordered_map<int, std::vector<int>> hist;
        for (size_t i = 0; i < spoint_cloud_->size (); i++) {
            auto& point = spoint_cloud_->at(i);
            int x_idx = (point.xy_ang - min_x_) / this->grid_size_;
            int y_idx = (point.z_ang - min_y_) / this->grid_size_;
            hist[idx(x_idx, y_idx)].push_back(i);
        }

        int write_pos = 0u;
        for (auto & [i, vs]: hist)
        {
            this->cell_start_[i] = write_pos;
            memcpy(&this->ids_[write_pos], vs.data(), vs.size() * sizeof(int));
            write_pos += vs.size();
            this->cell_end_[i] = write_pos;
        }
    }

    /**
     * @brief get the nearest neighbor of a Point<T> on rang image
     * @param cur_id id of the current Point<T>
     * @param max_dis max length
     * @param visited visited array
     * @param min_dis the distance between current SPoint<T> and its nearest neighbor
     * @param with_dir true if only search SPoint<T> on the same line on spherical cloud
     * @param positive true if search right along the line
     * @return the id of the nearest neighbor
     */ 
    int get_nearest_nbr(
        int cur_id, T max_dis, std::vector<bool>& visited, 
        float& min_dis, bool with_dir, bool positive
    ) const {
        auto& cur_point = spoint_cloud_->at(cur_id);
        int x_idx = (cur_point.xy_ang - min_x_) / this->grid_size_;
        int y_idx = (cur_point.z_ang - min_y_) / this->grid_size_;
        float y_mod = std::fmod(cur_point.z_ang - min_y_, this->grid_size_);
        
        int span_grids = std::ceil(max_dis / static_cast<float>(this->grid_size_));
        
        min_dis = std::numeric_limits<float>::max();
        int min_dis_idx = -1;

        int x_start = positive ? x_idx : x_idx - span_grids;
        int x_end = positive ? x_idx + span_grids : x_idx;
        
        int y_start = y_mod < this->grid_size_ / 2 ? y_idx - 1 : y_idx;
        int y_end = y_mod < this->grid_size_ / 2 ? y_idx : y_idx + 1;

        for (int i = x_start; i <= x_end; i++)
        for (int j = y_start; j <= y_end; j++)
        {
            if (this->cell_start_.find(idx(i, j)) == this->cell_start_.end())
                continue;

            for (int m = this->cell_start_.at(idx(i, j));
                    m < this->cell_end_.at(idx(i, j)); m++)
            {
                int id = this->ids_[m];
                if (visited[id]) continue;

                SPoint<T> diff = SPoint<T>::diff(cur_point, spoint_cloud_->at(id));
                if (with_dir && (
                        positive != !!(diff.xy_ang >= 0) || 
                        std::abs(diff.z_ang) > max_dis * conf::pl_phi_theta ||
                        std::abs(diff.xy_ang) > max_dis
                )) continue;

                float distance = std::sqrt(
                    diff.xy_ang * diff.xy_ang + 
                    diff.z_ang * diff.z_ang * conf::pl_phi_theta * conf::pl_phi_theta + 
                    diff.depth * diff.depth * conf::pl_depth_theta * conf::pl_depth_theta
                );
                if (distance < min_dis) {
                    min_dis = distance;
                    min_dis_idx = id;
                }
            }
        }
        return min_dis_idx;
    }
};

#endif
