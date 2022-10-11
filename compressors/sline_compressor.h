#ifndef SLINE_COMPRESSOR
#define SLINE_COMPRESSOR

#include <deque>
#include <vector>

#include "common/types.h"
#include "common/sensor_meta.h"
#include "compressors/base.h"
#include "compressors/general.h"


/**
 * @brief extracting lines on spherical point cloud and
 *      perform delta coding on each line.
 * @note angle resolution = resolution / max_depth
 */
class SLineCompressor
: public Compressor<SPointCloud<float>> {
protected:
    const SPointCloud<float>* cloud_;
    float max_dis_;
    float resolution_;
    float max_depth_;
    float grid_size_;
    conf::SensorMeta& sensor_;

    std::vector<std::deque<SPoint<int>>> lines_;
    std::vector<int> remaining_points_;
public:
    SLineCompressor(
        const float max_dis, 
        const float resolution, 
        const float grid_size,
        conf::SensorMeta& sensor);

    virtual size_t compress (
        const SPointCloud<float>& input, 
        std::vector<unsigned char>& output);

    virtual std::vector<int>& get_outliers();
protected:
    virtual void find_polylines();
};


/**
 * @brief extracting lines on spherical point cloud and
 *      perform delta coding on each line.
 * @note angle resolution = resolution / max_depth
 */
class SLineCompressorInterLine
: public SLineCompressor {
private:
    enum REF_TYPE {
        BOTTOM_LEFT = 0,
        UPPER_RIGHT,
        UPPER_MIDDLE,
        UPPER_LEFT
    };
    OffTheShelf off_the_shelf_;
public:
    SLineCompressorInterLine(
        const float max_dis, 
        const float resolution, 
        const float grid_size,
        conf::SensorMeta& sensor,
        OffTheShelf off_the_shelf);

    virtual size_t compress (
        const SPointCloud<float>& input, 
        std::vector<unsigned char>& output);

    virtual size_t decompress (
        const std::vector<unsigned char>& input,
        SPointCloud<float>& output);
protected:
    virtual void generate_consensus_reference_line(
        const std::deque<int>& ref_indices,
        std::deque<SPoint<int>>& ref_line
    );

    virtual const SPoint<int> *get_ref_point(
        const std::deque<SPoint<int>>& ref_line,
        const std::deque<SPoint<int>>& cur_line,
        const int& j,
        const int& ref_point_th,
        const int& depth_diff_th,
        size_t& left_pos,
        size_t& right_pos,
        REF_TYPE& type,
        bool decode = false
    );

    virtual const SPoint<int> *get_ref_point(
        const std::deque<SPoint<int>>& ref_line,
        const SPoint<int>& cur_point,
        const int& ref_point_th,
        const int& depth_diff_th
    );
};


/**
 * @brief divide lines into several groups
 *      depending on the depth.
 */
class SLineCompressorMultiLevels
: public SLineCompressor {
    std::vector<std::deque<int>> lines_;
public:
    SLineCompressorMultiLevels(
        const float max_dis, 
        const float resolution, 
        const float grid_size,
        conf::SensorMeta& sensor);

    virtual size_t compress (
        const SPointCloud<float>& input, 
        std::vector<unsigned char>& output);
protected:
    virtual void find_polylines();
};

#endif
