#include <algorithm>    /* For sort */
#include <chrono>
#include <deque>
#include <iostream>
#include <numeric>      /* For accumulate */
#include <vector>

#include "common/config.h"
#include "common/types.h"
#include "common/index.h"
#include "common/polylines.h"
#include "common/sensor_meta.h"
#include "compressors/sline_compressor.h"
#include "compressors/entropy.h"
#include "compressors/general.h"


SLineCompressor::SLineCompressor(
    const float max_dis, 
    const float resolution, 
    const float grid_size,
    conf::SensorMeta& sensor)
: cloud_(nullptr)
, max_dis_(max_dis)
, resolution_(resolution)
, max_depth_(0.0f)
, grid_size_(grid_size)
, sensor_(sensor)
, lines_{}
, remaining_points_{}
{}

size_t SLineCompressor::compress (
    const SPointCloud<float>& input, 
    std::vector<unsigned char>& output
) {
    output.clear();
    if (input.empty()) return 0UL;

    cloud_ = &input;
    
    max_depth_ = get_max_depth(input);
    std::cout << "max_depth: " << max_depth_ << "\nang_resolution: " 
        << resolution_ / max_depth_ << std::endl;
    
    find_polylines();

    // intra-line coding
    std::vector <int> lengths, dx, dy, ddepth;
    std::vector <unsigned char> lengths_chunk, 
        dx_chunk, dy_chunk, ddepth_chunk;
    size_t lengths_size, dx_size, dy_size, ddepth_size;

    for (size_t i = 0; i < lines_.size(); i++)
    {
        auto& cur_line = lines_[i];

        lengths.push_back(cur_line.size());
        dx.push_back(cur_line.front().xy_ang);
        dy.push_back(cur_line.front().z_ang);
        ddepth.push_back(cur_line.front().depth);

        for (size_t j = 0; j < cur_line.size() - 1; j++)
        {
            dx.push_back(cur_line[j + 1].xy_ang - cur_line[j].xy_ang);
            dy.push_back(cur_line[j + 1].z_ang - cur_line[j].z_ang);
            ddepth.push_back(cur_line[j + 1].depth - cur_line[j].depth);
        }
    }

    // organize output
    EntropyCompressor compressor;
    DeflateCompressor<int> deflate;
    
    lengths_size = deflate.compress(lengths, lengths_chunk);
    dx_size = deflate.compress(dx, dx_chunk);
    dy_size = compressor.compress(dy, dy_chunk);
    ddepth_size = compressor.compress(ddepth, ddepth_chunk);

    size_t write_pos = 0UL;

    output.resize(
        sizeof(size_t) + lengths_size +
        sizeof(size_t) + dx_size +
        sizeof(size_t) + dy_size +
        sizeof(size_t) + ddepth_size
    );
    memcpy(output.data() + write_pos, &lengths_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, lengths_chunk.data(), sizeof(lengths_size));
    write_pos += sizeof(lengths_size);

    memcpy(output.data() + write_pos, &dx_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, dx_chunk.data(), sizeof(dx_size));
    write_pos += sizeof(dx_size);

    memcpy(output.data() + write_pos, &dy_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, dy_chunk.data(), sizeof(dy_size));
    write_pos += sizeof(dy_size);

    memcpy(output.data() + write_pos, &ddepth_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, ddepth_chunk.data(), sizeof(ddepth_size));
    write_pos += sizeof(ddepth_size);

    return output.size();
}

void SLineCompressor::find_polylines()
{
    const float ang_resolution = resolution_ / max_depth_;

    SPointCloudIndex<float> index (grid_size_);
    index.set_input(*cloud_);

    std::vector<bool> visited(cloud_->size(), false);

    for (size_t i = 0; i < cloud_->size(); i++)
    {
        if (visited[i]) continue;

        visited[i] = true;
        std::deque <int> line {static_cast<int>(i)};
        
        extend_polyline<SPointCloudIndex<float>>(
            index, max_dis_, visited, line, true
        );
        
        if (line.size() > 5) 
        {
            lines_.emplace_back();
            for (auto j: line)
            {
                lines_.back().emplace_back(
                    std::round(cloud_->at(j).xy_ang / ang_resolution),
                    std::round(cloud_->at(j).z_ang / ang_resolution),
                    std::round(cloud_->at(j).depth / resolution_)
                );
            }
        }
        else
        {
            for (auto j: line)
            {
                visited[j] = false;
            }
        }
    }
    for (size_t i = 0; i < cloud_->size(); i++)
    {
        if (!visited[i])
            remaining_points_.push_back(i);
    }

    std::cout << "number of lines_: " << lines_.size() << std::endl;
    int num_points = std::accumulate(lines_.begin(), lines_.end(), 0, 
        [](int np, std::deque<SPoint<int>>& line){
            return np + line.size();
        }
    );
    std::cout << "number of points: " << num_points << std::endl;
    
}

std::vector<int>& SLineCompressor::get_outliers ()
{
    return remaining_points_;
}


SLineCompressorInterLine::SLineCompressorInterLine(
    const float max_dis, 
    const float resolution, 
    const float grid_size,
    conf::SensorMeta& sensor,
    OffTheShelf off_the_shelf)
: SLineCompressor(max_dis, resolution, grid_size, sensor)
, off_the_shelf_(off_the_shelf)
{}

size_t SLineCompressorInterLine::compress (
    const SPointCloud<float>& input, 
    std::vector<unsigned char>& output
) {
    using clock = std::chrono::high_resolution_clock;
    using duration = std::chrono::duration<double>;
    clock::time_point start, end;
    duration diff;

    output.clear();
    if (input.empty()) return 0UL;

    cloud_ = &input;
    
    max_depth_ = get_max_depth(input);
    std::cout << "max_depth: " << max_depth_ << "\nang_resolution: " 
        << resolution_ / max_depth_ << std::endl;
    
    TIME_START();
    find_polylines();
    TIME_END("find polylines");

    std::sort(lines_.begin(), lines_.end(), 
        [](std::deque<SPoint<int>>& t1, std::deque<SPoint<int>>& t2){
            if (t1.front().z_ang == t2.front().z_ang)
                return t1.front().xy_ang < t2.front().xy_ang;
            else
                return t1.front().z_ang < t2.front().z_ang;
        }
    );
    
    TIME_START();
    // inter-line coding
    std::vector <int> lengths, head_dx, head_dy, head_ddepth,
        dx, dy, ddepth, bit_indicator;
    std::vector<unsigned char> dx_char;

    std::vector <unsigned char> lengths_chunk, head_dx_chunk,
        head_dy_chunk, head_ddepth_chunk, 
        dx_chunk, dy_chunk, ddepth_chunk, bit_chunk;
    size_t lengths_size, head_dx_size, head_dy_size, head_ddepth_size,
        dx_size, dy_size, ddepth_size, bit_indicator_size;

    // thresholds
    const float ang_resolution = resolution_ / max_depth_;
    const int ref_point_th = std::round(sensor_.theta_range / sensor_.width / ang_resolution);
    const int z_ang_th = std::round((sensor_.phi_max - sensor_.phi_min) / sensor_.height / ang_resolution);
    const int depth_diff_th = conf::depth_diff_th / resolution_;

    // a deque storing all the reference lines_ of current line
    std::deque<int> ref_indices;

    for (size_t i = 0; i < lines_.size(); i++)
    {
        auto& cur_line = lines_[i];
        
        // get the reference lines_ of the current line
        while (
            !ref_indices.empty() && 
            cur_line.front().z_ang - lines_[ref_indices.front()].front().z_ang > z_ang_th
        ) {
            ref_indices.pop_front();
        }

        // consensus reference line
        std::deque<SPoint<int>> ref_line;
        generate_consensus_reference_line(ref_indices, ref_line);

        lengths.push_back(cur_line.size());
        if (i > 0) // skip the head of the first line
        {
            auto& pre_line = lines_[i - 1];
            const auto *ref_point = get_ref_point(ref_line,
                cur_line.front(), ref_point_th, depth_diff_th);

            head_dx.push_back(cur_line.front().xy_ang - pre_line.front().xy_ang);
            head_dy.push_back(cur_line.front().z_ang - pre_line.front().z_ang);
            if (ref_point)
                head_ddepth.push_back(cur_line.front().depth - ref_point->depth);
            else
                head_ddepth.push_back(cur_line.front().depth - pre_line.front().depth);
            //head_ddepth.push_back(cur_line.front().depth);
        }
        // align the line and its reference lines_
        size_t left_pos = 0UL, right_pos = 0UL;

        for (size_t j = 0; j < cur_line.size() - 1; j++)
        {
            REF_TYPE type;
            const auto *ref_point = get_ref_point(ref_line,
                cur_line, j, ref_point_th, depth_diff_th,
                left_pos, right_pos, type);

            dx.push_back(cur_line[j + 1].xy_ang - cur_line[j].xy_ang);
            dy.push_back(cur_line[j + 1].z_ang - cur_line[j].z_ang);
            if (!ref_point)
                ddepth.push_back(cur_line[j + 1].depth - cur_line[j].depth);
            else
            {
                ddepth.push_back(cur_line[j + 1].depth - ref_point->depth);
                bit_indicator.push_back(type);
            }
            /*if (!ref_point)
                ddepth.push_back(cur_line[j + 1].depth - cur_line[j].depth);
            else
                ddepth.push_back(cur_line[j + 1].depth - ref_point->depth);
            */
        }
        ref_indices.push_back(i);
    }
    TIME_END("coding");

    bool dx_to_char = false;
    // convert dx from int vector to unsigned char vector
    if (!dx.empty() && *std::max_element(dx.begin(), dx.end()) < 256)
        dx_to_char = true;

    // organize output
    EntropyCompressor compressor;
    DeflateCompressor<unsigned char> deflate_uc;
    DeflateCompressor<int> deflate;
    DeflateCompressor<unsigned char> lzma_uc;
    LzmaCompressor<int> lzma;
    DeflateCompressor<unsigned char> bz2_uc;
    BZip2Compressor<int> bz2;
    
    lengths_size = compressor.compress(lengths, lengths_chunk);
    head_dy_size = compressor.compress(head_dy, head_dy_chunk);
    
    dy_size = compressor.compress(dy, dy_chunk);
    bit_indicator_size = compressor.compress(bit_indicator, bit_chunk);
    switch (off_the_shelf_)
    {
        case OffTheShelf::deflate_:
        {
            TIME_START();
            head_ddepth_size = deflate.compress(head_ddepth, head_ddepth_chunk);
            ddepth_size = deflate.compress(ddepth, ddepth_chunk);
            
            head_dx_size = deflate.compress(head_dx, head_dx_chunk);
            // convert dx from int vector to unsigned char vector
            if (dx_to_char)
            {
                dx_char.assign (dx.begin(), dx.end());
                dx_size = deflate_uc.compress(dx_char, dx_chunk);
            }
            else
                dx_size = deflate.compress(dx, dx_chunk);
            TIME_END("encode arrays");
            break;
        }
        case OffTheShelf::lzma_:
        {
            TIME_START();
            head_ddepth_size = lzma.compress(head_ddepth, head_ddepth_chunk);
            ddepth_size = lzma.compress(ddepth, ddepth_chunk);

            head_dx_size = lzma.compress(head_dx, head_dx_chunk);
            // convert dx from int vector to unsigned char vector
            if (dx_to_char)
            {
                dx_char.assign (dx.begin(), dx.end());
                dx_size = lzma_uc.compress(dx_char, dx_chunk);
            }
            else
                dx_size = lzma.compress(dx, dx_chunk);
            TIME_END("encode arrays");
            break;
        }
        case OffTheShelf::bzip2_:
        {
            TIME_START();
            head_ddepth_size = bz2.compress(head_ddepth, head_ddepth_chunk);
            ddepth_size = bz2.compress(ddepth, ddepth_chunk);
            
            head_dx_size = bz2.compress(head_dx, head_dx_chunk);
            // convert dx from int vector to unsigned char vector
            if (dx_to_char)
            {
                dx_char.assign (dx.begin(), dx.end());
                dx_size = bz2_uc.compress(dx_char, dx_chunk);
            }
            else
                dx_size = bz2.compress(dx, dx_chunk);
            TIME_END("encode arrays");
            break;
        }
        default:
        {
            TIME_START();
            head_ddepth_size = compressor.compress(head_ddepth, head_ddepth_chunk);
            ddepth_size = compressor.compress(ddepth, ddepth_chunk);
            TIME_END("encode arrays 1");

            TIME_START();
            size_t temp_size = 0UL;

            head_dx_size = deflate.compress(head_dx, head_dx_chunk);
            temp_size += head_dx.size() * 4;
            // convert dx from int vector to unsigned char vector
            if (dx_to_char)
            {
                dx_char.assign (dx.begin(), dx.end());
                dx_size = deflate_uc.compress(dx_char, dx_chunk);
                temp_size += dx_char.size();
            }
            else
            {
                dx_size = deflate.compress(dx, dx_chunk);
                temp_size += dx.size() * 4;
            }
            std::cout << "temp_size: " << temp_size << std::endl;
            TIME_END("encode arrays 2");
            break;
        }
    }

    std::cout << "names: lengths_size head_dx_size head_dy_size head_ddepth_size " <<
        "dx_size dy_size ddepth_size bit_indicator_size" << std::endl;

    std::cout << "sizes: " << lengths_size << " " << head_dx_size << " " << 
        head_dy_size << " " << head_ddepth_size << " " << dx_size << " " <<
        dy_size << " " << ddepth_size << " " << bit_indicator_size << std::endl;

    size_t write_pos = 0UL;

    output.resize(
        sizeof(float) +     // max_depth_
        sizeof(int) * 3 +   // first head coordinates
        sizeof(size_t) + lengths_size +
        sizeof(size_t) + head_dx_size +
        sizeof(size_t) + head_dy_size +
        sizeof(size_t) + head_ddepth_size +
        sizeof(bool) + sizeof(size_t) + dx_size +
        sizeof(size_t) + dy_size +
        sizeof(size_t) + ddepth_size +
        sizeof(size_t) + bit_chunk.size()
    );
    memcpy(output.data() + write_pos, &max_depth_, sizeof(float));
    write_pos += sizeof(float);
    if (lines_.empty())
    {
        int zero = 0;
        memcpy(output.data() + write_pos, &zero, sizeof(int));
        write_pos += sizeof(int);
        memcpy(output.data() + write_pos, &zero, sizeof(int));
        write_pos += sizeof(int);
        memcpy(output.data() + write_pos, &zero, sizeof(int));
        write_pos += sizeof(int);
    }
    else
    {
        memcpy(output.data() + write_pos, &lines_[0].front().xy_ang, sizeof(int));
        write_pos += sizeof(int);
        memcpy(output.data() + write_pos, &lines_[0].front().z_ang, sizeof(int));
        write_pos += sizeof(int);
        memcpy(output.data() + write_pos, &lines_[0].front().depth, sizeof(int));
        write_pos += sizeof(int);
    }
    
    memcpy(output.data() + write_pos, &lengths_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, lengths_chunk.data(), lengths_size);
    write_pos += lengths_size;

    memcpy(output.data() + write_pos, &head_dx_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, head_dx_chunk.data(), head_dx_size);
    write_pos += head_dx_size;

    memcpy(output.data() + write_pos, &head_dy_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, head_dy_chunk.data(), head_dy_size);
    write_pos += head_dy_size;

    memcpy(output.data() + write_pos, &head_ddepth_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, head_ddepth_chunk.data(), head_ddepth_size);
    write_pos += head_ddepth_size;

    memcpy(output.data() + write_pos, &dx_to_char, sizeof(bool));
    write_pos += sizeof(bool);
    
    memcpy(output.data() + write_pos, &dx_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, dx_chunk.data(), dx_size);
    write_pos += dx_size;

    memcpy(output.data() + write_pos, &dy_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, dy_chunk.data(), dy_size);
    write_pos += dy_size;

    memcpy(output.data() + write_pos, &ddepth_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, ddepth_chunk.data(), ddepth_size);
    write_pos += ddepth_size;

    memcpy(output.data() + write_pos, &bit_indicator_size, sizeof(size_t));
    write_pos += sizeof(size_t);
    memcpy(output.data() + write_pos, bit_chunk.data(), bit_indicator_size);
    write_pos += bit_indicator_size;

    return output.size();
}

void SLineCompressorInterLine::
generate_consensus_reference_line(
    const std::deque<int>& ref_indices,
    std::deque<SPoint<int>>& ref_line
) {
    ref_line.clear();

    for (auto iter = ref_indices.begin(); iter != ref_indices.end(); iter++)
    {
        auto& cur_ref_line = lines_[*iter];

        if (ref_line.empty())
        {
            ref_line = cur_ref_line;
        }
        else
        {
            // find the place to insert
            size_t left_insert_pos = ref_line.size();
            for (size_t j = 0; j < ref_line.size(); j++)
            {
                if (cur_ref_line.front().xy_ang < ref_line[j].xy_ang)
                {
                    left_insert_pos = j;
                    break;
                }
            }
            size_t resume_pos = left_insert_pos;
            for (size_t j = left_insert_pos; j < ref_line.size(); j++)
            {
                if (cur_ref_line.back().xy_ang < ref_line[j].xy_ang)
                {
                    resume_pos = j;
                    break;
                }
            }
            ref_line.erase(
                ref_line.begin() + left_insert_pos, 
                ref_line.begin() + resume_pos);
            
            ref_line.insert(
                ref_line.begin() + left_insert_pos,
                cur_ref_line.begin(),
                cur_ref_line.end());
        }
    }
}

const SPoint<int> *SLineCompressorInterLine::
get_ref_point(
    const std::deque<SPoint<int>>& ref_line,
    const std::deque<SPoint<int>>& cur_line,
    const int& j,
    const int& ref_point_th,
    const int& depth_diff_th,
    size_t& left_pos,
    size_t& right_pos,
    REF_TYPE& type,
    bool decode
) {
    // set left_pos as the righ_most point in ref_line
    // whose xy_ang is not greater than cur_line[j].xy_ang
    while (
        left_pos < ref_line.size() && 
        ref_line[left_pos].xy_ang <= cur_line[j].xy_ang
    ) {
        left_pos++;
    }
    left_pos --;

    // discard unsatisfiable left_pos
    if (
        left_pos + 1 == 0 ||                // out of bound
        left_pos + 1 >= ref_line.size() ||  // out of bound
        cur_line[j].xy_ang - ref_line[left_pos].xy_ang > ref_point_th
    ) {
        return nullptr;
    }
    else
    {
        // compute right_pos if left_pos is valid
        right_pos = left_pos + 1;
        while (
            right_pos < ref_line.size() && 
            ref_line[right_pos].xy_ang < cur_line[j + 1].xy_ang
        ) {
            right_pos++;
        }

        // discard unsatisfiable right_pos
        if (
            right_pos >= ref_line.size() ||
            ref_line[right_pos].xy_ang - cur_line[j + 1].xy_ang > ref_point_th)
        {
            return nullptr;
        }
        else
        {
            auto& upper_left = ref_line[left_pos];
            auto& upper_right = ref_line[right_pos];
            auto& upper_middle = ref_line[right_pos - 1];
            auto& bottom_left = cur_line[j];
            auto& bottom_right = cur_line[j + 1];
            if (
                std::abs(upper_right.depth - upper_left.depth) > depth_diff_th ||
                std::abs(upper_right.depth - bottom_left.depth) > depth_diff_th ||
                std::abs(upper_left.depth - bottom_left.depth) > depth_diff_th
            ) {
                if (!decode)
                {
                    const float dis1 = std::abs(upper_left.depth - bottom_right.depth);
                    const float dis2 = std::abs(upper_middle.depth - bottom_right.depth);
                    const float dis3 = std::abs(upper_right.depth - bottom_right.depth);
                    const float dis4 = std::abs(bottom_left.depth - bottom_right.depth);
                    if (dis1 < dis2 && dis1 < dis3 && dis1 < dis4)
                    {
                        type = UPPER_LEFT;
                        return &upper_left;
                    }
                    else if (dis2 < dis3 && dis2 < dis4)
                    {
                        type = UPPER_MIDDLE;
                        return &upper_middle;
                    }
                    else
                    {
                        if (dis3 < dis4)
                        {
                            type = UPPER_RIGHT;
                            return &upper_right;
                        }
                        else
                        {
                            type = BOTTOM_LEFT;
                            return &bottom_left;
                        }
                    }
                }
                else
                {
                    if (type == UPPER_LEFT) return &upper_left;
                    else if (type == UPPER_MIDDLE) return &upper_middle;
                    else if (type == UPPER_RIGHT) return &upper_right;
                    else if (type == BOTTOM_LEFT) return &bottom_left;
                    else
                    {
                        std::cout << "unknow REF_TYPE" << std::endl;
                        exit(-1);
                    }
                }
            }
            else
            {
                return nullptr;
            }
        }
    }
}

const SPoint<int> *SLineCompressorInterLine::
get_ref_point(
    const std::deque<SPoint<int>>& ref_line,
    const SPoint<int>& cur_point,
    const int& ref_point_th,
    const int& depth_diff_th
) {
    size_t left_pos = 0UL;
    const SPoint<int> *ref_point;
    
    // set left_pos as the righ_most point in ref_line
    // whose xy_ang is not greater than cur_point
    while (
        left_pos < ref_line.size() && 
        ref_line[left_pos].xy_ang <= cur_point.xy_ang
    ) {
        left_pos++;
    }
    left_pos --;

    // discard unsatisfiable left_pos
    if (
        left_pos + 1 == 0 ||                // out of bound
        left_pos + 1 >= ref_line.size() ||  // out of bound
        cur_point.xy_ang - ref_line[left_pos].xy_ang > ref_point_th
    ) {
        ref_point = nullptr; //&cur_line[j];
    }
    else
    {
        ref_point = &ref_line[left_pos];
    }
    return ref_point;
}

size_t SLineCompressorInterLine::decompress (
    const std::vector<unsigned char>& input,
    SPointCloud<float>& output
) {
    using clock = std::chrono::high_resolution_clock;
    using duration = std::chrono::duration<double>;
    clock::time_point start, end;
    duration diff;

    output.clear();
    if (input.empty()) return 0UL;

    std::vector <unsigned char> lengths_chunk, head_dx_chunk,
        head_dy_chunk, head_ddepth_chunk, 
        dx_chunk, dy_chunk, ddepth_chunk, bit_chunk;
    size_t lengths_size, head_dx_size, head_dy_size, head_ddepth_size,
        dx_size, dy_size, ddepth_size, bit_indicator_size;
    std::vector <int> lengths, head_dx, head_dy, head_ddepth,
        dx, dy, ddepth, bit_indicator;
    std::vector <unsigned char> dx_char;
    size_t line_pos = 0UL, point_pos = 0UL, bit_indicator_pos = 0UL;

    size_t read_pos = 0UL;

    int temp_xy_ang, temp_z_ang, temp_depth;
    memcpy(&max_depth_, input.data() + read_pos, sizeof(float));
    read_pos += sizeof(float);
    memcpy(&temp_xy_ang, input.data() + read_pos, sizeof(int));
    read_pos += sizeof(int);
    memcpy(&temp_z_ang, input.data() + read_pos, sizeof(int));
    read_pos += sizeof(int);
    memcpy(&temp_depth, input.data() + read_pos, sizeof(int));
    read_pos += sizeof(int);

    memcpy(&lengths_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    lengths_chunk.resize(lengths_size);
    memcpy(lengths_chunk.data(), input.data() + read_pos, lengths_size);
    read_pos += lengths_size;

    memcpy(&head_dx_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    head_dx_chunk.resize(head_dx_size);
    memcpy(head_dx_chunk.data(), input.data() + read_pos, head_dx_size);
    read_pos += head_dx_size;

    memcpy(&head_dy_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    head_dy_chunk.resize(head_dy_size);
    memcpy(head_dy_chunk.data(), input.data() + read_pos, head_dy_size);
    read_pos += head_dy_size;

    memcpy(&head_ddepth_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    head_ddepth_chunk.resize(head_ddepth_size);
    memcpy(head_ddepth_chunk.data(), input.data() + read_pos, head_ddepth_size);
    read_pos += head_ddepth_size;

    bool dx_to_char = false;
    memcpy(&dx_to_char, input.data() + read_pos, sizeof(bool));
    read_pos += sizeof(bool);

    memcpy(&dx_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    dx_chunk.resize(dx_size);
    memcpy(dx_chunk.data(), input.data() + read_pos, dx_size);
    read_pos += dx_size;

    memcpy(&dy_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    dy_chunk.resize(dy_size);
    memcpy(dy_chunk.data(), input.data() + read_pos, dy_size);
    read_pos += dy_size;

    memcpy(&ddepth_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    ddepth_chunk.resize(ddepth_size);
    memcpy(ddepth_chunk.data(), input.data() + read_pos, ddepth_size);
    read_pos += ddepth_size;

    memcpy(&bit_indicator_size, input.data() + read_pos, sizeof(size_t));
    read_pos += sizeof(size_t);
    bit_chunk.resize(bit_indicator_size);
    memcpy(bit_chunk.data(), input.data() + read_pos, bit_indicator_size);
    read_pos += bit_indicator_size;

    TIME_START();
    EntropyCompressor compressor;
    DeflateCompressor<unsigned char> deflate_uc;
    DeflateCompressor<int> deflate;
    DeflateCompressor<unsigned char> lzma_uc;
    LzmaCompressor<int> lzma;
    DeflateCompressor<unsigned char> bz2_uc;
    BZip2Compressor<int> bz2;

    compressor.decompress(lengths_chunk, lengths);
    compressor.decompress(head_dy_chunk, head_dy);
    compressor.decompress(dy_chunk, dy);
    switch (off_the_shelf_)
    {
        case OffTheShelf::deflate_:
        {
            deflate.decompress(head_ddepth_chunk, head_ddepth);
            deflate.decompress(ddepth_chunk, ddepth);

            deflate.decompress(head_dx_chunk, head_dx);
            if (dx_to_char)
            {
                deflate_uc.decompress(dx_chunk, dx_char);
                dx.assign(dx_char.begin(), dx_char.end());
            }
            else
                deflate.decompress(dx_chunk, dx);
            break;
        }
        case OffTheShelf::lzma_:
        {
            lzma.decompress(head_ddepth_chunk, head_ddepth);
            lzma.decompress(ddepth_chunk, ddepth);

            lzma.decompress(head_dx_chunk, head_dx);
            if (dx_to_char)
            {
                lzma_uc.decompress(dx_chunk, dx_char);
                dx.assign(dx_char.begin(), dx_char.end());
            }
            else
                lzma.decompress(dx_chunk, dx);
            break;
        }
        case OffTheShelf::bzip2_:
        {
            bz2.decompress(head_ddepth_chunk, head_ddepth);
            bz2.decompress(ddepth_chunk, ddepth);

            bz2.decompress(head_dx_chunk, head_dx);
            if (dx_to_char)
            {
                bz2_uc.decompress(dx_chunk, dx_char);
                dx.assign(dx_char.begin(), dx_char.end());
            }
            else
                bz2.decompress(dx_chunk, dx);
            break;
        }
        default:
        {
            compressor.decompress(head_ddepth_chunk, head_ddepth);
            compressor.decompress(ddepth_chunk, ddepth);
            
            deflate.decompress(head_dx_chunk, head_dx);
            if (dx_to_char)
            {
                deflate_uc.decompress(dx_chunk, dx_char);
                dx.assign(dx_char.begin(), dx_char.end());
            }
            else
                deflate.decompress(dx_chunk, dx);
            break;
        }
    }
    compressor.decompress(bit_chunk, bit_indicator);

    TIME_END("decode arrays");

    // thresholds
    const float ang_resolution = resolution_ / max_depth_;
    const int ref_point_th = std::round(sensor_.theta_range / sensor_.width / ang_resolution);
    const int z_ang_th = std::round((sensor_.phi_max - sensor_.phi_min) / sensor_.height / ang_resolution);
    const int depth_diff_th = conf::depth_diff_th / resolution_;

    TIME_START();
    // a deque storing all the reference lines_ of current line
    std::deque<int> ref_indices;

    for (size_t i = 0; i < lengths.size(); i++)
    {
        size_t length = lengths[line_pos];
        std::deque<SPoint<int>> cur_line (length);

        // restore ay_ang and z_ang of the line head
        if (i == 0)
        {
            cur_line[0].xy_ang = temp_xy_ang;
            cur_line[0].z_ang = temp_z_ang;
        }
        else
        {
            auto& pre_line = lines_[i - 1];
            cur_line[0].xy_ang = head_dx[line_pos - 1] + pre_line.front().xy_ang;
            cur_line[0].z_ang = head_dy[line_pos - 1] + pre_line.front().z_ang;
        }
        
        // get the reference lines_ of the current line
        while (
            !ref_indices.empty() && 
            cur_line.front().z_ang - lines_[ref_indices.front()].front().z_ang > z_ang_th
        ) {
            ref_indices.pop_front();
        }
        // consensus reference line
        std::deque<SPoint<int>> ref_line;
        generate_consensus_reference_line(ref_indices, ref_line);

        // restore depth of the line head
        if (i == 0)
        {
            cur_line[0].depth = temp_depth;
        }
        else
        {
            auto& pre_line = lines_[i - 1];
            const auto *ref_point = get_ref_point(ref_line,
                cur_line.front(), ref_point_th, depth_diff_th);

            if (ref_point)
                cur_line[0].depth = head_ddepth[line_pos - 1] + ref_point->depth;
            else
                cur_line[0].depth = head_ddepth[line_pos - 1] + pre_line.front().depth;
            //cur_line[0].depth = head_ddepth[line_pos - 1];
        }

        // align the line and its reference lines_
        size_t left_pos = 0UL, right_pos = 0UL;

        for (size_t j = 0; j < length - 1; j++)
        {
            cur_line[j + 1].xy_ang = cur_line[j].xy_ang + dx[point_pos];
            cur_line[j + 1].z_ang = cur_line[j].z_ang + dy[point_pos];

            REF_TYPE type = REF_TYPE::BOTTOM_LEFT;
            if (!bit_indicator.empty())
            {
                type = static_cast<REF_TYPE>(bit_indicator[bit_indicator_pos]);
            }
            const auto *ref_point = get_ref_point(ref_line,
                cur_line, j, ref_point_th, depth_diff_th,
                left_pos, right_pos, type, true);
            
            if (!ref_point)
                cur_line[j + 1].depth = cur_line[j].depth + ddepth[point_pos];
            else
            {
                bit_indicator_pos++;
                cur_line[j + 1].depth = ref_point->depth + ddepth[point_pos];
            }
            point_pos++;
        }
        lines_.push_back(cur_line);
        ref_indices.push_back(i);
        line_pos++;
    }
    TIME_END("decoding");

    for (auto& line: lines_)
    {
        for (auto& p: line)
        {
            output.emplace_back(
                p.xy_ang * static_cast<float>(ang_resolution),
                p.z_ang * static_cast<float>(ang_resolution),
                p.depth * static_cast<float>(resolution_)
            );
        }
    }
    return output.size();
}

SLineCompressorMultiLevels::SLineCompressorMultiLevels(
    const float max_dis, 
    const float resolution, 
    const float grid_size,
    conf::SensorMeta& sensor)
: SLineCompressor(max_dis, resolution, grid_size, sensor)
{}

size_t SLineCompressorMultiLevels::compress (
    const SPointCloud<float>& input, 
    std::vector<unsigned char>& output
) {
    output.clear();
    if (input.empty()) return 0UL;
    
    cloud_ = &input;

    find_polylines();
    
    std::sort(lines_.begin(), lines_.end(), [&input](std::deque<int>& t1, std::deque<int>& t2){
        if (std::abs(input[t1.front()].xy_ang - input[t2.front()].xy_ang) < 1e-6f)
            return input[t1.front()].z_ang < input[t2.front()].z_ang;
        else
            return input[t1.front()].xy_ang < input[t2.front()].xy_ang;
    });

    std::vector<float> segs { 25.0f, 50.0f, get_max_depth(input) + 1e-6f };

    std::vector<int> lengths, head_ddepth, ddepth;
    std::vector<std::vector<int>> head_dx (segs.size()),
        head_dy (segs.size()), dx (segs.size()), dy (segs.size());
    
    std::vector<unsigned char> lengths_chunk, head_ddepth_chunk, ddepth_chunk;
    std::vector<std::vector<unsigned char>> head_dx_chunks (segs.size()),
        head_dy_chunks (segs.size()), dx_chunks (segs.size()), dy_chunks (segs.size());

    size_t lengths_size, head_ddepth_size, ddepth_size;
    std::vector<size_t> head_dx_sizes (segs.size()),
        head_dy_sizes (segs.size()), dx_sizes (segs.size()), dy_sizes (segs.size());

    for (size_t i = 0; i < lines_.size(); i++)
    {
        auto& cur_line = lines_[i];

        const float local_max_depth = input[*std::max_element(
            cur_line.begin(), cur_line.end(), 
            [&input](const int p1, const int p2){
                return input[p1].depth < input[p2].depth;
            }
        )].depth;

        size_t idx = 0;
        while (local_max_depth > segs[idx])
        {
            idx ++;
        }
        
        float local_ang_resolution = resolution_ / segs[idx];

        lengths.push_back(cur_line.size());
        if (i > 0)
        {
            head_dx[idx].push_back(std::round(input[cur_line.front()].xy_ang / local_ang_resolution) - std::round(input[lines_[i - 1].front()].xy_ang / local_ang_resolution));
            head_dy[idx].push_back(std::round(input[cur_line.front()].z_ang / local_ang_resolution));
            head_ddepth.push_back(std::round(input[cur_line.front()].depth / resolution_));
        }

        for (size_t j = 1; j < cur_line.size(); j++)
        {
            dx[idx].push_back(std::round(input[cur_line[j]].xy_ang / local_ang_resolution) - std::round(input[cur_line[j - 1]].xy_ang / local_ang_resolution));
            dy[idx].push_back(std::round(input[cur_line[j]].z_ang / local_ang_resolution) - std::round(input[cur_line[j - 1]].z_ang / local_ang_resolution));
            ddepth.push_back(std::round(input[cur_line[j]].depth / resolution_) - std::round(input[cur_line[j - 1]].depth / resolution_));
        }
    }

    DeflateCompressor<int> deflate;
    size_t compressed_size = 0UL;

    lengths_size = deflate.compress(lengths, lengths_chunk);
    head_ddepth_size = deflate.compress(head_ddepth, head_ddepth_chunk);
    ddepth_size = deflate.compress(ddepth, ddepth_chunk);
    compressed_size += lengths_size + head_ddepth_size + ddepth_size;

    for (size_t i = 0; i < segs.size(); i++)
    {
        head_dx_sizes[i] = deflate.compress(head_dx[i], head_dx_chunks[i]);
        head_dy_sizes[i] = deflate.compress(head_dy[i], head_dy_chunks[i]);
        dx_sizes[i] = deflate.compress(dx[i], dx_chunks[i]);
        dy_sizes[i] = deflate.compress(dy[i], dy_chunks[i]);
        compressed_size += head_dx_sizes[i] + head_dy_sizes[i] +
            dx_sizes[i] + dy_sizes[i];
    }
    return compressed_size;
}

void SLineCompressorMultiLevels::find_polylines()
{
    SPointCloudIndex<float> index (grid_size_);
    index.set_input(*cloud_);

    std::vector<bool> visited(cloud_->size(), false);

    for (size_t i = 0; i < cloud_->size(); i++)
    {
        if (visited[i]) continue;

        visited[i] = true;
        std::deque <int> line {static_cast<int>(i)};
        
        extend_polyline<SPointCloudIndex<float>>(
            index, max_dis_, visited, line, true
        );
        
        if (line.size() > 5) 
        {
            lines_.push_back(line);
        }
        else
        {
            for (auto j: line)
            {
                visited[j] = false;
            }
        }
    }
    for (size_t i = 0; i < cloud_->size(); i++)
    {
        if (!visited[i])
            remaining_points_.push_back(i);
    }

    std::cout << "number of lines_: " << lines_.size() << std::endl;
    int num_points = std::accumulate(lines_.begin(), lines_.end(), 0, 
        [](int np, std::deque<int>& line){
            return np + line.size();
        }
    );
    std::cout << "number of points: " << num_points << std::endl;
    
}
