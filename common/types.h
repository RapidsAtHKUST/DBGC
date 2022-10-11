#ifndef COMMON_TYPES
#define COMMON_TYPES

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

#include "common/config.h"

template<typename T>
struct Point {
    union {
        struct {
            T x;
            T y;
            T z;
        };
        T data[3];
    };

    Point(
        T x_arg = 0, 
        T y_arg = 0, 
        T z_arg = 0)
    : x(x_arg)
    , y(y_arg)
    , z(z_arg) {}

    Point& operator=(const Point& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    static Point diff(Point p1, Point p2)
    {
        Point result;
        result.x = p2.x - p1.x;
        result.y = p2.y - p1.y;
        result.z = p2.z - p1.z;
        return result;
    }
};


template<typename T>
using PointCloud = std::vector<Point<T>>;

namespace io {
    // type conversion
    inline void point_cloud_to_int (
        const PointCloud<float>& cloud,
        PointCloud<int>& cloud_int,
        float scale
    ) {
        cloud_int.resize (cloud.size());
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            cloud_int[i].x = std::round(cloud[i].x * scale);
            cloud_int[i].y = std::round(cloud[i].y * scale);
            cloud_int[i].z = std::round(cloud[i].z * scale);
        }
    }
    inline void point_cloud_to_int (
        const PointCloud<float>& cloud,
        PointCloud<int>& cloud_int,
        float x_scale,
        float y_scale,
        float z_scale
    ) {
        cloud_int.resize (cloud.size());
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            cloud_int[i].x = std::round(cloud[i].x * x_scale);
            cloud_int[i].y = std::round(cloud[i].y * y_scale);
            cloud_int[i].z = std::round(cloud[i].z * z_scale);
        }
    }
    
    inline void point_cloud_to_float (
        const PointCloud<int>& cloud,
        PointCloud<float>& cloud_float,
        float scale
    ) {
        cloud_float.resize (cloud.size());
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            cloud_float[i].x = cloud[i].x / scale;
            cloud_float[i].y = cloud[i].y / scale;
            cloud_float[i].z = cloud[i].z / scale;
        }
    }
    inline void point_cloud_to_float (
        const PointCloud<int>& cloud,
        PointCloud<float>& cloud_float,
        float x_scale,
        float y_scale,
        float z_scale
    ) {
        cloud_float.resize (cloud.size());
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            cloud_float[i].x = cloud[i].x / x_scale;
            cloud_float[i].y = cloud[i].y / y_scale;
            cloud_float[i].z = cloud[i].z / z_scale;
        }
    }
}

template<typename T>
struct SPoint {
    union {
        struct {
            T xy_ang;
            T z_ang;
            T depth;
        };
        T data[3];
    };

    SPoint(
        T xy_ang_arg = 0, 
        T z_ang_arg = 0, 
        T depth_arg = 0)
    : xy_ang(xy_ang_arg)
    , z_ang(z_ang_arg)
    , depth(depth_arg) {}

    SPoint& operator=(const SPoint& other)
    {
        xy_ang = other.xy_ang;
        z_ang = other.z_ang;
        depth = other.depth;
        return *this;
    }

    static SPoint diff(SPoint p1, SPoint p2)
    {
        SPoint result;
        result.xy_ang = p2.xy_ang - p1.xy_ang;
        result.z_ang = p2.z_ang - p1.z_ang;
        result.depth = p2.depth - p1.depth;
        return result;
    }
};

template<typename T>
using SPointCloud = std::vector<SPoint<T>>;

template<typename T>
T get_max_depth (const SPointCloud<T>& image) {
    return std::max_element(
        image.begin(), image.end(), 
        [](const SPoint<T>& p1, const SPoint<T>& p2){
            return p1.depth < p2.depth;
        }
    )->depth;
}

template<typename T>
T get_min_depth (const SPointCloud<T>& image) {
    return std::min_element(
        image.begin(), image.end(), 
        [](const SPoint<T>& p1, const SPoint<T>& p2){
            return p1.depth < p2.depth;
        }
    )->depth;
}

namespace io {
    inline void spoint_cloud_to_int (SPointCloud<float>& image, SPointCloud<int>& image_int, float depth_resolution, float intensity_scale)
    {
        image_int.resize (image.size());
        // get max depth
        float max_depth = get_max_depth(image);
        float ang_resolution = depth_resolution / max_depth;

        for (size_t i = 0; i < image.size(); ++i)
        {
            image_int[i].xy_ang = std::round(image[i].xy_ang / ang_resolution) * 2;
            image_int[i].z_ang = std::round(image[i].z_ang / ang_resolution) * 2;
            image_int[i].depth = std::round(image[i].depth / depth_resolution);
        }
    }

    inline void cartesian_to_spherical (
        PointCloud<float>& cloud,
        SPointCloud<float>& image
    ) {
        image.resize(cloud.size());

        for (size_t i = 0; i < cloud.size(); i++)
        {
            const auto& point = cloud.at(i);
            image[i].xy_ang = std::fmod(std::atan2(point.y, point.x) + 2 * conf::pi, 2 * conf::pi);

            float xy_depth = std::sqrt(point.y * point.y + point.x * point.x);
            image[i].z_ang = std::atan2(point.z, xy_depth);
            image[i].depth = std::sqrt(xy_depth * xy_depth + point.z * point.z);
        }
    }

    inline void spherical_to_cartesian (
        SPointCloud<float>& image,
        PointCloud<float>& cloud
    ) {
        cloud.resize(image.size());

        for (size_t i = 0; i < image.size(); i++)
        {
            const auto& spoint = image.at(i);
            cloud[i].x = spoint.depth * std::cos(spoint.z_ang) * std::cos(spoint.xy_ang);
            cloud[i].y = spoint.depth * std::cos(spoint.z_ang) * std::sin(spoint.xy_ang);
            cloud[i].z = spoint.depth * std::sin(spoint.z_ang);
        }
    }

    inline int round_with(float v1, float v2, float resolution)
    {
        return std::round(v1 / resolution) - std::round(v2 / resolution);
    }
}

#endif