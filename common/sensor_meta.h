#ifndef COMMON_SENEOR_META
#define COMMON_SENEOR_META

#include "common/config.h"

enum SensorIndex {
    KITTI = 0
};
namespace conf {
    struct SensorMeta
    {
        const float theta_range;
        const float phi_max;
        const float phi_min;
        const float depth_min;
        const float depth_max;
        const int height;
        const int width;
        const int nchannels;
    };
    inline SensorMeta sensors[] = {
        {   // KITTI
            deg2ang(360.0),
            deg2ang(2.5),
            deg2ang(-24.9),
            0.0,
            100,
            64,
            2000,
            1
        }
    };
}

#endif
