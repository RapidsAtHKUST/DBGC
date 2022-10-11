#ifndef ADAPTIVE_COMPRESSOR
#define ADAPTIVE_COMPRESSOR

#include <vector>

#include "compressors/base.h"
#include "compressors/general.h"
#include "common/types.h"


class AdaptiveCompressor
: public Compressor<PointCloud<float>> {
protected:
    float resolution_;
    float rate_;
    OffTheShelf off_the_shelf_;
    bool intra_line_coding_only_;
    const std::vector<float> segs_;
public:
    AdaptiveCompressor()
    : resolution_(0.0f)
    , rate_(-1.0f)
    , off_the_shelf_(OffTheShelf::arithmetic_)
    , intra_line_coding_only_(false)
    , segs_{0.0f, 25.0f, 50.0f, 100.0f} {}
    // , segs_{0.0f, 100.0f} {} for one level
    
    AdaptiveCompressor(
        float resolution, float rate = -1.0f,
        OffTheShelf off_the_shelf = OffTheShelf::arithmetic_,
        bool intra_line_coding_only = false)
    : resolution_(resolution)
    , rate_(rate)
    , off_the_shelf_(off_the_shelf)
    , intra_line_coding_only_(intra_line_coding_only)
    , segs_{0.0f, 25.0f, 50.0f, 100.0f} {}
    // , segs_{0.0f, 100.0f} {} for one level

    size_t compress(
        const PointCloud<float>& input, 
        std::vector<unsigned char>& output);

    size_t decompress(
        const std::vector<unsigned char>& input,
        PointCloud<float>& output);
};

#endif