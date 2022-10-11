#ifndef BASE_COMPRESSOR
#define BASE_COMPRESSOR

#include <string.h>
#include <vector>

#include "common/config.h"

#define PRINT_LOCAL_COMPRESSION_RATIO(x,y) \
    std::cout << "local original size: " << (x) * conf::bpB << '\n' \
              << "local compressed size: " << (y) * conf::bpB << '\n' \
              << "local compression ratio: " << (x) / static_cast<float>(y) << std::endl;

#define PRINT_COMPRESSION_RATIO(x,y) \
    std::cout << "original size: " << (x) * conf::bpB << '\n' \
              << "compressed size: " << (y) * conf::bpB << '\n' \
              << "compression ratio: " << (x) / static_cast<float>(y) << std::endl;

template <typename T>
class Compressor {
public:
    virtual size_t compress(const T& input, std::vector<unsigned char>& output) = 0;
    //virtual void decompress(std::vector<unsigned char> input, T output) = 0;
};

#endif