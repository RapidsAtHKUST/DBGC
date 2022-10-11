#ifndef ENTROPY_COMPRESSOR
#define ENTROPY_COMPRESSOR

#include <string.h>
#include <cmath>
#include <vector>
#include <map>
#include <unordered_map>

#include "compressors/base.h"


class EntropyCompressorNoDict
: public Compressor<std::vector<unsigned char>> {
public:
    size_t compress(
        const std::vector<unsigned char>& input, 
        std::vector<unsigned char>& output);

    size_t decompress(
        const std::vector<unsigned char>& input,
        std::vector<unsigned char>& output);
};

class EntropyCompressor
: public Compressor<std::vector<int>> {
public:
    size_t compress(
        const std::vector<int>& input, 
        std::vector<unsigned char>& output);
    size_t decompress(
        const std::vector<unsigned char>& input,
        std::vector<int>& output);
};


#endif