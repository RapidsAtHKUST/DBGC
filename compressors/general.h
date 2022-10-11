#ifndef GENERAL_COMPRESSORS
#define GENERAL_COMPRESSORS

#include <iostream>
#include <string>
#include <unordered_map>
#include <bzlib.h>
#include <lzma.h>
#define ZLIB_CONST
#include <zlib.h>

#include "compressors/base.h"

enum OffTheShelf : unsigned char {
    arithmetic_ = 0,
    entropy_ = 0,
    deflate_ = 1,
    lzma_ = 2,
    bzip2_ = 3,
    bz2_ = 3
};

const std::unordered_map<std::string, OffTheShelf> off_the_shelf_map {
    { "arithmetic", OffTheShelf::arithmetic_ },
    { "entropy", OffTheShelf::entropy_ },
    { "deflate", OffTheShelf::deflate_ },
    { "lzma", OffTheShelf::lzma_ },
    { "bzip2", OffTheShelf::bzip2_ },
    { "bz2", OffTheShelf::bz2_ }
};

template<typename T>
class DeflateCompressor: 
public Compressor<std::vector<T>> {
public:
    size_t compress(
        const std::vector<T>& input, 
        std::vector<unsigned char>& output)
    {
        if (input.empty()) return 0UL;

        const size_t in_size = input.size() * sizeof(T);
        size_t out_size = 0, delta_size = in_size;

        z_stream strm;
        strm.zalloc = Z_NULL;
        strm.zfree = Z_NULL;
        strm.next_in = reinterpret_cast<const unsigned char*>(input.data());
        strm.avail_in = in_size;

        int ret = ::deflateInit2(
            &strm, Z_BEST_COMPRESSION, Z_DEFLATED, 
            -15, 9, Z_DEFAULT_STRATEGY);
        if (ret != Z_OK)
        {
            std::cout << "Deflate Error in function deflateInit2." << std::endl;
            exit(-1);
        }

        do
        {
            output.resize(out_size + delta_size);
            strm.next_out = output.data() + out_size;
            strm.avail_out = delta_size;

            ret = deflate(&strm, Z_FINISH);
            if (ret != Z_OK && ret != Z_STREAM_END)
            {
                std::cout << "Deflate Error in function deflate." << std::endl;
                exit(-1);
            }
            out_size += delta_size;
            delta_size *= 2;
        }
        while (strm.avail_out == 0);

        out_size -= strm.avail_out;
        output.resize(out_size);
        return static_cast<size_t>(out_size);
    }

    size_t decompress(
        const std::vector<unsigned char>& input, 
        std::vector<T>& output)
    {
        const size_t in_size = input.size();
        size_t out_size = 0, delta_size = in_size + sizeof(T) - in_size % sizeof(T);

        z_stream strm;
        strm.zalloc = Z_NULL;
        strm.zfree = Z_NULL;
        strm.next_in = input.data();
        strm.avail_in = in_size;

        int ret = inflateInit2(&strm, -15);
        if (ret != Z_OK)
        {
            std::cout << "Deflate Error in function inflateInit2." << std::endl;
            exit(-1);
        }

        do
        {
            output.resize((out_size + delta_size) / sizeof(T));
            strm.next_out = reinterpret_cast<unsigned char*>(output.data() + out_size / sizeof(T));
            strm.avail_out = delta_size;

            ret = ::inflate(&strm, Z_FINISH);
            if (ret != Z_OK && ret != Z_STREAM_END && ret != Z_BUF_ERROR)
            {
                std::cout << "Deflate Error in function inflate." << std::endl;
                exit(-1);
            }
            out_size += delta_size;
            delta_size *= 2;
        }
        while (strm.avail_out == 0);

        out_size -= strm.avail_out;
        output.resize(out_size / sizeof(T));
        return static_cast<size_t>(out_size / sizeof(T));
    }
};


template <typename T>
class BZip2Compressor: 
public Compressor<std::vector<T>> {
public:
    size_t compress(
        const std::vector<T>& input, 
        std::vector<unsigned char>& output)
    {
        if (input.empty()) return 0UL;

        const unsigned int in_size = input.size() * sizeof(T);
        unsigned int out_size = in_size / 2;

        int ret;
        do
        {
            out_size *= 2;
            output.resize(out_size);

            ret = BZ2_bzBuffToBuffCompress (
                reinterpret_cast<char*>(output.data()), 
                &out_size,
                reinterpret_cast<char*>(const_cast<
                    std::vector<T>&
                >(input).data()),
                in_size, 9, 0, 30
            );
            if (ret != BZ_OK && ret != BZ_OUTBUFF_FULL)
            {
                std::cout << "BZip2 Error in function BZ2_bzBuffToBuffCompress." << std::endl;
                exit(-1);
            }
        }
        while (ret == BZ_OUTBUFF_FULL);

        output.resize(out_size);
        return static_cast<size_t>(out_size);
    }

    size_t decompress(
        const std::vector<unsigned char>& input, 
        std::vector<T>& output)
    {
        const unsigned int in_size = input.size();
        unsigned int out_size = in_size + sizeof(T) - in_size % sizeof(T);

        int ret;
        do
        {
            out_size *= 2;
            output.resize(out_size / sizeof(T));

            ret = BZ2_bzBuffToBuffDecompress (
                reinterpret_cast<char*>(output.data()), 
                &out_size,
                reinterpret_cast<char*>(const_cast<
                    std::vector<unsigned char>&
                >(input).data()),
                in_size, 0, 0
            );
            if (ret != BZ_OK && ret != BZ_OUTBUFF_FULL)
            {
                std::cout << "BZip2 Error in function BZ2_bzBuffToBuffDecompress." << std::endl;
                exit(-1);
            }
        }
        while (ret == BZ_OUTBUFF_FULL);

        output.resize(out_size / sizeof(T));
        return static_cast<size_t>(out_size / sizeof(T));
    }
};


template <typename T>
class LzmaCompressor: 
public Compressor<std::vector<T>> {
public:
    size_t compress(
        const std::vector<T>& input, 
        std::vector<unsigned char>& output)
    {
        if (input.empty()) return 0UL;

        const size_t in_size = input.size() * sizeof(T);
        size_t out_size = 0, delta_size = in_size;

        lzma_stream strm = LZMA_STREAM_INIT;
        lzma_ret ret = lzma_easy_encoder(&strm, 9, LZMA_CHECK_CRC64);
        if (ret != LZMA_OK)
        {
            std::cout << "LZMA Error in function lzma_easy_encoder." << std::endl;
            exit(-1);
        }

        strm.next_in = reinterpret_cast<const unsigned char*>(input.data());
        strm.avail_in = in_size;
        
        do
        {
            output.resize(out_size + delta_size);
            strm.next_out = output.data() + out_size;
            strm.avail_out = delta_size;

            ret = lzma_code(&strm, LZMA_FINISH);
            if (ret != LZMA_OK && ret != LZMA_STREAM_END)
            {
                std::cout << "LZMA Error in function lzma_code." << std::endl;
                exit(-1);
            }
            out_size += delta_size;
            delta_size *= 2;
        }
        while (strm.avail_out == 0);

        out_size -= strm.avail_out;
        output.resize(out_size);
        return static_cast<size_t>(out_size);
    }

    size_t decompress(
        const std::vector<unsigned char>& input, 
        std::vector<T>& output)
    {
        const size_t in_size = input.size();
        size_t out_size = 0, delta_size = in_size + sizeof(T) - in_size % sizeof(T);

        lzma_stream strm = LZMA_STREAM_INIT;
        lzma_ret ret = lzma_stream_decoder(&strm, UINT64_MAX, LZMA_CONCATENATED);
        if (ret != LZMA_OK)
        {
            std::cout << "LZMA Error in function lzma_easy_encoder." << std::endl;
            exit(-1);
        }
        
        strm.next_in = input.data();
        strm.avail_in = in_size;

        do
        {
            
            output.resize((out_size + delta_size) / sizeof(T));
            strm.next_out = reinterpret_cast<unsigned char*>(output.data() + out_size / sizeof(T));
            strm.avail_out = delta_size;

            ret = lzma_code(&strm, LZMA_FINISH);
            if (ret != LZMA_OK && ret != LZMA_STREAM_END)
            {
                std::cout << "LZMA Error in function lzma_code." << std::endl;
                exit(-1);
            }
            out_size += delta_size;
            delta_size *= 2;
        }
        while (strm.avail_out == 0);

        out_size -= strm.avail_out;
        output.resize(out_size / sizeof(T));
        return static_cast<size_t>(out_size / sizeof(T));
    }
};

#endif