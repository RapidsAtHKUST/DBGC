#include <stdio.h>
#include <chrono>
#include <iostream>
#include <vector>

#include "common/CLI11.hpp"
#include "common/types.h"
#include "common/utils.h"
#include "common/file_io.h"
#include "compressors/base.h"
#include "compressors/general.h"
#include "compressors/adaptive_compressor.h"


int main (int argc, char *argv[]) {
    CLI::App app{"App description"};

    std::string uncompressed_path = "", compressed_path = "";
    app.add_option("-i,--input", uncompressed_path, "uncompressed file path")->required();
    app.add_option("-o,--output", compressed_path, "compressed file path")->required();
    
    float resolution = 1.0f;
    app.add_option("-r,--resolution", resolution, "resolution")->required();
    
    float rate = -1.0f;
    app.add_option("--rate", rate, "rate");

    OffTheShelf off_the_shelf = OffTheShelf::arithmetic_;
    app.add_option("--off-the-shelf", off_the_shelf, "off-the-shelf compressor")
        ->transform(CLI::CheckedTransformer(off_the_shelf_map, CLI::ignore_case));

    bool intra_line_coding_only = false;
    app.add_option("--intra-line-coding-only", intra_line_coding_only, "intra line coding only");

    CLI11_PARSE(app, argc, argv);
    using clock = std::chrono::high_resolution_clock;
    using duration = std::chrono::duration<double>;

    size_t original_size, compressed_size;

    PointCloud<float> cloud;
    io::load<Point<float>> (uncompressed_path, cloud);
    original_size = cloud.size() * 3 * sizeof(float);

    std::cout << "Number of points: " << cloud.size() << std::endl;

    clock::time_point start = clock::now();

    AdaptiveCompressor compressor (resolution, rate, off_the_shelf, intra_line_coding_only);
    std::vector<unsigned char> output;
    compressed_size = compressor.compress(cloud, output);

    clock::time_point end = clock::now();
    duration diff = end - start;
    std::cout << "\n***** Compression Statistics *****\n";
    printf("Elapsed Time: %.9lf s\n", diff.count());
    
    PRINT_COMPRESSION_RATIO(original_size, compressed_size)

    io::dump_compressed(compressed_path, output);
    std::cout << "\nPeak Resident Set Size: " << mem::getValue() << " KB" << std::endl;
}