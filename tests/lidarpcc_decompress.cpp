#include <stdio.h>
#include <chrono>
#include <iostream>
#include <vector>

#include "common/CLI11.hpp"
#include "common/types.h"
#include "common/utils.h"
#include "common/file_io.h"
#include "compressors/adaptive_compressor.h"


int main (int argc, char *argv[]) {
    CLI::App app{"App description"};

    std::string compressed_path = "", reconstructed_path = "";
    app.add_option("-i,--input", compressed_path, "compressed file path")->required();
    app.add_option("-o,--output", reconstructed_path, "reconstructed file path")->required();
    
    CLI11_PARSE(app, argc, argv);
    using clock = std::chrono::high_resolution_clock;
    using duration = std::chrono::duration<double>;

    std::vector<unsigned char> output;
    io::load_compressed(compressed_path, output);

    clock::time_point start = clock::now();

    PointCloud<float> cloud;
    AdaptiveCompressor compressor1;
    compressor1.decompress(output, cloud);

    clock::time_point end = clock::now();
    duration diff = end - start;
    std::cout << "***** Decompression Statistics *****\n";
    std::cout << "Number of points: " << cloud.size() << '\n';
    printf("Elapsed Time: %.9lf s\n", diff.count());
    
    io::dump(reconstructed_path, cloud);
    std::cout << "\nPeak Resident Set Size: " << mem::getValue() << " KB" << std::endl;
}