# DBGC

## Introduction

LiDAR (Light Detection and Ranging) sensors produce 3D point clouds that capture the surroundings, and these data are used in applications such as autonomous driving, traffic monitoring, and remote surveys. LiDAR point clouds are usually compressed for efficient transmission and storage. However, to achieve a high compression ratio, existing work often sacrifices the geometric accuracy of the data, which hurts the effectiveness of downstream applications. Therefore, we propose a system that achieves a high compression ratio while preserving geometric accuracy. In our method, we first perform density-based clustering to distinguish the dense points from the sparse ones, because they are suitable for different compression methods. The clustering algorithm is optimized for our purpose and its parameter values are set to preserve accuracy. We then compress the dense points with an octree, and organize the sparse ones into polylines to reduce the redundancy. We further propose to compress the sparse points on the polylines by their spherical coordinates considering the properties of both the LiDAR sensors and the real-world scenes. Finally, we design suitable schemes to compress the remaining sparse points not on any polyline. Experimental results on DBGC, our prototype system, show that our scheme compressed large-scale real-world datasets by up to 19 times with an error bound under 0.02 meters for scenes of thousands of cubic meters. This result, together with the fast compression speed of DBGC, demonstrates the online compression of LiDAR data with high accuracy.

For the details, please refer to our EDBT'2023 paper "Density-Based Geometry Compression for LiDAR Point Clouds" by [Xibo Sun](https://github.com/xibosun) and [Prof. Qiong Luo](https://cse.hkust.edu.hk/~luo/). If you have any further question, please feel free to contact us.

Please cite our paper if you use our source code.

- "Xibo Sun, and Qiong Luo. Density-Based Geometry Compression for LiDAR Point Clouds. EDBT 2023."


## Compile

To compile the compressor, packages including Bzip2, LZMA, Zlib, and TBB should be installed. For example, on the Ubuntu system, one can run

```shell
sudo apt-get install -y libbz2-dev liblzma-dev zlib1g zlib1g-dev libtbb-dev
```

Then, one can compile the compressor by

```shell
make
```

## Standalone Compressor
### Compress

The command for compressing a point cloud is 

```shell
./build/lidarpcc_compress -i <original-path> -o <compressed-path> -r <2q_xyz>
```

where `<original-path>` is the original point cloud file, `<compressed-path>` is the compressed file, and `q_xyz` is the error bound in the Cartesian coordinate system.

The compressor supports a point cloud with the `.bin` extension (the [KITTI](http://www.cvlibs.net/datasets/kitti/raw_data.php) format) or the `.txt` extension storing a coordinate matrix. Specifically, each `.txt` file contains a matrix of floating-point numbers, where each row has four floating numbers representing the x, y, z, and intensity of a point.

Other configurable parameters are described as follows.

| Flag                     | Description                                                       | Valid Value                    |
|--------------------------|-------------------------------------------------------------------|--------------------------------|
| --rate                   | The rate of dense points                                          | from 0 to 1                    |
| --off-the-shelf          | Off-the-shelf compressor for arrays in the coordinate compression | entropy, bz2, deflate, or lzma |
| --intra-line-coding-only | Using intra-line coding only                                      | on/off                         |

### Decompress

A compressed file can be decompressed into a point cloud by running

```shell
./build/lidarpcc_decompress -i <compressed-path> -o <reconstructed-path>
```

where `<compressed-path>` is the compressed point cloud file and `<reconstructed-path>` is the file path of the reconstructed point cloud.

## DBGC system
### Server

After compiling the code, one can also run the following command to start the DBGC server

```shell
./build/server -d <compressed-dir> -p <server-port>
```

where `<compressed-dir>` is the path for storing point clouds, and `<server-port>` is the server's port number for communication.

### Client

Then, the client can be started by

```shell
./build/client --dir <original-dir> -r <2q_xyz> -n <server-ip> -p <server-port>
```

where `<original-dir>` is the path storing the original point clouds, `<server-ip>` is the IP address of the server, and `<server-port>` is the port number of the server. The client will compress all point clouds in `<original-dir>` and send them to the server. 

By default, as the server receives the compressed data, it decompresses it and stores the reconstructed point cloud. One can let the server directly store the compressed point cloud by setting the `--store-compressed` flag to `false`.

One can also let the server store the data in a database rather than the file system. In this case, the ODBC package should be installed. The example command on Ubuntu is

```shell
sudo apt-get install -y unixodbc-dev
```

Then, to run the program with database support, one should first compile to code by

```shell
make to_db
```

and then start the server by

```shell
./build/server -d <dsn-name> -p <server-port>
```

where `<dsn-name>` is the data source name of the database. The command for the client is the same as above.


## Development

We also package compression and decompression functions of DBGC into `./build/libDBGC.so`. One can easily adopt DBGC's compression scheme in other programs. An example file `example.cpp` is shown as follows.

```c++
#include <vector>
#include "common/file_io.h"
#include "compressors/adaptive_compressor.h"

int main (int argc, char *argv[]) {
    // read the original point cloud
    PointCloud<float> cloud;
    io::load<Point<float>> (<original-path>, cloud);

    // create a compressor with the error bound of 0.02 m
    AdaptiveCompressor compressor (0.04);

    // output compressed sequence
    std::vector<unsigned char> output;

    // perform the compression
    compressor.compress(cloud, output);

    // create a reconstructed point cloud
    PointCloud<float> re_cloud;

    // create a decompressor
    AdaptiveCompressor decompressor;

    // perform the decompression
    decompressor.decompress(output, re_cloud);
}
```

After that, one can compile `example.cpp` by

```shell
g++ -std=c++17 -I<DBGC-DIR> -L<DBGC-DIR>/build -lDBGC -rpath=<DBGC-DIR>/build example.cpp -o example
```
