#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <vector>

#include "common/CLI11.hpp"
#include "common/types.h"
#include "common/utils.h"
#include "common/file_io.h"
#include "common/thread_pool.h"
#include "compressors/general.h"
#include "compressors/adaptive_compressor.h"

void error(const char *msg)
{
    perror(msg);
    printf("%s", msg);
    exit(0);
}

int main (int argc, char ** argv)
{
    CLI::App app{"App description"};

    std::string dir = "", server_name = "";
    uint16_t portno = 0;
    app.add_option("-d,--dir", dir, "file load dir")->required();
    app.add_option("-n,--name", server_name, "server name")->required();
    app.add_option("-p,--port", portno, "server port")->required();
    
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

    int sockfd;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    if (argc < 3) {
       fprintf(stderr,"usage %s hostname port\n", argv[0]);
       exit(0);
    }
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(server_name.c_str());
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");

    {
        ThreadPool compress_pool(8);
        std::mutex upload_mutex;

        for (const auto & entry : std::filesystem::directory_iterator(dir))
        {
            if (io::ends_with(entry.path(), ".bin"))
            {
                compress_pool.enqueue([&](
                    const std::filesystem::__cxx11::directory_entry &entry
                ){
                    PointCloud<float> cloud;
                    io::load<Point<float>> (entry.path(), cloud);

                    AdaptiveCompressor compressor (resolution, rate, off_the_shelf, intra_line_coding_only);
                    std::vector<unsigned char> output;
                    compressor.compress(cloud, output);
                    {
                        std::unique_lock<std::mutex> lock(upload_mutex);

                        std::cout << output.size() << std::endl;
                        int n = -1;
                        char buffer[16];
                        unsigned long temp_size = output.size();
                        n = write(sockfd,(void *)&temp_size,sizeof(unsigned long));
                        n = write(sockfd,output.data(),output.size());
                        if (n < 0) 
                            error("ERROR writing to socket");
                        bzero(buffer,16);
                        n = read(sockfd,buffer,16);
                        if (n < 0) 
                            error("ERROR reading from socket");
                        printf("%s\n",buffer); 
                    }
                }, entry);
            }
        }
    }
}