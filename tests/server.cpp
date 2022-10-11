#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>

#ifdef TO_DB
#include <sql.h>
#include <sqlext.h>
#endif

#include "common/CLI11.hpp"
#include "common/types.h"
#include "common/utils.h"
#include "common/file_io.h"
#include "common/thread_pool.h"
#include "compressors/adaptive_compressor.h"

void error(const char *msg)
{
    perror(msg);
    printf("%s", msg);
    exit(1);
}


#ifdef TO_DB
static void extract_error(
    const char *fn,
    SQLHANDLE handle,
    SQLSMALLINT type)
{
    printf("%s", fn);
}

static char dec_hex(
    int v)
{
    if (v >= 0 && v < 10) return v + 48;
    else if (v >= 10 && v < 16) return v + 87;
    else
    {
        printf("convert error");
        exit(-1);
    }
}
#endif

int main (int argc, char *argv[]) {
    CLI::App app{"App description"};

#ifdef TO_DB
    std::string dsn_name;
    app.add_option("-d,--dsn-name", dsn_name, "database dsn name")->required();
#else
    std::string dir = "";
    app.add_option("-d,--dir", dir, "file store dir")->required();
#endif
    uint16_t portno = 0;
    bool store_compressed = false;
    app.add_option("-p,--port", portno, "server port")->required();
    app.add_option("--store-compressed", store_compressed, "store compressed");
    
    CLI11_PARSE(app, argc, argv);

    int sockfd, newsockfd;
    socklen_t clilen;
    unsigned char* buffer = new unsigned char[1000000];
    struct sockaddr_in serv_addr, cli_addr;
    if (argc < 2) {
        fprintf(stderr,"ERROR, no port provided\n");
        exit(1);
    }
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
       error("ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
            sizeof(serv_addr)) < 0) 
            error("ERROR on binding");
    listen(sockfd,20);
    clilen = sizeof(struct sockaddr_in);
    newsockfd = accept(sockfd, 
        (struct sockaddr *) &cli_addr, 
        &clilen);
    if (newsockfd < 0) 
        error("ERROR on accept");

#ifdef TO_DB
    SQLHENV env;
    SQLHDBC dbc;
    SQLHSTMT stmt;
    SQLRETURN ret; /* ODBC API return status */
    SQLCHAR outstr[1024];
    SQLSMALLINT outstrlen;

    /* Allocate an environment handle */
    SQLAllocHandle(SQL_HANDLE_ENV, SQL_NULL_HANDLE, &env);
    /* We want ODBC 3 support */
    SQLSetEnvAttr(env, SQL_ATTR_ODBC_VERSION, (void *) SQL_OV_ODBC3, 0);
    /* Allocate a connection handle */
    SQLAllocHandle(SQL_HANDLE_DBC, env, &dbc);
    /* Connect to the DSN TEST_PSQL */
    dsn_name = "DSN=" + dsn_name + ";";
    ret = SQLDriverConnect(dbc, NULL, (SQLCHAR *)dsn_name.c_str(), SQL_NTS,
                            outstr, sizeof(outstr), &outstrlen,
                            SQL_DRIVER_COMPLETE);
    if (SQL_SUCCEEDED(ret)) {
        printf("Connected\n");
        if (ret == SQL_SUCCESS_WITH_INFO) {
            printf("Driver reported the following diagnostics\n");
            extract_error("SQLDriverConnect", dbc, SQL_HANDLE_DBC);
        }
    }
    /* Allocate a statement handle */
    SQLAllocHandle(SQL_HANDLE_STMT, dbc, &stmt);
#else
    std::filesystem::path dir_path {dir};
    if (!std::filesystem::exists(dir_path))
        std::filesystem::create_directory(dir_path);
#endif
    
    {
        ThreadPool decompress_pool(8);

        std::mutex upload_mutex;
        int num = 0;
        
        while (true)
        {
            unsigned long temp_size;
            read(newsockfd,(void*)&temp_size,sizeof(unsigned long));

            bzero(buffer,1000000);
            unsigned long offset = 0;
            do {
                unsigned long size = read(newsockfd,buffer + offset,1000000);
                std::cout << temp_size << " " << size << std::endl;
                if (size < 0) error("ERROR reading from socket");
                offset += size;
            }
            while (temp_size != offset);
            
            int n = write(newsockfd,"success",8);
            if (n < 0) error("ERROR writing to socket");
            std::cout << temp_size << std::endl;
            std::vector<unsigned char> output(buffer, buffer + temp_size);

            if (store_compressed)
            {
#ifdef TO_DB
                std::string temp (temp_size * 2, '\0');
                for (int i = 0; i < temp_size; i++)
                {
                    temp[i * 2 + 0] = dec_hex(output[i] / 16);
                    temp[i * 2 + 1] = dec_hex(output[i] % 16);
                }
                temp = "insert into pc values (" + std::to_string(num) + ", '\\x" + temp + "')";
                printf("%s",temp.c_str());
                ret = SQLExecDirect(stmt, (SQLCHAR *)(temp.c_str()), SQL_NTS);
                if (SQL_SUCCEEDED(ret)) {
                    printf("Insert Success\n");
                    if (ret == SQL_SUCCESS_WITH_INFO)
                    {
                        printf("Driver reported the following diagnostics\n");
                        extract_error("SQLExecDirect", dbc, SQL_HANDLE_DBC);
                    }
                }
#else
                io::dump_compressed(dir + "/" + std::to_string(num) + ".lcc", output);
#endif
                num += 1;
            }
            else
            {
                decompress_pool.enqueue([&](std::vector<unsigned char> output){
                    PointCloud<float> cloud;
                    AdaptiveCompressor compressor1;
                    compressor1.decompress(output, cloud);

                    {
                        std::unique_lock<std::mutex> lock(upload_mutex);
#ifdef TO_DB
                        std::string temp ("insert into points values (");
                        for (unsigned long i = 0; i < cloud.temp_size(); i++)
                        {
                            const auto p = cloud[i];
                            if (i == 0) 
                                temp += std::to_string(p.x) + ", ";
                            else
                                temp += ", (" + std::to_string(p.x) + ", ";
                            temp += std::to_string(p.y) + ", ";
                            temp += std::to_string(p.z) + ", ";
                            temp += std::to_string(num) + ")";
                        }
                        ret = SQLExecDirect(stmt, (SQLCHAR *)(temp.c_str()), SQL_NTS);
                        if (SQL_SUCCEEDED(ret)) {
                            printf("Insert Success\n");
                            if (ret == SQL_SUCCESS_WITH_INFO)
                            {
                                printf("Driver reported the following diagnostics\n");
                                extract_error("SQLExecDirect", dbc, SQL_HANDLE_DBC);
                            }
                        }
#else
                        io::dump(dir + "/" + std::to_string(num) + ".bin", cloud);
#endif
                        num += 1;
                    }
                }, output);
            }
        }
    }
}