#ifndef COMMON_FILE_IO
#define COMMON_FILE_IO

#include <fcntl.h>    /* For O_RDONLY, etc. */
#include <malloc.h>   /* For memalign() */
#include <string.h>   /* For memcpy() */
#include <sys/stat.h> /* For stat() */
#include <unistd.h>   /* For open(), etc. */

#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>


namespace io
{
    class PointCloudReader {
    private:
        static const int PAGE_SIZE = 4096;
        static const int IO_REQ_SIZE = PAGE_SIZE * 32;
    public:
        static std::shared_ptr<char []> read_file_dio (const char *path, size_t size)
        {
            auto file_fd = open(path, O_RDONLY | O_DIRECT, S_IRUSR | S_IWUSR);
            std::shared_ptr<char []> buf ((char *)memalign(PAGE_SIZE, size + IO_REQ_SIZE), free);
            size_t read_size = 0;
            for (size_t i = 0; i < size; i += IO_REQ_SIZE)
            {
                auto it_beg = i;
                auto ret = pread(file_fd, &buf[it_beg], IO_REQ_SIZE, it_beg);
                if (ret != IO_REQ_SIZE && read_size + ret != size)
                {
                    std::cout << "Binary file read error: " << i << " " << it_beg
                        << " " << IO_REQ_SIZE << " " << ret << std::endl;
                    exit(1);
                }
                read_size += ret;
            }
            close(file_fd);
            return buf;
        }

        static void write_file_binary (const char *path, const unsigned char *buf, size_t size)
        {
            auto file_fd = open(path, O_WRONLY | O_CREAT, S_IRUSR | S_IWUSR);
            size_t write_size = write(file_fd, buf, size);
            if (write_size != size)
            {
                std::cout << "Binary file write error." << std::endl;
                exit(1);
            }
            close(file_fd);
        }

        static size_t get_file_size (const char *path)
        {
            struct stat st;
            stat(path, &st);
            return st.st_size;
        }
        
        template<typename T>
        static void load_bin (const std::string& path, std::vector<T>& cloud)
        {
            cloud.clear();

            size_t file_size = get_file_size(path.c_str());
            std::shared_ptr<char []> buf = read_file_dio(path.c_str(), file_size);
            std::shared_ptr<float []> float_array = std::reinterpret_pointer_cast<float []>(buf);

            for (size_t i = 0; i < file_size / 16; ++i)
            {
                cloud.emplace_back (
                    float_array[i * 4 + 0],
                    float_array[i * 4 + 1],
                    float_array[i * 4 + 2]
                );
            }
        }

        template<typename T>
        static void load_txt (const std::string& path, std::vector<T>& cloud)
        {
            cloud.clear();

            std::ifstream file(path);
            if (!file.is_open())
            {
                std::cout << "File is not open!" << std::endl;
                exit(-1);
            }

            float x, y, z, intensity;
            while(file >> x >> y >> z >> intensity)
            {
                cloud.emplace_back (x, y, z);
            }
            file.close();
        }

        template<typename T>
        static void dump_bin (const std::string& path, std::vector<T>& cloud)
        {
            PointCloudReader::write_file_binary (
                path.c_str(), 
                reinterpret_cast<unsigned char*>(cloud.data()), 
                cloud.size() * sizeof(T)
            );
        }

        template<typename T>
        static void dump_txt (const std::string& path, std::vector<T>& cloud)
        {
            std::ofstream ofs (path);
            if (!ofs.is_open())
            {
                std::cout << "File is not open!" << std::endl;
            }
            for (size_t i = 0; i < cloud.size(); i++) {
                ofs << cloud[i].data[0] << " ";
                ofs << cloud[i].data[1] << " ";
                ofs << cloud[i].data[2] << " ";
                ofs << "0.0" << '\n';
            }
            ofs.close();
        }

        template<typename T>
        static void dump_ply (const std::string& path, std::vector<T>& cloud)
        {
            std::ofstream ofs (path);
            if (!ofs.is_open())
            {
                std::cout << "File is not open!" << std::endl;
            }
            ofs << "ply" << '\n';
            ofs << "format ascii 1.0" << '\n';
            ofs << "element vertex " << cloud.size() << '\n';
            ofs << "property float x" << '\n';
            ofs << "property float y" << '\n';
            ofs << "property float z" << '\n';
            ofs << "element face 0" << '\n';
            ofs << "end_header" << '\n';
            for (size_t i = 0; i < cloud.size(); i++) {
                ofs << cloud[i].data[0] << " ";
                ofs << cloud[i].data[1] << " ";
                ofs << cloud[i].data[2] << '\n';
            }
            ofs.close();
        }
    };

    inline bool ends_with (const std::string &path, const std::string &ending)
    {
        if (path.length() >= ending.length()) {
            return (0 == path.compare (path.length() - ending.length(), ending.length(), ending));
        } else {
            return false;
        }
    }
    
    inline bool starts_with (const std::string &path, const std::string &starting)
    {
       if (path.length() >= starting.length()) {
            return (0 == path.rfind(starting, 0));
        } else {
            return false;
        }
    }
    
    template<typename T>
    void load (const std::string& path, std::vector<T>& cloud)
    {
        if (ends_with(path, ".bin"))
        {
            PointCloudReader::load_bin (path, cloud);
        }
        else if (ends_with(path, ".txt"))
        {
            PointCloudReader::load_txt (path, cloud);
        }
        else
        {
            std::cout << "The file type is unknown!" << std::endl;
            exit(-1);
        }
    }

    template<typename T>
    void dump (const std::string& path, std::vector<T>& cloud)
    {
        if (ends_with(path, ".bin"))
        {
            PointCloudReader::dump_bin (path, cloud);
        }
        else if (ends_with(path, ".txt"))
        {
            PointCloudReader::dump_txt (path, cloud);
        }
        else if (ends_with(path, ".ply"))
        {
            PointCloudReader::dump_ply (path, cloud);
        }
        else
        {
            std::cout << "The file type is unknown!" << std::endl;
            exit(-1);
        }
    }

    inline void load_compressed (const std::string& path, std::vector<unsigned char>& compressed)
    {
        size_t file_size = PointCloudReader::get_file_size(path.c_str());
        std::shared_ptr<char []> buf = PointCloudReader::read_file_dio(path.c_str(), file_size);
        
        compressed.resize(file_size);
        memcpy(compressed.data(), buf.get(), file_size);
    }

    inline void dump_compressed (const std::string& path, std::vector<unsigned char>& compressed)
    {
        PointCloudReader::write_file_binary(path.c_str(), compressed.data(), compressed.size());
    }
}

#endif
