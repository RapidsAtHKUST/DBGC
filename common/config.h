#ifndef COMMON_CONFIG
#define COMMON_CONFIG


#define TIME_START() start = clock::now()
#define TIME_END(name) \
    end = clock::now(); \
    diff = end - start; \
    printf("time for %s: %.9lf s\n", name, diff.count())


namespace conf
{
    //=====================================================
    // constants
    constexpr int bpB = 8;
    constexpr int num_codes = (1U << 8) - 1;
    constexpr double pi = 3.14159265358979323846;

    inline constexpr float deg2ang (float deg)
    {
        return deg * pi / 180.0;
    }

    //=====================================================
    // thresholds
    constexpr int max_recru_levels = 4096;
    constexpr float depth_diff_th = 2.0;
    constexpr float pl_phi_theta = 5;
    constexpr float pl_depth_theta = 0.001;
}

#endif