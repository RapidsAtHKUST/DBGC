#ifndef PARTITION_BY_LAMBDA
#define PARTITION_BY_LAMBDA

#include <functional>
#include <vector>

#include "common/types.h"


template <typename T>
void filter (
    const std::vector<T>& input,
    std::vector<T>& output,
    std::function<bool(const T&)> f
) {
    output.clear();
    for (size_t i = 0; i < input.size(); i++)
    {
        const T& p = input[i];
        if (f(p))
        {
            output.push_back(p);
        }
    }
}

#endif