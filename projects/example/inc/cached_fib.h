#pragma once

#include <string>
#include <vector>

constexpr auto CACHED_FIB_TOPIC = "cached_fib";

class CachedFib
{
private:
    std::vector<int> cache_;

public:
    explicit CachedFib(std::size_t);
    int getFib(std::size_t);
};
