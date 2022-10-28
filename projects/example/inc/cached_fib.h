#pragma once

#include <string>
#include <vector>

constexpr auto CachedFibTopic = "cached_fib";

class CachedFib
{
private:
    std::vector<int> cache;

public:
    explicit CachedFib(const std::size_t);
    int getFib(const std::size_t);
};
