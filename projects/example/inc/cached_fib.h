#pragma once

#include <string>
#include <vector>

constexpr auto CachedFibTopic = "cached_fib";

class CachedFib
{
private:
    std::vector<int> cache;

public:
    explicit CachedFib(std::size_t);
    int getFib(std::size_t);
};
