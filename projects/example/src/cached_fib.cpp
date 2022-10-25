#include "cached_fib.h"

#include <iostream>
#include <vector>

CachedFib::CachedFib(const std::size_t n)
{
    this->cache.push_back(0);
    this->cache.push_back(1);
    for (std::size_t i = 2; i < n; i++) {
        this->cache.push_back(cache[i - 1] + this->cache[i - 2]);
    }
}

int CachedFib::getFib(const std::size_t n)
{
    if (this->cache.size() < n) {
        for (std::size_t i = cache.size(); i < n; i++) {
            this->cache.push_back(cache[i - 1] + this->cache[i - 2]);
        }
    }
    std::cout << this->cache[n - 1] << std::endl;
    return this->cache[n - 1];
}
