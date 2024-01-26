#include <concepts>
#include <iostream>

template <typename T>
concept arithmetic = std::integral<T> or std::floating_point<T>;

namespace utils
{
template <arithmetic T>
bool checkBounds(T val, T lbnd, T ubnd)
{
    if (val < lbnd || val > ubnd) {
        std::cerr << "Error, " << typeid(T).name() << "(" << val << ") is out of bounds" << std::endl
                  << "lbnd: " << lbnd << std::endl
                  << "ubnd: " << ubnd << std::endl;
        return false;
    }
    return true;
}
}  // namespace utils
