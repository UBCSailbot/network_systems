#pragma once

#include <concepts>
#include <optional>
#include <sstream>

template <typename T>
concept arithmetic = std::integral<T> or std::floating_point<T>;

namespace utils
{
template <arithmetic T>
std::optional<std::string> isOutOfBounds(T val, T lbnd, T ubnd)
{
    if (val < lbnd || val > ubnd) {
        std::stringstream ss;
        ss << typeid(T).name() << "(" << val << ") is out of bounds" << std::endl
           << "lbnd: " << lbnd << std::endl
           << "ubnd: " << ubnd << std::endl;
        return ss.str();
    }
    return std::nullopt;
}

}  // namespace utils
