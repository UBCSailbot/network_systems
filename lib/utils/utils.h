#pragma once

#include <boost/math/special_functions.hpp>
#include <concepts>
#include <optional>
#include <sstream>

// Define a concept for arithmetic types
template <typename T>
concept arithmetic = std::integral<T> or std::floating_point<T>;
template <typename T>
concept not_float = not std::floating_point<T>;

namespace utils
{

/**
 * @brief Check if an input value is within its bounds
 *
 * @tparam T   arithmetic type to check
 * @param val  value to check
 * @param lbnd lower bound
 * @param ubnd upper bound
 * @return error string if out of bounds, std::nullopt if okay
 */
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

/**
 * @brief Calculate floating point equality using the GoogleTest default definition
 *        http://google.github.io/googletest/reference/assertions.html#floating-point
 *
 * @tparam T       A floating point type (float, double, etc)
 * @param to_check Value to compare to expected
 * @param expected Expected value
 * @param err_msg  String that gets printed to stderr on failure
 * @return true if "to_check" is close enough to "expected", false otherwise
 */
template <std::floating_point T>
bool isFloatEQ(T to_check, T expected, const std::string & err_msg)
{
    constexpr int ALLOWED_ULP_DIFF = 4;

    int diff = boost::math::float_distance(to_check, expected);
    if (std::abs(diff) <= ALLOWED_ULP_DIFF) {
        return true;
    }
    if (!err_msg.empty()) {
        std::cerr << err_msg << std::endl;
    }
    return false;
}

/**
 * @brief Calls default isFloatEq<T>(T, T, string) with an empty error string
 *
 */
template <std::floating_point T>
bool isFloatEQ(T to_check, T expected)
{
    return isFloatEQ<T>(to_check, expected, "");
}

class FailTracker
{
public:
    void track(bool was_success)
    {
        if (!was_success) {
            fail_count_++;
        }
    }
    void     reset() { fail_count_ = 0; }
    bool     failed() const { return fail_count_ != 0; }
    uint32_t failCount() const { return fail_count_; }

private:
    uint32_t fail_count_ = 0;
};

}  // namespace utils
