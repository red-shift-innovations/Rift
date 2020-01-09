#pragma once

#include <cmath>
#include <stdexcept>
#include <sstream>
#include <type_traits>

namespace rift {

template<class T>
using BoundType = std::conditional_t<std::is_floating_point_v<T>, long long, T>;

template<
    class T,
    BoundType<T> U = std::numeric_limits<BoundType<T>>::min,
    BoundType<T> V = std::numeric_limits<BoundType<T>>::max>
struct Bounded {
    /** Construct a Bounded number. Do bounds checking. */
    Bounded(T const &num) {
        if (num < U) {
            std::stringstream ss;
            ss << "Attempted to construct a Bounded number with value " << num
               << " which is smaller than the lower bound " << U;
            throw std::runtime_error(ss.str());
        } else if (num > V) {
            std::stringstream ss;
            ss << "Attempted to construct a Bounded number with value " << num
               << " which is larger than the upper bound " << V;
            throw std::runtime_error(ss.str());
        }
        mNum = num;
    }

private:
    /** The underlying number */
    T mNum;
};

}