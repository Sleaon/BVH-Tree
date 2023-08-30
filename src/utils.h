#pragma once
#ifndef BVH_TREE_UTILS_H
#define BVH_TREE_UTILS_H

#include <atomic>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace bvh {

// template <typename T,
//           std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
// T robust_min(T a, T b) {
//   return a < b ? a : b;
// }
// template <typename T,
//           std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
// T robust_max(T a, T b) {
//   return a > b ? a : b;
// }
template <typename T,
          std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
inline bool NumberEQ(T a, T b) {
  constexpr T epslion = std::numeric_limits<T>::epsilon() * 10;
  return std::abs(a - b) <= epslion;
}
template <typename T,
          std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
inline bool NumberNE(T a, T b) {
  constexpr T epslion = std::numeric_limits<T>::epsilon() * 10;
  return std::abs(a - b) > epslion;
}

template <typename T,
          std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
T SafeInverse(T x) {
  return std::fabs(x) <= std::numeric_limits<T>::epsilon()
             ? std::copysign(std::numeric_limits<T>::max(), x)
             : static_cast<T>(1.) / x;
}
template <size_t Begin, size_t End, typename F>
void StaticFor(F&& f) {
  if constexpr (Begin < End) {
    f(Begin);
    StaticFor<Begin + 1, End>(std::forward<F>(f));
  }
}

/// Computes the maximum between an atomic variable and a value, and returns the
/// value previously held by the atomic variable.
template <typename T>
T atomic_max(std::atomic<T>& atomic, const T& value) {
  auto prev_value = atomic.load();
  while (prev_value < value && !atomic.compare_exchange_weak(prev_value,
  value))
    ;
  return prev_value;
}

}  // namespace bvh

#endif
