#pragma once
#ifndef BVH_TREE_VECTOR_COMPUTE_H
#define BVH_TREE_VECTOR_COMPUTE_H
#include "array"
#include "utils.h"

namespace bvh {
template <typename T, size_t Dim, typename F>
inline std::array<T, Dim> Compute(F&& f) {
  std::array<T, Dim> v;
  StaticFor<0, Dim>([&](size_t i) { v[i] = f(i); });
  return v;
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorAdd(const std::array<T, Dim>& a,
                                    const std::array<T, Dim>& b) {
  return Compute<T, Dim>([&](size_t i) { return a[i] + b[i]; });
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorAdd(const std::array<T, Dim>& a, const T b) {
  return Compute<T, Dim>([&](size_t i) { return a[i] + b; });
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorSub(const std::array<T, Dim>& a, const T b) {
  return Compute<T, Dim>([&](size_t i) { return a[i] - b; });
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorSub(const std::array<T, Dim>& a,
                                    const std::array<T, Dim>& b) {
  return Compute<T, Dim>([&](size_t i) { return a[i] - b[i]; });
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorSub(const std::array<T, Dim>& a) {
  return Compute<T, Dim>([&](size_t i) { return -a[i]; });
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorMul(const std::array<T, Dim>& a,
                                    const std::array<T, Dim>& b) {
  return Compute<T, Dim>([&](size_t i) { return a[i] * b[i]; });
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorMul(const std::array<T, Dim>& a, const T b) {
  return Compute<T, Dim>([&](size_t i) { return a[i] * b; });
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorDiv(const std::array<T, Dim>& a,
                                    const std::array<T, Dim>& b) {
  if constexpr (std::is_floating_point_v<T>) {
    return Compute<T, Dim>([&](size_t i) { return a[i] * SafeInverse(b[i]); });
  } else {
    return Compute<T, Dim>([&](size_t i) {
      return b[i] != 0 ? a[i] / b[i] : std::numeric_limits<T>::max();
    });
  }
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorDiv(const std::array<T, Dim>& a, const T b) {
  if constexpr (std::is_floating_point_v<T>) {
    T div = SafeInverse(b);
    return Compute<T, Dim>([&](size_t i) { return a[i] * div; });
  } else {
    return Compute<T, Dim>([&](size_t i) {
      return b != 0 ? a[i] / b : std::numeric_limits<T>::max();
    });
  }
}

template <typename T, size_t Dim>
inline bool VectorEQ(const std::array<T, Dim>& a, const std::array<T, Dim>& b) {
  if constexpr (std::is_floating_point_v<T>) {
    for (size_t i = 0; i < Dim; ++i) {
      if (NumberNE(a[i], b[i])) {
        return false;
      }
    }
    return true;
  } else {
    return a == b;
  }
}

template <typename T, size_t Dim>
inline bool VectorNE(const std::array<T, Dim>& a, const std::array<T, Dim>& b) {
  if constexpr (std::is_floating_point_v<T>) {
    for (size_t i = 0; i < Dim; ++i) {
      if (NumberNE(a[i], b[i])) {
        return true;
      }
    }
    return false;
  } else {
    return a != b;
  }
}

}  // namespace bvh

#endif