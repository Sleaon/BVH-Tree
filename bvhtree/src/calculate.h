#pragma once
#ifndef BVH_TREE_CALCULATE_H
#define BVH_TREE_CALCULATE_H

#include <cstdint>

#include "fast_vector.h"
#include "utils.h"

namespace bvh {

template <typename T, size_t Dim>
inline FastVector<T, Dim> ComputeCentrePoint(VectorList<T, Dim> list) {
  FastVector<T, Dim> r;
  for (auto& vector : list) {
    r = r + vector;
  }
  return r / static_cast<T>(list.size());
}

template <typename T, size_t Dim>
inline T Sum(FastVector<T, Dim> v) {
  T r = 0;
  StaticFor<0, Dim>([&](size_t i) { r += v[i]; });
  return r;
}

template <typename T, size_t Dim>
inline T ComputeDistance(const FastVector<T, Dim>& v1,
                         const FastVector<T, Dim>& v2) {
  auto diff = v1 - v2;
  diff = diff * diff;
  return Sum(diff);
}

template <typename T, size_t Dim>
inline T ComputeDistance(const FastVector<T, Dim>& point,
                         const FastVector<T, Dim>& begin,
                         const FastVector<T, Dim>& end) {
  auto line_vec = end - begin;                       // AB
  auto point_vec = point - begin;                    // AP
  auto c = Sum(line_vec * point_vec);                //|AB*AP|
  if (c <= 0) return ComputeDistance(point, begin);  //|AP|

  auto d = ComputeDistance(begin, end);            //|AB|^2
  if (c >= d) return ComputeDistance(point, end);  //|BP|

  auto r = c / d;
  auto p_shadow = begin + line_vec * r;

  return ComputeDistance(point, p_shadow);  //|CP|
}

template <typename T, size_t Dim>
inline void ComputeBoxExtend(std::array<T*, 2 * Dim>&& source,
                             std::array<const T*, 2 * Dim>&& extend) {
  static_assert(Dim == 2 || Dim == 3, "Dim must be 2 or 3");
  if constexpr (Dim == 2) {
    *source[0] = *source[0] > *extend[0] ? *source[0] : *extend[0];
    *source[1] = *source[1] > *extend[1] ? *source[1] : *extend[1];
    *source[2] = *source[2] > *extend[2] ? *extend[2] : *source[2];
    *source[3] = *source[3] > *extend[3] ? *extend[3] : *source[3];
  }
}

template <typename T, size_t Dim>
inline std::tuple<T, T, T, T> ComputeBoxRange(
    const VectorList<T, Dim>& source) {
  static_assert(Dim == 2 || Dim == 3, "Dim must be 2 or 3");
  if constexpr (Dim == 2) {
    T upper = -std::numeric_limits<T>::max();
    T lower = std::numeric_limits<T>::max();
    T left = std::numeric_limits<T>::max();
    T right = -std::numeric_limits<T>::max();
    for (auto&& v : source) {
      if (v[0] > right) {
        right = v[0];
      } else if (v[0] < left) {
        left = v[0];
      }
      if (v[1] > upper) {
        upper = v[1];
      } else if (v[1] < lower) {
        lower = v[1];
      }
    }
    return std::make_tuple(upper, lower, left, right);
  }
}

template <typename T, size_t Dim>
inline bool ComputeBoxContain(const std::array<T, 2 * Dim>&& source,
                              const FastVector<T, Dim>& point) {
  static_assert(Dim == 2 || Dim == 3, "Dim must be 2 or 3");
  if constexpr (Dim == 2) {
    if (point[0] <= source[1] && point[0] >= source[2] && point[1] <= source[0] &&
        point[1] >= source[3]) {
      return true;
    }
  }
  return false;
}

}  // namespace bvh
#endif