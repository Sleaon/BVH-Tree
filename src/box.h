#pragma once
#ifndef BVH_TREE_BOX_H
#define BVH_TREE_BOX_H

#include "fast_vector.h"
#include "shape.h"

namespace bvh {
// template <typename T, size_t Dim>
// class Box {
//  public:
//   Box() = default;
//   BBox()
//       : min(min), max(max) {}
//   explicit BBox(const FastVector<T, Dim>& point)
//       : BBox(point, point) {}

//   BVH_ALWAYS_INLINE BBox& extend(const Vec<T, N>& point) {
//     return extend(BBox(point));
//   }

//   BVH_ALWAYS_INLINE BBox& extend(const BBox& other) {
//     min = robust_min(min, other.min);
//     max = robust_max(max, other.max);
//     return *this;
//   }

//   BVH_ALWAYS_INLINE Vec<T, N> get_diagonal() const { return max - min; }
//   BVH_ALWAYS_INLINE Vec<T, N> get_center() const {
//     return (max + min) * static_cast<T>(0.5);
//   }

//   BVH_ALWAYS_INLINE T get_half_area() const {
//     auto d = get_diagonal();
//     static_assert(N == 2 || N == 3);
//     if constexpr (N == 3) return (d[0] + d[1]) * d[2] + d[0] * d[1];
//     if constexpr (N == 2) return d[0] + d[1];
//     return static_cast<T>(0.);
//   }

//   BVH_ALWAYS_INLINE static constexpr BBox make_empty() {
//     return BBox(Vec<T, N>(+std::numeric_limits<T>::max()),
//                 Vec<T, N>(-std::numeric_limits<T>::max()));
//   }

//  private:
//   FastVector<T, Dim> upper_left, upper_right, lower_left, lower_right;
// };

}  // namespace bvh

#endif