#pragma once
#ifndef BVH_TREE_RAY_CASTING_H
#define BVH_TREE_RAY_CASTING_H

#include <algorithm>

#include "calculate.h"
#include "fast_vector.h"
#include "shape_algorithm.h"
#include "utils.h"
namespace bvh {

template <typename T>
class Polygon;

template <typename T, size_t Dim, typename Shape>
class RayCasting : public ContianAlgorithm<T, Dim, Shape> {
 public:
  Status Do(const FastVector<T, Dim>& point, const Shape& shape,
            bool* is_contian) override {
    return Status::MakeNotSupport();
  }
};

template <typename T>
class RayCasting<T, 2, Polygon<T>> : public ContianAlgorithm<T, 2, Polygon<T>> {
 public:
  Status Do(const FastVector<T, 2>& point, const Polygon<T>& polygon,
            bool* is_contian) override {
    uint32_t count = 0;
    try {
      auto edges = polygon.GetEdges();
      for (size_t i = 0; i < edges; ++i) {
        size_t begin_index;
        size_t end_index;
        if (i == 0) {
          begin_index = edges - 1;
          end_index = i;
        } else {
          begin_index = i - 1;
          end_index = i;
        }
        auto& begin = polygon.GetPeak(begin_index);
        auto& end = polygon.GetPeak(end_index);

        if (begin[0] <= point[0] && end[0] <= point[0]) {
          continue;
        }

        if (point[0] < begin[0] && point[1] == begin[1]) {
          ++count;
          continue;
        }

        if ((begin[1] < point[1] && end[1] >= point[1]) ||
            (begin[1] >= point[1] && end[1] < point[1])) {
          T intersection_x;
          if (begin[0] == end[0]) {
            intersection_x = end[0];
          } else {
            intersection_x = begin[0] + (point[1] - begin[1]) *
                                            (end[0] - begin[0]) /
                                            (end[1] - begin[1]);
          }
          if (point[0] <= intersection_x) {
            ++count;
            continue;
          }
        }
      }
      *is_contian = count % 2 !=0;
      return Status::MakeOK();
    } catch (std::exception& e) {
      return Status::MakeError(e.what());
    }
  }
};
}  // namespace bvh

#endif