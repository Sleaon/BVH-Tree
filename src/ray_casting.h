#pragma once
#ifndef BVH_TREE_RAY_CASTING_H
#define BVH_TREE_RAY_CASTING_H

#include <bitset>

#include "calculate.h"
#include "fast_vector.h"
#include "utils.h"
namespace bvh {

template <typename T, size_t Dim, size_t Edges>
class RayCasting {
  static Status Do(const FastVector<T, Dim>& point,
                   const VectorList<T, Dim, Edges>& peek_list,
                   uint32_t* intersection_count) {
    return Status::MakeNotSupport();
  }
};

template <typename T, size_t Edges>
class RayCasting<T, 2, Edges> {
  static Status Do(const FastVector<T, 2>& point,
                   const VectorList<T, 2, Edges>& peek_list,
                   uint32_t* intersection_count) {
    std::bitset<Edges> result;
    try {
      StaticFor<0, Edges>([&](size_t i) {
        size_t begin_index;
        size_t end_index;
        if (i == 0) {
          begin_index = Edges - 1;
          end_index = i;
        } else {
          begin_index = i - 1;
          end_index = i;
        }
        auto& begin = peek_list[begin_index];
        auto& end = peek_list[end_index];
        if (point == begin || point == end) {
          result[i] = true;
        }

        if ((begin[1] < point[1] && end[1] >= point[1]) ||
            (begin[1] >= point[1] && end[1] < point[1])) {
          auto intersection_x = begin[0] + (point[1] - begin[1]) *
                                               (end[0] - begin[0]) /
                                               (end[1] - begin[1]);
          if (point[0] > intersection_x || intersection_x == point[0]) {
            result[i] = true;
          }
        }
        result[i] = false;
      });
      *intersection_count = result.count();
      return Status::MakeOK();
    } catch (std::exception& e) {
      return Status::MakeError(e.what());
    }
  }
};
}  // namespace bvh

#endif