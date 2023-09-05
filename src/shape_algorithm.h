#pragma once
#ifndef BVH_TREE_SHAPE_ALGORITHM_H
#define BVH_TREE_SHAPE_ALGORITHM_H

#include "fast_vector.h"
#include "status.h"
namespace bvh {

template <typename T, size_t Dim, typename ShapeImpl>
class ContianAlgorithm {
 public:
  virtual Status Do(const FastVector<T, Dim>& point, const ShapeImpl& shape,
                    bool* is_contian) = 0;
};

template <typename T, size_t Dim, typename ShapeImpl>
class NearestAlgorithm {
 public:
  virtual Status Do(const FastVector<T, Dim>& point, const ShapeImpl& shape,
                    T* distance) = 0;
};
}  // namespace bvh

#endif