
#pragma once
#ifndef BVH_TREE_SHAPE_H
#define BVH_TREE_SHAPE_H
#include <vector>

#include "fast_vector.h"
#include "status.h"
#include "shape_algorithm.h"
#include "box.h"

namespace bvh {
template <typename T, size_t Dim>
class Shape {
 public:
  virtual bool Contain(const FastVector<T, Dim>& point) const = 0;

  virtual T Distance(const FastVector<T, Dim>& point) const = 0;
  virtual inline uint64_t GetId() const = 0;
  virtual inline const FastVector<T, 2>& GetCentre() const = 0;
  virtual inline const Box<T,Dim>& GetBox() const = 0;
  virtual ~Shape(){};
};
}  // namespace bvh

#endif