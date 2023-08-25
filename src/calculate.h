#pragma once
#ifndef BVH_TREE_CALCULATE_H
#define BVH_TREE_CALCULATE_H

#include <cstdint>

#include "fast_vector.h"
#include "utils.h"
#include "status.h"

namespace bvh {

template <typename T, size_t Dim, size_t Edges>
FastVector<T, Dim> ComputeCentrePoint(VectorList<T,Dim,Edges> list) {
  FastVector<T, Dim> r;
  for (auto& vector : list) {
    r = r + vector;
  }
  return r / Edges;
}

// template <typename T, size_t Dim, size_t Edges>
// Status Intersect(const FastVector<T,Dim>& point, const VectorList<T,Dim,Edges>& peek_list, uint32_t* intersection_count) {
//   return Status::MakeNotSupport();
// }

// template <typename T,  size_t Edges>
// Status Intersect<T,2,Edges>(const FastVector<T,2>& point, const VectorList<T,2,Edges>& peek_list, uint32_t* intersection_count) {
//   }

}  // namespace bvh
#endif