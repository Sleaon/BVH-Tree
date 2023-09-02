#pragma once
#ifndef BVH_TREE_CLOSEST_POINT_H
#define BVH_TREE_CLOSEST_POINT_H

#include <vector>

#include "calculate.h"
#include "fast_vector.h"
#include "shape_algorithm.h"
#include "utils.h"
namespace bvh {

template <typename T>
class Polygon;

template <typename T, size_t Dim>
class ClosestPoint : public NearstAlgorithm<T, Dim, Polygon<T>> {
 public:
  Status Do(const FastVector<T, Dim>& point, const Polygon<T>& polygon,
            T* distance) override {
    auto min_pd = std::numeric_limits<T>::max();
    size_t min_index;
    for (auto i : polygon.GetPeeaksByQuadrant(point)) {
      auto pd = ComputeDistance(point, polygon.GetPeak(i));
      if (pd < min_pd) {
        min_index = i;
        min_pd = pd;
      }
    }
    FastVector<T, Dim> begin;
    FastVector<T, Dim> end1;
    FastVector<T, Dim> end2;
    auto edges = polygon.GetEdges();
    if (min_index == 0) {
      begin = polygon.GetPeak(min_index);
      end1 = polygon.GetPeak(edges - 1);
      end2 = polygon.GetPeak(min_index + 1);
    } else if (min_index == edges - 1) {
      begin = polygon.GetPeak(min_index);
      end1 = polygon.GetPeak(min_index - 1);
      end2 = polygon.GetPeak(0);
    } else {
      begin = polygon.GetPeak(min_index);
      end1 = polygon.GetPeak(min_index - 1);
      end2 = polygon.GetPeak(min_index + 1);
    }
    auto dist1 = ComputeDistance(point, begin, end1);
    auto dist2 = ComputeDistance(point, begin, end2);
    *distance = dist1 < dist2 ? dist1 : dist2;
    return Status::MakeOK();
  }
};

}  // namespace bvh

#endif