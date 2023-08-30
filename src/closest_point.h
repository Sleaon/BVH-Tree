#pragma once
#ifndef BVH_TREE_CLOSEST_POINT_H
#define BVH_TREE_CLOSEST_POINT_H

#include <vector>

#include "calculate.h"
#include "fast_vector.h"
#include "utils.h"
namespace bvh {

template <typename T, size_t Dim, size_t Edges>
class ClosestPoint {
 public:
  Status Do(const FastVector<T, Dim>& point,
            const Diagram<T,Edges>& diagram,
            T* distance) {
    auto min_pd= std::numeric_limits<T>::max();
    size_t min_index;
    for (auto i : diagram.GetPeeaksByQuadrant(point)) {
      auto pd = ComputeDistance(point, diagram.GetPeak(i));
      if(pd < min_pd){
        min_index = i;
        min_pd = pd;
      }
    }
    FastVector<T,Dim> begin;
    FastVector<T,Dim> end1;
    FastVector<T,Dim> end2;
    if(min_index == 0){
        begin = diagram.GetPeak(min_index);
        end1 = diagram.GetPeak(Edges-1);
        end2 = diagram.GetPeak(min_index+1);
    }else if (min_index == Edges-1){
        begin = diagram.GetPeak(min_index);
        end1 = diagram.GetPeak(min_index-1);
        end2 = diagram.GetPeak(0);
    }else{
        begin = diagram.GetPeak(min_index);
        end1 = diagram.GetPeak(min_index-1);
        end2 = diagram.GetPeak(min_index+1);
    }
    auto dist1 = ComputeDistance(point,begin,end1);
    auto dist2 = ComputeDistance(point,begin,end2);
    *distance = dist1 < dist2 ? dist1:dist2;
    return Status::MakeOK();
  }
};

}  // namespace bvh

#endif