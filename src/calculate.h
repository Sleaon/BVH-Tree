#pragma once
#ifndef BVH_TREE_CALCULATE_H
#define BVH_TREE_CALCULATE_H

#include <cstdint>

#include "fast_vector.h"
#include "utils.h"
#include "status.h"

namespace bvh {

template <typename T, size_t Dim>
FastVector<T, Dim> ComputeCentrePoint(VectorList<T,Dim> list) {
  FastVector<T, Dim> r;
  for (auto& vector : list) {
    r = r + vector;
  }
  return r / list.size();
}

template <typename T, size_t Dim>
T Sum(FastVector<T,Dim> v) {
  T r=0;
  StaticFor<0, Dim>([&](size_t i) { r +=v[i]; });
  return r;
}

template <typename T, size_t Dim>
T ComputeDistance(const FastVector<T,Dim>& v1, const FastVector<T,Dim>& v2){
  auto diff = v1-v2;
  diff = diff*diff;
  return Sum(diff);
}

template <typename T, size_t Dim>
T ComputeDistance(const FastVector<T,Dim>& point, const FastVector<T,Dim>& begin, const FastVector<T,Dim>& end){
  auto line_vec = end - begin; //AB
	auto point_vec = point - begin; //AP
	auto c = Sum(line_vec * point_vec); //|AB*AP|
	if (c <= 0) return ComputeDistance(point,begin); //|AP|
 
	auto d = ComputeDistance(begin,end); //|AB|^2
	if (c >= d) return ComputeDistance(point,end); //|BP|
 
	auto r = c / d; 
	auto p_shadow = begin + line_vec * r;
 
	return ComputeDistance(point,p_shadow); //|CP|

}

}  // namespace bvh
#endif