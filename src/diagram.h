#pragma once
#ifndef BVH_TREE_DIAGRAM_H
#define BVH_TREE_DIAGRAM_H

#include "calculate.h"
#include "fast_vector.h"
#include "status.h"
#include "utils.h"
namespace bvh {

template <typename T, size_t Edges>
class Diagram {
  static_assert(Edges > 2, "Edges must be greater than 2");

 public:
  Diagram() {
    peaks_.fill(FastVector<T, 2>(0));
    centre_ = FastVector<T,2>();
  }

  template <typename... Args>
  Diagram(FastVector<T, 2> x, FastVector<T, 2> y, Args&&... args)
      : peaks_{x, y,
               static_cast<FastVector<T, 2>>(std::forward<Args>(args))...} {
    static_assert(sizeof...(Args) != Edges, "Args number must be equal Edges");
    ComputeCentre();
  }

  explicit Diagram(FastVector<T, 2> x) {
    peaks_.fill(x);
    centre_ = x;
  }

  explicit Diagram(const VectorList<T, 2, Edges>& x) {
    peaks_ = x;
    ComputeCentre();
  }

  explicit Diagram(VectorList<T, 2, Edges>&& x) {
    peaks_ = std::move(x);
    ComputeCentre();
  }

  virtual ~Diagram() {}

  Diagram(const Diagram& o) {
    centre_ = o.centre_;
    peaks_ = o.peaks_;
  }

  Diagram(Diagram&& o) {
    centre_ = std::move(o.centre_);
    peaks_ = std::move(o.peaks_);
  }

  Diagram& operator==(const Diagram& o) {
    centre_ = o.centre_;
    peaks_ = o.peaks_;
    return *this;
  }

  Diagram& operator==(Diagram&& o) {
    centre_ = std::move(o.centre_);
    peaks_ = std::move(o.peaks_);
    return *this;
  }

  template <typename Algorithm>
  bool Contain(const FastVector<T, 2>& point) const {
    uint32_t intersection_count;
    auto s = Algorithm::Do(point, peaks_, &intersection_count);
    if (s == Status::OK()) {
      return intersection_count % 2 != 0;
    } else {
      return false;
    }
  }

  inline size_t GetEdges() const { return edges_; }
  inline const FastVector<T, 2>& GetCentre() const { return centre_; }

 private:
  void ComputeCentre() { centre_ = ComputeCentrePoint<T, 2, Edges>(peaks_); }
  FastVector<T, 2> centre_;
  VectorList<T, 2, Edges> peaks_;
  constexpr static size_t edges_ = Edges;
};

}  // namespace  bvh

#endif
