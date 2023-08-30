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
  Diagram() : Diagram(0) {}
  explicit Diagram(uint64_t id) : id_(id) {
    peaks_.fill(FastVector<T, 2>(0));
    centre_ = FastVector<T, 2>();
  }

  template <typename... Args>
  Diagram(uint64_t id, FastVector<T, 2> x, FastVector<T, 2> y, Args&&... args)
      : id_(id),
        peaks_{x, y,
               static_cast<FastVector<T, 2>>(std::forward<Args>(args))...} {
    static_assert(sizeof...(Args) != Edges, "Args number must be equal Edges");
    ComputeCentre();
  }

  Diagram(uint64_t id, FastVector<T, 2> x) : id_(id) {
    peaks_.fill(x);
    centre_ = x;
  }

  Diagram(uint64_t id, const VectorList<T, 2, Edges>& x) : id_(id) {
    peaks_ = x;
    ComputeCentre();
  }

  Diagram(uint64_t id, VectorList<T, 2, Edges>&& x) : id_(id) {
    peaks_ = std::move(x);
    ComputeCentre();
  }

  virtual ~Diagram() {}

  Diagram(const Diagram& o) {
    centre_ = o.centre_;
    peaks_ = o.peaks_;
    id_ = o.id_;
  }

  Diagram(Diagram&& o) {
    centre_ = std::move(o.centre_);
    peaks_ = std::move(o.peaks_);
    id_ = o.id_;
  }

  Diagram& operator==(const Diagram& o) {
    centre_ = o.centre_;
    peaks_ = o.peaks_;
    id_ = o.id_;
    return *this;
  }

  Diagram& operator==(Diagram&& o) {
    centre_ = std::move(o.centre_);
    peaks_ = std::move(o.peaks_);
    id_ = o.id_;
    return *this;
  }

  bool operator<(const Diagram& o) const { return id_ < o.id_; }

  template <typename Algorithm>
  bool Contain(const FastVector<T, 2>& point, Algorithm alg) const {
    uint32_t intersection_count;
    auto s = alg.Do(point, peaks_, &intersection_count);
    if (s == Status::OK()) {
      return intersection_count % 2 != 0;
    } else {
      return false;
    }
  }

  inline size_t GetEdges() const { return edges_; }
  inline uint64_t GetId() const { return id_; }
  inline const FastVector<T, 2>& GetCentre() const { return centre_; }

 private:
  void ComputeCentre() { centre_ = ComputeCentrePoint<T, 2, Edges>(peaks_); }
  FastVector<T, 2> centre_;
  VectorList<T, 2, Edges> peaks_;
  uint64_t id_;
  constexpr static size_t edges_ = Edges;
};

}  // namespace  bvh

#endif
