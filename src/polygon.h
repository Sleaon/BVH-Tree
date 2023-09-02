#pragma once
#ifndef BVH_TREE_POLYGON_H
#define BVH_TREE_POLYGON_H
#include <vector>

#include "shape.h"
#include "calculate.h"
#include "fast_vector.h"
#include "status.h"
#include "utils.h"
#include "ray_casting.h"
#include "closest_point.h"
namespace bvh {

template <typename T>
class Polygon: public Shape<T,2> {

 public:
  Polygon() : Polygon(0) {}
  explicit Polygon(uint64_t id) : id_(id),edges_(1) {
    peaks_.emplace_back(FastVector<T, 2>(0));
    centre_ = FastVector<T, 2>();
    SplitPeek();
  }

  template <typename... Args>
  Polygon(uint64_t id, FastVector<T, 2> x, FastVector<T, 2> y, Args&&... args)
      : id_(id),
        peaks_{x, y, static_cast<FastVector<T, 2>>(std::forward<Args>(args))...},
        edges_(sizeof...(Args)+2) {
    ComputeCentre();
    SplitPeek();
  }

  Polygon(uint64_t id, FastVector<T, 2> x) : id_(id),edges_(1) {
    peaks_.emplace_back(x);
    centre_ = x;
    SplitPeek();
  }

  Polygon(uint64_t id, const VectorList<T, 2>& x) : id_(id) {
    peaks_ = x;
    edges_ = x.size();
    ComputeCentre();
    SplitPeek();
  }

  Polygon(uint64_t id, VectorList<T, 2>&& x) : id_(id) {
    edges_ = x.size();
    peaks_ = std::move(x);
    ComputeCentre();
    SplitPeek();
  }

  virtual ~Polygon() {}

  Polygon(const Polygon& o) {
    centre_ = o.centre_;
    peaks_ = o.peaks_;
    id_ = o.id_;
    edges_ = o.edges_;
    quadrant_ = o.quadrant_;
  }

  Polygon(Polygon&& o) {
    centre_ = std::move(o.centre_);
    peaks_ = std::move(o.peaks_);
    id_ = o.id_;
    edges_ = o.edges_;
    quadrant_ = std::move(o.quadrant_);
  }

  Polygon& operator=(const Polygon& o) {
    centre_ = o.centre_;
    peaks_ = o.peaks_;
    id_ = o.id_;
    edges_ = o.edges_;
    quadrant_ = o.quadrant_;
    return *this;
  }

  Polygon& operator=(Polygon&& o) {
    centre_ = std::move(o.centre_);
    peaks_ = std::move(o.peaks_);
    id_ = o.id_;
    edges_ = o.edges_;
    quadrant_ = std::move(o.quadrant_);
    return *this;
  }

  bool operator<(const Polygon& o) const { return id_ < o.id_; }

  bool Contain(const FastVector<T, 2>& point) const override {
    uint32_t intersection_count;
    bool is_contian;
    static RayCasting<T,2,Polygon<T>> alg;
    auto s = alg.Do(point, *this, &is_contian);
    if (s == Status::OK()) {
      return is_contian;
    } else {
      return false;
    }
  }

  T Distance(const FastVector<T, 2>& point) const override{
    T distance;
    static ClosestPoint<T,2> alg;
    auto s = alg.Do(point, *this, &distance);
    if (s == Status::OK()) {
      return distance;
    } else {
      return std::numeric_limits<T>::max();
    }
  }

  inline uint64_t GetId() const override{ return id_; }
  inline const FastVector<T, 2>& GetCentre() const override { return centre_; }
  inline size_t GetEdges() const { return edges_; }
  inline const FastVector<T, 2>& GetPeak(size_t i) const {return peaks_[i]; }
  inline const std::vector<size_t>& GetPeeaksByQuadrant(
      const FastVector<T, 2>& point) const {
    if (point[0] < centre_[0]) {
      if (point[1] < centre_[1]) {
        return quadrant_[2];
      } else {
        return quadrant_[1];
      }
    } else {
      if (point[1] < centre_[1]) {
        return quadrant_[3];
      } else {
        return quadrant_[0];
      }
    }
  }

 private:
  void ComputeCentre() { centre_ = ComputeCentrePoint<T, 2>(peaks_); }
  void SplitPeek() {
    for(auto i=0;i<edges_;++i){
      auto& p = this->peaks_[i];
      auto& c = this->centre_;
      if (p == c) [[unlikely]] {
          this->quadrant_[0].emplace_back(i);
          this->quadrant_[1].emplace_back(i);
          this->quadrant_[2].emplace_back(i);
          this->quadrant_[3].emplace_back(i);
      }

      if (p[0] < c[0]) {
        if (p[1] < c[1]) {
          this->quadrant_[2].emplace_back(i);
        } else {
          this->quadrant_[1].emplace_back(i);
        }
      } else {
        if (p[1] < c[1]) {
          this->quadrant_[3].emplace_back(i);
        } else {
          this->quadrant_[0].emplace_back(i);
        }
      }

    }
  }
  FastVector<T, 2> centre_;
  VectorList<T, 2> peaks_;
  std::array<std::vector<size_t>, 4> quadrant_;
  uint64_t id_;
  size_t edges_;
};

}  // namespace  bvh

#endif
