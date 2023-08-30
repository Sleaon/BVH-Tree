#pragma once
#ifndef BVH_TREE_DIAGRAM_H
#define BVH_TREE_DIAGRAM_H
#include <vector>

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
    SplitPeek();
  }

  template <typename... Args>
  Diagram(uint64_t id, FastVector<T, 2> x, FastVector<T, 2> y, Args&&... args)
      : id_(id),
        peaks_{x, y, static_cast<FastVector<T, 2>>(std::forward<Args>(args))...} {
    static_assert(sizeof...(Args) != Edges, "Args number must be equal Edges");
    ComputeCentre();
    SplitPeek();
  }

  Diagram(uint64_t id, FastVector<T, 2> x) : id_(id) {
    peaks_.fill(x);
    centre_ = x;
    SplitPeek();
  }

  Diagram(uint64_t id, const VectorList<T, 2, Edges>& x) : id_(id) {
    peaks_ = x;
    ComputeCentre();
    SplitPeek();
  }

  Diagram(uint64_t id, VectorList<T, 2, Edges>&& x) : id_(id) {
    peaks_ = std::move(x);
    ComputeCentre();
    SplitPeek();
  }

  virtual ~Diagram() {}

  Diagram(const Diagram& o) {
    centre_ = o.centre_;
    peaks_ = o.peaks_;
    id_ = o.id_;
    quadrant_ = o.quadrant_;
  }

  Diagram(Diagram&& o) {
    centre_ = std::move(o.centre_);
    peaks_ = std::move(o.peaks_);
    id_ = o.id_;
    quadrant_ = std::move(o.quadrant_);
  }

  Diagram& operator==(const Diagram& o) {
    centre_ = o.centre_;
    peaks_ = o.peaks_;
    id_ = o.id_;
    quadrant_ = o.quadrant_;
    return *this;
  }

  Diagram& operator==(Diagram&& o) {
    centre_ = std::move(o.centre_);
    peaks_ = std::move(o.peaks_);
    id_ = o.id_;
    quadrant_ = std::move(o.quadrant_);
    return *this;
  }

  bool operator<(const Diagram& o) const { return id_ < o.id_; }

  template <typename Algorithm>
  bool Contain(const FastVector<T, 2>& point, Algorithm alg) const {
    uint32_t intersection_count;
    auto s = alg.Do(point, *this, &intersection_count);
    if (s == Status::OK()) {
      return intersection_count % 2 != 0;
    } else {
      return false;
    }
  }

  template <typename Algorithm>
  T Distance(const FastVector<T, 2>& point, Algorithm alg) const {
    T distance;
    auto s = alg.Do(point, *this, &distance);
    if (s == Status::OK()) {
      return distance;
    } else {
      return std::numeric_limits<T>::max();
    }
  }

  inline size_t GetEdges() const { return edges_; }
  inline uint64_t GetId() const { return id_; }
  inline const FastVector<T, 2>& GetPeak(size_t i) const {return peaks_[i]; }
  inline const FastVector<T, 2>& GetCentre() const { return centre_; }
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
  void ComputeCentre() { centre_ = ComputeCentrePoint<T, 2, Edges>(peaks_); }
  void SplitPeek() {
    StaticFor<0, Edges>([this](size_t i) {
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
    });
  }
  FastVector<T, 2> centre_;
  VectorList<T, 2, Edges> peaks_;
  std::array<std::vector<size_t>, 4> quadrant_;
  uint64_t id_;
  constexpr static size_t edges_ = Edges;
};

}  // namespace  bvh

#endif
