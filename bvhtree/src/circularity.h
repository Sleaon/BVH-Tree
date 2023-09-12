#pragma once
#ifndef BVH_TREE_CIRCULARITY_H
#define BVH_TREE_CIRCULARITY_H
#include <vector>

#include "calculate.h"
#include "fast_vector.h"
#include "shape.h"
#include "status.h"
#include "utils.h"
namespace bvh {

template <typename T>
class Circularity : public Shape<T, 2> {
 public:
  Circularity() : Circularity(0) {}

  explicit Circularity(uint64_t id) : id_(id), radius_(0) {
    centre_ = FastVector<T, 2>();
    ComputeRange();
  }

  Circularity(uint64_t id, T radius) : id_(id), radius_(radius) {
    centre_ = FastVector<T, 2>();
    ComputeRange();
  }

  Circularity(uint64_t id, T radius, const FastVector<T, 2>& x)
      : id_(id), radius_(radius), centre_(x) {
    ComputeRange();
  }

  ~Circularity() override {}

  Circularity(const Circularity& o) {
    id_ = o.id_;
    radius_ = o.radius_;
    centre_ = o.centre_;
    box_ = o.box_;
  }

  Circularity(Circularity&& o) {
    id_ = o.id_;
    radius_ = o.radius_;
    centre_ = std::move(o.centre_);
    box_ = std::move(o.box_);
  }

  Circularity& operator=(const Circularity& o) {
    id_ = o.id_;
    radius_ = o.radius_;
    centre_ = o.centre_;
    box_ = o.box_;
    return *this;
  }

  Circularity& operator=(Circularity&& o) {
    id_ = o.id_;
    radius_ = o.radius_;
    centre_ = std::move(o.centre_);
    box_ = std::move(o.box_);
    return *this;
  }

  bool operator<(const Circularity& o) const { return id_ < o.id_; }

  bool Contain(const FastVector<T, 2>& point) const override {
    T dist = ComputeDistance(centre_, point);
    return dist < radius_;
  }

  T Distance(const FastVector<T, 2>& point) const override {
    T dist = ComputeDistance(centre_, point);
    if (dist < radius_) {
      return 0;
    } else {
      return dist - radius_;
    }
  }

  inline uint64_t GetId() const override { return id_; }
  inline const FastVector<T, 2>& GetCentre() const override { return centre_; }
  inline const T GetRadius() const { return radius_; }
  inline const Box<T, 2>& GetBox() const { return box_; }

 private:
  void ComputeRange() {
    T upper, lower, left, right;
    upper = centre_[1] + radius_;
    lower = centre_[1] - radius_;
    left = centre_[0] - radius_;
    right = centre_[0] + radius_;
    box_ = Box<T, 2>(upper, lower, left, right);
  }

 private:
  uint64_t id_;
  T radius_;
  FastVector<T, 2> centre_;
  Box<T, 2> box_;
};

}  // namespace  bvh

#endif