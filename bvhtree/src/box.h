#pragma once
#ifndef BVH_TREE_BOX_H
#define BVH_TREE_BOX_H

#include "calculate.h"
#include "fast_vector.h"

namespace bvh {
template <typename T, size_t Dim>
class Box {};

template <typename T>
class Box<T, 2> {
 public:
  Box() = default;
  Box(T upper, T lower, T left, T right)
      : upper_(upper), lower_(lower), left_(left), right_(right) {
    center_[0] = left / static_cast<T>(2) + right / static_cast<T>(2);
    center_[1] = upper / static_cast<T>(2) + lower / static_cast<T>(2);
  }
  explicit Box(const FastVector<T, 2>& point) {
    upper_ = point[1];
    lower_ = point[1];
    left_ = point[0];
    right_ = point[0];
    center_ = point;
  }
  Box(const Box& o) {
    upper_ = o.upper_;
    right_ = o.right_;
    left_ = o.left_;
    lower_ = o.lower_;
    center_ = o.center_;
  }

  Box(Box&& o) {
    upper_ = o.upper_;
    lower_ = o.lower_;
    left_ = o.left_;
    right_ = o.right_;
    center_ = std::move(o.center_);
  }

  Box& operator=(const Box& o) {
    upper_ = o.upper_;
    lower_ = o.lower_;
    left_ = o.left_;
    right_ = o.right_;
    center_ = o.center_;
    return *this;
  }

  Box& operator=(Box&& o) {
    upper_ = o.upper_;
    lower_ = o.lower_;
    left_ = o.left_;
    right_ = o.right_;
    center_ = std::move(o.center_);
    return *this;
  }

  bool operator==(const Box& o) {
    return VectorEQ<T, 4>(
        std::array<T, 4>{upper_, lower_, left_, right_},
        std::array<T, 4>{o.upper_, o.lower_, o.left_, o.right_});
  }

  Box& extend(const FastVector<T, 2>& point) { return extend(Box(point)); }

  Box& extend(const Box& o) {
    ComputeBoxExtend<T, 2>(
        std::array<T*, 4>{&upper_, &right_, &left_, &lower_},
        std::array<const T*, 4>{&o.upper_, &o.right_, &o.left_, &o.lower_});
    center_[0] = left_ / static_cast<T>(2) + right_ / static_cast<T>(2);
    center_[1] = upper_ / static_cast<T>(2) + lower_ / static_cast<T>(2);
    return *this;
  }

  inline bool Contain(const FastVector<T, 2>& point) const {
    return ComputeBoxContain({upper_, right_, left_, lower_}, point);
  }

  inline T Distance(const FastVector<T, 2>& point) const {
    if (point[1] > upper_) {
      if (point[0] > right_) {
        return ComputeDistance(point, FastVector<T, 2>(upper_, right_));
      } else if (point[0] < left_) {
        return ComputeDistance(point, FastVector<T, 2>(upper_, left_));
      } else {
        T temp = point[1] - upper_;
        return temp * temp;
      }
    } else if (point[1] < lower_) {
      if (point[0] > right_) {
        return ComputeDistance(point, FastVector<T, 2>(lower_, right_));
      } else if (point[0] < left_) {
        return ComputeDistance(point, FastVector<T, 2>(lower_, left_));
      } else {
        T temp = lower_ - point[1];
        return temp * temp;
      }
    } else {
      if (point[0] > right_) {
        T temp = point[0] - right_;
        return temp * temp;
      } else if (point[0] < left_) {
        T temp = left_ - point[0];
        return temp * temp;
      } else {
        return static_cast<T>(0);
      }
    }
  }

  inline const FastVector<T, 2>& GetCenter() const { return center_; }

  static constexpr Box MakeEmpty() {
    return Box(-std::numeric_limits<T>::max(), std::numeric_limits<T>::max(),
               std::numeric_limits<T>::max(), -std::numeric_limits<T>::max());
  }

 private:
  T upper_;
  T lower_;
  T left_;
  T right_;
  FastVector<T, 2> center_;
};

}  // namespace bvh

#endif