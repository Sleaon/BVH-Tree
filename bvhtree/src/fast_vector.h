#pragma once
#ifndef BVH_TREE_FAST_VECTOR_H
#define BVH_TREE_FAST_VECTOR_H

#include <array>
#include <vector>

#include "status.h"
#include "vector_compute.h"
namespace bvh {

template <typename T, size_t Dim>
class FastVector;
template <typename T, size_t Dim>
using VectorList = std::vector<FastVector<T, Dim>>;

template <typename T, size_t Dim>
class FastVector {
 public:
  template <typename TT, size_t DisplayDim, size_t RealDim>
  friend FastVector<TT, DisplayDim> ComputeCentrePoint(
      VectorList<TT, DisplayDim> list);

  template <typename TT, size_t DisplayDim>
  friend TT Sum(FastVector<TT, DisplayDim> v);

  static Status VectorToFastVector(const std::vector<T>& v, FastVector* fv) {
    if (v.size() != Dim) {
      return Status::MakeError("dim is wrong");
    }
    std::array<T, Dim> tmp{0};
    std::copy(v.begin(), v.end(), tmp.begin());
    *fv = FastVector(tmp);
    return Status::MakeOK();
  }

  FastVector() noexcept { values_.fill(0); }

  template <typename... Args>
  FastVector(T x, T y, Args&&... args)
      : values_{x, y, static_cast<T>(std::forward<Args>(args))...} {}

  explicit FastVector(T x) noexcept { values_.fill(x); }

  explicit FastVector(const std::array<T, Dim>& x) noexcept {
    std::copy(x.begin(), x.end(), values_.begin());
  }

  explicit FastVector(size_t real_dim,
                      const std::array<T, ComputeRealDim<T>(Dim)>& x) noexcept {
    values_ = x;
  }

  FastVector(const FastVector<T, Dim>& o) noexcept { values_ = o.values_; }

  FastVector(FastVector<T, Dim>&& o) noexcept {
    values_ = std::move(o.values_);
  }

  constexpr inline size_t GetDim() { return dim_; }
  constexpr inline size_t GetRealDim() { return real_dim_; }

  FastVector<T, Dim>& operator=(const FastVector<T, Dim>& o) noexcept {
    if (this != &o) {
      values_ = o.values_;
    }
    return *this;
  }

  FastVector<T, Dim>& operator=(FastVector<T, Dim>&& o) noexcept {
    if (this != &o) {
      values_ = std::move(o.values_);
    }
    return *this;
  }

  ~FastVector() noexcept {}

  T& operator[](size_t i) { return values_.at(i); }
  T operator[](size_t i) const { return values_.at(i); }
  FastVector<T, Dim> operator+(const FastVector<T, Dim>& other) const {
    return FastVector(real_dim_,
                      VectorAdd<T, real_dim_>(values_, other.values_));
  }
  FastVector<T, Dim> operator+(T value) const {
    return FastVector(real_dim_, VectorAdd<T, real_dim_>(values_, value));
  }
  FastVector<T, Dim> operator-(const FastVector<T, Dim>& other) const {
    return FastVector(real_dim_,
                      VectorSub<T, real_dim_>(values_, other.values_));
  }
  FastVector<T, Dim> operator-(T value) const {
    return FastVector(real_dim_, VectorSub<T, real_dim_>(values_, value));
  }
  FastVector<T, Dim> operator-() const {
    return FastVector(real_dim_, VectorSub<T, real_dim_>(values_));
  }
  FastVector<T, Dim> operator*(const FastVector<T, Dim>& other) const {
    return FastVector(real_dim_,
                      VectorMul<T, real_dim_>(values_, other.values_));
  }
  FastVector<T, Dim> operator*(T value) const {
    return FastVector(real_dim_, VectorMul<T, real_dim_>(values_, value));
  }

  FastVector<T, Dim> operator/(const FastVector<T, Dim>& other) const {
    return FastVector(real_dim_,
                      VectorDiv<T, real_dim_>(values_, other.values_));
  }
  FastVector<T, Dim> operator/(T value) const {
    return FastVector(real_dim_, VectorDiv<T, real_dim_>(values_, value));
  }
  bool operator==(const FastVector<T, Dim>& other) const {
    if (this == &other) {
      return true;
    } else {
      return VectorEQ<T, Dim, real_dim_>(values_, other.values_);
    }
  }
  bool operator!=(const FastVector<T, Dim>& other) const {
    if (this == &other) {
      return false;
    } else {
      return VectorNE<T, Dim, real_dim_>(values_, other.values_);
    }
  }

 private:
  static constexpr size_t dim_ = Dim;
  static constexpr size_t real_dim_ = ComputeRealDim<T>(Dim);
  std::array<T, real_dim_> values_;
};

}  // namespace bvh

#endif
