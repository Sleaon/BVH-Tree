#pragma once
#ifndef BVH_TREE_FAST_VECTOR_H
#define BVH_TREE_FAST_VECTOR_H

#include <array>

#include "vector_compute.h"
namespace bvh {
template <typename T, size_t Dim>
class FastVector {
 public:
  FastVector() { values_.fill(0); }
  template <typename... Args>
  FastVector(T x, T y, Args&&... args)
      : values_{x, y, static_cast<T>(std::forward<Args>(args))...} {}
  explicit FastVector(T x) noexcept { values_.fill(x); }
  explicit FastVector(std::array<T, Dim>&& x) noexcept {
    values_ = std::move(x);
  }
  explicit FastVector(const std::array<T, Dim>& x) noexcept { values_ = x; }
  FastVector(const FastVector<T, Dim>& o) noexcept { values_ = o.values_; }

  FastVector(FastVector<T, Dim>&& o) noexcept {
    values_ = std::move(o.values_);
  }

  inline size_t GetDim() const { return dim_; }

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

  T& operator[](size_t i) { return values_[i]; }
  T operator[](size_t i) const { return values_[i]; }
  FastVector<T, Dim> operator+(const FastVector<T, Dim>& other) {
    return FastVector(VectorAdd(values_, other.values_));
  }
  FastVector<T, Dim> operator+(T value) {
    return FastVector(VectorAdd(values_, value));
  }
  FastVector<T, Dim> operator-(const FastVector<T, Dim>& other) {
    return FastVector(VectorSub(values_, other.values_));
  }
  FastVector<T, Dim> operator-(T value) {
    return FastVector(VectorSub(values_, value));
  }
  FastVector<T, Dim> operator-() { return FastVector(VectorSub(values_)); }
  FastVector<T, Dim> operator*(const FastVector<T, Dim>& other) {
    return FastVector(VectorMul(values_, other.values_));
  }
  FastVector<T, Dim> operator*(T value) {
    return FastVector(VectorMul(values_, value));
  }

  FastVector<T, Dim> operator/(const FastVector<T, Dim>& other) {
    return FastVector(VectorDiv(values_, other.values_));
  }
  FastVector<T, Dim> operator/(T value) {
    return FastVector(VectorDiv(values_, value));
  }
  bool operator==(const FastVector<T, Dim>& other) {
    if (this == &other) {
      return true;
    } else {
      return VectorEQ(values_, other.values_);
    }
  }
  bool operator!=(const FastVector<T, Dim>& other) {
    if (this == &other) {
      return false;
    } else {
      return VectorNE(values_, other.values_);
    }
  }

 private:
  constexpr static size_t dim_ = Dim;
  std::array<T, Dim> values_;
};

template <typename T, size_t Dim, size_t Edges>
using VectorList = std::array<FastVector<T, Dim>, Edges>;

}  // namespace bvh

#endif
