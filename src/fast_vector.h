#pragma once
#ifndef BVH_TREE_FAST_VECTOR_H
#define BVH_TREE_FAST_VECTOR_H

#include <array>
#include <vector>

#include "status.h"
#include "vector_compute.h"
namespace bvh {
template <typename T, size_t Dim>
class FastVector {
 public:
  static Status VectorToFastVector(const std::vector<T>& v, FastVector* fv) {
    if (v.size() != Dim) {
      return Status::MakeError("dim is wrong");
    }
    std::array<T, Dim> tmp;
    std::copy(v.begin(), v.end(), tmp.begin());
    *fv = FastVector(tmp);
    return Status::MakeOK();
  }

  FastVector() noexcept {
    values_ = new std::array<T, Dim>;
    values_->fill(0);
  }

  template <typename... Args>
  FastVector(T x, T y, Args&&... args) noexcept {
    values_ = new std::array<T, Dim>{
        x, y, static_cast<T>(std::forward<Args>(args))...};
  }

  explicit FastVector(T x) noexcept {
    values_ = new std::array<T, Dim>;
    values_->fill(x);
  }

  explicit FastVector(const std::array<T, Dim>& x) noexcept {
    values_ = new std::array<T, Dim>(x);
  }

  FastVector(const FastVector<T, Dim>& o) noexcept {
    values_ = new std::array<T, Dim>(*o.values_);
  }

  FastVector(FastVector<T, Dim>&& o) noexcept {
    values_ = o.values_;
    o.values_ = nullptr;
  }

  inline size_t GetDim() const { return dim_; }

  FastVector<T, Dim>& operator=(const FastVector<T, Dim>& o) noexcept {
    if (this != &o) {
      if (values_ != nullptr) {
        delete values_;
      }
      values_ = new std::array<T, Dim>(*o.values_);
    }
    return *this;
  }
  FastVector<T, Dim>& operator=(FastVector<T, Dim>&& o) noexcept {
    if (this != &o) {
      values_ = o.values_;
      o.values_ = nullptr;
    }
    return *this;
  }
  ~FastVector() noexcept {
    if (values_ != nullptr) {
      delete values_;
    }
  }

  T& operator[](size_t i) { return values_->at(i); }
  T operator[](size_t i) const { return values_->at(i); }
  FastVector<T, Dim> operator+(const FastVector<T, Dim>& other) const {
    return FastVector(VectorAdd<T, Dim>(*values_, *other.values_));
  }
  FastVector<T, Dim> operator+(T value) const {
    return FastVector(VectorAdd<T, Dim>(*values_, value));
  }
  FastVector<T, Dim> operator-(const FastVector<T, Dim>& other) const {
    return FastVector(VectorSub<T, Dim>(*values_, *other.values_));
  }
  FastVector<T, Dim> operator-(T value) const {
    return FastVector(VectorSub<T, Dim>(*values_, value));
  }
  FastVector<T, Dim> operator-() const {
    return FastVector(VectorSub<T, Dim>(*values_));
  }
  FastVector<T, Dim> operator*(const FastVector<T, Dim>& other) const {
    return FastVector(VectorMul<T, Dim>(*values_, *other.values_));
  }
  FastVector<T, Dim> operator*(T value) const {
    return FastVector(VectorMul<T, Dim>(*values_, value));
  }

  FastVector<T, Dim> operator/(const FastVector<T, Dim>& other) const {
    return FastVector(VectorDiv<T, Dim>(*values_, *other.values_));
  }
  FastVector<T, Dim> operator/(T value) const {
    return FastVector(VectorDiv<T, Dim>(*values_, value));
  }
  bool operator==(const FastVector<T, Dim>& other) const {
    if (this == &other) {
      return true;
    } else {
      return VectorEQ<T, Dim>(*values_, *other.values_);
    }
  }
  bool operator!=(const FastVector<T, Dim>& other) const {
    if (this == &other) {
      return false;
    } else {
      return VectorNE<T, Dim>(*values_, *other.values_);
    }
  }

 private:
  constexpr static size_t dim_ = Dim;
  std::array<T, Dim>* values_;
};

template <typename T, size_t Dim, size_t Edges>
using VectorList = std::array<FastVector<T, Dim>, Edges>;

}  // namespace bvh

#endif
