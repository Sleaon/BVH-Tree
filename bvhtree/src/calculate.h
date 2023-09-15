#pragma once
#ifndef BVH_TREE_CALCULATE_H
#define BVH_TREE_CALCULATE_H
#define USE_AVX

#include <cstdint>

#include "fast_vector.h"
#include "utils.h"
namespace bvh {

template <typename T, size_t DisplayDim, size_t RealDim>
inline FastVector<T, DisplayDim> ComputeCentrePoint(
    VectorList<T, DisplayDim> list) {
#ifdef USE_AVX
  std::array<T, RealDim> result;
  if constexpr (std::is_same_v<T, float>) {
    static_assert(RealDim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = RealDim / 4;
    __m128 avx_dim = _mm_set1_ps(static_cast<float>(DisplayDim));
    StaticFor<0, num>([&](size_t i) {
      __m128 avx_sum = _mm_set1_ps(0.0);
      size_t index = i * 4;
      for (auto&& v : list) {
        __m128 avx_v = _mm_loadu_ps(&(v.values_[index]));
        avx_sum = _mm_add_ps(avx_v, avx_sum);
      }
      avx_sum = _mm_div_ps(avx_sum, avx_dim);
      _mm_storeu_ps(&result[index], avx_sum);
    });

  } else if constexpr (std::is_same_v<T, double>) {
    static_assert(RealDim % 2 == 0, "Dim should be a multiple of 2");
    constexpr size_t num = RealDim / 2;
    __m128d avx_dim = _mm_set1_pd(static_cast<double>(DisplayDim));
    StaticFor<0, num>([&](size_t i) {
      __m128d avx_sum = _mm_set1_pd(0.0);
      size_t index = i * 2;
      for (auto&& v : list) {
        __m128d avx_v = _mm_loadu_pd(&(v.values_[index]));
        avx_sum = _mm_add_pd(avx_v, avx_sum);
      }
      avx_sum = _mm_div_pd(avx_sum, avx_dim);
      _mm_storeu_pd(&result[index], avx_sum);
    });
  } else if constexpr (std::is_same_v<T, int32_t>) {
    static_assert(RealDim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = RealDim / 4;
    __m128 avx_dim = _mm_set1_ps(static_cast<float>(DisplayDim));
    StaticFor<0, num>([&](size_t i) {
      __m128i avx_sum = _mm_set1_epi32(0);
      size_t index = i * 4;
      for (auto&& v : list) {
        __m128i avx_v = _mm_loadu_si128(
            reinterpret_cast<const __m128i*>(&(v.values_[index])));
        avx_sum = _mm_add_epi32(avx_v, avx_sum);
      }
      avx_sum = _mm_cvtps_epi32(_mm_div_ps(_mm_cvtepi32_ps(avx_sum), avx_dim));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(&result[index]), avx_sum);
    });
  } else {
    throw std::runtime_error("Unsupported data type");
  }

  return FastVector<T, DisplayDim>(RealDim, result);

#else
  FastVector<T, Dim> r;
  for (auto& vector : list) {
    r = r + vector;
  }
  return r / static_cast<T>(list.size());
#endif
}

template <typename T, size_t Dim>
inline T Sum(FastVector<T, Dim> v) {
#ifdef USE_AVX
  std::array<T, 4> result;
  if constexpr (std::is_same_v<T, float>) {
    constexpr size_t num = v.GetRealDim() / 4;
    __m128 avx_sum = _mm_set1_ps(0.0);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128 avx_v = _mm_loadu_ps(&(v.values_[index]));
      avx_v = _mm_hadd_ps(avx_v, avx_v);
      avx_v = _mm_hadd_ps(avx_v, avx_v);
      avx_sum = _mm_add_ps(avx_sum, avx_v);
    });
    _mm_storeu_ps(&result[0], avx_sum);
  } else if constexpr (std::is_same_v<T, double>) {
    constexpr size_t num = v.GetRealDim() / 2;
    __m128d avx_sum = _mm_set1_pd(0.0);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128d avx_v = _mm_loadu_pd(&(v.values_[index]));
      avx_v = _mm_hadd_pd(avx_v, avx_v);
      avx_v = _mm_hadd_pd(avx_v, avx_v);
      avx_sum = _mm_add_pd(avx_sum, avx_v);
    });
    _mm_storeu_pd(&result[0], avx_sum);
  } else if constexpr (std::is_same_v<T, int32_t>) {
    constexpr size_t num = v.GetRealDim() / 4;
    __m128i avx_sum = _mm_set1_epi32(0);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128i avx_v =
          _mm_loadu_si128(reinterpret_cast<__m128i*>(&(v.values_[index])));
      avx_v = _mm_hadd_epi32(avx_v, avx_v);
      avx_v = _mm_hadd_epi32(avx_v, avx_v);
      avx_sum = _mm_add_epi32(avx_sum, avx_v);
    });
    _mm_storeu_si128(reinterpret_cast<__m128i*>(&result[0]), avx_sum);
  } else {
    throw std::runtime_error("Unsupported data type");
  }
  return result[0];
#else
  T r = 0;
  StaticFor<0, Dim>([&](size_t i) { r += v[i]; });
  return r;
#endif
}

template <typename T, size_t Dim>
inline T ComputeDistance(const FastVector<T, Dim>& v1,
                         const FastVector<T, Dim>& v2) {
  auto diff = v1 - v2;
  diff = diff * diff;
  return Sum(diff);
}

template <typename T, size_t Dim>
inline T ComputeDistance(const FastVector<T, Dim>& point,
                         const FastVector<T, Dim>& begin,
                         const FastVector<T, Dim>& end) {
  auto line_vec = end - begin;                       // AB
  auto point_vec = point - begin;                    // AP
  auto c = Sum(line_vec * point_vec);                //|AB*AP|
  if (c <= 0) return ComputeDistance(point, begin);  //|AP|

  auto d = ComputeDistance(begin, end);            //|AB|^2
  if (c >= d) return ComputeDistance(point, end);  //|BP|

  auto r = c / d;
  auto p_shadow = begin + line_vec * r;

  return ComputeDistance(point, p_shadow);  //|CP|
}

template <typename T, size_t Dim>
inline void ComputeBoxExtend(std::array<T*, 2 * Dim>&& source,
                             std::array<const T*, 2 * Dim>&& extend) {
  static_assert(Dim == 2 || Dim == 3, "Dim must be 2 or 3");
  if constexpr (Dim == 2) {
    *source[0] = *source[0] > *extend[0] ? *source[0] : *extend[0];
    *source[1] = *source[1] > *extend[1] ? *source[1] : *extend[1];
    *source[2] = *source[2] > *extend[2] ? *extend[2] : *source[2];
    *source[3] = *source[3] > *extend[3] ? *extend[3] : *source[3];
  }
}

template <typename T, size_t Dim>
inline std::tuple<T, T, T, T> ComputeBoxRange(
    const VectorList<T, Dim>& source) {
  static_assert(Dim == 2 || Dim == 3, "Dim must be 2 or 3");
  if constexpr (Dim == 2) {
    T upper = -std::numeric_limits<T>::max();
    T lower = std::numeric_limits<T>::max();
    T left = std::numeric_limits<T>::max();
    T right = -std::numeric_limits<T>::max();
    for (auto&& v : source) {
      if (v[0] > right) {
        right = v[0];
      } else if (v[0] < left) {
        left = v[0];
      }
      if (v[1] > upper) {
        upper = v[1];
      } else if (v[1] < lower) {
        lower = v[1];
      }
    }
    return std::make_tuple(upper, lower, left, right);
  }
}

template <typename T, size_t Dim>
inline bool ComputeBoxContain(const std::array<T, 2 * Dim>&& source,
                              const FastVector<T, Dim>& point) {
  static_assert(Dim == 2 || Dim == 3, "Dim must be 2 or 3");
  if constexpr (Dim == 2) {
    if (point[0] <= source[1] && point[0] >= source[2] &&
        point[1] <= source[0] && point[1] >= source[3]) {
      return true;
    }
  }
  return false;
}

}  // namespace bvh
#endif