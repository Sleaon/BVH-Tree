#pragma once
#ifndef BVH_TREE_VECTOR_COMPUTE_H
#define BVH_TREE_VECTOR_COMPUTE_H
#include "array"
#include "utils.h"

#ifdef USE_AVX
#pragma message "Use AVX"
#include <bitset>

#include "immintrin.h"
#endif
namespace bvh {

template <typename T>
constexpr inline size_t ComputeRealDim(size_t Dim) {
#ifdef USE_AVX
  auto value = sizeof(T) * Dim * 8;
  if (value < 128) {
    return 128 / 8 / sizeof(T);
  }

  if (value >= 512) {
    return ((value + 511) / 512) * 512 / 8 / sizeof(T);
  }
  value--;
  value |= value >> 1;
  value |= value >> 2;
  value |= value >> 4;
  value |= value >> 8;
  value |= value >> 16;
  value |= value >> 32;
  return (value + 1) / 8 / sizeof(T);
#else
  return Dim;
#endif
}

template <typename T, size_t Dim, typename F>
inline std::array<T, Dim> Compute(F&& f) {
  std::array<T, Dim> v;
  StaticFor<0, Dim>([&](size_t i) { v[i] = f(i); });
  return v;
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorAdd(const std::array<T, Dim>& a,
                                    const std::array<T, Dim>& b) {
#ifdef USE_AVX
  std::array<T, Dim> result;
  if constexpr (std::is_same_v<T, float>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128 avx_a = _mm_loadu_ps(&a[index]);
      __m128 avx_b = _mm_loadu_ps(&b[index]);
      __m128 avx_result = _mm_add_ps(avx_a, avx_b);
      _mm_storeu_ps(&result[index], avx_result);
    });

  } else if constexpr (std::is_same_v<T, double>) {
    static_assert(Dim % 2 == 0, "Dim should be a multiple of 2");
    constexpr size_t num = Dim / 2;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 2;
      __m128d avx_a = _mm_loadu_pd(&a[index]);
      __m128d avx_b = _mm_loadu_pd(&b[index]);
      __m128d avx_result = _mm_add_pd(avx_a, avx_b);
      _mm_storeu_pd(&result[index], avx_result);
    });
  } else if constexpr (std::is_same_v<T, int32_t>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128i avx_a =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&a[index]));
      __m128i avx_b =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&b[index]));
      __m128i avx_result = _mm_add_epi32(avx_a, avx_b);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(&result[index]), avx_result);
    });
  } else {
    throw std::runtime_error("Unsupported data type");
  }
  return result;
#else
  return Compute<T, Dim>([&](size_t i) { return a[i] + b[i]; });
#endif
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorAdd(const std::array<T, Dim>& a, T b) {
#ifdef USE_AVX
  std::array<T, Dim> result;
  if constexpr (std::is_same_v<T, float>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    __m128 avx_b = _mm_set1_ps(b);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128 avx_a = _mm_loadu_ps(&a[index]);
      __m128 avx_result = _mm_add_ps(avx_a, avx_b);
      _mm_storeu_ps(&result[index], avx_result);
    });

  } else if constexpr (std::is_same_v<T, double>) {
    static_assert(Dim % 2 == 0, "Dim should be a multiple of 2");
    constexpr size_t num = Dim / 2;
    __m128d avx_b = _mm_set1_pd(b);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 2;
      __m128d avx_a = _mm_loadu_pd(&a[index]);
      __m128d avx_result = _mm_add_pd(avx_a, avx_b);
      _mm_storeu_pd(&result[index], avx_result);
    });
  } else if constexpr (std::is_same_v<T, int32_t>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    __m128i avx_b = _mm_set1_epi32(b);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128i avx_a =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&a[index]));
      __m128i avx_result = _mm_add_epi32(avx_a, avx_b);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(&result[index]), avx_result);
    });
  } else {
    throw std::runtime_error("Unsupported data type");
  }
  return result;
#else
  return Compute<T, Dim>([&](size_t i) { return a[i] + b[i]; });
#endif
}
template <typename T, size_t Dim>
inline std::array<T, Dim> VectorSub(const std::array<T, Dim>& a, const T b) {
#ifdef USE_AVX
  std::array<T, Dim> result;
  if constexpr (std::is_same_v<T, float>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    __m128 avx_b = _mm_set1_ps(b);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128 avx_a = _mm_loadu_ps(&a[index]);
      __m128 avx_result = _mm_sub_ps(avx_a, avx_b);
      _mm_storeu_ps(&result[index], avx_result);
    });

  } else if constexpr (std::is_same_v<T, double>) {
    static_assert(Dim % 2 == 0, "Dim should be a multiple of 2");
    constexpr size_t num = Dim / 2;
    __m128d avx_b = _mm_set1_pd(b);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 2;
      __m128d avx_a = _mm_loadu_pd(&a[index]);
      __m128d avx_result = _mm_sub_pd(avx_a, avx_b);
      _mm_storeu_pd(&result[index], avx_result);
    });
  } else if constexpr (std::is_same_v<T, int32_t>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    __m128i avx_b = _mm_set1_epi32(b);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128i avx_a =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&a[index]));
      __m128i avx_result = _mm_sub_epi32(avx_a, avx_b);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(&result[index]), avx_result);
    });
  } else {
    throw std::runtime_error("Unsupported data type");
  }
  return result;
#else
  return Compute<T, Dim>([&](size_t i) { return a[i] - b; });
#endif
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorSub(const std::array<T, Dim>& a,
                                    const std::array<T, Dim>& b) {
#ifdef USE_AVX
  std::array<T, Dim> result;
  if constexpr (std::is_same_v<T, float>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128 avx_a = _mm_loadu_ps(&a[index]);
      __m128 avx_b = _mm_loadu_ps(&b[index]);
      __m128 avx_result = _mm_sub_ps(avx_a, avx_b);
      _mm_storeu_ps(&result[index], avx_result);
    });

  } else if constexpr (std::is_same_v<T, double>) {
    static_assert(Dim % 2 == 0, "Dim should be a multiple of 2");
    constexpr size_t num = Dim / 2;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 2;
      __m128d avx_a = _mm_loadu_pd(&a[index]);
      __m128d avx_b = _mm_loadu_pd(&b[index]);
      __m128d avx_result = _mm_sub_pd(avx_a, avx_b);
      _mm_storeu_pd(&result[index], avx_result);
    });
  } else if constexpr (std::is_same_v<T, int32_t>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128i avx_a =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&a[index]));
      __m128i avx_b =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&b[index]));
      __m128i avx_result = _mm_sub_epi32(avx_a, avx_b);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(&result[index]), avx_result);
    });
  } else {
    throw std::runtime_error("Unsupported data type");
  }
  return result;
#else
  return Compute<T, Dim>([&](size_t i) { return a[i] - b[i]; });
#endif
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorSub(const std::array<T, Dim>& a) {
#ifdef USE_AVX
  std::array<T, Dim> result;
  if constexpr (std::is_same_v<T, float>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    const __m128 sign_mask = _mm_set1_ps(-0.0f);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128 avx_a = _mm_loadu_ps(&a[index]);
      __m128 avx_result = _mm_xor_ps(avx_a, sign_mask);
      _mm_storeu_ps(&result[index], avx_result);
    });

  } else if constexpr (std::is_same_v<T, double>) {
    static_assert(Dim % 2 == 0, "Dim should be a multiple of 2");
    constexpr size_t num = Dim / 2;
    const __m128d sign_mask = _mm_set1_pd(-0.0);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 2;
      __m128d avx_a = _mm_loadu_pd(&a[index]);
      __m128d avx_result = _mm_xor_pd(avx_a, sign_mask);
      _mm_storeu_pd(&result[index], avx_result);
    });
  } else if constexpr (std::is_same_v<T, int32_t>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    const __m128i sign_mask = _mm_set1_epi32(0x80000000);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128i avx_a =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&a[index]));
      __m128i avx_result = _mm_xor_si128(avx_a, sign_mask);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(&result[index]), avx_result);
    });
  } else {
    throw std::runtime_error("Unsupported data type");
  }
  return result;
#else
  return Compute<T, Dim>([&](size_t i) { return -a[i]; });
#endif
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorMul(const std::array<T, Dim>& a,
                                    const std::array<T, Dim>& b) {
#ifdef USE_AVX
  std::array<T, Dim> result;
  if constexpr (std::is_same_v<T, float>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128 avx_a = _mm_loadu_ps(&a[index]);
      __m128 avx_b = _mm_loadu_ps(&b[index]);
      __m128 avx_result = _mm_mul_ps(avx_a, avx_b);
      _mm_storeu_ps(&result[index], avx_result);
    });

  } else if constexpr (std::is_same_v<T, double>) {
    static_assert(Dim % 2 == 0, "Dim should be a multiple of 2");
    constexpr size_t num = Dim / 2;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 2;
      __m128d avx_a = _mm_loadu_pd(&a[index]);
      __m128d avx_b = _mm_loadu_pd(&b[index]);
      __m128d avx_result = _mm_mul_pd(avx_a, avx_b);
      _mm_storeu_pd(&result[index], avx_result);
    });
  } else if constexpr (std::is_same_v<T, int32_t>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128i avx_a =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&a[index]));
      __m128i avx_b =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&b[index]));
      __m128i avx_result = _mm_mul_epi32(avx_a, avx_b);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(&result[index]), avx_result);
    });
  } else {
    throw std::runtime_error("Unsupported data type");
  }
  return result;
#else
  return Compute<T, Dim>([&](size_t i) { return a[i] * b[i]; });
#endif
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorMul(const std::array<T, Dim>& a, const T b) {
#ifdef USE_AVX
  std::array<T, Dim> result;
  if constexpr (std::is_same_v<T, float>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    __m128 avx_b = _mm_set1_ps(b);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128 avx_a = _mm_loadu_ps(&a[index]);
      __m128 avx_result = _mm_mul_ps(avx_a, avx_b);
      _mm_storeu_ps(&result[index], avx_result);
    });

  } else if constexpr (std::is_same_v<T, double>) {
    static_assert(Dim % 2 == 0, "Dim should be a multiple of 2");
    constexpr size_t num = Dim / 2;
    __m128d avx_b = _mm_set1_pd(b);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 2;
      __m128d avx_a = _mm_loadu_pd(&a[index]);
      __m128d avx_result = _mm_mul_pd(avx_a, avx_b);
      _mm_storeu_pd(&result[index], avx_result);
    });
  } else if constexpr (std::is_same_v<T, int32_t>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    __m128i avx_b = _mm_set1_epi32(b);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128i avx_a =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&a[index]));
      __m128i avx_result = _mm_mul_epi32(avx_a, avx_b);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(&result[index]), avx_result);
    });
  } else {
    throw std::runtime_error("Unsupported data type");
  }
  return result;
#else
  return Compute<T, Dim>([&](size_t i) { return a[i] * b; });
#endif
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorDiv(const std::array<T, Dim>& a,
                                    const std::array<T, Dim>& b) {
#ifdef USE_AVX
  std::array<T, Dim> result;
  if constexpr (std::is_same_v<T, float>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128 avx_a = _mm_loadu_ps(&a[index]);
      __m128 avx_b = _mm_loadu_ps(&b[index]);
      __m128 avx_result = _mm_div_ps(avx_a, avx_b);
      _mm_storeu_ps(&result[index], avx_result);
    });

  } else if constexpr (std::is_same_v<T, double>) {
    static_assert(Dim % 2 == 0, "Dim should be a multiple of 2");
    constexpr size_t num = Dim / 2;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 2;
      __m128d avx_a = _mm_loadu_pd(&a[index]);
      __m128d avx_b = _mm_loadu_pd(&b[index]);
      __m128d avx_result = _mm_div_pd(avx_a, avx_b);
      _mm_storeu_pd(&result[index], avx_result);
    });
  } else if constexpr (std::is_same_v<T, int32_t>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128i avxi_a =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&a[index]));
      __m128i avxi_b =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&b[index]));
      __m128 avxf_a = _mm_cvtepi32_ps(avxi_a);
      __m128 avxf_b = _mm_cvtepi32_ps(avxi_b);
      __m128 avxf_result = _mm_div_ps(avxf_a, avxf_b);
      __m128i avxi_result = _mm_cvtps_epi32(avxf_result);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(&result[index]), avxi_result);
    });
  } else {
    throw std::runtime_error("Unsupported data type");
  }
  return result;
#else
  if constexpr (std::is_floating_point_v<T>) {
    return Compute<T, Dim>([&](size_t i) { return a[i] * SafeInverse(b[i]); });
  } else {
    return Compute<T, Dim>([&](size_t i) {
      return b[i] != 0 ? a[i] / b[i] : std::numeric_limits<T>::max();
    });
  }
#endif
}

template <typename T, size_t Dim>
inline std::array<T, Dim> VectorDiv(const std::array<T, Dim>& a, const T b) {
#ifdef USE_AVX
  std::array<T, Dim> result;
  if (std::fabs(b) <= std::numeric_limits<T>::epsilon()) {
    result.fill(std::numeric_limits<T>::max());
    return result;
  }
  if constexpr (std::is_same_v<T, float>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    __m128 avx_b = _mm_set1_ps(b);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128 avx_a = _mm_loadu_ps(&a[index]);
      __m128 avx_result = _mm_div_ps(avx_a, avx_b);
      _mm_storeu_ps(&result[index], avx_result);
    });

  } else if constexpr (std::is_same_v<T, double>) {
    static_assert(Dim % 2 == 0, "Dim should be a multiple of 2");
    constexpr size_t num = Dim / 2;
    __m128d avx_b = _mm_set1_pd(b);
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 2;
      __m128d avx_a = _mm_loadu_pd(&a[index]);
      __m128d avx_result = _mm_div_pd(avx_a, avx_b);
      _mm_storeu_pd(&result[index], avx_result);
    });
  } else if constexpr (std::is_same_v<T, int32_t>) {
    static_assert(Dim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = Dim / 4;
    __m128 avxf_b = _mm_cvtepi32_ps(_mm_set1_epi32(b));
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128i avxi_a =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&a[index]));
      __m128 avxf_a = _mm_cvtepi32_ps(avxi_a);
      __m128 avxf_result = _mm_div_ps(avxf_a, avxf_b);
      __m128i avxi_result = _mm_cvtps_epi32(avxf_result);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(&result[index]), avxi_result);
    });
  } else {
    throw std::runtime_error("Unsupported data type");
  }
  return result;
#else
  if constexpr (std::is_floating_point_v<T>) {
    T div = SafeInverse(b);
    return Compute<T, Dim>([&](size_t i) { return a[i] * div; });
  } else {
    return Compute<T, Dim>([&](size_t i) {
      return b != 0 ? a[i] / b : std::numeric_limits<T>::max();
    });
  }
#endif
}

template <typename T, size_t Dim, size_t RealDim>
inline bool VectorEQ(const std::array<T, RealDim>& a,
                     const std::array<T, RealDim>& b) {
#ifdef USE_AVX
  std::bitset<RealDim> result;
  if constexpr (std::is_same_v<T, float>) {
    static_assert(RealDim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = RealDim / 4;
    const __m128 upper_epsilon =
        _mm_set1_ps(std::numeric_limits<float>::epsilon() * 10);
    const __m128 lower_epsilon =
        _mm_set1_ps(-(std::numeric_limits<float>::epsilon() * 10));
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128 avx_a = _mm_loadu_ps(&a[index]);
      __m128 avx_b = _mm_loadu_ps(&b[index]);
      __m128 diff = _mm_sub_ps(avx_a, avx_b);
      __m128 cmp_upper = _mm_cmp_ps(diff, upper_epsilon, _CMP_LE_OS);
      __m128 cmp_lower = _mm_cmp_ps(diff, lower_epsilon, _CMP_GE_OS);
      __m128 avx_result = _mm_and_ps(cmp_upper, cmp_lower);
      int mask = _mm_movemask_ps(avx_result);
      StaticFor<0, 4>([&](size_t j) { result[i * 4 + j] = (mask >> j) & 1; });
    });

  } else if constexpr (std::is_same_v<T, double>) {
    static_assert(RealDim % 2 == 0, "Dim should be a multiple of 2");
    constexpr size_t num = RealDim / 2;
    const __m128d upper_epsilon =
        _mm_set1_pd(std::numeric_limits<double>::epsilon() * 10);
    const __m128d lower_epsilon =
        _mm_set1_pd(-(std::numeric_limits<double>::epsilon() * 10));
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 2;
      __m128d avx_a = _mm_loadu_pd(&a[index]);
      __m128d avx_b = _mm_loadu_pd(&b[index]);
      __m128d diff = _mm_sub_pd(avx_a, avx_b);
      __m128d cmp_upper = _mm_cmp_pd(diff, upper_epsilon, _CMP_LE_OS);
      __m128d cmp_lower = _mm_cmp_pd(diff, lower_epsilon, _CMP_GE_OS);
      __m128d avx_result = _mm_and_pd(cmp_upper, cmp_lower);
      int mask = _mm_movemask_pd(avx_result);
      StaticFor<0, 2>([&](size_t j) { result[i * 2 + j] = (mask >> j) & 1; });
    });
  } else if constexpr (std::is_same_v<T, int32_t>) {
    static_assert(RealDim % 4 == 0, "Dim should be a multiple of 4");
    constexpr size_t num = RealDim / 4;
    StaticFor<0, num>([&](size_t i) {
      size_t index = i * 4;
      __m128i avx_a =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&a[index]));
      __m128i avx_b =
          _mm_loadu_si128(reinterpret_cast<const __m128i*>(&b[index]));
      __m128i avx_result = _mm_cmpeq_epi32(avx_a, avx_b);
      int mask = _mm_movemask_epi8(avx_result);
      StaticFor<0, 4>([&](size_t j) { result[i * 4 + j] = (mask >> j) & 1; });
    });
  } else {
    throw std::runtime_error("Unsupported data type");
  }
  constexpr size_t dim_mask = ~((1 << Dim) - 1);
  result |= dim_mask;
  return result.all();
#else
  if constexpr (std::is_floating_point_v<T>) {
    for (size_t i = 0; i < Dim; ++i) {
      if (NumberNE(a[i], b[i])) {
        return false;
      }
    }
    return true;
  } else {
    return a == b;
  }
#endif
}

template <typename T, size_t Dim, size_t RealDim>
inline bool VectorNE(const std::array<T, RealDim>& a,
                     const std::array<T, RealDim>& b) {
#ifdef USE_AVX
  return !VectorEQ<T, Dim, RealDim>(a, b);
#else
  if constexpr (std::is_floating_point_v<T>) {
    for (size_t i = 0; i < Dim; ++i) {
      if (NumberNE(a[i], b[i])) {
        return true;
      }
    }
    return false;
  } else {
    return a != b;
  }
#endif
}

}  // namespace bvh

#endif