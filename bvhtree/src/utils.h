#pragma once
#ifndef BVH_TREE_UTILS_H
#define BVH_TREE_UTILS_H

#include <stdio.h>

#include <atomic>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "status.h"

namespace bvh {

template <typename T,
          std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
inline bool NumberEQ(T a, T b) {
  constexpr T epslion = std::numeric_limits<T>::epsilon() * 10;
  return std::abs(a - b) <= epslion;
}
template <typename T,
          std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
inline bool NumberNE(T a, T b) {
  constexpr T epsilon = std::numeric_limits<T>::epsilon() * 10;
  return std::abs(a - b) > epsilon;
}

template <typename T,
          std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
T SafeInverse(T x) {
  return std::fabs(x) <= std::numeric_limits<T>::epsilon()
             ? std::copysign(std::numeric_limits<T>::max(), x)
             : static_cast<T>(1.) / x;
}
template <size_t Begin, size_t End, typename F>
void StaticFor(F&& f) {
  if constexpr (Begin < End) {
    f(Begin);
    StaticFor<Begin + 1, End>(std::forward<F>(f));
  }
}

/// Computes the maximum between an atomic variable and a value, and returns the
/// value previously held by the atomic variable.
template <typename T>
T atomic_max(std::atomic<T>& atomic, const T& value) {
  auto prev_value = atomic.load();
  while (prev_value < value && !atomic.compare_exchange_weak(prev_value, value))
    ;
  return prev_value;
}

template <typename T>
T Str2Num(std::string_view str) {
  T result;
  std::istringstream stream(str.data());
  stream >> result;
  return result;
}

inline std::vector<std::string_view> Split(std::string_view input,
                                           char delimiter) {
  std::vector<std::string_view> result;
  size_t start = 0;

  while (start < input.size()) {
    size_t end = input.find(delimiter, start);
    if (end == std::string_view::npos) {
      end = input.size();
    }
    result.push_back(input.substr(start, end - start));
    start = end + 1;
  }
  return result;
}

/// @brief load diagram data from file, format must be "[name] [peak];[peak]",
/// peak format is "x,y,z"
/// @tparam T number type
/// @param file_name The filename containing the path
/// @param data return data
/// @return execution state
template <typename T>
Status LoadDataFromFile(
    std::string_view file_name,
    std::map<std::string, std::vector<std::vector<T>>>* data) {
  FILE* file = fopen(file_name.data(), "r");  // 打开文件

  if (file == nullptr) {
    return Status::MakeError(std::string(file_name) + " file not found");
  }
  data->clear();
  char line[512];
  while (fgets(line, sizeof(line), file) != nullptr) {
    auto d = Split(line, ' ');
    if (d.size() != 2) {
      return Status::MakeError("format wrong");
    }
    auto d_name = d[0];
    auto peaks_str = Split(d[1], ';');
    try {
      std::vector<std::vector<T>> peaks;
      for (auto&& peak_str : peaks_str) {
        auto peak_value_str = Split(peak_str, ',');
        std::vector<T> peak;
        for (auto&& value_str : peak_value_str) {
          peak.emplace_back(Str2Num<T>(value_str));
        }
        peaks.emplace_back(std::move(peak));
      }
      data->emplace(d_name, std::move(peaks));
    } catch (std::exception& e) {
      return Status::MakeError(std::string("peaks load failed, cause: ") +
                               e.what());
    }
  }

  fclose(file);  // 关闭文件
  return Status::MakeOK();
}

}  // namespace bvh

#endif
