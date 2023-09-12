#pragma once
#ifndef BVH_TREE_BVH_H
#define BVH_TREE_BVH_H
#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "bvh_node.h"
#include "circularity.h"
#include "fast_vector.h"
#include "polygon.h"
#include "shape.h"
#include "status.h"
#include "utils.h"
namespace bvh {
template <typename T, size_t Dim>
class BVHTreeBuilder;

template <typename T, size_t Dim,
          typename = std::enable_if_t<std::is_arithmetic_v<T>>>
class BVHTree {};

template <typename T>
class BVHTree<T, 2> {
  friend BVHTreeBuilder<T, 2>;

 public:
  BVHTree() : id_(0) { root_ = new BVHNode<T, 2>(); }
  ~BVHTree() { delete root_; }
  BVHTree(const BVHTree&) = delete;
  BVHTree& operator=(const BVHTree&) = delete;

  Status InsertAndRebuild(std::string_view name,
                          const std::vector<std::vector<T>>& polygon) {
    return Status::MakeNotSupport();
  }

  Status DeleteAndRebuild(std::string_view name) {
    return Status::MakeNotSupport();
  }

  Status FindAllByContain(const std::array<T, 2>& point,
                          std::vector<std::string>* areas_name) {
    areas_name->clear();
    FastVector<T, 2> tmp(point);
    std::vector<uint64_t> ids;
    auto s = root_->FindAllByContain(tmp, &ids);
    if (s != Status::OK()) {
      return s;
    }
    for (auto i : ids) {
      auto name_it = ivf_name_.find(i);
      if (name_it == ivf_name_.end()) {
        return Status::MakeError("status error");
      }
      areas_name->emplace_back(name_it->second);
    }
    return Status::MakeOK();
  }

  Status FindOneByContain(const std::array<T, 2>& point,
                          std::string* area_name) {
    FastVector<T, 2> tmp(point);
    uint64_t id;
    auto s = root_->FindOneByContain(tmp, &id);
    if (s != Status::OK()) {
      return s;
    }
    auto name_it = ivf_name_.find(id);
    if (name_it == ivf_name_.end()) {
      return Status::MakeError("status error");
    }
    *area_name = name_it->second;
    return Status::MakeOK();
  }

  Status FindNearest(const std::array<T, 2>& point, T max_dist,
                     std::string* area_name) {
    FastVector<T, 2> tmp(point);
    ShapeInfo<T, 2> info;
    auto s = root_->FindNearest(tmp, max_dist * max_dist, &info);
    if (s != Status::OK()) {
      return s;
    }
    auto name_it = ivf_name_.find(info.id);
    if (name_it == ivf_name_.end()) {
      return Status::MakeError("status error");
    }
    *area_name = name_it->second;
    return Status::MakeOK();
  }

  Status FindNearest(const std::array<T, 2>& point, uint32_t top_k, T max_dist,
                     std::vector<std::string>* areas_name) {
    FastVector<T, 2> tmp(point);
    std::set<ShapeInfo<T, 2>> info_set;
    auto s = root_->FindNearest(tmp, top_k, max_dist * max_dist, &info_set);
    if (s != Status::OK()) {
      return s;
    }
    for (auto&& info : info_set) {
      auto name_it = ivf_name_.find(info.id);
      if (name_it == ivf_name_.end()) {
        return Status::MakeError("status error");
      }
      areas_name->emplace_back(name_it->second);
    }
    return Status::MakeOK();
  }

  size_t GetElementNum() const { return root_->GetElementNum(); }

  size_t GetHight() const { return root_->GetHight(); }

 private:
  Status Insert(std::string_view name, VectorList<T, 2>& vl) {
    auto shape_ptr = new Polygon<T>(id_++, std::move(vl));
    ivf_name_.emplace(shape_ptr->GetId(), name.data());
    return root_->Insert(shape_ptr);
  }

  Status Insert(std::string_view name, const FastVector<T, 2>& centre,
                T radius) {
    auto shape_ptr = new Circularity<T>(id_++, radius, centre);
    ivf_name_.emplace(shape_ptr->GetId(), name.data());
    return root_->Insert(shape_ptr);
  }
  Status Delete() { return Status::MakeNotSupport(); }

  Status Build(size_t max_capacity) { return root_->Build(max_capacity); }

 private:
  std::atomic<uint64_t> id_;
  BVHNode<T, 2>* root_;
  std::unordered_map<uint64_t, std::string> ivf_name_;
};

template <typename T, size_t Dim>
class BVHTreeBuilder {
 public:
  BVHTreeBuilder() {
    is_builded_ = false;
    is_building_ = false;
    bvh_tree_ptr_ = std::make_shared<BVHTree<T, Dim>>();
  }

  ~BVHTreeBuilder() {}
  Status Insert(std::string_view name,
                const std::vector<std::vector<T>>& polygon) {
    if (is_building_.load()) {
      return Status::MakeError("Is building");
    }
    if (polygon.empty()) {
      return Status::MakeOK();
    }

    if (polygon[0].size() != Dim) {
      return Status::MakeError("Dim is not " + std::to_string(Dim));
    }
    VectorList<T, Dim> vl(polygon.size());
    uint32_t i = 0;
    for (auto&& v : polygon) {
      FastVector<T, Dim> edge;
      auto s = FastVector<T, Dim>::VectorToFastVector(v, &edge);
      if (s != Status::OK()) {
        return s;
      }
      vl[i++] = std::move(edge);
    }
    return bvh_tree_ptr_->Insert(name, vl);
  }
  Status Insert(std::string_view name, const std::array<T, Dim>& centre,
                T radius) {
    return bvh_tree_ptr_->Insert(name, FastVector<T, Dim>(centre), radius);
  }

  Status Delete() { return Status::MakeNotSupport(); }

  Status Build(size_t max_capacity) {
    if (!is_building_.exchange(true)) {
      bvh_tree_ptr_->Build(max_capacity);
      is_builded_.store(true);
      return Status::MakeOK();
    }
    return Status::MakeError("Is builded");
  }

  std::shared_ptr<BVHTree<T, Dim>> GetBVHTree() {
    if (is_builded_.load()) {
      return bvh_tree_ptr_;
    } else {
      return nullptr;
    }
  }

 private:
  std::atomic_bool is_building_;
  std::atomic_bool is_builded_;
  std::shared_ptr<BVHTree<T, Dim>> bvh_tree_ptr_;
};
}  // namespace bvh

#endif
