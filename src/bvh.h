#pragma once
#ifndef BVH_TREE_BVH_TREE_H
#define BVH_TREE_BVH_TREE_H
#include <algorithm>
#include <atomic>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "diagram.h"
#include "fast_vector.h"
#include "ray_casting.h"
#include "status.h"
#include "utils.h"
namespace bvh {

template <typename T, size_t Dim, size_t Edges>
class BVHTreeBuilder;

template <typename T, size_t Dim, size_t Edges>
class BVHTree {};

template <typename T, size_t Edges>
class BVHTree<T, 2, Edges> {
  friend BVHTreeBuilder<T, 2, Edges>;

 public:
  enum Operation { kINSERT, kDELETE };
  BVHTree():id_(0) { static_assert(Edges != 0, "edges can't be 0"); }
  ~BVHTree() {}

  Status UpdateAndRebuild(const Diagram<T, Edges>& diagram, Operation op) {
    return Status::MakeNotSupport();
  }

  Status FindAllByContain(const std::array<T, 2>& point,
                          std::vector<std::string>* areas_name) {
    areas_name->clear();
    FastVector<T, 2> tmp(point);

    auto it = data_.begin();
    RayCasting<T, 2, Edges> ray;
    while (true) {
      it = std::find_if(it, data_.end(), [&](const Diagram<T, Edges>& d) {
        return d.Contain(tmp, ray);
      });
      if (it != data_.end()) {
        auto name_it = ivf_name_.find(it->GetId());
        if (name_it == ivf_name_.end()) {
          return Status::MakeError("status error");
        }
        areas_name->emplace_back(name_it->second);
      } else {
        break;
      }
      ++it;
    }

    if (areas_name->size() > 0) {
      return Status::MakeOK();
    } else {
      return Status::MakeNotFound("");
    }
  }

  Status FindOneByContain(const std::array<T, 2>& point,
                          std::string* area_name) {
    FastVector<T, 2> tmp(point);
    RayCasting<T, 2, Edges> ray;
    auto it = std::find_if(
        data_.begin(), data_.end(),
        [&](const Diagram<T, Edges>& d) { return d.Contain(tmp, ray); });
    if (it != data_.end()) {
      auto name_it = ivf_name_.find(it->GetId());
      if (name_it == ivf_name_.end()) {
        return Status::MakeError("status error");
      }
      *area_name = it->second;
      return Status::MakeOK();
    }
    return Status::MakeNotFound("");
  }

  Status FindNearest(const std::array<T, 2>& point, std::string* area_name) {
    return Status::MakeNotFound("");
  }

  Status FindNearest(const std::array<T, 2>& point, uint32_t top_k,
                     std::vector<std::string>* areas_name) {
    return Status::MakeNotFound("");
  }

  size_t GetElementNum() const { return data_.size(); }

 private:
  Status Insert(std::string name, const Diagram<T, Edges>& diagram) {
    data_.emplace(diagram);
    atomic_max<uint64_t>(id_, diagram.GetId());
    ivf_name_.emplace(diagram.GetId(), name);
    return Status::MakeOK();
  }

  Status Delete() { return Status::MakeNotSupport(); }

  Status Build() { return Status::MakeOK(); }

  std::atomic<uint64_t> id_;

  // todo change tmp
  std::set<Diagram<T, Edges>> data_;
  std::unordered_map<uint64_t, std::string> ivf_name_;
};

template <typename T, size_t Dim, size_t Edges>
class BVHTreeBuilder {
 public:
  BVHTreeBuilder() : id_(0), is_building_(false), is_builded_(false) {
    bvh_tree_ptr_ = std::make_shared<BVHTree<T, Dim, Edges>>();
  }
  Status Insert(std::string_view name,
                const std::vector<std::vector<T>>& diagram) {
    if (is_building_.load()) {
      return Status::MakeError("Is building");
    }
    if (diagram.empty()) {
      return Status::MakeOK();
    }
    if (diagram.size() != Edges) {
      return Status::MakeError("Edges is not " + std::to_string(Edges));
    }

    if (diagram[0].size() != Dim) {
      return Status::MakeError("Dim is not " + std::to_string(Dim));
    }
    auto id = id_++;
    VectorList<T, Dim, Edges> vl;
    uint32_t i = 0;
    for (auto&& v : diagram) {
      FastVector<T, Dim> edge;
      auto s = FastVector<T, Dim>::VectorToFastVector(v, &edge);
      if (s != Status::OK()) {
        return s;
      }
      vl[i++] = std::move(edge);
    }
    data_.emplace(name, Diagram{id, vl});
    return Status::MakeOK();
  }

  Status Delete() { return Status::MakeNotSupport(); }

  Status Build() {
    if (!is_building_.exchange(true)) {
      for (auto&& [name, d] : data_) {
        bvh_tree_ptr_->Insert(name, d);
      }
      bvh_tree_ptr_->Build();
      is_builded_.store(true);
      return Status::MakeOK();
    }
    return Status::MakeError("Is builded");
  }

  std::shared_ptr<BVHTree<T, Dim, Edges>> GetBVHTree() {
    if (is_builded_.load()) {
      return bvh_tree_ptr_;
    } else {
      return nullptr;
    }
  }

 private:
  std::atomic<uint64_t> id_;
  std::atomic_bool is_building_;
  std::atomic_bool is_builded_;
  std::shared_ptr<BVHTree<T, Dim, Edges>> bvh_tree_ptr_;
  // todo change tmp
  std::map<std::string, Diagram<T, Edges>> data_;
};
}  // namespace bvh

#endif
