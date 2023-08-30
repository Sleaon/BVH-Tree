#pragma once
#ifndef BVH_TREE_BVH_TREE_H
#define BVH_TREE_BVH_TREE_H
#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "status.h"
#include "utils.h"
#include "fast_vector.h"
#include "diagram.h"
#include "ray_casting.h"
#include "closest_point.h"

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
      it = std::find_if(it, data_.end(), [&](auto& d_pair) {
        auto&[_,d] = d_pair;
        return d.Contain(tmp, ray);
      });
      if (it != data_.end()) {
        auto name_it = ivf_name_.find(it->second.GetId());
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
        [&](auto& d_pair) {
        auto&[_,d] = d_pair;
           return d.Contain(tmp, ray); });
    if (it != data_.end()) {
      auto name_it = ivf_name_.find(it->second.GetId());
      if (name_it == ivf_name_.end()) {
        return Status::MakeError("status error");
      }
      *area_name = name_it->second;
      return Status::MakeOK();
    }
    return Status::MakeNotFound("");
  }

  Status FindNearest(const std::array<T, 2>& point, T max_dist, std::string* area_name) {
    FastVector<T, 2> tmp(point);
    ClosestPoint<T,2,Edges> alg;
    uint64_t min_id=-1;
    T min_dist = max_dist;
    for(auto&& [id,d]:data_){
      T dist = d.Distance(tmp,alg);
      if(dist < min_dist){
        min_id = id;
        min_dist = dist;
        continue;
      }
      if(dist == min_dist){
        T dist_c = ComputeDistance(tmp,d.GetCentre());
        T min_dist_c = ComputeDistance(tmp,data_[min_id].GetCentre());
        if(dist_c < min_dist_c){
        min_id = id;
        min_dist = dist;
        }
      }
    }
      auto name_it = ivf_name_.find(min_id);
      if (name_it == ivf_name_.end()) {
    return Status::MakeNotFound("");
      }
      *area_name = name_it->second;
      return Status::MakeOK();
  }

  Status FindNearest(const std::array<T, 2>& point, uint32_t top_k, T max_dist,
                     std::vector<std::string>* areas_name) {
    return Status::MakeNotSupport();
  }

  size_t GetElementNum() const { return data_.size(); }

 private:
  Status Insert(std::string name, const Diagram<T, Edges>& diagram) {
    data_.emplace(diagram.GetId(),diagram);
    atomic_max<uint64_t>(id_, diagram.GetId());
    ivf_name_.emplace(diagram.GetId(), name);
    return Status::MakeOK();
  }

  Status Delete() { return Status::MakeNotSupport(); }

  Status Build() { return Status::MakeOK(); }

  std::atomic<uint64_t> id_;

  // todo change tmp
  std::unordered_map<uint64_t, Diagram<T, Edges>> data_;
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
  std::unordered_map<std::string, Diagram<T, Edges>> data_;
};
}  // namespace bvh

#endif
