#pragma once
#ifndef BVH_TREE_BVH_NODE_H
#define BVH_TREE_BVH_NODE_H
#include <random>
#include <set>
#include <unordered_map>

#include "box.h"
#include "fast_vector.h"
#include "shape.h"
namespace bvh {

template <typename T, size_t Dim>
struct ShapeInfo {
  uint64_t id;
  T dist;
  mutable T c_dist;
  mutable bool has_c_dist;
  const FastVector<T, Dim>* centre;
  const FastVector<T, Dim>* point;
  ShapeInfo()
      : id(-1),
        dist(std::numeric_limits<T>::max()),
        c_dist(0),
        has_c_dist(false),
        centre(nullptr),
        point(nullptr) {}
  ShapeInfo(uint64_t id, T dist, const FastVector<T, Dim>* centre,
            const FastVector<T, Dim>* point)
      : id(id),
        dist(dist),
        c_dist(0),
        has_c_dist(false),
        centre(centre),
        point(point) {}
  inline T GetCentreDist() const {
    if (!has_c_dist) [[unlikely]] {
      if(point==nullptr || centre == nullptr){
        return std::numeric_limits<T>::max();
      }
      c_dist = ComputeDistance(*point, *centre);
      has_c_dist = true;
    }
    return c_dist;
  }

  bool operator<(const ShapeInfo<T,Dim>& o) const {
    if (dist == o.dist) [[unlikely]] {
      return this->GetCentreDist() < o.GetCentreDist();
    } else {
      return dist < o.dist;
    }
  }
};

template <typename T, size_t Dim>
struct TopKHelper {
  uint32_t capacity;
  T dist_upper;
  std::set<ShapeInfo<T, Dim>> nodes;

  TopKHelper(uint32_t top_k, T dist_upper)
      : capacity(top_k), dist_upper(dist_upper) {}

  void InsertNearestShape(ShapeInfo<T, Dim>&& info) {
    if (info.dist > dist_upper) {
      return;
    }

    if(nodes.empty()){
      nodes.emplace(std::move(info));
      return;
    }

    if (info.dist > nodes.rbegin()->dist) {
      return;
    }
    nodes.emplace(std::move(info));
    if (nodes.size() > capacity) {
      auto num = nodes.size() - capacity;
      for (size_t i = 0; i < num; ++i) {
        nodes.erase(std::prev(nodes.end()));
      }
    }
  }

  inline void Merge(TopKHelper<T, Dim>&& helper) {
    nodes.merge(helper.nodes);
    if (nodes.size() > capacity) {
      auto num = nodes.size() - capacity;
      for (size_t i = 0; i < num; ++i) {
        nodes.erase(std::prev(nodes.end()));
      }
    }
  }

  inline void Merge(std::set<ShapeInfo<T, Dim>>& info_set) {
    nodes.merge(info_set);
    if (nodes.size() > capacity) {
      auto num = nodes.size() - capacity;
      for (size_t i = 0; i < num; ++i) {
        nodes.erase(std::prev(nodes.end()));
      }
    }
  }
};

template <typename T, size_t Dim>
class BVHNode {};

template <typename T>
class BVHNode<T, 2> {
 public:
  BVHNode()
      : is_leaf_(true),
        size_(0),
        hight_(1),
        box_(Box<T, 2>::MakeEmpty()),
        parent_(nullptr),
        upper_left_(nullptr),
        upper_right_(nullptr),
        lower_left_(nullptr),
        lower_right_(nullptr),
        data_() {}

  BVHNode(BVHNode<T, 2>* parent)
      : is_leaf_(true),
        size_(0),
        hight_(1),
        box_(Box<T, 2>::MakeEmpty()),
        parent_(parent),
        upper_left_(nullptr),
        upper_right_(nullptr),
        lower_left_(nullptr),
        lower_right_(nullptr),
        data_() {}

  ~BVHNode() {
    if (is_leaf_) {
      for (auto [_, shape] : data_) {
        delete shape;
      }
    } else {
      delete upper_left_;
      delete upper_right_;
      delete lower_left_;
      delete lower_right_;
    }
  }

  Status FindAllByContain(const FastVector<T, 2>& point,
                          std::vector<uint64_t>* area_ids) const {
    if (!box_.Contain(point)) {
      return Status::MakeNotFound();
    }
    if (is_leaf_) {
      size_t count = 0;
      auto it = data_.begin();
      while (true) {
        it = std::find_if(it, data_.end(), [&](auto& s_pair) {
          auto& [_, s] = s_pair;
          return s->Contain(point);
        });
        if (it != data_.end()) {
          area_ids->emplace_back(it->first);
          count++;
        } else [[likely]] {
          break;
        }
        ++it;
      }

      if (count > 0) {
        return Status::MakeOK();
      } else {
        return Status::MakeNotFound();
      }
    } else {
      auto count = 0;
      auto s = upper_left_->FindAllByContain(point, area_ids);
      if (s == Status::OK()) {
        count++;
      } else if (s == Status::ERROR()) {
        return s;
      }
      s = upper_right_->FindAllByContain(point, area_ids);
      if (s == Status::OK()) {
        count++;
      } else if (s == Status::ERROR()) {
        return s;
      }
      s = lower_left_->FindAllByContain(point, area_ids);
      if (s == Status::OK()) {
        count++;
      } else if (s == Status::ERROR()) {
        return s;
      }
      s = lower_right_->FindAllByContain(point, area_ids);
      if (s == Status::OK()) {
        count++;
      } else if (s == Status::ERROR()) {
        return s;
      }
      if (count > 0) {
        return Status::MakeOK();
      } else {
        return Status::MakeNotFound();
      }
    }
  }

  Status FindOneByContain(const FastVector<T, 2>& point,
                          uint64_t* area_id) const {
    if (!box_.Contain(point)) {
      return Status::MakeNotFound();
    }
    if (is_leaf_) {
      auto it = std::find_if(data_.begin(), data_.end(), [&](auto& s_pair) {
        auto& [_, s] = s_pair;
        return s->Contain(point);
      });
      if (it != data_.end()) {
        *area_id = it->first;
        return Status::MakeOK();
      } else [[likely]] {
        return Status::MakeNotFound();
      }
    } else {
      auto s = upper_left_->FindOneByContain(point, area_id);
      if (s == Status::OK()) {
        return Status::MakeOK();
      }
      s = upper_right_->FindOneByContain(point, area_id);
      if (s == Status::OK()) {
        return Status::MakeOK();
      }
      s = lower_left_->FindOneByContain(point, area_id);
      if (s == Status::OK()) {
        return Status::MakeOK();
      }
      s = lower_right_->FindOneByContain(point, area_id);
      if (s == Status::OK()) {
        return Status::MakeOK();
      }
      return Status::MakeNotFound();
    }
  }

  Status FindNearest(const FastVector<T, 2>& point, T max_dist,
                     ShapeInfo<T, 2>* shape_info) const {
    if (box_.Distance(point) > max_dist) {
      return Status::MakeNotFound();
    }

    if (is_leaf_) {
      uint64_t min_id = -1;
      T min_dist = max_dist + 1;
      FastVector<T, 2> min_centre;
      for (auto&& [id, shape] : data_) {
        T dist = shape->Distance(point);
        if (dist < min_dist) {
          min_id = id;
          min_dist = dist;
          min_centre = shape->GetCentre();
          continue;
        }
        if (dist == min_dist) {
          T dist_c = ComputeDistance(point, shape->GetCentre());
          T min_dist_c = ComputeDistance(point, min_centre);
          if (dist_c < min_dist_c) {
            min_id = id;
            min_dist = dist;
          }
        }
      }
      if (min_dist <= max_dist) {
        shape_info->id = min_id;
        shape_info->dist = min_dist;
        return Status::MakeOK();
      } else {
        return Status::MakeNotFound();
      }
    } else {
      auto count = 0;
      ShapeInfo<T, 2> info;
      ShapeInfo<T, 2> min_info;

      auto s = upper_left_->FindNearest(point, max_dist, &info);
      if (s == Status::OK()) {
        min_info = info;
        count++;
      } else if (s == Status::ERROR()) {
        return s;
      }
      s = upper_right_->FindNearest(point, max_dist, &info);
      if (s == Status::OK()) {
        if (info.dist < min_info.dist) {
          min_info = info;
          count++;
        }
      } else if (s == Status::ERROR()) {
        return s;
      }
      s = lower_left_->FindNearest(point, max_dist, &info);
      if (s == Status::OK()) {
        if (info.dist < min_info.dist) {
          min_info = info;
          count++;
        }
      } else if (s == Status::ERROR()) {
        return s;
      }
      s = lower_right_->FindNearest(point, max_dist, &info);
      if (s == Status::OK()) {
        if (info.dist < min_info.dist) {
          min_info = info;
          count++;
        }
      } else if (s == Status::ERROR()) {
        return s;
      }
      if (count > 0) {
        *shape_info = min_info;
        return Status::MakeOK();
      } else {
        return Status::MakeNotFound();
      }
    }
  }

  Status FindNearest(const FastVector<T, 2>& point, uint32_t top_k, T max_dist,
                     std::set<ShapeInfo<T,2>>* info_set) const {
    if (box_.Distance(point) > max_dist) {
      return Status::MakeNotFound();
    }
    info_set->clear();
    TopKHelper<T, 2> helper(top_k, max_dist);

    if (is_leaf_) {
      for (auto&& [id, shape] : data_) {
        T dist = shape->Distance(point);
        helper.InsertNearestShape(
            ShapeInfo<T,2>(id, dist, &(shape->GetCentre()), &point));
      }
      if (!helper.nodes.empty()) {
        *info_set = std::move(helper.nodes);
        return Status::MakeOK();
      } else {
        return Status::MakeNotFound();
      }
    } else {
      std::set<ShapeInfo<T,2>> info_tmp;

      auto s = upper_left_->FindNearest(point, top_k, max_dist, &info_tmp);
      if (s == Status::OK()) {
        helper.Merge(info_tmp);
      } else if (s == Status::ERROR()) {
        return s;
      }
      s = upper_right_->FindNearest(point, top_k, max_dist, &info_tmp);
      if (s == Status::OK()) {
        helper.Merge(info_tmp);
      } else if (s == Status::ERROR()) {
        return s;
      }
      s = lower_left_->FindNearest(point, top_k, max_dist, &info_tmp);
      if (s == Status::OK()) {
        helper.Merge(info_tmp);
      } else if (s == Status::ERROR()) {
        return s;
      }
      s = lower_right_->FindNearest(point, top_k, max_dist, &info_tmp);
      if (s == Status::OK()) {
        helper.Merge(info_tmp);
      } else if (s == Status::ERROR()) {
        return s;
      }
      if (!helper.nodes.empty()) {
        *info_set = std::move(helper.nodes);
        return Status::MakeOK();
      } else {
        return Status::MakeNotFound();
      }
    }
  }

  Status Insert(Shape<T, 2>* shape_ptr) {
    auto [it, ok] = data_.emplace(shape_ptr->GetId(), shape_ptr);
    if (!ok) {
      return Status::MakeError("ID is exist");
    }
    size_++;
    box_.extend(shape_ptr->GetBox());
    return Status::MakeOK();
  }

  Status Build(size_t max_capacity) {
    if (size_ > max_capacity) {
      if constexpr (std::is_floating_point_v<T>) {
        return Split<std::uniform_real_distribution<>>(max_capacity);
      } else {
        return Split<std::uniform_int_distribution<>>(max_capacity);
      }
    }
    return Status::MakeOK();
  }

  size_t GetElementNum() const { return size_; }

  size_t GetHight() const { return hight_; }

 private:
  template <typename RandomDist>
  Status Split(size_t max_capacity) {
    is_leaf_ = false;
    upper_left_ = new BVHNode<T, 2>(this);
    upper_right_ = new BVHNode<T, 2>(this);
    lower_left_ = new BVHNode<T, 2>(this);
    lower_right_ = new BVHNode<T, 2>(this);
    UpdateHight(1);
    std::vector<Shape<T, 2>*> shapes(data_.size());
    std::transform(data_.begin(), data_.end(), shapes.begin(),
                   [](const auto& pair) { return pair.second; });
    std::sort(shapes.begin(), shapes.end(),
              [](Shape<T, 2>* left, Shape<T, 2>* right) {
                return left->GetCentre()[0] < right->GetCentre()[0];
              });
    T median_x = shapes[shapes.size() / 2]->GetCentre()[0];

    std::sort(shapes.begin(), shapes.end(),
              [](Shape<T, 2>* left, Shape<T, 2>* right) {
                return left->GetCentre()[1] < right->GetCentre()[1];
              });
    T median_y = shapes[shapes.size() / 2]->GetCentre()[1];

    std::random_device rd;
    std::mt19937 gen(rd());
    constexpr T upper_limit = std::numeric_limits<T>::epsilon() * 20;
    constexpr T lower_limit = -std::numeric_limits<T>::epsilon() * 10;
    RandomDist distribution(lower_limit, upper_limit);

    Status s;
    for (auto [id, shape] : data_) {
      if (shape->GetCentre()[0] > median_x + distribution(gen)) {
        if (shape->GetCentre()[1] > median_y + distribution(gen)) {
          s = upper_right_->Insert(shape);
        } else {
          s = lower_right_->Insert(shape);
        }
      } else {
        if (shape->GetCentre()[1] > median_y + distribution(gen)) {
          s = upper_left_->Insert(shape);
        } else {
          s = lower_left_->Insert(shape);
        }
      }
      if (s != Status::OK()) [[unlikely]] {
        return s;
      }
    }
    data_.clear();
    s = upper_left_->Build(max_capacity);
    if (s != Status::OK()) [[unlikely]] {
      return s;
    }
    s = upper_right_->Build(max_capacity);
    if (s != Status::OK()) [[unlikely]] {
      return s;
    }
    s = lower_left_->Build(max_capacity);
    if (s != Status::OK()) [[unlikely]] {
      return s;
    }
    s = lower_right_->Build(max_capacity);
    if (s != Status::OK()) [[unlikely]] {
      return s;
    }
    return Status::MakeOK();
  }

  void UpdateHight(size_t hight) {
    hight_ = std::max(hight + 1, hight_);
    if (parent_ != nullptr) {
      parent_->UpdateHight(hight_);
    }
  }

 private:
  bool is_leaf_;
  size_t size_;
  size_t hight_;
  Box<T, 2> box_;
  BVHNode<T, 2>* parent_;
  BVHNode<T, 2>* upper_left_;
  BVHNode<T, 2>* upper_right_;
  BVHNode<T, 2>* lower_left_;
  BVHNode<T, 2>* lower_right_;
  std::unordered_map<uint64_t, Shape<T, 2>*> data_;
};
}  // namespace bvh

#endif