# BVH-Tree
## Introduction
In order to quickly detect the region in which a given coordinate is located on a map, I have created a detection tool based on the BVH Tree approach. This tool can determine which regions the current coordinate is contained in or closest to.
This project is inspired by madmann91's [bvh](https://github.com/madmann91/bvh) implementation.

## Features 
* Supports two-dimensional polygons of arbitrary shapes.
* Supports finding polygons that contain a target point.
* Supports finding the closest polygons to a target point.

## Future Goals
- [ ] top-k nearest neighbor search
- [ ] support for AVX512
- [ ] support for circular shape
- [ ] dynamic addition and removal of shapes
- [ ] Ensure thread safety

## Usage
* Build
  ```c++
  bvh::BVHTreeBuilder<double, 2> bt_builder;
  for (auto&& [name, shape] : shapes) {
    bt_builder.Insert(name, shape);
  }
  bt_builder.Build(10); //Each leaf node contains a maximum of 10 shapes
  auto bt = bt_builder.GetBVHTree();
  ```
* Find
  ```c++
  auto s = bt->FindOneByContain(point, &r);
  s = bt->FindAllByContain(point, &vr);
  s = bt->FindNearest(point, 100, &r);

  if(s==bvh::Status::OK()){
        // Here, a successful retrieval of the target places the result in 'r'
  }
  if(s==bvh::Status::NOT_FOUND()){
        // When no results meeting the criteria are found
  } 
  if(s==bvh::Status::ERROR()){
        // error, message can use s.msg()
  } 
  if(s==bvh::Status::NOT_SUPPORT()){
        // At the moment, this functionality is still unavailable.
  } 
  ```