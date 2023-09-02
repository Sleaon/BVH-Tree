#include <chrono>
#include <iostream>
#include <random>
#include <string>

#include "bvh_tree.h"
using namespace bvh;
int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout
        << "arg size must be 2. first is data file path. second is cycle num"
        << std::endl;
    return 1;
  }
  std::string file_path = argv[1];
  int cycle_num = std::atoi(argv[2]);
  std::map<std::string, std::vector<std::vector<double>>> data;
  auto s = LoadDataFromFile<double>(file_path, &data);
  if (s != Status::OK()) {
    std::cout << "data file loads failed, cause: " << s.GetMsg() << std::endl;
    return 1;
  }
  std::cout << "diagram num: " << data.size() << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  BVHTreeBuilder<double, 2> bt_builder;
  for (auto&& [name, peaks] : data) {
    bt_builder.Insert(name, peaks);
  }
  bt_builder.Build();
  auto bt = bt_builder.GetBVHTree();
  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  std::cout << "build cost " << duration.count() << " us" << std::endl;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> intDist(0, 999);

  std::uniform_real_distribution<> realDist(1000.0, 2000.0);
  std::vector<std::array<double, 2>> points;
  points.reserve(1000);
  for (auto i = 0; i < 1000; ++i) {
    points.emplace_back(std::array<double, 2>{realDist(gen), realDist(gen)});
  }

  std::string r;
  start = std::chrono::high_resolution_clock::now();
  for (auto i = 0; i < cycle_num; ++i) {
    auto s = bt->FindOneByContain(points[intDist(gen)], &r);
  }
  end = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  std::cout << "find contain cost " << duration.count() << " us" << std::endl;

  start = std::chrono::high_resolution_clock::now();
  for (auto i = 0; i < cycle_num; ++i) {
    auto s = bt->FindNearest(points[intDist(gen)], 100, &r);
  }
  end = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  std::cout << "find nearst cost " << duration.count() << " us" << std::endl;

  return 0;
}