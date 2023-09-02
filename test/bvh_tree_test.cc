
#include <map>
#include <vector>

#include "bvh.h"
#include "gtest/gtest.h"
#include "ray_casting.h"
#include "utils.h"
using namespace bvh;
TEST(BVHtree, base) {
  BVHTreeBuilder<float, 2> bt_builder;
  bt_builder.Insert(
      "A1", std::vector<std::vector<float>>{{0, 0}, {2, 0}, {2, 2}, {0, 2}});
  bt_builder.Insert(
      "A2", std::vector<std::vector<float>>{{1, 0}, {3, 0}, {3, 2}, {1, 2}});
  bt_builder.Insert("A3", std::vector<std::vector<float>>{
                              {10, 0}, {12, 0}, {12, 2}, {10, 2}});
  bt_builder.Insert("A4", std::vector<std::vector<float>>{
                              {0, 10}, {2, 10}, {2, 12}, {0, 12}});
  bt_builder.Insert("A5", std::vector<std::vector<float>>{
                              {20, 1}, {22, 1}, {22, 3}, {20, 3}});
  bt_builder.Insert("A6", std::vector<std::vector<float>>{
                              {26, 3}, {22, 1}, {22, 4}, {20, 3}, {74, 43}});
  bt_builder.Build();
  auto bt = bt_builder.GetBVHTree();
  EXPECT_EQ(6, bt->GetElementNum());
}

TEST(BVHtree, FindContain) {
  BVHTreeBuilder<float, 2> bt_builder;
  bt_builder.Insert(
      "A1", std::vector<std::vector<float>>{{0, 0}, {2, 0}, {2, 2}, {0, 2}});
  bt_builder.Insert(
      "A2", std::vector<std::vector<float>>{{1, 0}, {3, 0}, {3, 2}, {1, 2}});
  bt_builder.Insert("A3", std::vector<std::vector<float>>{
                              {10, 0}, {12, 0}, {12, 2}, {10, 2}});
  bt_builder.Insert("A4", std::vector<std::vector<float>>{
                              {0, 10}, {2, 10}, {2, 12}, {0, 12}});
  bt_builder.Insert("A5", std::vector<std::vector<float>>{
                              {20, 1}, {22, 1}, {22, 3}, {20, 3}});
  bt_builder.Insert("A6", std::vector<std::vector<float>>{{1484.45, 1703.77},
                                                          {1485.83, 1697.83},
                                                          {1482.48, 1697.05},
                                                          {1481.1, 1702.99}});
  bt_builder.Build();
  auto bt = bt_builder.GetBVHTree();

  std::array<float, 2> p1{0.5, 0.5};
  std::vector<std::string> r;
  auto s = bt->FindAllByContain(p1, &r);
  ASSERT_EQ(1, r.size());
  EXPECT_EQ("A1", r[0]);

  std::array<float, 2> p2{1.5, 1.5};
  s = bt->FindAllByContain(p2, &r);
  ASSERT_EQ(2, r.size());
  EXPECT_EQ("A1", r[0]);
  EXPECT_EQ("A2", r[1]);

  std::array<float, 2> p3{1, 1};
  s = bt->FindAllByContain(p3, &r);
  ASSERT_EQ(2, r.size());
  EXPECT_EQ("A1", r[0]);
  EXPECT_EQ("A2", r[1]);

  std::array<float, 2> p4{10, 0};
  s = bt->FindAllByContain(p4, &r);
  ASSERT_EQ(1, r.size());
  EXPECT_EQ("A3", r[0]);

  std::array<float, 2> p5{30, 33.3};
  s = bt->FindAllByContain(p5, &r);
  EXPECT_EQ(0, r.size());

  std::array<float, 2> p6{1430.0804099329575, 1110.208726214445};
  s = bt->FindAllByContain(p6, &r);
  EXPECT_EQ(0, r.size());
}

TEST(BVHtree, FindNearst) {
  BVHTreeBuilder<float, 2> bt_builder;
  bt_builder.Insert(
      "A1", std::vector<std::vector<float>>{{0, 0}, {2, 0}, {2, 2}, {0, 2}});
  bt_builder.Insert(
      "A2", std::vector<std::vector<float>>{{1, 0}, {3, 0}, {3, 2}, {1, 2}});
  bt_builder.Insert("A3", std::vector<std::vector<float>>{
                              {10, 0}, {12, 0}, {12, 2}, {10, 2}});
  bt_builder.Insert("A4", std::vector<std::vector<float>>{
                              {0, 10}, {2, 10}, {2, 12}, {0, 12}});
  bt_builder.Insert("A5", std::vector<std::vector<float>>{
                              {20, 1}, {22, 1}, {22, 3}, {20, 3}});
  bt_builder.Insert("A6", std::vector<std::vector<float>>{{1484.45, 1703.77},
                                                          {1485.83, 1697.83},
                                                          {1482.48, 1697.05},
                                                          {1481.1, 1702.99}});
  bt_builder.Build();
  auto bt = bt_builder.GetBVHTree();

  std::array<float, 2> p1{-0.5, 0.5};
  std::string r;
  auto s = bt->FindNearest(p1, 100, &r);
  EXPECT_EQ("A1", r);

  std::array<float, 2> p2{5.6, 1.5};
  s = bt->FindNearest(p2, 100, &r);
  EXPECT_EQ("A2", r);

  std::array<float, 2> p3{1.5, 11.5};
  s = bt->FindNearest(p3, 100, &r);
  EXPECT_EQ("A4", r);

  std::array<float, 2> p4{0.9, 3.5};
  s = bt->FindNearest(p4, 100, &r);
  EXPECT_EQ("A1", r);

  std::array<float, 2> p5{2.1, 3.5};
  s = bt->FindNearest(p5, 100, &r);
  EXPECT_EQ("A2", r);

  std::array<float, 2> p6{100, 10l};
  s = bt->FindNearest(p6, 5, &r);
  EXPECT_EQ(true, s == Status::NOT_FOUND());
}

TEST(BVHtree, really_find) {
  std::map<std::string, std::vector<std::vector<double>>> data;
  auto s = LoadDataFromFile<double>("../map.txt", &data);
  ASSERT_EQ(true, Status::OK() == s);
  BVHTreeBuilder<double, 2> bt_builder;
  for (auto&& [name, peaks] : data) {
    bt_builder.Insert(name, peaks);
  }
  bt_builder.Build();
  auto bt = bt_builder.GetBVHTree();
  std::string r;

  std::array<double, 2> p1{1475.3286456324336, 1179.755355715824};
  s = bt->FindNearest(p1, 100, &r);
  EXPECT_EQ("3AA.23", r);

  std::array<double, 2> p2{1405.6059966986068, 1074.0189993185923};
  s = bt->FindOneByContain(p2, &r);
  EXPECT_EQ("5AA.49", r);

  std::array<double, 2> p3{1411.930003168527, 1095.3849971266463};
  s = bt->FindOneByContain(p3, &r);
  EXPECT_EQ("4AA.43", r);

  std::array<double, 2> p4{1430.0804099329575, 1110.208726214445};
  s = bt->FindNearest(p4, 100, &r);
  EXPECT_EQ("4AA.39", r);
}