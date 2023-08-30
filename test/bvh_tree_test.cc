
#include <map>
#include <vector>

#include "bvh.h"
#include "gtest/gtest.h"
#include "ray_casting.h"
using namespace bvh;
TEST(BVHtree, base) {
  BVHTreeBuilder<float, 2, 4> bt_builder;
  bt_builder.Insert("A1",std::vector<std::vector<float>>{{0, 0}, {2, 0}, {2, 2}, {0, 2}});  
  bt_builder.Insert("A2", std::vector<std::vector<float>>{{1, 0}, {3, 0}, {3, 2}, {1, 2}});  
  bt_builder.Insert("A3", std::vector<std::vector<float>>{{10, 0}, {12, 0}, {12, 2}, {10, 2}});  
  bt_builder.Insert("A4", std::vector<std::vector<float>>{{0, 10}, {2, 10}, {2, 12}, {0, 12}});  
  bt_builder.Insert("A5", std::vector<std::vector<float>>{{20, 1}, {22, 1}, {22, 3}, {20, 3}});  
  bt_builder.Build();
  auto bt = bt_builder.GetBVHTree();

  EXPECT_EQ(5, bt->GetElementNum());

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
}