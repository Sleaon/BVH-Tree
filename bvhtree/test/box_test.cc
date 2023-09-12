#include "fast_vector.h"
#include "gtest/gtest.h"
#include "polygon.h"
using namespace bvh;
TEST(Boxs, base) {
  Box<float, 2> box(6, 0, 0, 6);
  auto center = box.GetCenter();
  FastVector<float, 2> a{3.0, 3.0};
  EXPECT_EQ(true, a == center);

  FastVector<float, 2> b{1.0, 20.0};
  auto r = box.Contain(b);
  EXPECT_EQ(false, r);

  FastVector<float, 2> c{0, 6};
  r = box.Contain(c);
  EXPECT_EQ(true, r);

  FastVector<float, 2> d{2, 3};
  r = box.Contain(d);
  EXPECT_EQ(true, r);

  auto dist = box.Distance(d);
  EXPECT_FLOAT_EQ(0, dist);

  FastVector<float, 2> e{10, 3};
  dist = box.Distance(e);
  EXPECT_FLOAT_EQ(16, dist);

  auto box1 = Box<float, 2>::MakeEmpty();
  Box<float, 2> box2(10, 2, 2, 10);
  FastVector<float, 2> f{2.0, 2.0};
  FastVector<float, 2> g{10.0, 10.0};
  box1.extend(f);
  box1.extend(g);
  EXPECT_EQ(true, box1 == box2);
}