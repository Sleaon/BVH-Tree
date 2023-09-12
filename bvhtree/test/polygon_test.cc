
#include "closest_point.h"
#include "fast_vector.h"
#include "gtest/gtest.h"
#include "polygon.h"
#include "ray_casting.h"
using namespace bvh;
TEST(Polygons, base) {
  Polygon<int> a;
  FastVector<int, 2> b{0, 0};
  EXPECT_EQ(true, b == a.GetCentre());
  EXPECT_EQ(1, a.GetEdges());
  EXPECT_EQ(0, a.GetId());

  FastVector<int, 2> c{1, 1};
  Polygon<int> d(0, c);
  EXPECT_EQ(true, c == d.GetCentre());

  FastVector<float, 2> e{1, 1};
  FastVector<float, 2> f{1, 4};
  FastVector<float, 2> g{4, 4};
  FastVector<float, 2> h{4, 1};
  FastVector<float, 2> i{2.5, 2.5};
  Polygon<float> j(0, e, f, g, h);
  EXPECT_EQ(true, i == j.GetCentre());
  EXPECT_EQ(4, j.GetEdges());
}

TEST(Polygons, contain) {
  FastVector<float, 2> a{1.0, 1.0};
  FastVector<float, 2> b{1.0, 20.0};
  FastVector<float, 2> c{50.0, 20.0};
  FastVector<float, 2> d{50.0, 1.0};
  Polygon<float> e(0, a, b, c, d);
  FastVector<float, 2> p1{2.0, 2.0};
  auto r = e.Contain(p1);
  EXPECT_EQ(true, r);

  FastVector<float, 2> p2{1.0, 1.0};
  r = e.Contain(p2);
  EXPECT_EQ(true, r);

  FastVector<float, 2> p3{40.5, 1.0};
  r = e.Contain(p3);
  EXPECT_EQ(true, r);

  FastVector<float, 2> p4{40.5, 50.3};
  r = e.Contain(p4);
  EXPECT_EQ(false, r);
}

TEST(Polygons, distance) {
  FastVector<float, 2> a{1.0, 1.0};
  FastVector<float, 2> b{1.0, 20.0};
  FastVector<float, 2> c{50.0, 20.0};
  FastVector<float, 2> d{50.0, 1.0};
  Polygon<float> e(0, a, b, c, d);

  FastVector<float, 2> p1{1.0, 1.0};
  auto dist = e.Distance(p1);
  EXPECT_DOUBLE_EQ(0, dist);

  FastVector<float, 2> p2{1.0, 10.0};
  dist = e.Distance(p1);
  EXPECT_DOUBLE_EQ(0, dist);

  FastVector<float, 2> p3{0.0, 1.0};
  dist = e.Distance(p3);
  EXPECT_DOUBLE_EQ(1.0, dist);

  FastVector<float, 2> p4{55.0, 10.0};
  dist = e.Distance(p4);
  EXPECT_DOUBLE_EQ(25.0, dist);
}