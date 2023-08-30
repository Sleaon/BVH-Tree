
#include "diagram.h"

#include "fast_vector.h"
#include "gtest/gtest.h"
#include "ray_casting.h"
using namespace bvh;
TEST(Diagnostics, base) {
  Diagram<int, 5> a;
  FastVector<int, 2> b{0, 0};
  EXPECT_EQ(true, b == a.GetCentre());
  EXPECT_EQ(5, a.GetEdges());
  EXPECT_EQ(0, a.GetId());

  FastVector<int, 2> c{1, 1};
  Diagram<int, 5> d(0, c);
  EXPECT_EQ(true, c == d.GetCentre());

  FastVector<float, 2> e{1, 1};
  FastVector<float, 2> f{1, 4};
  FastVector<float, 2> g{4, 4};
  FastVector<float, 2> h{4, 1};
  FastVector<float, 2> i{2.5, 2.5};
  Diagram<float, 4> j(0, e, f, g, h);
  EXPECT_EQ(true, i == j.GetCentre());
}

TEST(Diagnostics, contain) {
  FastVector<float, 2> a{1.0, 1.0};
  FastVector<float, 2> b{1.0, 20.0};
  FastVector<float, 2> c{50.0, 20.0};
  FastVector<float, 2> d{50.0, 1.0};
  Diagram<float, 4> e(0, a, b, c, d);
  RayCasting<float, 2, 4> ray;
  FastVector<float, 2> p1{2.0, 2.0};
  auto r = e.Contain(p1, ray);
  EXPECT_EQ(true, r);

  FastVector<float, 2> p2{1.0, 1.0};
  r = e.Contain<RayCasting<float, 2, 4>>(p2, ray);
  EXPECT_EQ(true, r);

  FastVector<float, 2> p3{40.5, 1.0};
  r = e.Contain<RayCasting<float, 2, 4>>(p3, ray);
  EXPECT_EQ(true, r);

  FastVector<float, 2> p4{40.5, 50.3};
  r = e.Contain<RayCasting<float, 2, 4>>(p4, ray);
  EXPECT_EQ(false, r);
}