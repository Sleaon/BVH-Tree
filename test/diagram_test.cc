
#include "diagram.h"

#include "fast_vector.h"
#include "gtest/gtest.h"
#include "ray_casting.h"
using namespace bvh;
TEST(Diagnostics, base) {
  Diagram<int, 5> a;
  FastVector<int, 2> b{0, 0};
  EXPECT_EQ(true, b == a.GetCentre());

  FastVector<int, 2> c{1, 1};
  Diagram<int, 5> d(c);
  EXPECT_EQ(true, c == d.GetCentre());

  FastVector<float, 2> e{1, 1};
  FastVector<float, 2> f{1, 4};
  FastVector<float, 2> g{4, 4};
  FastVector<float, 2> h{4, 1};
  FastVector<float, 2> i{2.5, 2.5};
  Diagram<float, 4> j(e,f,g,h);
  EXPECT_EQ(true, i == j.GetCentre());

}