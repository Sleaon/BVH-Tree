#include "fast_vector.h"

#include "gtest/gtest.h"
using namespace bvh;
TEST(FAST_VECTOR, base) {
  FastVector<float, 3> a;
  FastVector<float, 3> b{0, 0, 0};
  EXPECT_EQ(true, a == b);

  FastVector<float, 3> c(1);
  FastVector<float, 3> d{1, 1, 1};
  EXPECT_EQ(true, c == d);

  std::array<float, 3> e{3, 5, 10};
  FastVector<float, 3> f(e);
  FastVector<float, 3> g{3, 5, 10};
  EXPECT_EQ(true, f == g);
}

TEST(FAST_VECTOR, add) {
  FastVector<float, 3> b{0, 0, 0};
  FastVector<float, 3> c{1.0, 2.0, 3.0};
  EXPECT_EQ(true, b + c == c);

  FastVector<float, 3> d{5.2, 2.3, 6.5};
  FastVector<float, 3> e{6.2, 4.3, 9.5};
  EXPECT_EQ(true, c + d == e);

  FastVector<double, 5> f{5.2, 2.3, 6.5, 2, 3};
  FastVector<double, 5> g{6.2, 4.3, 9.5, 6, 7};
  FastVector<double, 5> h{11.4, 6.6, 16.0, 8, 10};
  EXPECT_EQ(true, f + g == h);
}
TEST(FAST_VECTOR, sub) {
  FastVector<float, 3> c{1.000, 2.000, 6.500};
  FastVector<float, 3> d{5.200, 2.300, 3.500};
  FastVector<float, 3> e{-4.2, -0.300, 3.000};
  EXPECT_EQ(true, c - d == e);

  FastVector<double, 5> f{5.2, 2.3, 6.5, 2, 3};
  FastVector<double, 5> g{6.2, 4.3, 9.5, 6, 7};
  FastVector<double, 5> h{1.0, 2.0, 3.0, 4, 4};
  EXPECT_EQ(true, g - f == h);

  FastVector<float, 3> i{4, 4, 5};
  FastVector<float, 3> j{-4, -4, -5};
  EXPECT_EQ(true, i == -j);
}

TEST(FAST_VECTOR, mul) {
  FastVector<float, 3> c{1.0, 2.0, 6.5};
  FastVector<float, 3> d{5.2, 2.3, 3.5};
  FastVector<float, 3> e{5.2, 4.6, 22.75};
  EXPECT_EQ(true, c * d == e);

  FastVector<double, 5> f{5.200, 2.300, 6.500, 2.232323232300, 3.00};
  FastVector<double, 5> g{6.200, 4.300, 9.500, 6.666666666600, 7.000};
  FastVector<double, 5> h{32.2400, 9.8900, 61.7500, 14.88215488185117845118, 21};
  EXPECT_EQ(true, g * f == h);
}
TEST(FAST_VECTOR, div) {
  FastVector<float, 3> c{5, 2.0, 9.9};
  FastVector<float, 3> d{2, 4, 3};
  FastVector<float, 3> e{2.5, 0.5, 3.3};
  EXPECT_EQ(true, c / d == e);

  FastVector<double, 5> f{-10.00, 100.00, 33.00, 33.3300, -10.00};
  FastVector<double, 5> g{2.00, 200.00, 11.00, 3.00, 3.00};
  FastVector<double, 5> h{-5.000, 0.5000, 3.000, 11.1100, -10.0 / 3.0};
  EXPECT_EQ(true, f / g == h);

  FastVector<int, 5> i{10,24,32,65,100};
  FastVector<int, 5> j{2,6,32,5,101};
  FastVector<int, 5> k{5,4,1,13,0};
  EXPECT_EQ(true, f / g == h);

}