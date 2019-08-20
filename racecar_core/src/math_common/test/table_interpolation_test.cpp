//
// Created by yenkn on 19-3-17.
//

#include <gtest/gtest.h>

#include <map>
#include <racecar_core/math_common/table_interpolation.h>

TEST(TableInterpolation, shouldInterpolate)
{
  TableInterpolation interpolator(InterpolateMode::Spline, BoundaryType::Clamp);
  std::map<double, double> testData = {
      {-0.6, -0.20},
      {0.5, 0.06},
      {1.0, 0.08},
      {1.6, 0.10},
  };
  interpolator.setPoints(testData);

  EXPECT_NEAR(interpolator.interpolate(0.12), 0.7, 0.001);
  EXPECT_NEAR(interpolator.interpolate(1.5), 0.9, 0.1);

  EXPECT_EQ(interpolator.interpolate(0.01), 0.1); // Clamp
  EXPECT_EQ(interpolator.interpolate(2.2), 0.9);
}

TEST(TableInterpolation, shouldInterpolateSin)
{
  TableInterpolation interpolator(InterpolateMode::Spline, BoundaryType::Clamp);
  std::map<double, double> testData;
  for(int i = 0; i < 10; i++) {
    testData[i] = sin(i);
  }
  interpolator.setPoints(testData);

  EXPECT_NEAR(interpolator.interpolate(0.4), sin(0.4), 0.01);
  EXPECT_NEAR(interpolator.interpolate(1.5), sin(1.5), 0.01);
}

int main(int argc,char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}