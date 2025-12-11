#include "dancer_geometry/point.hpp"
#include "dancer_geometry/angle.hpp"
#include <gtest/gtest.h>

using namespace dancer_geometry;

TEST(TestGeometry, testPoint) {
}

TEST(TestGeometry, testAngle) {
  std::vector<Angle> angle_set;

  angle_set.emplace_back(90);
  angle_set.emplace_back(0);
  angle_set.emplace_back(30);
  angle_set.emplace_back(60);
  angle_set.emplace_back(45);
  ASSERT_FLOAT_EQ(45.0, Angle::mean(angle_set).get_signed());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

