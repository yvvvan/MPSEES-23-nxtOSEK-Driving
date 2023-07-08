#include <gtest/gtest.h>

#include "slam/localization.hpp"

TEST(localization_ci_test, adjust_driving_direction) {
  Localization localization;

  localization.adjust_driving_direction(0.0);
  EXPECT_EQ(localization.driving_direction.at(0), 1);
  EXPECT_EQ(localization.driving_direction.at(1), 0);

  localization.adjust_driving_direction(90.0);
  EXPECT_EQ(localization.driving_direction.at(0), 0);
  EXPECT_EQ(localization.driving_direction.at(1), 1);

  localization.adjust_driving_direction(90.0);
  EXPECT_EQ(localization.driving_direction.at(0), -1);
  EXPECT_EQ(localization.driving_direction.at(1), 0);

  localization.adjust_driving_direction(60.0);
  EXPECT_EQ(localization.driving_direction.at(0), -0.5);
  EXPECT_EQ(localization.driving_direction.at(1), -0.87);

  localization.adjust_driving_direction(-90.0);
  EXPECT_EQ(localization.driving_direction.at(0), -0.87);
  EXPECT_EQ(localization.driving_direction.at(1), 0.5);
}
