#include <gtest/gtest.h>

#include "modules/slam/coordinates.hpp"
#include "modules/slam/localization.hpp"

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

TEST(localization_ci_test, track_map) {
  BlackBoard &blackboard = BlackBoard::getInstance();
  Localization localization;

  blackboard.connection_map = {// S W N E
                               {0, {3, 8, -1, 1}},  {1, {9, 0, -1, 2}},
                               {2, {6, 1, -1, -1}}, {3, {7, 0, 8, 4}},
                               {4, {10, 3, -1, 5}}, {5, {7, 4, -1, 6}},
                               {6, {-1, 5, 2, -1}}, {7, {-1, 3, 10, 5}}};

  blackboard.intersection_handled = false;
  blackboard.last_intersection = 0;
  blackboard.next_intersection = 1;
  blackboard.turn_direction = direction_t::STRAIGHT;

  localization.track_map();

  EXPECT_EQ(blackboard.next_intersection.get(), 1);

  blackboard.intersection_handled = true;
  localization.track_map();

  EXPECT_EQ(blackboard.next_intersection.get(), 2);
  EXPECT_EQ(blackboard.last_intersection.get(), 1);

  blackboard.intersection_handled = false;
  localization.track_map();

  EXPECT_EQ(blackboard.next_intersection.get(), 2);
  EXPECT_EQ(blackboard.last_intersection.get(), 1);

  blackboard.intersection_handled = true;
  blackboard.turn_direction = direction_t::RIGHT;
  localization.track_map();

  EXPECT_EQ(blackboard.next_intersection.get(), 6);
  EXPECT_EQ(blackboard.last_intersection.get(), 2);
}
