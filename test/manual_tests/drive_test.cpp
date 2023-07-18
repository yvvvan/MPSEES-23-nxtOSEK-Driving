#include "modules/movement/Drive.hpp"

#include <gtest/gtest.h>

#include "communication/internal/BlackBoard.cpp"

TEST(drive_test, turn_left) {
  auto &blackBoard{BlackBoard::getInstance()};
  blackBoard.running = true;
  blackBoard.navigation_enabled = true;

  auto &drive = Drive::getInstance();
  drive.turn_left();
}

TEST(drive_test, stop) {
  auto &blackBoard{BlackBoard::getInstance()};
  blackBoard.running = true;
  blackBoard.navigation_enabled = true;

  auto &drive = Drive::getInstance();
  drive.coast();
}
