#include <thread>

#include "modules/control/RobotController.hpp"
#include "modules/lane_detection/LaneDetection.hpp"

int test_turn_main() {
  auto &drive{Drive::getInstance()};
  auto &blackBoard{BlackBoard::getInstance()};

  blackBoard.running = true;

  auto laneDetection = LaneDetection(LaneDetectionMode::CAMERA);
  std::thread([&laneDetection]() mutable { laneDetection.run(nullptr); }).detach();

  drive.set_speed(0.75);
  drive.move_forward(0);
  std::this_thread::sleep_for(std::chrono::milliseconds{2000});

  drive.turn_left();

  drive.set_speed(0.75);
  drive.move_forward(0);
  std::this_thread::sleep_for(std::chrono::milliseconds{2000});

  drive.turn_right();

  drive.set_speed(0.75);
  drive.move_forward(0);
  std::this_thread::sleep_for(std::chrono::milliseconds{2000});

  blackBoard.running = false;

  drive.coast();
  std::this_thread::sleep_for(std::chrono::milliseconds{200});

  return 0;
}

int main() {
  auto &blackBoard{BlackBoard::getInstance()};
  blackBoard.running = true;

  //return test_turn_main();

  // create a new thread and detach which handles lane detection
  std::thread([]() {
    LaneDetection laneDetection(LaneDetectionMode::CAMERA);
    laneDetection.run(nullptr);
  }).detach();

  // TODO: ORB and webserver threads

  std::thread([&blackBoard]() {
    RobotController robotController;
    while (blackBoard.running.get()) {
      robotController.execute();
    }
  }).join();

  return 0;
}
