#include <thread>

#include "modules/control/RobotController.hpp"
#include "modules/lane_detection/LaneDetection.hpp"

int main() {
  // create a new thread and detach which handles lane detection
  LaneDetection laneDetection(LaneDetectionMode::CAMERA);
  std::thread([&laneDetection]() mutable { laneDetection.run(nullptr); }).detach();

  // TODO: ORB and webserver threads

  std::thread(&RobotController::run).join();

  return 0;
}
