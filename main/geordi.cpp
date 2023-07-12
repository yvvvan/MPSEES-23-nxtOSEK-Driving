#include <thread>

#include "communication/internal/BlackBoard.cpp"

#include "modules/control/RobotController.hpp"
#include "modules/lane_detection/LaneDetection.hpp"
#include "modules/slam/navigation.hpp"

// TODO: detect power fault

int main() {
  auto &blackBoard{BlackBoard::getInstance()};
  blackBoard.running = true;

  auto colorSensor = ColorSensor();

  // set target color
  blackBoard.target_color_name.set("orange");

  // create a new thread and detach which handles lane detection
  std::thread([]() {
    LaneDetection laneDetection(LaneDetectionMode::CAMERA);
    laneDetection.run(nullptr);
  }).detach();

  // TODO: webserver thread

  std::thread([&blackBoard]() {
    blackBoard.destination.set(8);
    blackBoard.next_intersection.set(4);
    blackBoard.last_intersection.set(7);
    blackBoard.intersection_handled.set(true);
    Navigation navigation;
    navigation.exec_thread();
  }).detach();

  std::thread([&blackBoard]() {
    namespace ch = std::chrono;
    RobotController robotController;
    blackBoard.navigation_enabled.set(true);
    while (blackBoard.running.get()) {
      // take start time - we ensure that every iteration takes 25 ms, thus we
      // have exactly 40 iterations per second
      auto start = ch::high_resolution_clock::now();
      // execute the robot controller
      robotController.execute();
      // calculate taken time
      auto duration = ch::duration_cast<std::chrono::milliseconds>(
                          ch::high_resolution_clock::now() - start)
                          .count();
      // sleep so that the total time is 25 ms
      std::this_thread::sleep_for(std::chrono::milliseconds{25 - duration});
    }
  }).join();

  return 0;
}