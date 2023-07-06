#include <thread>

#include "communication/serial/BuildHat.hpp"
#include "communication/serial/ISerialRead.hpp"
#include "modules/movement/IMovement.hpp"
#include "modules/movement/Drive.hpp"
#include "remote_control/DS4.hpp"

#include "communication/internal/BlackBoard.cpp"
#include "modules/lane_detection/LaneDetection.hpp"
#include "modules/control/RobotController.hpp"

int test_main() {
  ColorSensor colorSensor;

  auto colorRanges = ColorSensor::parse_calibration_file();

  // for twenty seconds, get and print color values every 200 ms
  for (int i = 0; i < 100; i++) {
    auto color = colorSensor.get_color();

    bool found = false;
    for (auto &colorRange : colorRanges) {
      if (color.sat <= COLOR_MIN_SAT && color.val <= COLOR_MIN_VAL) {
        // not a color (most likely the floor)
        break;
      }

      if (colorRange.contains(color, true)) {
        std::cout << colorRange.name << std::endl;
        found = true;
        break;
      }
    }

    if (!found) {
      std::cout << "unknown color: " << color << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{200});
  }

  return 0;
}

// TODO: detect power fault

int main() {
  auto &blackBoard{BlackBoard::getInstance()};
  blackBoard.running = true;

  //return test_main(); // globals: min cal offset = 2

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