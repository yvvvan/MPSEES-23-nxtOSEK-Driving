#include <csignal>
#include <thread>

#include "communication/internal/BlackBoard.cpp"
#include "communication/serial/BuildHat.hpp"
#include "communication/serial/ISerialRead.hpp"
#include "modules/control/RobotController.hpp"
#include "modules/lane_detection/LaneDetection.hpp"
#include "modules/movement/Drive.hpp"
#include "modules/movement/IMovement.hpp"
#include "remote_control/DS4.hpp"

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

  auto colorSensor = ColorSensor();
/*
  // create a new thread and detach which handles lane detection
  std::thread([]() {
    LaneDetection laneDetection(LaneDetectionMode::CAMERA);
    laneDetection.run(nullptr);
  }).detach();

  while (!blackBoard.lane_detection_ready.get()) {
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
  }

  auto &drive = Drive::getInstance();
  drive.set_speed(CAR_SPEED);

  drive.move_forward(0);
  std::this_thread::sleep_for(std::chrono::milliseconds{1000});

  drive.turn_left();

  return 0;*/

  // set target color
  blackBoard.target_color_name.set("orange");

  // create a new thread and detach which handles lane detection
  std::thread([]() {
    LaneDetection laneDetection(LaneDetectionMode::CAMERA);
    laneDetection.run(nullptr);
  }).detach();

  // TODO: webserver thread

  std::thread([&blackBoard]() {
    namespace ch = std::chrono;
    RobotController robotController;
    while (blackBoard.running.get()) {
      // take start time - we ensure that every iteration takes 25 ms, thus we have exactly 40 iterations per second
      auto start = ch::high_resolution_clock::now();
      // execute the robot controller
      robotController.execute();
      // calculate taken time
      auto duration = ch::duration_cast<std::chrono::milliseconds>(ch::high_resolution_clock::now() - start).count();
      // sleep so that the total time is 25 ms
      std::this_thread::sleep_for(std::chrono::milliseconds{25 - duration});
    }
  }).join();

  return 0;
}