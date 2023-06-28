#include <thread>

#include "communication/serial/BuildHat.hpp"
#include "communication/serial/ISerialRead.hpp"
#include "modules/movement/IMovement.hpp"
#include "modules/movement/Drive.hpp"
#include "remote_control/DS4.hpp"

#include "communication/internal/BlackBoard.cpp"
#include "modules/lane_detection/LaneDetection.hpp"
#include "modules/control/RobotController.hpp"

void windDriveUp(double wind_start_speed, double wind_target_speed, int wind_time_milliseconds, int wind_step_count) {
  std::thread([wind_start_speed, wind_target_speed, wind_time_milliseconds, wind_step_count]() {
    IMovement &drive = Drive::getInstance();
    drive.set_speed(wind_start_speed);
    uint16_t wind_sleep = wind_time_milliseconds / wind_step_count;
    double stepSize = (wind_target_speed - wind_start_speed) / ((double) wind_step_count);
    for (int step = 0; step <= wind_step_count; ++step) {
      double value = wind_start_speed + (step * stepSize);
      drive.set_speed(value);
      std::this_thread::sleep_for(std::chrono::milliseconds(wind_sleep));
    }
  }).detach();
}

void windDriveDown(double wind_start_speed, double wind_target_speed, int wind_time_milliseconds, int wind_step_count) {
  std::thread([wind_start_speed, wind_target_speed, wind_time_milliseconds, wind_step_count]() {
    IMovement &drive = Drive::getInstance();
    drive.set_speed(wind_start_speed);
    uint16_t wind_sleep = wind_time_milliseconds / wind_step_count;
    double stepSize = (wind_target_speed - wind_start_speed) / ((double) wind_step_count);
    for (int step = 0; step <= wind_step_count; ++step) {
      double value = wind_target_speed - (step * stepSize);
      std::this_thread::sleep_for(std::chrono::milliseconds(wind_sleep));
      drive.set_speed(value);
    }
  }).detach();
}

int test_turn_main() {
  auto &drive{Drive::getInstance()};

  drive.turn_left();

  drive.coast();
  std::this_thread::sleep_for(std::chrono::milliseconds{200});

  return 0;
}

int main() {
  //return test_turn_main();

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