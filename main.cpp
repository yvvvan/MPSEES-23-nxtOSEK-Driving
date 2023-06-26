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

int actual_factual_main() {
  // get the instances of the classes
  auto &drive{Drive::getInstance()};
  auto &blackBoard{BlackBoard::getInstance()};

  // let all start
  blackBoard.running = true;

  // create a new thread and detach which handles lane detection
  LaneDetection laneDetection(LaneDetectionMode::CAMERA);
  std::thread([&laneDetection]() mutable { laneDetection.run(nullptr); }).detach();

  // save current time
  auto start = std::chrono::system_clock::now();
  auto lap = std::chrono::system_clock::now();

  // set the speed of the car to maximum
  drive.set_speed(.75);

  // store one angle in the past to ensure that the car does not turn too fast
  double last_angle = 0;

  while (blackBoard.running.get()) {
    // take lap time
    lap = std::chrono::system_clock::now();

    // check whether ten seconds have elapsed
    if (std::chrono::duration_cast<std::chrono::seconds>(lap - start).count() > 10) {
      // initiate termination of the program
      blackBoard.running = false;
      break;
    }

    // get the angle from the blackboard
    double angle = blackBoard.offset_middle_line.get();

    // calculate the distance between the last angle and the current angle
    double diff = std::abs(angle - last_angle);

    // difference is to big, use middle of both angles
    if (diff > 60) {
      angle = (angle + last_angle) / 2;
    } else {
      // difference is small, use current angle
      last_angle = angle;
    }

    // check whether the angle is too big, if so, set it to the maximum with regard to the sign
    if (std::abs(angle) > 175) angle = angle / std::abs(angle) * 175;

    // use the angle to control the car
    drive.move_forward(angle);
  }

  // coast motors and wait for other threads to finish
  drive.coast();
  std::this_thread::sleep_for(std::chrono::milliseconds{500});

  return 0;
}

int main() {
  //return actual_factual_main();

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