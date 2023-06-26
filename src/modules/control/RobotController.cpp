#include "RobotController.hpp"

#include "modules/color_sensor/ColorSensor.hpp"

#include <thread>

RobotController::RobotController() {
  init();
}

RobotController::~RobotController() {
  terminate();
}

void RobotController::init() {
  // init angle
  last_angle = 0;

  // set the speed of the car
  drive.set_speed(.75); // TODO: make configurable

  // initiate color sensor
  // TODO: actually use the calibration and the color sensor
  blackBoard.colors = ColorSensor::parse_calibration_file();

  // wait for lane detection start up
  while (!blackBoard.lane_detection_ready.get()) {
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
  }

  // take start time
  start = std::chrono::system_clock::now();
}

void RobotController::terminate() {
  // coast motors and wait for other threads to finish
  drive.coast();
  std::this_thread::sleep_for(std::chrono::milliseconds{500});
}

void RobotController::execute() {
  // take lap time
  lap = std::chrono::system_clock::now();

  // TODO: remove for actual code
  // check whether ten seconds have elapsed
  if (std::chrono::duration_cast<std::chrono::seconds>(lap - start).count() > 10) {
    // initiate termination of the program
    blackBoard.running = false;
    return;
  }

  /*if (!blackBoard.buildHatReady.get()) {
    return;
  }*/
/*
  auto color = colorSensor.get_color();
  for (auto const &range : blackBoard.colors.get()) {
    if (range.contains(color)) {
      std::cout << "----------------------------------------" << std::endl;
      std::cout << ">>> Seeing color: " << range.name << std::endl;
      std::cout << "----------------------------------------" << std::endl;
    }
  }
*/
  // get the angle from the blackboard
  double angle = blackBoard.offset_middle_line.get();

  // calculate the distance between the last angle and the current angle
  double diff = std::abs(angle - last_angle);

  // difference is too big, use middle of both angles
  if (diff > 60) {
    angle = (angle + last_angle) / 2;
  } else {
    // difference is small, use current angle
    last_angle = angle;
  }

  // check whether the angle is too big, if so, set it to the maximum with regard to the sign
  if (std::abs(angle) > 175) angle = angle / std::abs(angle) * 175;

  // set the speed of the car
  drive.set_speed(.75); // TODO: make configurable

  // use the angle to control the car
  drive.move_forward(angle);
}
