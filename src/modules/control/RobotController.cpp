#include "RobotController.hpp"

#include "modules/control/PDController.hpp"
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
  drive.set_speed(2.0/3.0); // TODO: make configurable

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
  // create a new PD controller
  // TODO: move back to globals.hpp
  static PDController pd_controller(KP, KD);

  // take lap time
  lap = std::chrono::system_clock::now();

  // TODO: remove for actual code
  // check whether ten seconds have elapsed
  if (std::chrono::duration_cast<std::chrono::seconds>(lap - start).count() > 15) {
    // initiate termination of the program
    blackBoard.running = false;
    return;
  }

  if (!blackBoard.buildHatReady.get()) {
    return;
  }

  /*+++++++++++++++++++++++++++++++++++++++*/

  /* TODO
  if (blackBoard.is_dead_end.get()) {
    drive.turn_left();
    drive.turn_left();
    return;
  }*/
/*
  static int wait_time_ms = 750;

  auto [leftIntersection, middleIntersection, rightIntersection] = blackBoard.exits_intersection.get();

  if (leftIntersection||middleIntersection||rightIntersection) {
    drive.set_speed(0.5);
  } else {
    drive.set_speed(2.0/3.0);
  }

  if (leftIntersection) {
    if (!middleIntersection && !rightIntersection) {
      if (!should_turn_left) {
        should_turn_left = true;
        leftTurnTime = std::chrono::system_clock::now();
      }

      // when the car is in the intersection for more than wait_time_ms milliseconds, turn left
      auto now = std::chrono::system_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - leftTurnTime).count() > wait_time_ms) {
        should_turn_left = false;
        drive.turn_left();
        return;
      }
    } else {
      should_turn_left = false;
    }
  }

  if (rightIntersection) {
    if (!middleIntersection && !leftIntersection) {
      if (!should_turn_right) {
        should_turn_right = true;
        rightTurnTime = std::chrono::system_clock::now();
      }

      // when the car is in the intersection for more than wait_time_ms milliseconds, turn right
      auto now = std::chrono::system_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - rightTurnTime).count() > wait_time_ms) {
        should_turn_right = false;
        drive.turn_right();
        return;
      }
    } else {
      should_turn_right = false;
    }
  }
*/
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

  // calculate the angle using the PD controller
  double error = angle;
  double control_signal = pd_controller.calculateControl(error);

  // adjust the angle
  angle += control_signal;

  // check whether the angle is too big, if so, set it to the maximum with regard to the sign
  if (std::abs(angle) > 175) angle = angle / std::abs(angle) * 175;

  // use the angle to control the car
  drive.move_forward(angle);

  // update last angle
  last_angle = angle;
}
