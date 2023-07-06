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
  // set the speed of the car
  drive.set_speed(CAR_SPEED);

  // prepare color sensor
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

void RobotController::processColorSensor() {
  bool found_color = false;
  auto color = colorSensor.get_color();
  if (color.sat > COLOR_MIN_SAT && color.val > COLOR_MIN_VAL) {
    for (auto const &range : blackBoard.colors.get()) {
      if (range.contains(color, true)) {
        blackBoard.current_color = range;
        found_color = true;

        //std::cout << "I see the color: " << range.name << std::endl;

        // check whether the car is on the target color, if any
        auto target_color = blackBoard.target_color.get();
        blackBoard.on_target_color.set(target_color && range == target_color.value());
        break;
      }
    }
  }

  if (!found_color) {
    blackBoard.current_color.set(std::nullopt);
    blackBoard.on_target_color.set(false);
  }
}

double RobotController::getProcessedLaneAngle() {
  // create a new PD controller
  static PDController pd_controller(KP, KD);

  // get the angle from the blackboard
  double angle = blackBoard.offset_middle_line.get();

  // calculate the angle using the PD controller
  double control_signal = pd_controller.calculateControl(angle);

  // adjust the angle
  angle += control_signal;

  // clip angle to -180 to 180
  if (std::abs(angle) > 180) angle = (angle > 0 ? 180 : -180);

  return angle;
}

void RobotController::execute() {
  // only do anything at all when the build hat is ready
  if (!blackBoard.buildHatReady.get()) {
    return;
  }

  // take lap time
  lap = std::chrono::system_clock::now();

  // TODO: remove for actual code
  {
    // check whether ten seconds have elapsed
    if (std::chrono::duration_cast<std::chrono::seconds>(lap - start).count() > 50) {
      // initiate termination of the program
      blackBoard.running = false;
      return;
    }
  }

  // process the color sensor
  processColorSensor();

  /* ++++++++++++++ BEGIN check special conditions BEGIN ++++++++++++++ */

  // check whether the car is on the target color, if so, coast
  if (blackBoard.on_target_color.get()) {
      drive.coast(); // TODO: maybe break?
      return;
  }

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

  /* ++++++++++++++  END  check special conditions  END  ++++++++++++++ */

  // no special conditions apply, just drive according to the line
  double angle = getProcessedLaneAngle();
  drive.move_forward(angle);
}
