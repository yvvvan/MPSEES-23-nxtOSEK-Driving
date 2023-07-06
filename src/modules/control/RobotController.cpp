#include "RobotController.hpp"

#include <thread>

#include "modules/color_sensor/ColorSensor.hpp"
#include "modules/control/PDController.hpp"

RobotController::RobotController() { init(); }

RobotController::~RobotController() { terminate(); }

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

        // std::cout << "I see the color: " << range.name << std::endl;

        // check whether the car is on the target color, if any
        auto target_color = blackBoard.target_color.get();
        blackBoard.on_target_color.set(target_color &&
                                       range == target_color.value());
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
  if (!blackBoard.buildHatReady.get() || !blackBoard.lane_detection_ready.get() || !blackBoard.color_sensor_ready.get()) {
    return;
  }

  // TODO: remove for actual code
  {
    // take lap time
    lap = std::chrono::system_clock::now();

    // check whether ten seconds have elapsed
    if (std::chrono::duration_cast<std::chrono::seconds>(lap - start).count() > 60) {
      // initiate termination of the program
      blackBoard.running = false;
      return;
    }
  }

  // process the color sensor
  processColorSensor();

  /* ++++++++++++++ BEGIN check special conditions BEGIN ++++++++++++++ */

  static int min_intersection_counter = 10;
  static int min_not_intersection_counter = 10;

  if (blackBoard.is_lower_intersection.get()) {
    intersection_counter++;
    not_intersection_counter = 0;
    if (intersection_counter > min_intersection_counter / 2) {
      drive.set_speed(CAR_MIN_SPEED);
    }

    if (intersection_counter > min_intersection_counter) {
      yeaCommaProbablyAnIntersection = true;
    }
  } else {
    not_intersection_counter++;
    if (intersection_counter <= min_intersection_counter || yeaCommaProbablyAnIntersection) {
      intersection_counter = 0;
    }
  }

  if (yeaCommaProbablyAnIntersection) {
    if (blackBoard.is_lower_intersection.get()) {
      yeaCommaProbablyAnIntersection = false;
      intersection_counter = 0;
      not_intersection_counter = 0;
      drive.set_speed(CAR_SPEED);
      drive.turn_left();
      return;
    }
  }

  static bool on_color = false;
  // check whether the car is on the target color, if so, coast
  if (blackBoard.on_target_color.get()) {
    if (!on_color) {
      on_color = true;
      std::cout << "I am on the target color!" << std::endl;
    }
    drive.coast();
    return;
  } else {
    on_color = false;
  }

  /* ++++++++++++++  END  check special conditions  END  ++++++++++++++ */

  // no special conditions apply, just drive according to the line
  double angle = getProcessedLaneAngle();
  drive.move_forward(angle);
}
