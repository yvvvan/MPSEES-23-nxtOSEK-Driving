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
        auto target_color = blackBoard.target_color_name.get();
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
  double raw_angle = angle;

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
  if (!blackBoard.buildHatReady.get() ||
      !blackBoard.lane_detection_ready.get() ||
      !blackBoard.color_sensor_ready.get() ||
      !blackBoard.navigation_enabled.get()) {
    std::cout << "RobotController: waiting for other systems to be ready"
              << std::endl;
    drive.coast();
    return;
  }

  // process the color sensor
  processColorSensor();

  /* ++++++++++++++ BEGIN check special conditions BEGIN ++++++++++++++ */

  static bool on_color = false;
  // check whether the car is on the target color, if so, coast
  if (blackBoard.on_target_color.get()) {
    if (!on_color) {
      on_color = true;
      std::cout << "I am on the target color!" << std::endl;
    }
    this->blackBoard.destination_reached.set(true);
    this->blackBoard.navigation_enabled.set(false);  // TODO necessary
    drive.coast();
    return;
  } else {
    on_color = false;
  }

  static int min_intersection_counter = 10;

  // check for lower intersection
  if (blackBoard.is_lower_intersection.get()) {
    intersection_counter++;
    not_intersection_counter = 0;
    if (intersection_counter > min_intersection_counter / 2) {
      drive.set_speed(CAR_MIN_SPEED);
      blackBoard.intersection_handled.set(false);
    }

    if (intersection_counter > min_intersection_counter) {
      yeaCommaProbablyAnIntersection = true;
    }
  } else {
    not_intersection_counter++;
    if (intersection_counter <= min_intersection_counter ||
        yeaCommaProbablyAnIntersection) {
      intersection_counter = 0;
    }
  }

  // if lower intersection is not detected anymore
  static int probably_intersection_counter = 0;
  if (yeaCommaProbablyAnIntersection &&
      !blackBoard.is_lower_intersection.get()) {
    probably_intersection_counter++;
  }

  // go into intersection handling if intersection
  // was not detected for some frames
  if (probably_intersection_counter > 5) {
    // reset counter
    probably_intersection_counter = 0;
    yeaCommaProbablyAnIntersection = false;
    intersection_counter = 0;
    not_intersection_counter = 0;

    // set next turning direction
    direction_t direction = blackBoard.direction.get();
    if (direction == direction_t::LEFT) {
      drive.turn_left();
    } else if (direction == direction_t::RIGHT) {
      drive.turn_right();
    } else if (direction == direction_t::UNKNOWN) {
      drive.turn_left();
      drive.turn_left();
    }
    std::cout << "Intersection detected "
              << "Direction: " << direction << std::endl;

    drive.set_speed(CAR_SPEED);
    blackBoard.intersection_handled.set(true);
    return;
  }

  // check if no lanes are detected
  static int no_lanes_counter = 0;
  if (blackBoard.lane_count.get() < 1) {
    no_lanes_counter++;
  } else {
    no_lanes_counter = 0;
  }

  // turn left if no lanes are detected
  if (no_lanes_counter > 10) {
    drive.turn_left();
    return;
  }

  /* ++++++++++++++  END  check special conditions  END  ++++++++++++++ */

  // no special conditions apply, just drive according to the line
  double angle = getProcessedLaneAngle();

  log_file.open("/home/pi/sample_log.txt", std::ios::app);
  log_file << angle << " " << 0.6
           << " "  // should be the speed, but it's always the same
           << blackBoard.is_intersection.get() << " "
           << blackBoard.exits_intersection.get()[0] << " "
           << blackBoard.exits_intersection.get()[1] << " "
           << blackBoard.exits_intersection.get()[2] << " "
           << "\n"
           << std::endl;
  log_file.close();

  drive.move_forward(angle);
}
