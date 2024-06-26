#include "Drive.hpp"

#include <thread>

#include "globals.hpp"

IMovement &Drive::getInstance() {
  static Drive instance;
  return instance;
}

Drive::Drive()
    : left(Motor::getInstance(PORT_LEFT_MOTOR, INVERT_LEFT_MOTOR)),
      right(Motor::getInstance(PORT_RIGHT_MOTOR, INVERT_RIGHT_MOTOR)) {}

Drive::~Drive() { coast(); }

void Drive::stop() {
  left.stop();
  right.stop();
}

void Drive::coast() {
  left.coast();
  right.coast();
}

void Drive::set_speed(double s) {
  if (std::abs(s) > 1.0) {
    std::cerr << "Drive: speed must be between -1.0 and 1.0" << std::endl;
    return;
  }
  this->speed = s;
}

void Drive::move_forward(double angle) {
  if (std::abs(angle) > 180.0) {
    std::cerr << "Drive: angle must be between -180.0 and 180.0" << std::endl;
    return;
  }

  // calculate effective speed, which is a function of the angle
  double effective_speed =
      (1 - this->speed) / (FORWARD_ANGLE / 2.0) * std::abs(angle) + this->speed;

  // limit effective speed to 90% to avoid drawing to much current
  if (effective_speed > .9) {
    effective_speed = .9;
  }

  double adapted_speed;
  if (std::abs(angle) < FORWARD_ANGLE) {
    // if the angle is considered "forward", use regular calculation for turning
    adapted_speed =
        (FORWARD_SCALE_FACTOR * std::abs(angle) + 1) * effective_speed;
  } else {
    // otherwise, turn in place
    adapted_speed = -effective_speed;
  }

  if (angle < 0) {
    // turn left
    left.set_speed(adapted_speed);
    right.set_speed(effective_speed);
  } else {
    // turn right
    left.set_speed(effective_speed);
    right.set_speed(adapted_speed);
  }
}

void Drive::move_backward() {
  left.set_speed(-this->speed);
  right.set_speed(-this->speed);
}

void Drive::turn(bool _left) {
  // save previous speed
  double prev_speed = this->speed;

  double angle = (_left ? -180 : 180);

  static const int wind_up_time_ms = 1000;
  static const int step_size_ms = 5;
  static const double start_speed = 0.2;
  static const double target_speed = 0.9;

  static const int turn_time_ms = TURN_TIME;

  // set both motors to coast
  coast();
  std::this_thread::sleep_for(std::chrono::milliseconds{500});

  // wind up motors
  WindUp(start_speed, target_speed, wind_up_time_ms,
         wind_up_time_ms / step_size_ms);

  // take start time
  auto start = std::chrono::system_clock::now();
  auto lap = std::chrono::system_clock::now();

  while (std::chrono::duration_cast<std::chrono::milliseconds>(lap - start)
             .count() < turn_time_ms) {
    lap = std::chrono::system_clock::now();
    // drive forward
    move_forward(angle);
    std::this_thread::sleep_for(
        std::chrono::milliseconds{static_cast<int>(step_size_ms / 2.0)});
  }

  blackboard.has_turned = true;

  // restore speed
  this->speed = prev_speed;
}

void Drive::turn_left() {
  std::cout << "Turning left" << std::endl;
  turn(true);
}

void Drive::turn_right() {
  std::cout << "Turning right" << std::endl;
  turn(false);
}

void Drive::WindUp(double wind_start_speed, double wind_target_speed,
                   int wind_time_milliseconds, int wind_step_count) {
  std::thread([wind_start_speed, wind_target_speed, wind_time_milliseconds,
               wind_step_count]() {
    IMovement &drive = Drive::getInstance();
    drive.set_speed(wind_start_speed);
    uint16_t wind_sleep = wind_time_milliseconds / wind_step_count;
    double stepSize =
        (wind_target_speed - wind_start_speed) / ((double)wind_step_count);
    for (int step = 0; step <= wind_step_count; ++step) {
      double value = wind_start_speed + (step * stepSize);
      drive.set_speed(value);
      std::this_thread::sleep_for(std::chrono::milliseconds(wind_sleep));
    }
  }).detach();
}

void Drive::WindDown(double wind_start_speed, double wind_target_speed,
                     int wind_time_milliseconds, int wind_step_count) {
  std::thread([wind_start_speed, wind_target_speed, wind_time_milliseconds,
               wind_step_count]() {
    IMovement &drive = Drive::getInstance();
    drive.set_speed(wind_start_speed);
    uint16_t wind_sleep = wind_time_milliseconds / wind_step_count;
    double stepSize =
        (wind_target_speed - wind_start_speed) / ((double)wind_step_count);
    for (int step = 0; step <= wind_step_count; ++step) {
      double value = wind_target_speed - (step * stepSize);
      std::this_thread::sleep_for(std::chrono::milliseconds(wind_sleep));
      drive.set_speed(value);
    }
  }).detach();
}
