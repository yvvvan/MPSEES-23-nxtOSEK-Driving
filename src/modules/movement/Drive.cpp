#include "Drive.hpp"

#include "globals.hpp"

IMovement &Drive::getInstance() {
  static Drive instance;
  return instance;
}

Drive::Drive() : left(Motor::getInstance(3)), right(Motor::getInstance(0, true)) {}

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

  double adapted_speed;
  if (std::abs(angle) < FORWARD_ANGLE) {
    // if the angle is considered "forward", use regular calculation for turning
    adapted_speed = (FORWARD_SCALE_FACTOR * std::abs(angle) + 1) * this->speed;
  } else {
    // otherwise, turn in place
    adapted_speed = -this->speed;
  }

  if (angle < 0) {
    // turn left
    left.set_speed(adapted_speed);
    right.set_speed(this->speed);
  } else {
    // turn right
    left.set_speed(this->speed);
    right.set_speed(adapted_speed);
  }
}