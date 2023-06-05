#include "Drive.hpp"

#include "globals.hpp"

#include <thread>

Drive::Drive() : left(3), right(0, true) {
  enabled = left.isEnabled() && right.isEnabled();
}

void Drive::stop() {
  if (!enabled) return;
  left.stop();
  right.stop();
}

void Drive::coast() {
  if (!enabled) return;
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

bool Drive::isEnabled() const {
  return this->enabled;
}

void Drive::forward(double angle) {
  if (!enabled) return;

  if (std::abs(angle) > 180.0) {
    std::cerr << "Drive: angle must be between -180.0 and 180.0" << std::endl;
    return;
  }

  double adapted_speed = (-10.0 / 9.0) * angle + 100;
  adapted_speed /= 100.0;
  adapted_speed *= this->speed;

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

void Drive::backward(double angle) {
  if (!enabled) return;

  if (std::abs(angle) > 180.0) {
    std::cerr << "Drive: angle must be between -180.0 and 180.0" << std::endl;
    return;
  }

  double adapted_speed = (-10.0 / 9.0) * angle + 100;
  adapted_speed /= 100.0;
  adapted_speed *= this->speed;

  if (angle < 0) {
    // turn left
    left.set_speed(-adapted_speed);
    right.set_speed(-this->speed);
  } else {
    // turn right
    left.set_speed(-this->speed);
    right.set_speed(-adapted_speed);
  }
}

void Drive::windUp() {
  std::thread([this]() {
    uint16_t wind_sleep = WIND_TIME_SECONDS * 1000 / WIND_STEP_COUNT;
    double stepSize = (1.0f - WIND_MIN_SPEED) / ((double) WIND_STEP_COUNT);
    for (int step = 0; step <= WIND_STEP_COUNT; ++step) {
      double value = WIND_MIN_SPEED + (step * stepSize);
      std::this_thread::sleep_for(std::chrono::milliseconds(wind_sleep));
      this->set_speed(value);
    }
  }).detach();
}

void Drive::windDown() {
  std::thread([this]() {
    uint16_t wind_sleep = WIND_TIME_SECONDS * 1000 / WIND_STEP_COUNT;
    double stepSize = (1.0f - WIND_MIN_SPEED) / ((double) WIND_STEP_COUNT);
    for (int step = 0; step <= WIND_STEP_COUNT; ++step) {
      double value = 1.0f - (step * stepSize);
      std::this_thread::sleep_for(std::chrono::milliseconds(wind_sleep));
      this->set_speed(value);
    }
  }).detach();
}
