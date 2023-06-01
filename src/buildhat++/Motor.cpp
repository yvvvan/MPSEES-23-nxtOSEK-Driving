#include "Motor.hpp"

Motor::Motor(uint8_t const &id, bool inverted) {
  if (id > 3) {
    std::cerr << "Motor: id must be between 0 and 3" << std::endl;
    return;
  }

  this->id = id;
  this->inverted = inverted;

  // set pwm limit
  hat.serial_write_line("port " + std::to_string(this->id) + " ; plimit 1");

  // set pwm params TODO: find good values
  hat.serial_write_line("port " + std::to_string(this->id) + " ; pwmparams 0.65 0.01");

  this->enabled = true;
}

bool Motor::isEnabled() const {
  return this->enabled;
}

void Motor::stop() {
  hat.serial_write_line("port " + std::to_string(id) + " ; stop");
}

void Motor::coast() {
  hat.serial_write_line("port " + std::to_string(id) + " ; coast");
}

void Motor::set_speed(double speed) {
  if (inverted) speed = -speed;

  if (std::abs(speed) > 1.0) {
    std::cerr << "Motor: speed must be between -1.0 and 1.0" << std::endl;
    return;
  }

  hat.serial_write_line("port " + std::to_string(id) + " ; pwm ; set " + std::to_string(speed));
}
