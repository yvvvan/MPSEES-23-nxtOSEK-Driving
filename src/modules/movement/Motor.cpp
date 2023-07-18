#include "Motor.hpp"

#define DEBUG_MOTOR_COMMANDS false

IMotor &Motor::getInstance(uint8_t _id, bool _inverted) {
  // local "hack" to avoid a whole IMotorFactory class (1/2)
  static Motor instance[4] = {Motor(0), Motor(1), Motor(2), Motor(3)};

  if (_id > 3) {
    std::cerr << "Motor: id must be between 0 and 3" << std::endl;
    throw std::invalid_argument("Motor: id must be between 0 and 3");
  }

  // local "hack" to avoid a whole IMotorFactory class (2/2)
  instance[_id].invert(_inverted);

  return instance[_id];
}

Motor::Motor(uint8_t id) {
  if (id > 3) {
    std::cerr << "Motor: id must be between 0 and 3" << std::endl;
    return;
  }

  this->id = id;

  // set pwm limit
  hat.serial_write_line("port " + std::to_string(this->id) + " ; plimit 1.0", DEBUG_MOTOR_COMMANDS);

  // set pwm params TODO: find good values
  hat.serial_write_line("port " + std::to_string(this->id) + " ; pwmparams 0.0001 0.0001", DEBUG_MOTOR_COMMANDS);
}

void Motor::stop() {
  hat.serial_write_line("port " + std::to_string(id) + " ; off", DEBUG_MOTOR_COMMANDS);
}

void Motor::coast() {
  hat.serial_write_line("port " + std::to_string(id) + " ; coast", DEBUG_MOTOR_COMMANDS);
}

void Motor::set_speed(double speed) {
  if (inverted) speed = -speed;

  if (std::abs(speed) > 1.0) {
    std::cerr << "Motor: speed must be between -1.0 and 1.0" << std::endl;
    return;
  }

  hat.serial_write_line("port " + std::to_string(id) + " ; pwm ; set " + std::to_string(speed), DEBUG_MOTOR_COMMANDS);
}

void Motor::invert(bool _inverted) {
  this->inverted = _inverted;
}
