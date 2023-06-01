#ifndef BUILDHAT_SRC_BUILDHAT_MOTOR_HPP_
#define BUILDHAT_SRC_BUILDHAT_MOTOR_HPP_

#include "BuildHat.hpp"

/**
 * @brief abstraction for motor access
 * 
 */
class Motor {
 public:
  /**
   * @brief Construct a new Motor object
   * 
   * @param id the id on the build hat 0-3; maps to A-D
   * @param inverted whether the motor direction is reverted (depends on how it is constructed)
   */
  Motor(uint8_t const &id, bool inverted = false);

  /**
   * @brief 
   * 
   * @return whether the motor is ready
   */
  [[nodiscard]] bool isEnabled() const;

  /**
   * @brief hard brake
   * 
   */
  void stop();

  /**
   * @brief set motor to coast
   * 
   */
  void coast();

  /**
   * @brief Set the motor speed, -1.0 to 1.0
   * 
   * @param speed 
   */
  void set_speed(double speed);

 private:
  BuildHat &hat = BuildHat::getInstance();

  uint8_t id;
  bool inverted = false;

  bool enabled = false;
};

#endif //BUILDHAT_SRC_BUILDHAT_MOTOR_HPP_
