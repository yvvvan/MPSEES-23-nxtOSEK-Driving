#ifndef BUILDHAT_SRC_BUILDHAT_MOTOR_HPP_
#define BUILDHAT_SRC_BUILDHAT_MOTOR_HPP_

#include <cstdint>

#include "communication/serial/ISerialWrite.hpp"
#include "communication/serial/BuildHat.hpp"
#include "modules/movement/IMotor.hpp"

/**
 * @brief abstraction for motor access
 * 
 */
class Motor : public IMotor {
 GEORDI_PUBLIC:
  static IMotor &getInstance(uint8_t id, bool inverted = false);

  ~Motor() override = default;

  void stop() override;
  void coast() override;
  void set_speed(double speed) override;

 GEORDI_PRIVATE:
  /**
   * @brief Construct a new Motor object
   *
   * @param id the id on the build hat 0-3; maps to A-D
   * @param inverted whether the motor direction is reverted (depends on how it is constructed)
   */
  explicit Motor(uint8_t id);

  void invert(bool inverted);

  ISerialWrite &hat = BuildHat::getInstance();

  uint8_t id;
  bool inverted = false;
};

#endif //BUILDHAT_SRC_BUILDHAT_MOTOR_HPP_
