#ifndef GEORDI_INCLUDE_MODULES_MOVEMENT_IMOTOR_HPP_
#define GEORDI_INCLUDE_MODULES_MOVEMENT_IMOTOR_HPP_

#include "globals.hpp"

class IMotor {
 GEORDI_PUBLIC:
  virtual ~IMotor() = default;

  /**
   * @brief set the motor speed, -1.0 to 1.0
   *
   * @param speed
   */
  virtual void set_speed(double speed) = 0;

  /**
   * @brief hard brake
   *
   */
  virtual void stop() = 0;

  /**
   * @brief set drive to coast
   *
   */
  virtual void coast() = 0;
};

#endif //GEORDI_INCLUDE_MODULES_MOVEMENT_IMOTOR_HPP_
