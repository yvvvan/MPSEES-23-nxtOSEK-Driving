#ifndef BUILDHAT_INCLUDE_MODULES_MOVEMENT_IMOVEMENT_HPP_
#define BUILDHAT_INCLUDE_MODULES_MOVEMENT_IMOVEMENT_HPP_

#include "globals.hpp"

class IMovement {
 GEORDI_PUBLIC:
  virtual ~IMovement() = default;

  /**
   * @brief Moves the robot forward along the given angle
   *
   * @param angle the angle to move along
   */
  virtual void move_forward(double angle) = 0;

  /**
   * @brief Moves the robot backward
   */
  virtual void move_backward() = 0;

  /**
   * @brief set the drive speed, -1.0 to 1.0
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

  /**
   * @brief turn left
   */
  virtual void turn_left() = 0;

  /**
   * @brief turn right
   */
  virtual void turn_right() = 0;
};

#endif //BUILDHAT_INCLUDE_MODULES_MOVEMENT_IMOVEMENT_HPP_
