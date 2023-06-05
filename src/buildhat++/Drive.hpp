#ifndef BUILDHAT_SRC_BUILDHAT_DRIVE_HPP_
#define BUILDHAT_SRC_BUILDHAT_DRIVE_HPP_

#include "BuildHat.hpp"

#include "Motor.hpp"

/**
 * @brief abstraction for drive access
 *
 */
class Drive {
 private:
  Motor left;
  Motor right;

  // the internal speed for the whole drive
  double speed = 1.0;

  bool enabled = false;

 public:
  /**
   * @brief Construct a new Drive object
   *
   */
  Drive();

  /**
   * @brief whether the drive is ready
   *
   * @return true when ready, false else
   */
  [[nodiscard]]  bool isEnabled() const;

  /**
   * @brief set the drive speed, -1.0 to 1.0
   *
   * @param speed
   */
  void set_speed(double speed);

  /**
   * @brief hard brake
   *
   */
  void stop();

  /**
  * @brief set drive to coast
  *
  */
  void coast();

  /**
   * @brief drive the robot forward along a given angle
   *
   * @param angle the angle in degrees
   */
  void forward(double angle);

  /**
   * @brief drive the robot backward along a given angle
   *
   * @param angle the angle in degrees
   */
  void backward(double angle);

  /**
   * @brief scale the drive speed up
   */
  void windUp();

  /**
   * @brief scale the drive speed down
   */
  void windDown();
};

#endif //BUILDHAT_SRC_BUILDHAT_DRIVE_HPP_
