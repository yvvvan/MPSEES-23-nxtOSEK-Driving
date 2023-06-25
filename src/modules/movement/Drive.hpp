#ifndef BUILDHAT_SRC_BUILDHAT_DRIVE_HPP_
#define BUILDHAT_SRC_BUILDHAT_DRIVE_HPP_

#include "Motor.hpp"

#include "modules/movement/IMovement.hpp"
#include "communication/internal/BlackBoard.hpp"

/**
 * @brief abstraction for drive access
 *
 */
class Drive : public IMovement {
 GEORDI_PRIVATE:
  /**
   * @brief Construct a new Drive object
   *
   */
  Drive();

  IMotor &left;
  IMotor &right;

  BlackBoard &blackboard = BlackBoard::getInstance();

  // the internal speed for the whole drive
  double speed = 1.0;

  void turn(bool left);

  static void WindUp(double wind_start_speed, double wind_target_speed, int wind_time_milliseconds, int wind_step_count);
  static void WindDown(double wind_start_speed, double wind_target_speed, int wind_time_milliseconds, int wind_step_count);

 GEORDI_PUBLIC:
  static IMovement &getInstance();

  ~Drive() override = default;

  /**
   * @brief set the drive speed, -1.0 to 1.0
   *
   * @param speed
   */
  void set_speed(double speed) override;

  /**
   * @brief hard brake
   *
   */
  void stop() override;

  /**
  * @brief set drive to coast
  *
  */
  void coast() override;

  /**
   * @brief drive the robot forward along a given angle
   *
   * @param angle the angle in degrees
   */
  void move_forward(double angle) override;

  /**
   * @brief turn left
   */
  void turn_left() override;

  /**
   * @brief turn right
   */
  void turn_right() override;
};

#endif //BUILDHAT_SRC_BUILDHAT_DRIVE_HPP_
