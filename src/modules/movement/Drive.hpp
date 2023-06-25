#ifndef BUILDHAT_SRC_BUILDHAT_DRIVE_HPP_
#define BUILDHAT_SRC_BUILDHAT_DRIVE_HPP_

#include "communication/serial/ISerialReadWrite.hpp"

#include "Motor.hpp"

#include "modules/movement/IMovement.hpp"

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

  // the internal speed for the whole drive
  double speed = 1.0;

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
};

#endif //BUILDHAT_SRC_BUILDHAT_DRIVE_HPP_
