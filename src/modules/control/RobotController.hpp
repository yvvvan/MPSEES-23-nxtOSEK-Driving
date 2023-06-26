#ifndef GEORDI_SRC_MODULES_CONTROL_ROBOTCONTROLLER_HPP_
#define GEORDI_SRC_MODULES_CONTROL_ROBOTCONTROLLER_HPP_

#include "globals.hpp"

#include "modules/movement/Drive.hpp"
#include "modules/color_sensor/ColorSensor.hpp"
#include "communication/internal/BlackBoard.hpp"

/**
 * @brief The RobotController class is responsible for controlling the robot.
 */
class RobotController {
 GEORDI_PUBLIC:
  RobotController();
  ~RobotController();

  /**
   * @brief Execute the RobotController once.
   */
  void execute();

 GEORDI_PRIVATE:
  void init();

  void terminate();

  // used for time keeping
  std::chrono::time_point<std::chrono::system_clock> start;
  std::chrono::time_point<std::chrono::system_clock> lap;

  // store one angle in the past to ensure that the car does not turn too fast
  double last_angle = 0;

  IMovement &drive = Drive::getInstance();
  BlackBoard &blackBoard = BlackBoard::getInstance();
  ColorSensor &colorSensor = ColorSensor::getInstance();
};

#endif //GEORDI_SRC_MODULES_CONTROL_ROBOTCONTROLLER_HPP_
