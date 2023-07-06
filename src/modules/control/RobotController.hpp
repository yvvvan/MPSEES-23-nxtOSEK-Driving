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

  void processColorSensor();

  double getProcessedLaneAngle();

  bool yeaCommaProbablyAnIntersection{false};
  uint8_t intersection_counter{0};
  uint8_t not_intersection_counter{0};

  bool yeaCommaProbablyADeadEnd{false};
  uint8_t dead_end_counter{0};
  uint8_t not_dead_end_counter{0};

  // used for time keeping
  std::chrono::time_point<std::chrono::system_clock> start;
  std::chrono::time_point<std::chrono::system_clock> lap;

  bool should_turn_left = false;
  std::chrono::time_point<std::chrono::system_clock> leftTurnTime;
  bool should_turn_right = false;
  std::chrono::time_point<std::chrono::system_clock> rightTurnTime;

  IMovement &drive = Drive::getInstance();
  BlackBoard &blackBoard = BlackBoard::getInstance();
  ColorSensor &colorSensor = ColorSensor::getInstance();
};

#endif //GEORDI_SRC_MODULES_CONTROL_ROBOTCONTROLLER_HPP_
