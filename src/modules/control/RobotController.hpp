#ifndef GEORDI_SRC_MODULES_CONTROL_ROBOTCONTROLLER_HPP_
#define GEORDI_SRC_MODULES_CONTROL_ROBOTCONTROLLER_HPP_

#include "globals.hpp"

#include "modules/movement/Drive.hpp"
#include "communication/internal/BlackBoard.hpp"

/**
 * @brief The RobotController class is responsible for controlling the robot.
 */
class RobotController {
 GEORDI_PUBLIC:
  /**
   * @brief Run the RobotController.
   */
  static void run();
};

#endif //GEORDI_SRC_MODULES_CONTROL_ROBOTCONTROLLER_HPP_
