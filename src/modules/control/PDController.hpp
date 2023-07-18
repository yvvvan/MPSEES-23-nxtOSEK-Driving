#ifndef GEORDI_SRC_MODULES_CONTROL_PDCONTROLLER_HPP_
#define GEORDI_SRC_MODULES_CONTROL_PDCONTROLLER_HPP_

#include <chrono>

class PDController {
private:
  double Kp;  // Proportional gain
  double Kd;  // Derivative gain
  double prevError;  // Previous error

  std::chrono::time_point<std::chrono::system_clock> prevTime;  // Time of previous error

public:
  PDController(double kp, double kd);

  double calculateControl(double error);
};

#endif //GEORDI_SRC_MODULES_CONTROL_PDCONTROLLER_HPP_
