#include "PDController.hpp"
#include "globals.hpp"


PDController::PDController(double kp, double kd)
    : Kp(kp), Kd(kd), prevError(0.0), prevTime(std::chrono::system_clock::now()) {}

double PDController::calculateControl(double error) {
  auto currentTime = std::chrono::system_clock::now();
  auto deltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - prevTime).count();
  double derivative = (error - prevError) / deltaTime;  // Calculate the derivative term
  double controlSignal = Kp * error + Kd * derivative;  // Calculate the control signal

  prevError = error;  // Update the previous error
  prevTime = currentTime;  // Update the previous time

  return controlSignal;
}

