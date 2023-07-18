#include "remote_control/DS4.hpp"
#include "modules/movement/Drive.hpp"
#include "communication/internal/BlackBoard.hpp"

int main () {
  auto &blackBoard = BlackBoard::getInstance();
  blackBoard.running = true;

  auto &drive = Drive::getInstance();
  drive.set_speed(0.7);

  Controller controller;

  // wait for controller to become available
  while (!controller.isAvailable()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  while (blackBoard.running.get()) {
    controller.execute();

    auto angle = controller.getAngle();
    auto speed = controller.getSpeed();

    if (speed < 0.1) {
      drive.coast();
      continue;
    }

    if (std::abs(angle) > 160) {
      drive.move_backward();
      continue;
    }

    drive.move_forward(angle);
  }

  return 0;
}