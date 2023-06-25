#include "RobotController.hpp"

#include <thread>

void RobotController::run() {
  IMovement &drive = Drive::getInstance();
  BlackBoard &blackBoard = BlackBoard::getInstance();

  // set the speed of the car
  drive.set_speed(.75);

  // store one angle in the past to ensure that the car does not turn too fast
  double last_angle = 0;

  // wait for lane detection start up
  while (!blackBoard.lane_detection_ready.get()) {
    std::this_thread::sleep_for(std::chrono::milliseconds{100});
  }

  // used for time keeping
  std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
  std::chrono::time_point<std::chrono::system_clock> lap;

  while (blackBoard.running.get()) {
    // take lap time
    lap = std::chrono::system_clock::now();

    // check whether ten seconds have elapsed
    if (std::chrono::duration_cast<std::chrono::seconds>(lap - start).count() > 10) {
      // initiate termination of the program
      blackBoard.running = false;
      return;
    }

    // get the angle from the blackboard
    double angle = blackBoard.offset_middle_line.get();

    // calculate the distance between the last angle and the current angle
    double diff = std::abs(angle - last_angle);

    // difference is too big, use middle of both angles
    if (diff > 60) {
      angle = (angle + last_angle) / 2;
    } else {
      // difference is small, use current angle
      last_angle = angle;
    }

    // check whether the angle is too big, if so, set it to the maximum with regard to the sign
    if (std::abs(angle) > 175) angle = angle / std::abs(angle) * 175;

    // use the angle to control the car
    drive.move_forward(angle);
  }

  // coast motors and wait for other threads to finish
  drive.coast();
  std::this_thread::sleep_for(std::chrono::milliseconds{500});
}
