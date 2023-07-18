#include <gtest/gtest.h>
#include <signal.h>

#include <fstream>
#include <thread>

#include "remote_control/DS4.hpp"
#include "communication/internal/BlackBoard.hpp"
#include "modules/lane_detection/LaneDetection.hpp"
#include "modules/movement/Drive.hpp"

bool stop = false;

/**
 * @brief Control the robot with the DS4 controller and create a log file
 *        containing the speed, angle, intersection flag, and exit flags as
 *        well as a video of the pi camera. (If #define WRITE_VIDEO 1 is set)
 */
TEST(Controller_Test, ExecutionTest) {
  /* handle keyboard interrupt */
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = [](int s) {
    std::cout << "Caught SIGINT, exiting..." << std::endl;
    stop = true;
  };

  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, nullptr);

  double fps = 20.0;             // Frames per second
  cv::Size frameSize(640, 480);  // Frame size

  cv::Mat image;

  BlackBoard &blackboard = BlackBoard::getInstance();
  auto &drive = Drive::getInstance();
  Controller controller;

  // Create new log file
  std::ofstream logFile("/home/pi/log.txt");

  while (!controller.isAvailable() && !stop) {
    std::this_thread::sleep_for(std::chrono::milliseconds{500});
  }

  std::thread([]() {
    LaneDetection laneDetection(LaneDetectionMode::CAMERA);
    laneDetection.run(nullptr);
  }).detach();

  drive.set_speed(1.0);

  auto time = std::chrono::system_clock::now();
  while (controller.isAvailable() && !stop) {
    controller.execute();
    double angle = controller.getAngle();
    double speed = controller.getSpeed();

    if (speed > 0.3) {
      drive.move_forward(angle);
    } else {
      drive.coast();
    }
    auto curr_time = std::chrono::system_clock::now();
    logFile << speed << ", " << angle << ", "
            << blackboard.is_intersection.get() << ", "
            << blackboard.exits_intersection.get().at(0) << ", "
            << blackboard.exits_intersection.get().at(1) << ", "
            << blackboard.exits_intersection.get().at(2) << ", "
            << std::chrono::duration_cast<std::chrono::milliseconds>(curr_time -
                                                                     time)
                   .count()
            << "\n";
  }

  drive.coast();
  std::this_thread::sleep_for(std::chrono::milliseconds{200});
  logFile.close();
}
