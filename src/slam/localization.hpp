#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include <System.h>

#include <opencv2/opencv.hpp>

#include "blackboard/BlackBoard.hpp"

class Localization {
 public:
  // TODO should this be a singleton?
  /**
   * @brief Construct a new Localization object
   *
   * @param vocabularyFile path to the ORB_SLAM vocabulary file
   * (ORB_SLAM3/Vocabulary/ORBvoc.txt)
   * @param configFile path to the ORB_SLAM camera config file
   *
   * @return Localization&
   */
  Localization(std::string vocabularyFile, std::string configFile);

  /**
   * @brief Destroy the Localization object
   */
  ~Localization();

  /**
   * @brief Runs the localization thread
   *
   * @return int 0 on success, -1 on failure
   */
  int exec_thread();

  /**
   * @brief reset the clock, to start tracking
   */
  void reset_clock();

  /**
   * @brief set the Coordinates according to the driving direction
   */
  Coordinates driving_tracking();

 private:
  /* ORB SLAM config files */
  std::string vocabularyFile;
  std::string configFile;

  /* ORB SLAM system */
  ORB_SLAM3::System *slam;

  /* Current Driving Direction, starting with (1,0) */
  array<double, 2> driving_direction = {1, 0};
  double angle = 0;
  std::chrono::system_clock::time_point time = std::chrono::system_clock::now();

  /* Blackboard */
  BlackBoard &blackboard = BlackBoard::getInstance();
};

#endif  // LOCALIZATION_HPP