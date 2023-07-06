#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

// #define USE_ORB_SLAM 1

#ifdef USE_ORB_SLAM
#include <System.h>
#endif

#include <opencv2/opencv.hpp>

#include "blackboard/BlackBoard.hpp"

#define INTERSECTION_SEC_RANGE 1

class Localization {
 public:
  // TODO should this be a singleton?
#ifdef USE_ORB_SLAM
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
#endif
  Localization();

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
  Coordinates driving_tracking(long time_difference);

  /* Function to change the driving direction */
  void adjust_driving_direction(double angle);

  /**
   * @brief Handle angle at intersections
   */
  void handle_intersection(double angle, long time_difference);

  /* Current Driving Direction, starting with (1,0) */
  std::array<double, 2> driving_direction{};

 private:
#ifdef USE_ORB_SLAM
  /* ORB SLAM config files */
  std::string vocabularyFile;
  std::string configFile;

  /* ORB SLAM system */
  ORB_SLAM3::System *slam;
#endif

  double accum_angle;
  long accum_time;
  bool intersection = false;
  bool intersection_driven = false;
  std::chrono::system_clock::time_point time;

  /* Blackboard */
  BlackBoard &blackboard = BlackBoard::getInstance();
};

#endif  // LOCALIZATION_HPP