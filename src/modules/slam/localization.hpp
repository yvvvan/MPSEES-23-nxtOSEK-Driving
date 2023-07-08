#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include <opencv2/opencv.hpp>

#include "communication/internal/BlackBoard.hpp"
#include "coordinates.hpp"

#define INTERSECTION_SEC_RANGE 1

class Localization {
 public:
  // TODO should this be a singleton?
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

  double accum_angle;
  long accum_time;
  bool intersection = false;
  bool intersection_driven = false;
  std::chrono::system_clock::time_point time;

  /* Blackboard */
  BlackBoard &blackboard = BlackBoard::getInstance();
};

#endif  // LOCALIZATION_HPP