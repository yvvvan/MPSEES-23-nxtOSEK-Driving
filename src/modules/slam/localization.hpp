#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include <opencv2/opencv.hpp>

#include "communication/internal/BlackBoard.hpp"
#include "coordinates.hpp"

#define INTERSECTION_SEC_RANGE 1

class Localization {
  // clang-format off
  GEORDI_PUBLIC:

  Localization();
  // clang-format on
  /**
   * @brief Destroy the Localization object
   */
  ~Localization();

  /**
   * @brief reset the clock, to start tracking
   */
  void reset_clock();

  /**
   * @brief set the Coordinates according to the driving direction
   */
  Coordinates track_driving_params(long time_difference);

  /* Current Driving Direction, starting with (1,0) */
  std::array<double, 2> driving_direction{};

  // clang-format off
  GEORDI_PRIVATE:

  /**
   * @brief Handle angle at intersections
   */
  void handle_intersection(double angle, long time_difference);
  // clang-format on

  /* Function to change the driving direction */
  void adjust_driving_direction(double angle);

  double accum_angle;
  long accum_time;
  bool intersection = false;
  bool intersection_driven = false;
  std::chrono::system_clock::time_point time;

  bool intersection_handled_last = false;

  /* Blackboard */
  BlackBoard &blackboard = BlackBoard::getInstance();
};

#endif  // LOCALIZATION_HPP