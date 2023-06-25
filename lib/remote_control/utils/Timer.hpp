#ifndef BUILDHAT_SRC_UTILS_TIMER_HPP_
#define BUILDHAT_SRC_UTILS_TIMER_HPP_

#include <chrono>

#include "globals.hpp"

/**
 * @brief timer class
 *
 */
class Timer {
 GEORDI_PRIVATE:
  std::chrono::system_clock::time_point time;

 GEORDI_PUBLIC:
  /**
   * @brief Construct a new Timer object
   *
   * starts the timer
   */
  Timer();

  /**
   * @brief Construct a new Timer object
   *
   * @param _startTimeMS the start time in milliseconds
   */
  explicit Timer(int _startTimeMS);

  /**
   * @brief start the timer
   *
   */
  void start();

  /**
   * @brief get the time in seconds
   *
   * @return double the time in seconds
   */
  double getTimeS();

  /**
   * @brief get the time in milliseconds
   *
   * @return double the time in milliseconds
   */
  double getTimeMS();

  /**
   * @brief reset the timer
   *
   */
  void reset();
};

#endif //BUILDHAT_SRC_UTILS_TIMER_HPP_
