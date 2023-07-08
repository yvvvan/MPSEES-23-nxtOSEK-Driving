#ifndef BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
#define BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_

#include "globals.hpp"

#include "SmartMember.hpp"
#include "modules/color_sensor/ColorTypes.hpp"

#include <array>
#include <mutex>
#include <vector>
#include <optional>

/**
 * @brief common, synchronised data exchange
 *
 */
class BlackBoard {
 GEORDI_PUBLIC:
  /**
   * @brief Get the Instance object
   *
   * @return BlackBoard&
   */
  static BlackBoard &getInstance();

  // Delete the copy constructor and copy assignment operator
  BlackBoard(BlackBoard const &) = delete;
  BlackBoard &operator=(BlackBoard const &) = delete;

  /**** add more member variables here ****/

  // general thread control
  SmartMember<bool> running{true};                                  // set false to stop all threads

  SmartMember<bool> buildHatReady{false};

  // Control
  SmartMember<bool> has_turned;                                     // flag indicating that the car has turned

  // Color sensor
  SmartMember<bool> color_sensor_ready{false};             // flag indicating that the color sensor is ready

  SmartMember<std::vector<Color::ColorRange> > colors;              // vector of colors
  SmartMember<std::optional<Color::ColorRange> > current_color;     // current color
  SmartMember<std::optional<std::string> > target_color_name;       // target color
  SmartMember<bool> on_target_color{false};                           // flag indicating that the car is on the target color

  // Lane detection
  SmartMember<bool> lane_detection_ready{false};                    // flag indicating that the lane detection is ready

  SmartMember<int> lane_count;                                      // count the lanes
  SmartMember<bool> has_left_lane;                                  // check if left lane exists
  SmartMember<bool> has_right_lane;                                 // check if right lane exists
  SmartMember<bool> is_dead_end;                                    // check if dead end exists
  SmartMember<bool> is_intersection;                                // check if intersection exists
  SmartMember<double> offset_middle_line;                           // current offset to middle line
  SmartMember<double> distance_intersection;                        // distance to intersection
  SmartMember<std::array<bool, 3>> exits_intersection;              // left, middle, right
  SmartMember<std::array<double, 3>> exits_distance_intersection;   // left, middle, right distance
  // END Lane detection

  /******* end of member variables *******/

 GEORDI_PRIVATE:
  BlackBoard() = default;
  ~BlackBoard() = default;

  static BlackBoard *instance;
  static std::mutex instance_mutex_;
};

#endif //BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
