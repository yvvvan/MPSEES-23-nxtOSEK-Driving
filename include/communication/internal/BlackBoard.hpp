#ifndef BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
#define BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_


#include <array>
#include <mutex>
#include <vector>
#include <optional>
#include <opencv2/opencv.hpp>

#include "SmartMember.hpp"
#include "globals.hpp"
#include "modules/slam/coordinates.hpp"
#include "modules/color_sensor/ColorTypes.hpp"

/**
 * @brief common, synchronised data exchange
 *
 */
class BlackBoard {
  // clang-format off
  GEORDI_PUBLIC:

  /**
   * @brief Get the Instance object
   *
   * @return BlackBoard&
   */
  static BlackBoard &getInstance();
  // clang-format on

  // Delete the copy constructor and copy assignment operator
  BlackBoard(BlackBoard const &) = delete;
  BlackBoard &operator=(BlackBoard const &) = delete;

  /**** add more member variables here ****/

  // general thread control
  SmartMember<bool> running{true};  // set false to stop all threads

  SmartMember<bool> buildHatReady{false};

  // Control
  SmartMember<bool> has_turned;  // flag indicating that the car has turned
  SmartMember<bool> intersection_handled;  // flag indicating that the
                                           // intersection has been handled
  SmartMember<direction_t>
      turn_direction;  // direction to turn to (left, right, straight)

  // Color sensor
  SmartMember<bool> color_sensor_ready{false};             // flag indicating that the color sensor is ready

  SmartMember<std::vector<Color::ColorRange> > colors;              // vector of colors
  SmartMember<std::optional<Color::ColorRange> > current_color;     // current color
  SmartMember<std::optional<std::string> > target_color_name;       // target color
  SmartMember<bool> on_target_color{false};                           // flag indicating that the car is on the target color

  // Lane detection
  SmartMember<bool> lane_detection_ready{
      false};  // flag indicating that the lane detection is ready

  SmartMember<int> lane_count;                                      // count the lanes
  SmartMember<bool> has_left_lane;                                  // check if left lane exists
  SmartMember<bool> has_right_lane;                                 // check if right lane exists
  SmartMember<bool> is_dead_end;                                    // check if dead end exists
  SmartMember<bool> is_intersection;                                // check if intersection exists
  SmartMember<bool> is_lower_intersection;                          // check if intersection exists in the lower half
  SmartMember<double> offset_middle_line;                           // current offset to middle line
  SmartMember<double> distance_intersection;                        // distance to intersection
  SmartMember<std::array<bool, 3>> exits_intersection;              // left, middle, right
  SmartMember<std::array<double, 3>> exits_distance_intersection;   // left, middle, right distance
  // END Lane detection

  /* Camera Frame */
  SmartMember<cv::Mat> frame;
  SmartMember<bool> camera_enabled;

  /* Localization & Mapping */
  SmartMember<double> angle;
  SmartMember<double> speed;
  SmartMember<bool> localization_enabled;
  SmartMember<bool> mapping_enabled;
  SmartMember<bool> mapping_finished;
  SmartMember<bool>
      direction_changed;  // if the direction needs to be read by motor
  SmartMember<direction_t>
      direction;  // the turning direction at an intersection

  SmartMember<Coordinates>
      coordinates;  // current coordinates returned by localization
  SmartMember<std::array<Coordinates, 4>>
      stations;  // coordinates of the 4 stations

  SmartMember<int> last_intersection;  // last intersection
  SmartMember<int>
      next_intersection;  // if mapping is finished, localization will update
                          // this value (and coordinates) based on the map
  SmartMember<std::map<int, std::array<int, 4>>> cost_exits_map;
  SmartMember<std::map<int, std::array<int, 4>>> connection_map;
  SmartMember<std::map<int, std::array<int, 12>>> duration_map;

  SmartMember<exit_t> exit;  // the exit to take at an intersection, it is just
                             // a translation of direction->exit

  /* Path Finding*/
  SmartMember<bool> navigation_enabled;
  SmartMember<int> destination;           // the station to go to
  SmartMember<bool> destination_reached;  // flag indicating that the
                                          // destination has been reached

  /******* end of member variables *******/
  // clang-format off
  GEORDI_PRIVATE:
  BlackBoard() = default;
  ~BlackBoard() = default;
  // clang-format on

  static BlackBoard *instance;
  static std::mutex instance_mutex_;
};

#endif  // BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
