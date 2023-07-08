#ifndef BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
#define BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_

#include "globals.hpp"

#include "SmartMember.hpp"
#include "modules/slam/coordinates.hpp"

#include <opencv2/opencv.hpp>
#include <array>
#include <mutex>

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

  // Control
  SmartMember<bool> has_turned;                                     // flag indicating that the car has turned

  // Lane detection
  SmartMember<bool> lane_detection_ready{false};                    // flag indicating that the lane detection is ready

  SmartMember<int> lane_count;                                      // count the lanes
  SmartMember<bool> is_dead_end;                                    // check if dead end exists
  SmartMember<bool> is_intersection;                                // check if intersection exists
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
  SmartMember<bool> direction_changed; // if the direction needs to be read by motor
  SmartMember<direction_t> direction; // the turning direction at an intersection

  SmartMember<Coordinates> coordinates; // current coordinates returned by localization
  SmartMember<std::array<Coordinates,4>> stations; // coordinates of the 4 stations
  SmartMember<std::map<int , std::array<int, 4>>> connection_map;
  SmartMember<std::map<int , std::array<int, 4>>> duration_map;  

  SmartMember<int> next_instersection; // if mapping is finished, localization will update this value (and coordinates) based on the map

  /* Path Finding*/
  SmartMember<bool> pathfinding_enabled;

  /******* end of member variables *******/

 GEORDI_PRIVATE:
  BlackBoard() = default;
  ~BlackBoard() = default;

  static BlackBoard *instance;
  static std::mutex instance_mutex_;
};

#endif //BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
