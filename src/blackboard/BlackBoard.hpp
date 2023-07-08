#ifndef BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
#define BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_

#include "SmartMember.hpp"
#include "slam/coordinates.hpp"

#include <mutex>
#include <opencv2/opencv.hpp>

/**
 * @brief common, synchronised data exchange
 * 
 */
class BlackBoard {
 public:
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

  SmartMember<double> speed;
  SmartMember<double> angle;

  /* Camera Frame */
  SmartMember<cv::Mat> frame;
  SmartMember<bool> camera_enabled;

  /* Lane Detection */
  SmartMember<bool> intersection_detected;
  SmartMember<std::array<bool,3>> exits_detected;
  SmartMember<station_t> station;
  

  /* Localization & Mapping */
  SmartMember<bool> localization_enabled;
  SmartMember<bool> mapping_enabled;
  SmartMember<bool> mapping_finished;
  SmartMember<bool> direction_changed; // if the direction needs to be read by motor
  SmartMember<direction_t> direction; // the turning direction at an intersection


  SmartMember<Coordinates> coordinates; // current coordinates returned by localization
  SmartMember<std::array<Coordinates,4>> stations; // coordinates of the 4 stations

  SmartMember<std::map<int , std::array<int, 4>>> cost_exits_map;
  SmartMember<std::map<int , std::array<int, 4>>> connection_map;
  SmartMember<std::map<int , std::array<int, 12>>> duration_map;  

  SmartMember<exit_t> exit; // the exit to take at an intersection, it is just a translation of direction->exit
  SmartMember<int> previous_intersection; 
  SmartMember<int> next_instersection; // if mapping is finished, localization will update this value (and coordinates) based on the map

  /* Path Finding*/
  SmartMember<bool> navigation_enabled;
  SmartMember<std::array<std::array<int,12>,2>> path_found; // path_found[0] = path, path_found[1] = exits

  /******* end of member variables *******/

 private:
  BlackBoard() = default;
  ~BlackBoard() = default;

  static BlackBoard *instance;
  static std::mutex instance_mutex_;
};

#endif //BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
