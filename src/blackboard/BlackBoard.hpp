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
  SmartMember<Coordinates> coordinates;
  SmartMember<direction_t> direction;
  SmartMember<bool> direction_changed;
  SmartMember<std::array<Coordinates,4>> stations;
  SmartMember<bool> mapping_finished;

  SmartMember<std::map<std::tuple<double, double> , std::array<int, 4>>> connection_map;
  SmartMember<std::map<std::tuple<double, double> , std::array<int, 4>>> duration_map;
  SmartMember<int> next_instersection;
  

  /* Path Finding*/
  SmartMember<bool> pathfinding_enabled;

  /******* end of member variables *******/

 private:
  BlackBoard() = default;
  ~BlackBoard() = default;

  static BlackBoard *instance;
  static std::mutex instance_mutex_;
};

#endif //BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
