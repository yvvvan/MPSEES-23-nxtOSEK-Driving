#ifndef BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
#define BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_

#include "SmartMember.hpp"
#include "../slam/coordinates.hpp"

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

  /* Localization */
  SmartMember<bool> localization_enabled;
  SmartMember<Coordinates> coordinates;

  /******* end of member variables *******/

 private:
  BlackBoard() = default;
  ~BlackBoard() = default;

  static BlackBoard *instance;
  static std::mutex instance_mutex_;
};

#endif //BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
