#ifndef BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
#define BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_

#include "globals.hpp"

#include "SmartMember.hpp"

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

  SmartMember<double> speed;
  SmartMember<double> angle;

  /******* end of member variables *******/

 GEORDI_PRIVATE:
  BlackBoard() = default;
  ~BlackBoard() = default;

  static BlackBoard *instance;
  static std::mutex instance_mutex_;
};

#endif //BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_HPP_
