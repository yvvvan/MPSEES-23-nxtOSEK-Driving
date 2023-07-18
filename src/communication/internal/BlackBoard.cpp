#ifndef BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_CPP_
#define BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_CPP_

#include "communication/internal/BlackBoard.hpp"

BlackBoard *BlackBoard::instance = nullptr;
std::mutex BlackBoard::instance_mutex_;

BlackBoard &BlackBoard::getInstance() {
  // Double-Checked Locking optimization
  if (instance == nullptr) {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (instance == nullptr) {
      instance = new BlackBoard();
    }
  }
  return *instance;
}

#endif // BUILDHAT_SRC_BLACKBOARD_BLACKBOARD_CPP_