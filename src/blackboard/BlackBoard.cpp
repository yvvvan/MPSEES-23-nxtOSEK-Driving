#include "BlackBoard.hpp"

/* singleton implementation */

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