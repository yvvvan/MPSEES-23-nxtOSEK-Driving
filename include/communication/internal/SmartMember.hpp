#ifndef BUILDHAT_SRC_BLACKBOARD_SMARTMEMBER_HPP_
#define BUILDHAT_SRC_BLACKBOARD_SMARTMEMBER_HPP_

#include <mutex>
#include <atomic>
#include <iostream>
#include <shared_mutex>

#include "globals.hpp"

/**
 * @brief wrapper for data to be put into the blackboard
 * enables thread safe access and sync in a more or less easy to use interface
 * 
 * @tparam T the actual type of the data to store
 */
template<typename T>
class SmartMember {
  GEORDI_PRIVATE:
      T value;
  std::atomic<int> readCount;
  mutable std::shared_mutex mutex;

  GEORDI_PUBLIC:
      /**
       * @brief Construct a new Smart Member object
       * readCount is initialized to 0
       *
       */
      SmartMember()
  : readCount(0) {}

  /**
   * @brief Construct a new Smart Member object with a start value
   * readCount is initialized to 0
   * 
   * @param initVal the value to store
   */
  explicit SmartMember(const T &initVal) : value(initVal), readCount(0) {}

  /**
   * @brief gets the value
   * increases readCount by one
   * 
   * @return T the value
   */
  T get() {
    std::shared_lock<std::shared_mutex> lock(mutex);
    readCount.fetch_add(1, std::memory_order_relaxed);
    return value;
  }

  /**
   * @brief Gets the value into the out-parameter if it is new
   * increases readCount by one
   * 
   * @param result the value or previous, if not new
   * @return 0 when read was successful, else if value was not new
   */
  int getIfNotYetRead(T &result) {
    int count = readCount.load(std::memory_order_acquire);
    if (count > 0) {
      return 1; // Already read
    }

    std::unique_lock<std::shared_mutex> lock(mutex);
    count = readCount.load(std::memory_order_relaxed);
    if (count > 0) {
      return 1; // Already read
    }

    readCount.fetch_add(1, std::memory_order_relaxed);
    result = value;
    return 0; // Success
  }

  /**
   * @brief updates the value
   * resets readCount to 0
   * 
   * @param newValue the new value
   */
  void set(T const &newValue) {
    std::unique_lock<std::shared_mutex> lock(mutex);
    value = newValue;
    readCount.store(0, std::memory_order_release);
  }

  /**
   * @brief value compare
   * 
   * @param other value to compare to
   */
  bool operator==(T const &other) const {
    std::shared_lock<std::shared_mutex> lock(mutex);
    return value == other;
  }

  /**
   * @brief value compare
   * 
   * @param other value to compare to
   */
  bool operator!=(T const &other) const {
    std::shared_lock<std::shared_mutex> lock(mutex);
    return value != other;
  }

  /**
   * @brief value set convenience by using assign
   * 
   * @param newValue value to assign
   * @return SmartMember& itself
   */
  SmartMember &operator=(T const &newValue) {
    set(newValue);
    return *this;
  }
};

#endif //BUILDHAT_SRC_BLACKBOARD_SMARTMEMBER_HPP_
