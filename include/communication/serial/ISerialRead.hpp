#ifndef BUILDHAT_SRC_BUILDHAT_ISERIALREAD_HPP_
#define BUILDHAT_SRC_BUILDHAT_ISERIALREAD_HPP_

#include <string>

#include "globals.hpp"

class ISerialRead {
 GEORDI_PUBLIC:
  virtual ~ISerialRead() = default;

  /**
   * @brief Reads a single line from serial
   *
   * @param log whether to log what was read
   * @param alt if non-empty, log this instead of the read line
   * @return std::string the line read""
   */
  virtual std::string serial_read_line(bool log = false, std::string const &alt = "") = 0;
};

#endif //BUILDHAT_SRC_BUILDHAT_ISERIALREAD_HPP_
