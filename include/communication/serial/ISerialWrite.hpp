#ifndef BUILDHAT_SRC_BUILDHAT_ISERIALWRITE_HPP_
#define BUILDHAT_SRC_BUILDHAT_ISERIALWRITE_HPP_

#include <string>

#include "globals.hpp"

class ISerialWrite {
 GEORDI_PUBLIC:
  virtual ~ISerialWrite() = default;

  /**
   * @brief Writes a single line to serial and flushes the write buffer
   *
   * @param data data to write, will be appended with \r to indicate end of line
   * @param log whether to log the write
   * @param alt if non-empty, log this instead of the data
   */
  virtual void serial_write_line(std::string const &data = "", bool log = true, std::string const &alt = "") = 0;
};

#endif //BUILDHAT_SRC_BUILDHAT_ISERIALWRITE_HPP_
