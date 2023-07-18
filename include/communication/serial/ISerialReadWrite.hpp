#ifndef BUILDHAT_SRC_BUILDHAT_ISERIALREADWRITE_HPP_
#define BUILDHAT_SRC_BUILDHAT_ISERIALREADWRITE_HPP_

#include "globals.hpp"

#include "ISerialRead.hpp"
#include "ISerialWrite.hpp"

class ISerialReadWrite : virtual public ISerialRead, virtual public ISerialWrite {
 GEORDI_PUBLIC:
  ~ISerialReadWrite() override = default;

  /**
   * @brief write data to serial and read the response (in a single transaction)
   *
   * @param data the data to write
   * @param log whether to log the write
   * @param alt if non-empty, log this instead of the data
   * @return std::string the response
   */
  virtual std::string serial_write_read(std::string const &data, bool log = false, std::string const &alt = "") = 0;
};

#endif //BUILDHAT_SRC_BUILDHAT_ISERIALREADWRITE_HPP_
