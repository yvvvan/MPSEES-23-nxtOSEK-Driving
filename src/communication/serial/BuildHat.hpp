#ifndef BUILDHAT_BUILDHAT_HPP
#define BUILDHAT_BUILDHAT_HPP

#include <mutex>
#include <string>
#include <fstream>
#include <iostream>
#include <experimental/filesystem>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "communication/serial/ISerialWrite.hpp"
#include "communication/serial/ISerialRead.hpp"
#include "communication/serial/ISerialReadWrite.hpp"

using namespace LibSerial;

namespace fs = std::experimental::filesystem;

/**
 * @brief abstraction over the serial communication interface
 * providing both read and write, as well as an internal firmware flashing service
 * 
 */
class BuildHat : public ISerialReadWrite {
 GEORDI_PUBLIC:
  /**
   * @brief Get the Instance object
   * 
   * @return BuildHat& 
   */
  static ISerialReadWrite &getInstance();

  // Delete the copy constructor and copy assignment operator
  BuildHat(BuildHat const &) = delete;
  BuildHat &operator=(BuildHat const &) = delete;

  void serial_write_line(std::string const &data = "", bool log = true, std::string const &alt = "") override;
  std::string serial_read_line(bool log = false, std::string const &alt = "") override;
  std::string serial_write_read(std::string const &data, bool log = false, std::string const &alt = "") override;

 GEORDI_PRIVATE:
  /**
   * @brief Construct a new Build Hat object
   * This includes checking the build hat state and, if necessary,
   * flashing the firmware
   * 
   */
  BuildHat();

  /**
   * @brief Destroy the Build Hat object
   * 
   */
  ~BuildHat() override;

  /**
   * @brief HatState, in terms of software flashed
   * 
   */
  enum class HatState : u_int8_t {
    OTHER, FIRMWARE, BOOTLOADER
  };

  /**
   * @brief serial interface
   * 
   */
  SerialStream serial_stream;

  /**
   * @brief the current state of the head
   * 
   */
  HatState state;

  /**
   * @brief whether or not the hat is ready for communication
   * 
   */
  bool ready;

  /* member functions */

  /**
   * @brief Update the hat state by querying the serial interface
   * 
   * @return 0 on successful update, else != 0
   */
  int update_hat_state();

  /**
   * @brief flashes the firmware onto the build hat
   * 
   * @return 0 on successful flash, else != 0
   */
  int load_firmware();

  /**
   * @brief calculates a checksum of the give data, required for firmware flashing
   * 
   * @param data the data to calculate the checksum of
   * @return uint32_t the checksum
   */
  static uint32_t checksum(std::string const &data);

  /**
   * @brief singleton instance
   * 
   */
  static BuildHat *instance;

  /**
   * @brief locks access to mutex creation
   * 
   */
  static std::mutex instance_mutex_;

  /**
   * @brief locks access to serial interface - recursive mutex to allow multiple calls internally
   * 
   */
  static std::recursive_mutex serial_access_mutex_;
};

#endif //BUILDHAT_BUILDHAT_HPP
