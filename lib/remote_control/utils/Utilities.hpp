#ifndef BUILDHAT_SRC_UTILS_UTILITIES_HPP_
#define BUILDHAT_SRC_UTILS_UTILITIES_HPP_

#include <string>

/**
 * @brief utility functions
 *
 */
struct Utilities {
  /**
   * @brief execute a command and return the output
   *
   * @param cmd the command to execute
   * @param async whether to execute the command asynchronously
   * @return std::string the output of the command
   */
  static std::string exec(std::string const &cmd, bool async = false);

  /**
   * @brief read a 16 bit integer from a buffer
   *
   * @param buf the buffer to read from
   * @param offset the offset to read from
   * @return int16_t the integer read
   */
  static int16_t read16LE(unsigned char const *buf, int offset);

  static void ltrim(std::string &s);
  static void rtrim(std::string &s);
  static void trim(std::string &s);
};

#endif //BUILDHAT_SRC_UTILS_UTILITIES_HPP_
