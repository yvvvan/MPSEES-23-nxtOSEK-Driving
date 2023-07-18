#ifndef GEORDI_SRC_MODULES_COLOR_SENSOR_COLORSENSOR_HPP_
#define GEORDI_SRC_MODULES_COLOR_SENSOR_COLORSENSOR_HPP_

#include "globals.hpp"

#include "modules/color_sensor/ColorTypes.hpp"

#include "communication/serial/BuildHat.hpp"
#include "communication/serial/ISerialReadWrite.hpp"

/**
 * @brief abstraction for color sensor access
 *
 */
class ColorSensor {
 GEORDI_PUBLIC:
  explicit ColorSensor(uint8_t port = PORT_COLOR_SENSOR);
  ~ColorSensor();

  static ColorSensor &getInstance();

  /**
   * @brief store calibration file:
   *    - read color from sensor
   *    - write color +- first standard deviation or +- 5% (whichever is larger) to file
   *    - repeat for all (four) colors
   */
  void calibrate();

  /**
   * @brief parse the calibration file
   *
   * @return std::vector<ColorRange>
   */
  static std::vector<Color::ColorRange> parse_calibration_file();

  /**
   * @brief get the color of the object in front of the sensor
   *
   * @return Color
   */
  Color::Color get_color();

 GEORDI_PRIVATE:
  bool available = false;
  uint8_t port{};

  static std::string get_calibration_file_path();

  ISerialReadWrite &buildHat = BuildHat::getInstance();
};

#endif //GEORDI_SRC_MODULES_COLOR_SENSOR_COLORSENSOR_HPP_
