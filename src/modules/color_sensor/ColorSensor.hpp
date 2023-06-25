#ifndef GEORDI_SRC_MODULES_COLOR_SENSOR_COLORSENSOR_HPP_
#define GEORDI_SRC_MODULES_COLOR_SENSOR_COLORSENSOR_HPP_

#include "globals.hpp"

#include "communication/internal/BlackBoard.hpp"
#include "communication/serial/ISerialReadWrite.hpp"
#include "communication/serial/BuildHat.hpp"

/**
 * @brief abstraction for color sensor access
 *
 */
class ColorSensor {
 GEORDI_PUBLIC:
  explicit ColorSensor(uint8_t port);

  struct Color {
    int hue;
    int saturation;
    int value;
  };

  /**
   * @brief Construct a new Color Sensor object
   *
   */
  ColorSensor() = default;

  /**
   * @brief Destroy the Color Sensor object
   *
   */
  ~ColorSensor() = default;

  /**
   * @brief get the color of the object in front of the sensor
   *
   * @return Color
   */
  Color get_color();

 GEORDI_PRIVATE:
  bool available = false;
  uint8_t port{};

  BlackBoard &blackBoard = BlackBoard::getInstance();
  ISerialReadWrite &buildHat = BuildHat::getInstance();
};

#endif //GEORDI_SRC_MODULES_COLOR_SENSOR_COLORSENSOR_HPP_
