#ifndef GEORDI_SRC_MODULES_COLOR_SENSOR_COLORSENSOR_HPP_
#define GEORDI_SRC_MODULES_COLOR_SENSOR_COLORSENSOR_HPP_

#include "globals.hpp"

#include "communication/serial/BuildHat.hpp"
#include "communication/serial/ISerialReadWrite.hpp"

/**
 * @brief abstraction for color sensor access
 *
 */
class ColorSensor {
 GEORDI_PUBLIC:
  explicit ColorSensor(uint8_t port = PORT_COLOR_SENSOR);

  struct Color {
    int hue;
    int sat;
    int val;

    friend std::ostream &operator<<(std::ostream &os, Color const &color) {
      return os << "{hue: " << color.hue << ", sat: " << color.sat << ", val: " << color.val << "}";
    }
  };

  struct ColorRange {
    std::string name;
    Color min;
    Color max;

    [[nodiscard]] bool contains(Color color) const {
      return color.hue >= min.hue && color.hue <= max.hue &&
          color.sat >= min.sat && color.sat <= max.sat &&
          color.val >= min.val && color.val <= max.val;
    }

    friend std::ostream &operator<<(std::ostream &os, ColorRange const &range) {
      return os << "ColorRange " << range.name
                << ": {min: " << range.min.hue << ", " << range.min.sat << ", " << range.min.val
                << ", max: " << range.max.hue << ", " << range.max.sat << ", " << range.max.val << "}";
    }
  };

  /**
   * @brief get the color of the object in front of the sensor
   *
   * @return Color
   */
  Color get_color();

 GEORDI_PRIVATE:
  bool available = false;
  uint8_t port{};

  ISerialReadWrite &buildHat = BuildHat::getInstance();
};

#endif //GEORDI_SRC_MODULES_COLOR_SENSOR_COLORSENSOR_HPP_
