#ifndef GEORDI_INCLUDE_MODULES_COLOR_SENSOR_COLORTYPES_HPP_
#define GEORDI_INCLUDE_MODULES_COLOR_SENSOR_COLORTYPES_HPP_

#include <string>
#include <iostream>

// TODO: figure out why they need to be here and not in the cpp file

namespace Color {
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

  [[nodiscard]] bool contains(Color color, bool only_hue = false) const {
    return (color.hue >= min.hue && color.hue <= max.hue) &&
        (only_hue || (color.sat >= min.sat && color.sat <= max.sat && color.val >= min.val && color.val <= max.val));
  }

  friend std::ostream &operator<<(std::ostream &os, ColorRange const &range) {
    return os << "ColorRange " << range.name
              << ": {min: " << range.min.hue << ", " << range.min.sat << ", " << range.min.val
              << ", max: " << range.max.hue << ", " << range.max.sat << ", " << range.max.val << "}";
  }
};

} // namespace Color

#endif //GEORDI_INCLUDE_MODULES_COLOR_SENSOR_COLORTYPES_HPP_
