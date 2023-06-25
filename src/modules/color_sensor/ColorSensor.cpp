#include "ColorSensor.hpp"

#include <cmath>

ColorSensor::ColorSensor(uint8_t port) {
  if (port > 3) {
    std::cerr << "ColorSensor: port must be between 0 and 3" << std::endl;
    return;
  }

  this->port = port;
}

ColorSensor::Color ColorSensor::get_color() {
  if (!available) {
    std::cerr << "ColorSensor: not available" << std::endl;
    return {};
  }

  // issue data request
  buildHat.serial_write_line("port " + std::to_string(this->port) + " ; selonce 6");

  // read data
  auto data = buildHat.serial_read_line();

  // parse data
  std::istringstream iss(data);
  int hue, sat, val;
  iss >> hue >> sat >> val;

  // decode data
  sat = static_cast<int>((sat / 1024.0) * 100);
  val = static_cast<int>((val / 1024.0) * 100);

  double s, c;
  s = std::sin( hue * DEG_2_RAD);
  c = std::cos( hue * DEG_2_RAD);

  hue = (static_cast<int>((std::atan2(s, c) * RAD_2_DEG) + 360)) % 360;

  return {hue, sat, val};
}
