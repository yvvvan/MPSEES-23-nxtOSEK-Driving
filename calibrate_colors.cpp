#include <array>
#include <cmath>
#include <thread>

#include "modules/color_sensor/ColorSensor.hpp"




// load calibration file, then ask for each color by name, check if measurement is within range
bool test_calibration() {
  ColorSensor colorSensor;
  auto colors = colorSensor.parse_calibration_file();

  // tell user what's happening
  std::cout << "> Testing calibration." << std::endl;

  for (auto &color : colors) {
    std::cout << ">> Testing " << color.name << ": Press enter when ready." << std::endl;
    std::cin.get();

    auto measuredColor = colorSensor.get_color();

    if (color.contains(measuredColor)) {
      std::cout << ">>> Measurement within range." << std::endl;
    } else {
      std::cerr << ">>> Measurement out of range: " << measuredColor << " vs " << color << std::endl;
    }
    std::cout << std::endl;
    std::cout << std::endl;
  }

  std::cout << std::endl;
  std::cout << "> Calibration test complete." << std::endl;

  return true;
}

int main() {
  ColorSensor colorSensor;
  colorSensor.calibrate();
  test_calibration();
  return 0;
}