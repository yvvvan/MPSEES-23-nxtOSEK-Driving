#include <array>
#include <cmath>
#include <thread>

#include "modules/color_sensor/ColorSensor.hpp"

void calibrate_colors() {
  ColorSensor colorSensor;

  const char *homeDir = std::getenv("HOME");
  std::string filePath = std::string(homeDir) + "/calibration.txt";

  std::ofstream file;
  file.open(filePath);

  auto calibrateColor = [&file, &colorSensor](const std::string &colorName) {
    std::cout << "Calibrating " << colorName << ": Press enter when ready." << std::endl;
    std::cin.get();

    std::array<int, 5> hue{};
    std::array<int, 5> sat{};
    std::array<int, 5> val{};
    double avg_hue = 0;
    double avg_sat = 0;
    double avg_val = 0;
    double hue_sd = 0;
    double sat_sd = 0;
    double val_sd = 0;

    // take five measurements over one second
    for (int i = 0; i < 5; i++) {
      auto color = colorSensor.get_color();
      hue[i] = color.hue;
      sat[i] = color.sat;
      val[i] = color.val;
      avg_hue += color.hue;
      avg_sat += color.sat;
      avg_val += color.val;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // calculate average#include <cmath>
    avg_hue /= 5;
    avg_sat /= 5;
    avg_val /= 5;

    // calculate first standard deviation
    for (int i = 0; i < 5; i++) {
      hue_sd += std::pow(hue[i] - avg_hue, 2);
      sat_sd += std::pow(sat[i] - avg_sat, 2);
      val_sd += std::pow(val[i] - avg_val, 2);
    }

    hue_sd = std::sqrt(hue_sd / 5);
    sat_sd = std::sqrt(sat_sd / 5);
    val_sd = std::sqrt(val_sd / 5);

    // print color with +- standard deviation to file
    file << colorName << std::endl;
    file << (avg_hue - hue_sd) << " " << (avg_hue + hue_sd) << std::endl;
    file << (avg_sat - sat_sd) << " " << (avg_sat + sat_sd) << std::endl;
    file << (avg_val - val_sd) << " " << (avg_val + val_sd) << std::endl;
  };

  calibrateColor("red");
  calibrateColor("blue");
  calibrateColor("green");
  calibrateColor("orange");

  file.close();

  std::cout << "Calibration complete." << std::endl;
}

std::vector<ColorSensor::ColorRange> parse_calibration_file() {
  std::ifstream file;
  file.open("calibration.txt");

  std::vector<ColorSensor::ColorRange> colors;

  std::string line;
  while (std::getline(file, line)) {
    ColorSensor::ColorRange color;
    color.name = line;
    std::getline(file, line);
    std::istringstream iss(line);
    iss >> color.min.hue >> color.max.hue;
    std::getline(file, line);
    iss = std::istringstream(line);
    iss >> color.min.sat >> color.max.sat;
    std::getline(file, line);
    iss = std::istringstream(line);
    iss >> color.min.val >> color.max.val;
    std::cout << color << std::endl;
    colors.push_back(color);
  }

  file.close();

  return colors;
}

// load calibration file, then ask for each color by name, check if measurement is within range
bool test_calibration() {
  auto colors = parse_calibration_file();

  ColorSensor colorSensor;

  for (auto &color : colors) {
    std::cout << "Testing " << color.name << ": Press enter when ready." << std::endl;
    std::cin.get();

    auto measuredColor = colorSensor.get_color();

    if (color.contains(measuredColor)) {
      std::cout << "Measurement within range." << std::endl;
    } else {
      std::cout << "Measurement out of range: " << measuredColor << " vs " << color << std::endl;
    }
  }

  return true;
}

int main() {
  calibrate_colors();
  test_calibration();
  return 0;
}