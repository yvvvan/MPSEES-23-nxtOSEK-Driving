#include "ColorSensor.hpp"

#include <cmath>
#include <thread>
#include <array>

ColorSensor &ColorSensor::getInstance() {
  static ColorSensor instance;
  return instance;
}

ColorSensor::ColorSensor(uint8_t port) {
  if (port > 3) {
    std::cerr << "ColorSensor: port must be between 0 and 3" << std::endl;
    return;
  }

  this->port = port;
  this->available = true;
}

void ColorSensor::calibrate() {
  std::string calibrationFilePath = get_calibration_file_path();
  std::ofstream file;
  file.open(calibrationFilePath);

  if (!file.is_open()) {
    std::cerr << "Could not open file " << calibrationFilePath << std::endl;
    return;
  }

  std::cout << "> Calibrating colors." << std::endl;

  auto calibrateColor = [this, &file](const std::string &colorName) {
    std::cout << ">> Calibrating " << colorName << ": Press enter when ready." << std::endl;
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
      auto color = get_color();
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

  std::cout << std::endl;
  std::cout << "> Calibration complete." << std::endl;
}

std::vector<Color::ColorRange> ColorSensor::parse_calibration_file() {
  std::string calibrationFilePath = get_calibration_file_path();
  std::ifstream file;
  file.open(calibrationFilePath);

  // check if open failed
  if (!file.is_open()) {
    std::cerr << "Failed to open calibration file " << calibrationFilePath << std::endl;
    return {};
  }

  std::vector<Color::ColorRange> colors;

  std::string line;
  while (std::getline(file, line)) {
    Color::ColorRange colorRange;
    colorRange.name = line;
    std::getline(file, line);
    std::istringstream iss(line);
    iss >> colorRange.min.hue >> colorRange.max.hue;
    std::getline(file, line);
    iss = std::istringstream(line);
    iss >> colorRange.min.sat >> colorRange.max.sat;
    std::getline(file, line);
    iss = std::istringstream(line);
    iss >> colorRange.min.val >> colorRange.max.val;
    colors.push_back(colorRange);
  }

  file.close();

  return colors;
}

Color::Color ColorSensor::get_color() {
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
  s = std::sin(hue * DEG_2_RAD);
  c = std::cos(hue * DEG_2_RAD);

  hue = (static_cast<int>((std::atan2(s, c) * RAD_2_DEG) + 360)) % 360;

  return {hue, sat, val};
}

std::string ColorSensor::get_calibration_file_path() {
  const char *homeDir = std::getenv("HOME");
  return std::string(homeDir) + "/calibration.txt";
}
