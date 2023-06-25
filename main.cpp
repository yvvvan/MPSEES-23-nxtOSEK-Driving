#include <thread>

#include "communication/serial/BuildHat.hpp"
#include "communication/serial/ISerialRead.hpp"
#include "modules/movement/IMovement.hpp"
#include "modules/movement/Drive.hpp"
#include "remote_control/DS4.hpp"

void windDriveUp(double wind_start_speed, double wind_target_speed, int wind_time_milliseconds, int wind_step_count) {
  std::thread([wind_start_speed, wind_target_speed, wind_time_milliseconds, wind_step_count]() {
    IMovement &drive = Drive::getInstance();
    drive.set_speed(wind_start_speed);
    uint16_t wind_sleep = wind_time_milliseconds / wind_step_count;
    double stepSize = (wind_target_speed - wind_start_speed) / ((double) wind_step_count);
    for (int step = 0; step <= wind_step_count; ++step) {
      double value = wind_start_speed + (step * stepSize);
      drive.set_speed(value);
      std::this_thread::sleep_for(std::chrono::milliseconds(wind_sleep));
    }
  }).detach();
}

void windDriveDown(double wind_start_speed, double wind_target_speed, int wind_time_milliseconds, int wind_step_count) {
  std::thread([wind_start_speed, wind_target_speed, wind_time_milliseconds, wind_step_count]() {
    IMovement &drive = Drive::getInstance();
    drive.set_speed(wind_start_speed);
    uint16_t wind_sleep = wind_time_milliseconds / wind_step_count;
    double stepSize = (wind_target_speed - wind_start_speed) / ((double) wind_step_count);
    for (int step = 0; step <= wind_step_count; ++step) {
      double value = wind_target_speed - (step * stepSize);
      std::this_thread::sleep_for(std::chrono::milliseconds(wind_sleep));
      drive.set_speed(value);
    }
  }).detach();
}

int main() {
  auto &d = Drive::getInstance();

  d.set_speed(1.0);

  d.move_forward(0);
  std::this_thread::sleep_for(std::chrono::milliseconds{6250});

  d.move_forward(-45);
  std::this_thread::sleep_for(std::chrono::milliseconds{700});

  d.move_forward(-120);
  std::this_thread::sleep_for(std::chrono::milliseconds{800});

  d.move_forward(0);
  std::this_thread::sleep_for(std::chrono::milliseconds{1500});

  d.coast();
  std::this_thread::sleep_for(std::chrono::milliseconds{200});
}

int main2() {
  auto &d = Drive::getInstance();

  d.set_speed(0.2);
  d.move_forward(0);
  std::this_thread::sleep_for(std::chrono::milliseconds{250});
  d.coast();

  (BuildHat::getInstance()).serial_write_line("vin", false);
  std::this_thread::sleep_for(std::chrono::milliseconds{250});
  std::cout << "Voltage at buildhat: " << (BuildHat::getInstance()).serial_read_line(false) << std::endl;

  return 0;
}

int main_controlled() {
  auto &drive{ Drive::getInstance() };
  Controller controller;

  while (!controller.isAvailable()) {
    std::this_thread::sleep_for(std::chrono::milliseconds{500});
  }

  /*(BuildHat::getInstance()).serial_write_line("clear_faults");
  std::cout << "<<< " << (BuildHat::getInstance()).serial_read_line(false) << std::endl;*/

  // save current time
  auto start = std::chrono::system_clock::now();
  auto lap = std::chrono::system_clock::now();

  drive.set_speed(1.0);

  while (controller.isAvailable()) {
    controller.execute();
    double angle = controller.getAngle();
    double speed = controller.getSpeed();

    // take lap time
    lap = std::chrono::system_clock::now();

    // check whether a seconds has elapsed
    if (std::chrono::duration_cast<std::chrono::seconds>(lap - start).count() > 0) {
      // print voltage
      (BuildHat::getInstance()).serial_write_line("vin", false);
      std::cout << "Voltage at buildhat: " << (BuildHat::getInstance()).serial_read_line(false) << std::endl;
      start = lap;
    }

    if (speed > 0.1) {
      drive.move_forward(angle);
    } else {
      drive.coast();
    }
  }

  drive.coast();
  std::this_thread::sleep_for(std::chrono::milliseconds{200});

  return 0;
}

