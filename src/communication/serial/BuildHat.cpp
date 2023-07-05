#include "BuildHat.hpp"

/* own headers */
#include "globals.hpp"
#include "remote_control/utils/Utilities.hpp"
#include "communication/internal/BlackBoard.hpp"

/* library headers */
#include <thread>

/* singleton implementation */

BuildHat *BuildHat::instance = nullptr;
std::mutex BuildHat::instance_mutex_;
std::recursive_mutex BuildHat::serial_access_mutex_;

ISerialReadWrite &BuildHat::getInstance() {
  // Double-Checked Locking optimization
  if (instance == nullptr) {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (instance == nullptr) {
      instance = new BuildHat();
    }
  }
  return *instance;
}

/********************* implementation *********************/

BuildHat::BuildHat() :
    serial_stream{SERIAL_DEVICE}, state{HatState::OTHER}, ready{false} {
  ready = true;

  if (!serial_stream.IsOpen() || update_hat_state() != 0) {
    ready = false;
    throw std::runtime_error("BuildHat: error initialising serial");
  }

  // turn echo off
  if (state == HatState::BOOTLOADER) {
    std::cout << "BuildHat: bootloader detected, flashing firmware..." << std::endl;
    if (load_firmware() != 0 || update_hat_state() != 0 && state != HatState::FIRMWARE) {
      ready = false;
      throw std::runtime_error("BuildHat: error loading firmware");
    }
  } else if (state == HatState::OTHER) {
    ready = false;
    throw std::runtime_error("BuildHat: unknown state");
  }

  // if we rebooted, turn echo off again
  serial_write_line("echo 0", false);

  // check voltage reported by BuildHat
  std::cout << "Voltage at buildhat: " << serial_write_read("vin", false) << std::endl;

  // clear any faults
  serial_write_line("clear_faults", false);

  ready = true;

  BlackBoard::getInstance().buildHatReady = true;
}

BuildHat::~BuildHat() {
  serial_stream.Close();
}

int BuildHat::update_hat_state() {
  // constants used for serial communication parsing
  const std::string FIRMWARE = "Firmware version: ";
  const std::string BOOTLOADER = "BuildHAT bootloader version";

  int inc_data = 0;
  while (true) {
    std::string line = serial_write_read("version", false);

    if (line.find(FIRMWARE) != std::string::npos) {
      std::cout << "BuildHAT has firmware: " << line << std::endl;
      state = HatState::FIRMWARE;
      break;
    } else if (line.find(BOOTLOADER) != std::string::npos) {
      state = HatState::BOOTLOADER;
      break;
    } else {
      inc_data++;
      std::cerr << "Error: unknown data received from BuildHAT: " << line << std::endl;
      if (inc_data > 20) {
        return -1;
      } else {
        // got data we don't understand, try again
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }
    }
  }

  return 0;
}

int BuildHat::load_firmware() {
  // get the path to the data directory
  fs::path data = fs::path("../../firmware/");

  // get the paths to the firmware, signature, and version files
  fs::path firm = data / "firmware.bin";
  fs::path sig = data / "signature.bin";
  fs::path ver = data / "version";

  // read the version number from the version file
  std::ifstream version_file(ver);
  uint32_t version = 0;
  if (version_file.is_open()) {
    version_file >> version;
    version_file.close();
  } else {
    std::cerr << "Failed to open version file: " << ver << std::endl;
    return 1;
  }

  // read the firmware file into a std::string
  std::ifstream firm_file(firm, std::ios::binary);
  if (!firm_file) {
    std::cerr << "Failed to open firmware file: " << firm << std::endl;
    return 1;
  }
  std::string firmware((std::istreambuf_iterator<char>(firm_file)), std::istreambuf_iterator<char>());
  firm_file.close();

  // read the signature file into a std::string
  std::ifstream sig_file(sig, std::ios::binary);
  if (!sig_file) {
    std::cerr << "Failed to open signature file: " << sig << std::endl;
    return 1;
  }
  std::string signature((std::istreambuf_iterator<char>(sig_file)), std::istreambuf_iterator<char>());
  sig_file.close();

  // clear write buffer
  serial_write_line();
  serial_write_line();

  serial_write_line("version");
  if (serial_stream.IsDataAvailable()) serial_read_line(true);

  // write firmware and signature to serial:
  // first, send clear
  serial_write_line("clear");

  // initiate firmware loading using load command with length and checksum as arguments
  std::string firmware_size = std::to_string(firmware.size());
  std::string firmware_checksum = std::to_string(checksum(firmware));
  std::string load_command = "load " + firmware_size + " " + firmware_checksum;
  serial_write_line(load_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // write firmware bytes to serial
  std::string start_of_frame = "\x02";
  std::string end_of_frame = "\x03";
  std::string firmware_payload = start_of_frame + firmware + end_of_frame;
  std::string signature_payload = start_of_frame + signature + end_of_frame;

  serial_write_line(firmware_payload, true, "--- firmware payload ---");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // write signature bytes to serial
  std::string signature_command = "signature " + std::to_string(signature.size());
  serial_write_line(signature_command);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // now send signature bytes
  serial_write_line(signature_payload, true, "--- signature payload ---");

  // send reboot command
  serial_write_line("reboot");

  // flush serial buffer, tell user
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  while (serial_stream.IsDataAvailable()) {
    serial_read_line(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Flashed firmware, rebooting..." << std::endl;

  // actually wait for reboot
  std::this_thread::sleep_for(std::chrono::milliseconds(3500));

  // read post reboot data
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  while (serial_stream.IsDataAvailable()) {
    serial_read_line(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Firmware flashed successfully" << std::endl;

  return 0;
}

uint32_t BuildHat::checksum(std::string const &data) {
  uint32_t u = 1;
  for (char c : data) {
    if ((u & 0x80000000) != 0) {
      u = (u << 1) ^ 0x1D872B41; // CRC32 polynomial
    } else {
      u = u << 1;
    }
    u = (u ^ static_cast<uint8_t>(c)) & 0xFFFFFFFF;
  }
  return u;
}

/********* public interface *********/

void BuildHat::serial_write_line(std::string const &data, bool log, std::string const &alt) {
  if (!ready) {
    std::cerr << "Error: BuildHat not ready" << std::endl;
    return;
  }

  {
    std::lock_guard<std::recursive_mutex> lock(serial_access_mutex_);

    serial_stream << data << '\r' << std::flush;
    serial_stream.DrainWriteBuffer();
  }

  if (log) std::cout << "> " << (alt.empty() ? data : alt) << std::endl;
}

// TODO: add *blocking* flag which determines whether we wait for data (if vs while)
std::string BuildHat::serial_read_line(bool log, std::string const &alt) {
  if (!ready) {
    std::cerr << "Error: BuildHat not ready" << std::endl;
    return {};
  }

  std::lock_guard<std::recursive_mutex> lock(serial_access_mutex_);

  std::string line;
  while (line.empty() && std::getline(serial_stream, line)) {
    Utilities::trim(line);
    if (log) std::cout << (alt.empty() ? line : alt) << std::endl;
  }
  return line;
}

std::string BuildHat::serial_write_read(std::string const &data, bool log, std::string const &alt) {
  std::lock_guard<std::recursive_mutex> lock(serial_access_mutex_);
  serial_write_line(data, log, alt);
  return serial_read_line(log, alt);
}
