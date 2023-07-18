#ifndef BUILDHAT_SRC_CONTROL_DS4_HPP_
#define BUILDHAT_SRC_CONTROL_DS4_HPP_

#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <shared_mutex>
#include <valarray>
#include <thread>
#include <atomic>

#include "utils/Timer.hpp"

// ds4 remote_control port (HumanInterfaceDevicePort)
constexpr int L2CAP_PSM_HIDP_CTRL = 0x11;
// ds4 report port
constexpr int L2CAP_PSM_HIDP_INTR = 0x13;

constexpr int REPORT_ID = 0x11;
constexpr size_t REPORT_SIZE = 79;

constexpr int DS4_REMOTE_BUTTON_COOLDOWN = 300;

constexpr int DS4_TERNARY_TRACKPAD0_ACTIVE = 0;
constexpr int DS4_TERNARY_TRACKPAD1_ACTIVE = 1;
constexpr int DS4_TERNARY_DPAD_NORTH = 2;
constexpr int DS4_TERNARY_DPAD_SOUTH = 3;
constexpr int DS4_TERNARY_DPAD_WEST = 4;
constexpr int DS4_TERNARY_DPAD_EAST = 5;
constexpr int DS4_TERNARY_BUTTON_CROSS = 6;
constexpr int DS4_TERNARY_BUTTON_CIRCLE = 7;
constexpr int DS4_TERNARY_BUTTON_SQUARE = 8;
constexpr int DS4_TERNARY_BUTTON_TRIANGLE = 9;
constexpr int DS4_TERNARY_BUTTON_LEFT1 = 10;
constexpr int DS4_TERNARY_BUTTON_RIGHT1 = 11;
constexpr int DS4_TERNARY_BUTTON_LEFT2 = 12;
constexpr int DS4_TERNARY_BUTTON_RIGHT2 = 13;
constexpr int DS4_TERNARY_BUTTON_LEFTSTICK = 14;
constexpr int DS4_TERNARY_BUTTON_RIGHTSTICK = 15;
constexpr int DS4_TERNARY_BUTTON_SHARE = 16;
constexpr int DS4_TERNARY_BUTTON_OPTIONS = 17;
constexpr int DS4_TERNARY_BUTTON_TRACKPAD = 18;
constexpr int DS4_TERNARY_BUTTON_PS = 19;

constexpr int DS4_HID_ENTRY_TERNARY_COUNT = 20;

constexpr int DS4_DIGITAL_PLUG_USB = 20;
constexpr int DS4_DIGITAL_PLUG_AUDIO = 21;
constexpr int DS4_DIGITAL_PLUG_MICROPHONE = 22;
constexpr int DS4_DIGITAL_IS_CONNECTED = 23;

constexpr int DS4_HID_ENTRY_DIGITAL_COUNT = 4;

constexpr int DS4_ANALOG_BATTERY = 24;
constexpr int DS4_ANALOG_LEFT_STICK_X = 25;
constexpr int DS4_ANALOG_LEFT_STICK_Y = 26;
constexpr int DS4_ANALOG_RIGHT_STICK_X = 27;
constexpr int DS4_ANALOG_RIGHT_STICK_Y = 28;
constexpr int DS4_ANALOG_LEFT2 = 29;
constexpr int DS4_ANALOG_RIGHT2 = 30;
constexpr int DS4_ANALOG_ACCELERATION_Y = 31;
constexpr int DS4_ANALOG_ACCELERATION_X = 32;
constexpr int DS4_ANALOG_ACCELERATION_Z = 33;
constexpr int DS4_ANALOG_ORIENTATION_ROLL = 34;
constexpr int DS4_ANALOG_ORIENTATION_YAW = 35;
constexpr int DS4_ANALOG_ORIENTATION_PITCH = 36;
constexpr int DS4_ANALOG_TIMESTAMP = 37;
constexpr int DS4_ANALOG_TRACKPAD0_TOUCH_ID = 38;
constexpr int DS4_ANALOG_TRACKPAD0_TOUCH_X = 39;
constexpr int DS4_ANALOG_TRACKPAD0_TOUCH_Y = 40;
constexpr int DS4_ANALOG_TRACKPAD1_TOUCH_ID = 41;
constexpr int DS4_ANALOG_TRACKPAD1_TOUCH_X = 42;
constexpr int DS4_ANALOG_TRACKPAD1_TOUCH_Y = 43;

constexpr int DS4_HID_ENTRY_ANALOG_COUNT = 20;

constexpr int DS4_HID_ENTRY_TOTAL_COUNT = 44;

// button / switch states
constexpr int ON = 1;
constexpr int OFF = 0;

#define UP     OFF
#define DOWN    ON
constexpr int HELD = 2;

// warning, heuristic data: "good enough"
constexpr int TRACKPAD_MAX_X = 959;
constexpr int TRACKPAD_MAX_Y = 466;
constexpr double TRACKPAD_ANGLE_TOP_RIGHT = (-64.08382462);
constexpr double TRACKPAD_ANGLE_BOTTOM_RIGHT = (-115.91617538);
constexpr double TRACKPAD_ANGLE_TOP_LEFT = 64.08382462;
constexpr double TRACKPAD_ANGLE_BOTTOM_LEFT = 115.91617538;

constexpr int MAX_RECONNECTS = 5;
constexpr int RECONNECT_COOLDOWN_S = 5;
constexpr int MIN_DISCONNECT_COUNT = 100;

// Error codes
constexpr int ERROR_UNIDENTIFIED_BOARD = (-1);
constexpr int ERROR_NO_NETWORK_CONNECTION = (-2);
constexpr int ERROR_NO_BLUETOOTH_CONNECTION = (-3);
constexpr int ERROR_NO_HCITOOL = (-4);

// joystick remote_control values
constexpr double CONTROLLER_SPEED_ADJUSTMENT = 0.7;
constexpr double CONTROLLER_DEAD_ZONE = 0.1;

/**
 * @brief The ControllerState struct
 * This struct holds the state of the controller.
 * It is used to send the state to the robot.
 */
struct ControllerState {
  uint8_t bigRumble = 0;
  uint8_t smallRumble = 0;
  uint8_t ledRed = 0;
  uint8_t ledGreen = 128;
  uint8_t ledBlue = 0;
  uint8_t flashOnTime = 0;
  uint8_t flashOffTime = 0;

  bool tryReconnect = false;
  bool disconnectRequested = false;
};

/**
 * @brief The AnalogStick struct
 * This struct holds the state of an analog stick.
 * It is used to calculate the angle and scale of the stick.
 */
struct AnalogStick {
  double x;
  double y;

  [[nodiscard]] double angle() const;;

  [[nodiscard]] double scale() const;;
};

/**
 * @brief The AnalogTrackpad struct
 * This struct holds the state of an analog trackpad.
 * It is used to calculate the angle and scale of the trackpad.
 * The trackpad is used to control the robot's speed and angle.
 */
struct AnalogTrackpad {
  double x;
  double y;

  bool active;

  [[nodiscard]] double angle() const;;

  [[nodiscard]] double scale() const;;
};

/**
 * @brief The Controller class
 * This class is used to read the state of a DualShock 4 controller.
 * It uses the hidraw interface to read the state of the controller.
 * The hidraw interface is used to avoid the need for root privileges.
 */
class Controller {
 GEORDI_PUBLIC:
  Controller();

  ~Controller();

  int execute();

  bool isAvailable() const;

  void disconnect(bool _reconnect = false);

  void idle(bool _idle);

  double getAngle();

  double getSpeed();

  int readHIDReport();

  bool run = false;

 GEORDI_PRIVATE:
  void initialize();

  ControllerState state;

  AnalogStick leftStick{};
  AnalogStick rightStick{};

  AnalogTrackpad finger0{};
  AnalogTrackpad finger1{};

  void setControllerLed(const uint8_t &_r, const uint8_t &_g, const uint8_t &_b);

  void setControllerRumble(const uint8_t &_rumbleSmall, const uint8_t &_rumbleBig);

  void onRemoteControl(uint8_t _id, int _value);

  std::shared_mutex scan_m;
  std::shared_mutex recv_m;
  std::shared_mutex send_m;
  std::shared_mutex connect_m;
  std::shared_mutex idle_m;

  bool isIdle = false;

  size_t ctlStatus = -1;
  int intStatus = -1;

  int ds4In = -1;
  int ds4Out = -1;

  epoll_event ev{};
  int epfd = -1;

  int invalidCount = -1;

  std::vector<std::string> availableControllers;

  sockaddr_l2 ds4InAddr = {0};
  sockaddr_l2 ds4OutAddr = {0};

  int retryCount{};
  std::atomic<bool> volatile mutable available{};
  bool lowBattery{};
  bool volatile mutable allowConnection{};

  Timer lowBatteryRumbleTimer{};

  u_char *hidBuffer = new u_char[REPORT_SIZE];

  struct HIDEntry {
    int value = UINT32_MAX;
    Timer time;

    void resetTimer();

    double getTimeMS();
  };

  std::array<HIDEntry, DS4_HID_ENTRY_TOTAL_COUNT> latestReport{};
  std::array<HIDEntry, DS4_HID_ENTRY_TOTAL_COUNT> currentReport{};
  std::array<HIDEntry, DS4_HID_ENTRY_TOTAL_COUNT> actualFactualReport{};

  int updateSettings();

  int initiateConnection();

  int scanForControllers();

  std::thread *idleThread = nullptr;

  void rgbw();

  volatile mutable bool stopped = false;
};

#endif //BUILDHAT_SRC_CONTROL_DS4_HPP_
