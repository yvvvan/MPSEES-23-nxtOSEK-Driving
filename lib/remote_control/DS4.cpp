#include "DS4.hpp"

#include "globals.hpp"
#include "utils/Utilities.hpp"

#include <mutex>
#include <sstream>

double AnalogStick::angle() const {
  // make sure "nothing" translates to zero
  // nothing means below an error of about 4%, sadly
  if (scale() < 0.044) return 0;
  double angle = std::atan2(y, x) * 180 / M_PI - 90;
  return -(angle > 180 ? angle - 360 : angle < -180 ? angle + 360 : angle);
}

double AnalogStick::scale() const {
  double temp = std::sqrt(std::pow(y, 2) + std::pow(x, 2));
  // Heuristically: doesn't go over 70% of the theoretical max in all directions
  // Therefore, declare that max (floor(0.7*(sqrt(2*127^2)) = 125)
  return temp > 125 ? 1 : temp / 125;
}

double AnalogTrackpad::angle() const {
  double angle = std::atan2(y, x) * 180 / M_PI - 90;
  return -(angle > 180 ? angle - 360 : angle < -180 ? angle + 360 : angle);
}

double AnalogTrackpad::scale() const {
  double degree = angle();
  double scale = 0;
  double length = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
  double slope = y / x;

  if ((degree > TRACKPAD_ANGLE_TOP_LEFT && TRACKPAD_ANGLE_TOP_RIGHT > degree)
      || (degree > TRACKPAD_ANGLE_BOTTOM_RIGHT || TRACKPAD_ANGLE_BOTTOM_LEFT < degree)) {
    // top or bottom
    scale = length / std::sqrt(std::pow((TRACKPAD_MAX_Y / slope), 2) + std::pow(TRACKPAD_MAX_Y, 2));
  } else {
    // left or right
    scale = length / std::sqrt(std::pow(TRACKPAD_MAX_X, 2) + std::pow(slope * TRACKPAD_MAX_Y, 2));
  }
  return scale;
}

/**********************************************************************************************************************/

void Controller::onRemoteControl(uint8_t _id, int _value) {
  uint8_t l2Value = 0, r2Value = 0;
  static auto rumble_time = Timer();
  if (rumble_time.getTimeS() > 0.5) setControllerRumble(0, 0);
  switch (_id) {
    case DS4_ANALOG_LEFT2: {
      l2Value = _value;
      break;
    }
    case DS4_ANALOG_RIGHT2: {
      r2Value = _value;
      break;
    }
    case DS4_DIGITAL_IS_CONNECTED: {
      break;
    }
    case DS4_TERNARY_BUTTON_LEFTSTICK: {
      break;
    }
    case DS4_TERNARY_BUTTON_TRIANGLE: {
      break;
    }
    case DS4_TERNARY_BUTTON_CIRCLE: {
      break;
    }
    case DS4_TERNARY_BUTTON_CROSS: {
      break;
    }
    case DS4_TERNARY_BUTTON_SQUARE: {
      break;
    }
    case DS4_TERNARY_DPAD_NORTH: {
      break;
    }
    case DS4_TERNARY_DPAD_SOUTH: {
      break;
    }
    case DS4_TERNARY_BUTTON_SHARE: {
      break;
    }
    case DS4_TERNARY_BUTTON_OPTIONS: {
      break;
    }
    case DS4_TERNARY_BUTTON_PS: {
      if (_value == HELD) {
        disconnect(false);
        return;
      }
      break;
    }
    case DS4_ANALOG_LEFT_STICK_X: {
      leftStick.x = _value;
      break;
    }
    case DS4_ANALOG_LEFT_STICK_Y: {
      leftStick.y = _value;
      break;
    }
    case DS4_ANALOG_RIGHT_STICK_X: {
      rightStick.x = _value;
      break;
    }
    case DS4_ANALOG_RIGHT_STICK_Y: {
      rightStick.y = _value;
      break;
    }
    case DS4_ANALOG_TRACKPAD0_TOUCH_X: {
      finger0.x = _value;
      break;
    }
    case DS4_ANALOG_TRACKPAD0_TOUCH_Y: {
      finger0.y = _value;
      break;
    }
    case DS4_ANALOG_TRACKPAD1_TOUCH_X: {
      finger1.x = _value;
      break;
    }
    case DS4_ANALOG_TRACKPAD1_TOUCH_Y: {
      finger1.y = _value;
      break;
    }
    case DS4_TERNARY_TRACKPAD0_ACTIVE: {
      finger0.active = _value;
      break;
    }
    case DS4_TERNARY_TRACKPAD1_ACTIVE: {
      finger1.active = _value;
      break;
    }
    default: {
      //LOG_ERROR("Got unhandled controller event (in menu): " + std::_id));
    }
  }
}

void Controller::setControllerLed(const uint8_t &_r, const uint8_t &_g, const uint8_t &_b) {
  state.ledRed = _r;
  state.ledGreen = _g;
  state.ledBlue = _b;
}

void Controller::setControllerRumble(const uint8_t &_rumbleSmall, const uint8_t &_rumbleBig) {
  state.smallRumble = _rumbleSmall;
  state.bigRumble = _rumbleBig;
}

Controller::Controller() {
  std::thread([this] { initialize(); }).detach();
}

[[noreturn]] void Controller::initialize() {
  while (true) {
    initiateConnection();

    updateSettings();

    while (readHIDReport() != 0);

    LOG("Controller at about " << currentReport[DS4_ANALOG_BATTERY].value << "0% charge.")

    available = true;

    while (execute() == 0);
  }
}

int Controller::initiateConnection() {
  retryCount = 0;
  available = false;
  lowBattery = false;
  allowConnection = true;

  std::unique_lock<std::shared_mutex> lock(scan_m);
  if (!allowConnection) return -1;

  // start out with invalid states
  ctlStatus = -1;
  intStatus = -1;

  // create two sockets, one for reading, one for sending
  ds4In = socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
  ds4Out = socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);

  ds4InAddr.l2_family = AF_BLUETOOTH;
  ds4OutAddr.l2_family = AF_BLUETOOTH;
  ds4InAddr.l2_psm = htobs(L2CAP_PSM_HIDP_INTR);
  ds4OutAddr.l2_psm = htobs(L2CAP_PSM_HIDP_CTRL);

  retryCount = 0;

  LOG("Scanning for devices..")

  while (true) {
    LOG("Scan [" << ++retryCount << "/inf]:")

    int temp = scanForControllers();

    if (temp == ERROR_NO_HCITOOL) return ERROR_NO_HCITOOL;
    else if (temp != -1) break;
  }

  if (availableControllers.empty()) {
    LOG_ERROR("No controller found, abort")
    return disconnect(), -1;
  }

  // attempt to connect to paired controllers
  for (auto &availableController : availableControllers) {
    char temp[18];
    strcpy(temp, availableController.c_str());
    str2ba(temp, &ds4OutAddr.l2_bdaddr);
    str2ba(temp, &ds4InAddr.l2_bdaddr);

    LOG("Attempting connection to [" << temp << "]")

    // attempt connection
    ctlStatus = connect(ds4Out, (struct sockaddr *) &ds4OutAddr, sizeof(ds4OutAddr));
    intStatus = connect(ds4In, (struct sockaddr *) &ds4InAddr, sizeof(ds4InAddr));

    if (ctlStatus == 0 && intStatus == 0) {
      fcntl(ds4In, F_SETFL, O_RDWR | O_NONBLOCK);
      fcntl(ds4Out, F_SETFL, O_RDWR | O_NONBLOCK);
      LOG("Connected to [" << temp << "]")

      available = true;

      break;
    } else {
      DEBUG_PRINT("Unsuccessful: " << errno << " " << strerror(errno))
    }
  }

  if (available) updateSettings();

  // create poller in order to only request hid latestReports when available
  epfd = epoll_create(1023);

  if (epfd == -1) {
    //LOG_ERROR("epoll_create failed");
    available = false;
  }

  ev.events = EPOLLIN;
  ev.data.fd = ds4In;

  if (epoll_ctl(epfd, EPOLL_CTL_ADD, ds4In, &ev) == -1) {
    LOG_ERROR("epoll_ctl failed")
    disconnect();
    available = false;
  }

  return 0;
}

int Controller::scanForControllers() {
  std::unique_lock<std::shared_mutex> lock(connect_m);
  // reset available list
  availableControllers.clear();

  /*// make sure hcitool is available
  std::string output = Utilities::exec("hcitool clock");

  if (output.find("not found") != std::string::npos) {
    LOG_ERROR("\"hcitool\" not found, do you have \"bluez-utils\" installed?")
    disconnect();
    return ERROR_NO_HCITOOL;
  } else if (output.find("Device is not available: No such device") != std::string::npos) {
    LOG_ERROR("\"hcitool\" latestReports anclass error, is hciconfig up?")
    disconnect();
    return ERROR_NO_HCITOOL;
  }*/

  // use hcitool for bluetooth scan
  std::stringstream data(
      Utilities::exec("hcitool scan --flush | grep Wireless | grep -Eo '([a-fA-F0-9]{2}:){5}[a-fA-F0-9]{2}'"));

  std::string address;
  while (std::getline(data, address)) {
    LOG("Found: [" + address + "]")
    availableControllers.push_back(address);
  }

  if (availableControllers.empty()) return -1;
  else return 0;
}

int Controller::execute() {
  if (!available) {
    return -1;
  }

  updateSettings();

  if (readHIDReport() != 0) {
    // disconnected
    for (int i = 0; i < DS4_HID_ENTRY_TOTAL_COUNT; i++) {
      onRemoteControl(i, false);
    }
  } else {
    // buttons, because they need special handling
    for (int i = 0; i < DS4_HID_ENTRY_TERNARY_COUNT; i++) {
      if (currentReport[i].value == DOWN) {
        if (actualFactualReport[i].getTimeMS() > DS4_REMOTE_BUTTON_COOLDOWN) {
          if (actualFactualReport[i].value == DOWN) {
            actualFactualReport[i].value = HELD;
          } else if (actualFactualReport[i].value == UP) {
            actualFactualReport[i].resetTimer();
            actualFactualReport[i].value = DOWN;
          }
          onRemoteControl(i, actualFactualReport[i].value);
        }
      } else {
        if (actualFactualReport[i].value != UP) {
          // button is up again
          if (actualFactualReport[i].value == HELD) {
            onRemoteControl(i, UP);
          }
          actualFactualReport[i].value = UP;
        }
      }
    }
    // everything else, because it doesn't
    for (int i = DS4_HID_ENTRY_TERNARY_COUNT; i < DS4_HID_ENTRY_TOTAL_COUNT; i++) {
      if (latestReport[i].value != currentReport[i].value) {
        onRemoteControl(i, currentReport[i].value);
      }
    }
  }

  if (lowBattery) updateSettings();

  return 0;
}

int Controller::updateSettings() {
  if (!isAvailable()) return -1;
  std::unique_lock<std::shared_mutex> lock(send_m);

  setControllerLed(26, 229, 26);

  if (state.disconnectRequested) {
    disconnect(state.tryReconnect);
    return -1;
  }

  auto *buf = new u_char[REPORT_SIZE];
  memset(buf, 0, REPORT_SIZE);

  buf[0] = 0x52;
  buf[1] = 0x11;
  buf[2] = 0x80;
  buf[4] = 0xff;

  if (!lowBattery) {
    buf[7] = state.smallRumble;
    buf[8] = state.bigRumble;
    buf[9] = state.ledRed;
    buf[10] = state.ledGreen;
    buf[11] = state.ledBlue;
    buf[12] = state.flashOnTime;
    buf[13] = state.flashOffTime;
  } else {
    if (lowBatteryRumbleTimer.getTimeS() < 2) {
      buf[7] = 128;
      buf[8] = 128;
      buf[9] = 128; // ??
    } else if (lowBatteryRumbleTimer.getTimeS() < 4) {
      buf[7] = 0;
      buf[8] = 0;
      buf[9] = 0; // ??
    } else lowBatteryRumbleTimer.reset();
  }

  //ctlStatus = send(ds4Out, "\x52\x11\x80\x00\xff\x00\x00\x00\x00\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00", 79, 0);

  ctlStatus = send(ds4Out, buf, REPORT_SIZE, 0);

  delete[] buf;

  //if (ctlStatus < 0) //LOG_VERBOSE("Sending failed: " << errno));

  return 0;
}

int Controller::readHIDReport() {
  std::unique_lock<std::shared_mutex> lock(recv_m);
  if (!isAvailable()) return -1;

  memset(hidBuffer, 0, REPORT_SIZE);

  int bytesRead = -1;

  // TODO: does ev need to be an array here?
  if (epoll_wait(epfd, &ev, 1023, 5000) == -1) {
    available = false;
    LOG_ERROR("Lost connection to controller (epoll_wait), attempting new connection..")
    close(ds4In);
    close(ds4Out);
    close(epfd);
    initiateConnection();
    return -1;
  } else {
    bytesRead = recv(ds4In, hidBuffer, REPORT_SIZE, 0);
  }

  // invalid answer, ignore
  if (bytesRead < REPORT_SIZE || hidBuffer[1] != REPORT_ID) {
    if (invalidCount++ > MIN_DISCONNECT_COUNT) {
      available = false;
      LOG_ERROR("Lost connection to controller (bytes_read), attempting new connection..")
      close(ds4In);
      close(ds4Out);
      close(epfd);
      initiateConnection();
      invalidCount = -1;
    }
    return -1;
  } else invalidCount = -1;

  latestReport = currentReport;

  int dpad = hidBuffer[8] % 16;

  // x-127, to move default position to x = 0, facing right
  currentReport[DS4_ANALOG_LEFT_STICK_X].value = hidBuffer[4] - 127;
  // -(x-127), to move default position to y = 0, facing top
  currentReport[DS4_ANALOG_LEFT_STICK_Y].value = -(hidBuffer[5] - 127);

  // x-127, to move default position to x = 0, facing right
  currentReport[DS4_ANALOG_RIGHT_STICK_X].value = hidBuffer[6] - 127;
  // -(x-127), to move default position to y = 0, facing top
  currentReport[DS4_ANALOG_RIGHT_STICK_Y].value = -(hidBuffer[7] - 127);

  currentReport[DS4_ANALOG_LEFT2].value = hidBuffer[11];
  currentReport[DS4_ANALOG_RIGHT2].value = hidBuffer[12];

  currentReport[DS4_TERNARY_DPAD_NORTH].value = dpad == 0 || dpad == 1 || dpad == 7;
  currentReport[DS4_TERNARY_DPAD_SOUTH].value = dpad == 3 || dpad == 4 || dpad == 5;
  currentReport[DS4_TERNARY_DPAD_WEST].value = dpad == 5 || dpad == 6 || dpad == 7;
  currentReport[DS4_TERNARY_DPAD_EAST].value = dpad == 1 || dpad == 2 || dpad == 3;

  currentReport[DS4_TERNARY_BUTTON_CROSS].value = (hidBuffer[8] & 32) != 0;
  currentReport[DS4_TERNARY_BUTTON_CIRCLE].value = (hidBuffer[8] & 64) != 0;
  currentReport[DS4_TERNARY_BUTTON_SQUARE].value = (hidBuffer[8] & 16) != 0;
  currentReport[DS4_TERNARY_BUTTON_TRIANGLE].value = (hidBuffer[8] & 128) != 0;

  currentReport[DS4_TERNARY_BUTTON_LEFT1].value = (hidBuffer[9] & 1) != 0;
  currentReport[DS4_TERNARY_BUTTON_LEFT2].value = (hidBuffer[9] & 4) != 0;
  currentReport[DS4_TERNARY_BUTTON_LEFTSTICK].value = (hidBuffer[9] & 64) != 0;

  currentReport[DS4_TERNARY_BUTTON_RIGHT1].value = (hidBuffer[9] & 2) != 0;
  currentReport[DS4_TERNARY_BUTTON_RIGHT2].value = (hidBuffer[9] & 8) != 0;
  currentReport[DS4_TERNARY_BUTTON_RIGHTSTICK].value = (hidBuffer[9] & 128) != 0;

  currentReport[DS4_TERNARY_BUTTON_SHARE].value = (hidBuffer[9] & 16) != 0;
  currentReport[DS4_TERNARY_BUTTON_OPTIONS].value = (hidBuffer[9] & 32) != 0;

  currentReport[DS4_TERNARY_BUTTON_TRACKPAD].value = (hidBuffer[10] & 2) != 0;
  currentReport[DS4_TERNARY_BUTTON_PS].value = (hidBuffer[10] & 1) != 0;

  currentReport[DS4_ANALOG_ACCELERATION_Y].value = Utilities::read16LE(hidBuffer, 16);
  currentReport[DS4_ANALOG_ACCELERATION_X].value = Utilities::read16LE(hidBuffer, 18);
  currentReport[DS4_ANALOG_ACCELERATION_Z].value = Utilities::read16LE(hidBuffer, 20);

  currentReport[DS4_ANALOG_ORIENTATION_ROLL].value = -Utilities::read16LE(hidBuffer, 22);
  currentReport[DS4_ANALOG_ORIENTATION_YAW].value = Utilities::read16LE(hidBuffer, 24);
  currentReport[DS4_ANALOG_ORIENTATION_PITCH].value = Utilities::read16LE(hidBuffer, 26);

  // heuristic measures lead to value ranges for the track pad of:
  // x: 0..1919
  // y: -933..0
  // So, to move the coordinates to the middle, subtract 960 from x
  // and negate the outcome of subtracting 467 from y:

  currentReport[DS4_ANALOG_TRACKPAD0_TOUCH_ID].value = hidBuffer[38] & 0x7f;
  currentReport[DS4_TERNARY_TRACKPAD0_ACTIVE].value = (hidBuffer[38] >> 7) == 0;
  currentReport[DS4_ANALOG_TRACKPAD0_TOUCH_X].value = (((hidBuffer[40] & 0x0f) << 8) | hidBuffer[39]) - 960;
  currentReport[DS4_ANALOG_TRACKPAD0_TOUCH_Y].value = -((hidBuffer[41] << 4 | ((hidBuffer[40] & 0xf0) >> 4)) - 467);

  currentReport[DS4_ANALOG_TRACKPAD1_TOUCH_ID].value = hidBuffer[42] & 0x7f;
  currentReport[DS4_TERNARY_TRACKPAD1_ACTIVE].value = (hidBuffer[42] >> 7) == 0;
  currentReport[DS4_ANALOG_TRACKPAD1_TOUCH_X].value = (((hidBuffer[44] & 0x0f) << 8) | hidBuffer[43]) - 960;
  currentReport[DS4_ANALOG_TRACKPAD1_TOUCH_Y].value = -((hidBuffer[45] << 4 | ((hidBuffer[44] & 0xf0) >> 4)) - 467);

  currentReport[DS4_ANALOG_TIMESTAMP].value = hidBuffer[10] >> 2;
  currentReport[DS4_ANALOG_BATTERY].value = hidBuffer[33] % 16;

  currentReport[DS4_DIGITAL_PLUG_USB].value = (hidBuffer[33] & 16) != 0;
  currentReport[DS4_DIGITAL_PLUG_AUDIO].value = (hidBuffer[33] & 32) != 0;
  currentReport[DS4_DIGITAL_PLUG_MICROPHONE].value = (hidBuffer[33] & 64) != 0;

  // battery check
  //LOG_VERBOSE("Controller at " << currentReport[DS4_ANALOG_BATTERY].value) + "0%+ charge.");

  switch (currentReport[DS4_ANALOG_BATTERY].value) {
    case 10:
    case 9:
    case 8:
    case 7:
    case 6:
    case 5: {
      lowBattery = false;
      break;
    }
    case 4: {
      lowBattery = false;
      if (!currentReport[DS4_DIGITAL_PLUG_USB].value) {
        LOG("Battery below 50%, please connect to charger.")
      }
      break;
    }
    case 3:
    case 2: {
      if (!currentReport[DS4_DIGITAL_PLUG_USB].value) {
        LOG("Battery below 30%, please connect to charger ASAP.")
        lowBattery = true;
        idle(false);
      } else lowBattery = false;
      break;
    }
    case 1:
    case 0: {
      if (!currentReport[DS4_DIGITAL_PLUG_USB].value) {
        LOG_ERROR("Battery under 10%, charge the controller immediately!")
        disconnect();
        idle(false);
        disconnect();
        lowBattery = true;
        allowConnection = false;
      } else lowBattery = false;
      break;
    }
  }

  return 0;
}

void Controller::dumpHIDReport() {
  readHIDReport();

  LOG(
      "\nHIDReport:\n"
          << "Time stamp:        " << currentReport[DS4_ANALOG_TIMESTAMP].value << "\n"
          << "Analog Data:\n"
          << "leftStick:         "
          << currentReport[DS4_ANALOG_LEFT_STICK_X].value << ", "
          << currentReport[DS4_ANALOG_LEFT_STICK_Y].value
          << " (x, y)\n"
          << "rightStick:        "
          << currentReport[DS4_ANALOG_RIGHT_STICK_X].value << ", "
          << currentReport[DS4_ANALOG_RIGHT_STICK_Y].value << " (x, y)\n"
          << "L2:                "
          << currentReport[DS4_ANALOG_LEFT2].value << " --> "
          << currentReport[DS4_TERNARY_BUTTON_LEFT2].value << "\n"
          << "R2:                "
          << currentReport[DS4_ANALOG_RIGHT2].value << " --> "
          << currentReport[DS4_TERNARY_BUTTON_RIGHT2].value << "\n"
          << "TrackPad0:         "
          << currentReport[DS4_ANALOG_TRACKPAD0_TOUCH_X].value << ", "
          << currentReport[DS4_ANALOG_TRACKPAD0_TOUCH_Y].value << ", "
          << currentReport[DS4_TERNARY_TRACKPAD0_ACTIVE].value << " (x, y, active)\n"
          << "TrackPad1:         "
          << currentReport[DS4_ANALOG_TRACKPAD1_TOUCH_X].value << ", "
          << currentReport[DS4_ANALOG_TRACKPAD1_TOUCH_Y].value << ", "
          << currentReport[DS4_TERNARY_TRACKPAD1_ACTIVE].value << " (x, y, active)\n"
          << "Acceleration X:    " << currentReport[DS4_ANALOG_ACCELERATION_X].value << "\n"
          << "Acceleration Y:    " << currentReport[DS4_ANALOG_ACCELERATION_Y].value << "\n"
          << "Acceleration Z:    " << currentReport[DS4_ANALOG_ACCELERATION_Z].value << "\n"
          << "Orientation Roll:  " << currentReport[DS4_ANALOG_ORIENTATION_ROLL].value << "\n"
          << "Orientation Yaw:   " << currentReport[DS4_ANALOG_ORIENTATION_YAW].value << "\n"
          << "Orientation Pitch: " << currentReport[DS4_ANALOG_ORIENTATION_PITCH].value << "\n"
          << "Buttons:\n"
          << "Cross:             " << currentReport[DS4_TERNARY_BUTTON_CROSS].value << "\n"
          << "Square:            " << currentReport[DS4_TERNARY_BUTTON_SQUARE].value << "\n"
          << "Triangle:          " << currentReport[DS4_TERNARY_BUTTON_TRIANGLE].value << "\n"
          << "Circle:            " << currentReport[DS4_TERNARY_BUTTON_CIRCLE].value << "\n"
          << "Options:           " << currentReport[DS4_TERNARY_BUTTON_OPTIONS].value << "\n"
          << "TrackPad:          " << currentReport[DS4_TERNARY_BUTTON_TRACKPAD].value << "\n"
          << "PS:                " << currentReport[DS4_TERNARY_BUTTON_PS].value << "\n"
          << "Share:             " << currentReport[DS4_TERNARY_BUTTON_SHARE].value << "\n"
          << "leftStick:         " << currentReport[DS4_TERNARY_BUTTON_LEFTSTICK].value << "\n"
          << "rightStick:        " << currentReport[DS4_TERNARY_BUTTON_RIGHTSTICK].value << "\n"
          << "D-Pad North:       " << currentReport[DS4_TERNARY_DPAD_NORTH].value << "\n"
          << "D-Pad East:        " << currentReport[DS4_TERNARY_DPAD_EAST].value << "\n"
          << "D-Pad South:       " << currentReport[DS4_TERNARY_DPAD_SOUTH].value << "\n"
          << "D-Pad West:        " << currentReport[DS4_TERNARY_DPAD_WEST].value << "\n"
          << "L1:                " << currentReport[DS4_TERNARY_BUTTON_LEFT1].value << "\n"
          << "L2:                " << currentReport[DS4_TERNARY_BUTTON_LEFT2].value << "\n"
          << "R1:                " << currentReport[DS4_TERNARY_BUTTON_RIGHT1].value << "\n"
          << "R2:                " << currentReport[DS4_TERNARY_BUTTON_RIGHT2].value << "\n"
          << "Plugs:\n"
          << "Mic:               " << currentReport[DS4_DIGITAL_PLUG_MICROPHONE].value << "\n"
          << "USB:               " << currentReport[DS4_DIGITAL_PLUG_USB].value << "\n"
          << "Audio:             " << currentReport[DS4_DIGITAL_PLUG_AUDIO].value << "\n"
          << "Battery:           " << currentReport[DS4_ANALOG_BATTERY].value)
}

void Controller::idle(bool _idle) {
  std::unique_lock<std::shared_mutex> lock(idle_m);
  if (isIdle == _idle) return; // no changes

  isIdle = _idle;

  if (isIdle) {
    idleThread = new std::thread([this] { rgbw(); });
    idleThread->detach();
  }
  lock.unlock();
}

void Controller::rgbw() {
  int r = 0;
  int g = 0;
  int b = 0;

  for (uint8_t idleLoopCount = 0; idleLoopCount < 10; idleLoopCount++) {
    for (r = 255; g < 255; g++) {
      if (!isIdle) goto IDLE_END;
      setControllerLed(r, g, b);
      usleep(3579);
    }
    // yellow
    for (; r > 0; r--) {
      if (!isIdle) goto IDLE_END;
      setControllerLed(r, g, b);
      usleep(3579);
    }
    // green
    for (; b < 255; b++) {
      if (!isIdle) goto IDLE_END;
      setControllerLed(r, g, b);
      usleep(3579);
    }
    // turquoise
    for (; g > 0; g--) {
      if (!isIdle) goto IDLE_END;
      setControllerLed(r, g, b);
      usleep(3579);
    }
    // blue
    for (; r < 255; r++) {
      if (!isIdle) goto IDLE_END;
      setControllerLed(r, g, b);
      usleep(3579);
    }
    // magenta
    for (; g < 255; g++) {
      if (!isIdle) goto IDLE_END;
      setControllerLed(r, g, b);
      usleep(3579);
    }
    // white
    for (; g > 0 && b > 0; b = g--) {
      if (!isIdle) goto IDLE_END;
      setControllerLed(r, g, b);
      usleep(3579);
    }
    //red
    idleLoopCount++;
  }

  goto IDLE_END;

  IDLE_END:

  isIdle = false;
  setControllerLed(26, 229, 26);
  updateSettings();
}

Controller::~Controller() {
  // For some reason the controller that came with the PS4 connects
  // differently with the OS then one I bought later.
  // As a result, the OS maintains the connection to this controller;
  // effectively stopping it from turning off on program shutdown.
  // I have yet to find out why this is the case and how to fix it
  // (can one update the DS4 firmware? This also could be due that the
  // misbehaving controller is registered as the main controller on my
  // PS4 - maybe that changes it behavior) - Anyway, for the time being
  // this means you either have to shut off the controller manually,
  // remove it as a bluetooth device representation, kill the OS,
  // or use a different (probably more up-to-date) controller.
  disconnect();
  delete[] hidBuffer;
}

bool Controller::isAvailable() const {
  return available;
}

void Controller::disconnect(bool _reconnect) {
  close(ds4In);
  close(ds4Out);

  available = false;

  if (_reconnect) initiateConnection();
}

double Controller::getAngle() {
  return rightStick.scale() > CONTROLLER_DEAD_ZONE ? rightStick.angle() : 0;
}

double Controller::getSpeed() {
  return rightStick.scale() > CONTROLLER_DEAD_ZONE ? rightStick.scale() * CONTROLLER_SPEED_ADJUSTMENT : 0;
}

void Controller::HIDEntry::resetTimer() { return time.reset(); }

double Controller::HIDEntry::getTimeMS() {
  return value == HELD ? 0 : time.getTimeMS();
}

bool Controller::isActivated() const {
  return m_isActivated;
}
