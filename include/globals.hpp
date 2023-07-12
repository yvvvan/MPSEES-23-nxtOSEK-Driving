#ifndef BUILDHAT_GLOBALS_HPP
#define BUILDHAT_GLOBALS_HPP

/* standard library headers */
#include <cmath>
#include <iostream>

/******************************* macros *******************************/

/* access macros */
#ifdef TEST_BUILD
#define GEORDI_PUBLIC public
#define GEORDI_PROTECTED protected
#define GEORDI_PRIVATE public
#else
#define GEORDI_PUBLIC public
#define GEORDI_PROTECTED protected
#define GEORDI_PRIVATE private
#endif

/* debug control, disable for release */
#define DEBUG 1

/* debug print macro */
#define DEBUG 1
#if DEBUG
#define DEBUG_PRINT(x) std::cout << x << std::endl;
#else
#define DEBUG_PRINT(x)
#endif

/* log macros */
#define LOG(x) std::cout << x << std::endl;
#define LOG_ERROR(x) std::cerr << x << std::endl;

/******************************* globals *******************************/

// math
constexpr double DEG_2_RAD = M_PI / 180.0;
constexpr double RAD_2_DEG = 180.0 / M_PI;

/* serial device */
constexpr char SERIAL_DEVICE[] = "/dev/serial0";

/* color sensor parameters */
constexpr int PORT_COLOR_SENSOR = 2;
constexpr int MIN_CALIBRATION_OFFSET = 3;

// everything below these values is considered black, thus, the floor (we ignore
// the hue)
constexpr int COLOR_MIN_SAT = 7;
constexpr int COLOR_MIN_VAL = 7;

/* drive parameters */
constexpr double CAR_SPEED = 2.0 / 3.0;
constexpr double CAR_MIN_SPEED = 0.571;

constexpr int PORT_LEFT_MOTOR = 3;
constexpr bool INVERT_LEFT_MOTOR = false;

constexpr int PORT_RIGHT_MOTOR = 0;
constexpr bool INVERT_RIGHT_MOTOR = true;

constexpr double FORWARD_ANGLE =
    120;  // everything above this angle is considered in place turning
constexpr double FORWARD_SCALE_FACTOR = (-1.0 / (FORWARD_ANGLE / 2.0));

/* PD controller parameters */
constexpr double KP = 0.25;
constexpr double KD = 0.25;

/* camera parameters */
constexpr int FPS = 30;
constexpr int IMAGE_WIDTH = 640;
constexpr int IMAGE_HEIGHT = 480;

// lane detection
constexpr int QUEUE_SIZE = FPS / 5;
constexpr int LANE_LOST_QUEUE_SIZE = FPS * 2;
constexpr int INTERSECTION_QUEUE_SIZE = FPS / 2;
constexpr double IMAGE_MIDDLE_X = IMAGE_WIDTH / 2.0;
constexpr double IMAGE_MIDDLE_Y = IMAGE_HEIGHT / 2.0;
constexpr double MISSING_LANE_MULTIPLIER = 2.0;
constexpr double LANE_DETECTION_MIN_SLOPE = 0.5;

// canny
constexpr int CANNY_THRESHOLD_1 = 50;
constexpr int CANNY_THRESHOLD_2 = 150;

// hough
constexpr int HOUGH_RHO = 1;
constexpr int HOUGH_THRESHOLD = 15;
constexpr int HOUGH_MIN_LINE_LEN = 10;
constexpr int HOUGH_MAX_LINE_GAP = 20;
constexpr double HOUGH_THETA = M_PI / 180.0;

#endif  // BUILDHAT_GLOBALS_HPP
