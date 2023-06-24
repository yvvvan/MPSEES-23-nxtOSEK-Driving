#ifndef BUILDHAT_GLOBALS_HPP
#define BUILDHAT_GLOBALS_HPP

/* standard library headers */
#include <iostream>

/******************************* macros *******************************/

/* access macros */
#ifdef TEST_BUILD
#define GEORDI_PUBLIC public
#define GEORDI_PROTECTED protected
#define GEORDI_PRIVATE protected
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

/* serial device */
constexpr char SERIAL_DEVICE[] = "/dev/serial0";

/* drive parameters */
constexpr double FORWARD_ANGLE = 120; // everything above this angle is considered in place turning
constexpr double FORWARD_SCALE_FACTOR = (-1.0 / (FORWARD_ANGLE / 2.0));

/* lane detection */
constexpr int IMAGE_WIDTH  = 640;
constexpr int IMAGE_HEIGHT = 480;
constexpr int FPS          = 20;
constexpr double IMAGE_MIDDLE = IMAGE_WIDTH / 2;

// canny
constexpr int CANNY_THRESHOLD_1 = 50;
constexpr int CANNY_THRESHOLD_2 = 150;

// hough
constexpr int HOUGH_RHO         = 1;
constexpr double HOUGH_THETA    = CV_PI / 180;
constexpr int HOUGH_THRESHOLD   = 15;
constexpr int HOUGH_MIN_LINELEN = 10;
constexpr int HOUGH_MAX_LINEGAP = 20;

// lane detection
constexpr double LANE_DETECTION_MIN_SLOPE = 0.5;
constexpr double MISSING_LANE_MULTIPLIER  = 0.12;

#endif //BUILDHAT_GLOBALS_HPP
