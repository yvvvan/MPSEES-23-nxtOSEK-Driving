#ifndef BUILDHAT_GLOBALS_HPP
#define BUILDHAT_GLOBALS_HPP

/* standard library headers */
#include <iostream>

/******************************* macros *******************************/

/* debug control, disable for release */
#define DEBUG 1

/* debug print macro */
#define DEBUG 1
#if DEBUG
#define DEBUG_PRINT(x) std::cout << x << std::endl;
#else
#define DEBUG_PRINT(x)
#endif

/******************************* globals *******************************/

/* serial device */
constexpr char SERIAL_DEVICE[] = "/dev/ttyACM0";

/* drive parameters */
constexpr double WIND_TIME_SECONDS  = 5.0f;
constexpr double WIND_MIN_SPEED     = 0.1f;
constexpr u_int16_t WIND_STEP_COUNT = 100;

#endif //BUILDHAT_GLOBALS_HPP
