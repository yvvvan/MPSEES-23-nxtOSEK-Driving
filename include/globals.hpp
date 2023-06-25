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

#endif //BUILDHAT_GLOBALS_HPP
