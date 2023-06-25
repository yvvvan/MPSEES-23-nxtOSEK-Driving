#ifndef COORDINATES_HPP
#define COORDINATES_HPP

typedef enum Direction {
  LEFT = 1,
  STRAIGHT = 2,
  RIGHT = 3,
  UNKNOWN = 0
} direction_t;

typedef enum Exit {
  WEST_EXIT = 1,
  NORTH_EXIT = 2,
  EAST_EXIT = 3,
  SOUTH_EXIT = 0
} exit_t;

typedef enum Station {
  STATION_YELLOW = 0,
  STATION_RED = 1,
  STATION_BLUE = 2,
  STATION_GREEN = 3,
  STATION_UNKNOWN = -1
} station_t;

class Coordinates {
 public:
  /**
   * @brief Construct a new Coordinates object
   *
   * @param x
   * @param y
   */
  Coordinates(double x, double y, double z);

  Coordinates();

  /**
   * @brief Destroy the Coordinates object
   *
   */
  ~Coordinates();

  /**
   * @brief add vector to coordinates
   */
  void add_vector(double x, double y, double z);

  double x;
  double y;
  double z;
};

#endif  // COORDINATES_HPP