#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "communication/internal/BlackBoard.hpp"

#define MAX_SIZE 12
#define INF 9999

class Navigation {
 public:
  /**
   * @brief Construct a new Mapping object
   *
   * @return Mapping&
   */
  Navigation();

  /**
   * @brief Destroy the Mapping object
   *
   */
  ~Navigation();

  /**
   * @brief Execute the mapping thread
   *
   * @return 0 on success, -1 on failure
   */
  int exec_thread();

  void track_map();

 private:
  BlackBoard &blackboard = BlackBoard::getInstance();

  int start = 2;
  int end = -1;

  std::array<std::array<int, 12>, 2>
      path_found;  // path_found[0] = path, path_found[1] = exits

  int G[MAX_SIZE][MAX_SIZE] = {
      // 0   1     2     3     4     5     6     7     8     9    10    11
      /*0*/ {0, 155, -1, 195, -1, -1, -1, -1, 170, -1, -1, -1},
      /*1*/ {155, 0, 170, -1, -1, -1, -1, -1, -1, 80, -1, -1},
      /*2*/ {-1, 170, 0, -1, -1, -1, 200, -1, -1, -1, -1, -1},
      /*3*/ {195, -1, -1, 0, 160, -1, -1, 215, 270, -1, -1, -1},
      /*4*/ {-1, -1, -1, 160, 0, 125, -1, 100, -1, -1, 50, -1},
      /*5*/ {-1, -1, -1, -1, 125, 0, 45, 195, -1, -1, -1, -1},
      /*6*/ {-1, -1, 200, -1, -1, 45, 0, -1, -1, -1, -1, 1},
      /*7*/ {-1, -1, -1, 215, 100, 195, -1, 0, -1, -1, 50, -1},
      /*8*/ {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
      /*9*/ {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
      /*10*/ {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
      /*11*/ {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};

  /* treat landmarks as intersection*/
  int C[MAX_SIZE][4] = {
      // S  W   N    E
      /*0*/ {3, 8, -1, 1},
      /*1*/ {9, 0, -1, 2},
      /*2*/ {6, 1, -1, -1},
      /*3*/ {7, 8, 0, 4},
      /*4*/ {10, 3, -1, 5},
      /*5*/ {7, 4, -1, 6},
      /*6*/ {-1, 5, 2, -1},
      /*7*/ {-1, 3, 10, 5},
  };

  // clang-format off
  /* maps exit of the intersection with the intersection connected to the exit
   */
  const int intersection_map[MAX_SIZE][4][2] = {
      // S  W   N    E
/*0*/ {{3, NORTH_EXIT}, {3, WEST_EXIT}, {-1, UNKNOWN_EXIT}, {1, WEST_EXIT}},
/*1*/ {{1, SOUTH_EXIT}, {0, EAST_EXIT}, {-1, UNKNOWN_EXIT}, {2, WEST_EXIT}},
/*2*/ {{6, NORTH_EXIT}, {1, EAST_EXIT}, {-1, UNKNOWN_EXIT}, {-1, UNKNOWN_EXIT}},
/*3*/ {{7, WEST_EXIT}, {0, WEST_EXIT}, {0, SOUTH_EXIT}, {4, WEST_EXIT}},
/*4*/ {{7, NORTH_EXIT}, {3, EAST_EXIT}, {-1, UNKNOWN_EXIT}, {5, WEST_EXIT}},
/*5*/ {{7, EAST_EXIT}, {4, EAST_EXIT}, {-1, UNKNOWN_EXIT}, {6, WEST_EXIT}},
/*6*/ {{-1, UNKNOWN_EXIT}, {5, EAST_EXIT}, {2, SOUTH_EXIT}, {-1, UNKNOWN_EXIT}},
/*7*/ {{-1, UNKNOWN_EXIT}, {3, SOUTH_EXIT}, {4, SOUTH_EXIT}, {5, SOUTH_EXIT}},
  };
  // clang-format on

  /* only need if there are two connections between 2 intersections, and need to
   * choose a shorter*/
  int C_E[8][4] = {
      // S  W   N    E
      /*0*/ {195, 170, -1, 155},
      /*1*/ {160, 155, -1, 170},
      /*2*/ {200, 170, -1, -1},
      /*3*/ {215, 270, 195, 160},
      /*4*/ {100, 160, -1, 125},
      /*5*/ {195, 125, -1, 45},
      /*6*/ {-1, 45, 200, -1},
      /*7*/ {-1, 215, 100, 195}};
};

#endif  // NAVIGATION_HPP
