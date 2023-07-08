#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "communication/internal/BlackBoard.hpp"
#include "localization.hpp"

#define max 12
#define INFINITY 9999

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

 private:
  BlackBoard &blackboard = BlackBoard::getInstance();

  Localization localization;

  int start = 2;
  int end = -1;

  std::array<std::array<int, 12>, 2>
      path_found;  // path_found[0] = path, path_found[1] = exits

  int G[max][max] = {
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
  int C[max][4] = {
      // S  W   N    E
      /*0*/ {3, 8, -1, 1},
      /*1*/ {9, 0, -1, 2},
      /*2*/ {6, 1, -1, -1},
      /*3*/ {7, 0, 8, 4},
      /*4*/ {10, 3, -1, 5},
      /*5*/ {7, 4, -1, 6},
      /*6*/ {-1, 5, 2, -1},
      /*7*/ {-1, 3, 10, 5},
  };

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
