
#include "mapping.hpp"

/**
 * @brief Get number of the exit where the robot is currently
 *
 * @param current_exit Vector, the robot drove last
 */
int get_exit_number(std::tuple<double, double> current_exit) {
  double current_exit_x = std::get<0>(current_exit);
  double current_exit_y = std::get<1>(current_exit);
  int exit = -1;
  if (current_exit_x >= current_exit_y) {
    if (fabs(current_exit_x) >= fabs(current_exit_y)) {
      exit = exit_t::WEST_EXIT;
    } else {
      exit = exit_t::NORTH_EXIT;
    }
  } else {
    if (fabs(current_exit_x) >= fabs(current_exit_y)) {
      exit = exit_t::EAST_EXIT;
    } else {
      exit = exit_t::SOUTH_EXIT;
    }
  }
  // std::cout << current_exit_x << " " << current_exit_y << " "<< exit  <<
  // std::endl;
  return exit;
}

/**
 * @brief get the exit which will be explored next
 *
 * @param exits Status of all four exits (list of integer, integers are
 * increased every time drive through the exit)
 * @param current_exit number of current exit
 *
 * @return direction: 1 - left
 *                    2 - straight
 *                    3 - right
 */
int get_next_direction(std::array<int, 4> exits, int current_exit) {
  int direction = direction_t::STRAIGHT;
  // get the most left exit
  for (int i = 1; i < 4; i += 1) {
    if (exits.at((current_exit + i) % 4) == 0) {
      direction = i;
      break;
    }
  }
  if (exits.at((current_exit + direction) % 4) == -1) {
    direction = direction_t::LEFT;
  }
  if (exits.at((current_exit + direction) % 4) == -1) {
    direction = direction_t::RIGHT;
  }

  // 1:left; 2:straight;3:right
  std::string text_direction = "";
  switch (direction) {
    case 1:
      text_direction = "left";
      break;
    case 2:
      text_direction = "straight";
      break;
    case 3:
    default:
      text_direction = "right";
  }
  // std::cout << "curent exit #" << current_exit << " go to " << text_direction
  // << std::endl;
  return direction;
}

Mapping::Mapping() {
  // No operation
}

Mapping::~Mapping() {
  // No operation
}

int Mapping::exec_thread() {
  const int map_size = 12;

  // a dictionary to index the intersections
  std::map<std::tuple<double, double>, int> index_intersection;
  // a dictionary of the visit state of intersections' exits (throughput)
  std::map<std::tuple<double, double>, std::array<int, 4>>
      throughtput_intersection;

  std::map<int, std::array<int, 4>> cost_intersection;  // costs of each exits
  std::map<int, std::array<int, 4>> connection_map;  // connection of each exits
  std::map<int, std::array<int, map_size>>
      cost_map;  // costs between intersections

  /* // abandon the matrix, not easy to set and get
  // int connection_matrix[this->mapsize][4] = {};
  // int cost_matrix[this->mapsize][this->mapsize] = {};
  // std::fill(*connection_matrix, *connection_matrix + this->mapsize*4, -1);
  // std::fill(*cost_matrix, *cost_matrix + this->mapsize*this->mapsize, 0);
  */

  std::tuple<double, double> intersection_current(0, 0);
  // std::tuple<double, double> intersection_last (0,0);
  int index_intersection_current = 0;
  int index_intersection_last = 0;

  // position
  double x = 0;
  double y = 0;
  double x_last = -1;
  double y_last = -1;

  // intersection detection
  bool intersection_detected = false;
  bool intersection_detected_last;
  auto intersection_time = std::chrono::system_clock::now();
  auto intersection_time_last = std::chrono::system_clock::now();
  bool exit_left_detected;
  bool exit_right_detected;
  bool exit_middle_detected;

  // exits
  exit_t exit_current;
  int exit_last;

  /* direction:
      "unkown", if don't at intersection;
      "left", "straight", "right", if at intersection */
  direction_t direction;
  direction_t direction_last;

  // station detection
  station_t station;
  station_t station_last = station_t::STATION_UNKNOWN;
  int station_connection[4][2] = {};
  std::fill(*station_connection, *station_connection + 4 * 2, -1);
  int station_duration[4][2] = {};
  std::fill(*station_duration, *station_duration + 4 * 2, 0);

  // start mapping, set finished flag to false
  this->blackboard.mapping_finished.set(false);
  this->blackboard.direction_changed.set(false);
  // stations position initial
  this->blackboard.station.set(station_t::STATION_UNKNOWN);
  this->blackboard.stations.set(
      std::array<Coordinates, 4>{Coordinates{0, 0, 0}, Coordinates{0, 0, 0},
                                 Coordinates{0, 0, 0}, Coordinates{0, 0, 0}});

  std::cout << "Mapping started ..." << std::endl;
  while (this->blackboard.mapping_enabled.get()) {
    // direction_last:   if at intersection( direction not unknow), pass the old
    // value to . direction:        initial direction with unknow
    if (direction != direction_t::UNKNOWN) {
      direction_last = direction;
    }
    direction = direction_t::UNKNOWN;

    // get the current position
    x_last = x;
    y_last = y;
    Coordinates coordinates = this->blackboard.coordinates.get();
    x = coordinates.x;
    y = coordinates.y;
    // if position not changed, continue
    if (x == x_last && y == y_last) continue;
    // std::cout << "----- current: " << x << " " << y << " last: " << x_last <<
    // " " << y_last << std::endl;

    // Landmark detection
    station = this->blackboard.station.get();
    if (station != station_t::STATION_UNKNOWN &&
        (station_connection[station][0] == -1 ||
         station_connection[station][1] == -1)) {
      station_last = station;
      std::array<Coordinates, 4> station_coordinates =
          this->blackboard.stations.get();
      switch (station) {
        case station_t::STATION_RED:
          station_coordinates.at(station_t::STATION_RED) = coordinates;
          break;
        case station_t::STATION_GREEN:
          station_coordinates.at(station_t::STATION_GREEN) = coordinates;
          break;
        case station_t::STATION_BLUE:
          station_coordinates.at(station_t::STATION_BLUE) = coordinates;
          break;
        case station_t::STATION_YELLOW:
          station_coordinates.at(station_t::STATION_YELLOW) = coordinates;
          break;
        case station_t::STATION_UNKNOWN:
        default:
          break;
      }

      // update the station connection(comming direction)
      if (index_intersection.size() > 0) {
        station_connection[station][0] = index_intersection_last;
        // station_connection[station][0] =
        // index_intersection.find(intersection_last)->second;
        station_duration[station][0] =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - intersection_time)
                .count();
      }

      // update the station coordinates
      this->blackboard.stations.set(station_coordinates);
    }

    // Intersection detection
    intersection_detected_last = intersection_detected;
    intersection_detected = this->blackboard.intersection_detected.get();
    std::array<bool, 3> exits_detected = this->blackboard.exits_detected.get();
    exit_left_detected = exits_detected.at(0);
    exit_middle_detected = exits_detected.at(1);
    exit_right_detected = exits_detected.at(2);

    /* on a rising edge of intersection detected */
    if (intersection_detected && !intersection_detected_last) {
      std::tuple<double, double> current_position(x, y);

      // current_exit: the direction of entering an intersection
      std::tuple<double, double> current_exit(x - x_last, y - y_last);

      // south:0; west:1; north:2; east:3
      exit_last = (exit_current + direction_last) % 4;
      exit_current = (exit_t)get_exit_number(current_exit);

      if (index_intersection.size() > 0) {
        index_intersection_last = index_intersection_current;
        // intersection_last = intersection_current;
        intersection_time_last = intersection_time;
      }
      intersection_time = std::chrono::system_clock::now();

      /* check if the intersection is in map and update coordinates */
      bool is_recorded_intersection = false;
      for (const auto& elem : throughtput_intersection) {
        double intersection_x = std::get<0>(elem.first);
        double intersection_y = std::get<1>(elem.first);
        if (x >= intersection_x - this->fuzziness &&
            x <= intersection_x + this->fuzziness &&
            y >= intersection_y - this->fuzziness &&
            y <= intersection_y + this->fuzziness) {
          is_recorded_intersection = true;
          intersection_current =
              std::make_tuple(intersection_x, intersection_y);
          break;
        }
      }

      // if new intersection
      if (!is_recorded_intersection) {
        intersection_current = current_position;
        std::array<int, 4> exits_visit_stat{0, 0, 0, 0};
        // mark not-exits at intersection
        if (!exit_left_detected)
          exits_visit_stat.at((exit_current + 1) % 4) = -1;
        if (!exit_middle_detected)
          exits_visit_stat.at((exit_current + 2) % 4) = -1;
        if (!exit_right_detected)
          exits_visit_stat.at((exit_current + 3) % 4) = -1;

        // comming direction +1
        exits_visit_stat.at((exit_current) % 4) += 1;
        // get next exit
        direction =
            (direction_t)get_next_direction(exits_visit_stat, exit_current);
        // leaving direction +1
        exits_visit_stat.at((exit_current + direction) % 4) += 1;
        // initial and update the throughput
        throughtput_intersection.insert(
            std::pair<std::tuple<double, double>, std::array<int, 4>>(
                intersection_current, exits_visit_stat));
        // initial the connection and duration
        index_intersection_current = index_intersection.size();
        connection_map.insert(std::pair<int, std::array<int, 4>>(
            index_intersection_current, exits_visit_stat));
        cost_intersection.insert(std::pair<int, std::array<int, 4>>(
            index_intersection_current, exits_visit_stat));
        cost_map.insert(std::pair<int, std::array<int, map_size>>(
            index_intersection_current, std::array<int, map_size>{}));
        // index the new intersection
        index_intersection.insert(std::pair<std::tuple<double, double>, int>(
            intersection_current, index_intersection_current));
      }
      // if intersection already visited
      else {
        index_intersection_current =
            index_intersection.find(intersection_current)->second;
        // get the exits stat
        std::array<int, 4> exits_visit_stat =
            throughtput_intersection.find(intersection_current)->second;
        // comming direction +1
        exits_visit_stat.at(exit_current) += 1;
        // get the next exit
        direction =
            (direction_t)get_next_direction(exits_visit_stat, exit_current);
        // leaving direction +1
        exits_visit_stat.at((exit_current + direction) % 4) += 1;
        // update the throughput
        throughtput_intersection.find(intersection_current)->second =
            exits_visit_stat;
      }

      // update the station connection(going direction)
      if (station_last != station_t::STATION_UNKNOWN &&
          station_connection[station_last][1] == -1) {
        station_connection[station_last][1] =
            index_intersection.find(intersection_current)->second;
      }

      // update the map
      if (index_intersection.size() > 1) {
        // index_intersection_current =
        // index_intersection.find(intersection_current)->second;
        // index_intersection_last =
        // index_intersection.find(intersection_last)->second;

        // set connection
        std::array<int, 4> connection_current =
            connection_map.find(index_intersection_current)->second;
        std::array<int, 4> connection_last =
            connection_map.find(index_intersection_last)->second;
        connection_current.at(exit_current) = index_intersection_last;
        connection_last.at(exit_last) = index_intersection_current;

        // set duration
        std::array<int, 4> durations_current =
            cost_intersection.find(index_intersection_current)->second;
        std::array<int, 4> durations_last =
            cost_intersection.find(index_intersection_last)->second;
        auto intersection_duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                intersection_time - intersection_time_last)
                .count();
        durations_current.at(exit_current) = intersection_duration;
        durations_last.at(exit_last) = intersection_duration;

        // update the connection_map and cost_intersection
        connection_map.find(index_intersection_current)->second =
            connection_current;
        connection_map.find(index_intersection_last)->second = connection_last;
        cost_intersection.find(index_intersection_current)->second =
            durations_current;
        cost_intersection.find(index_intersection_last)->second =
            durations_last;

        // update the cost_map
        std::array<int, map_size> cost_current =
            cost_map.find(index_intersection_current)->second;
        int original_duration = cost_current.at(index_intersection_last);
        if (original_duration > 0) {
          if (intersection_duration < original_duration) {
            cost_current.at(index_intersection_last) = intersection_duration;
            cost_map.find(index_intersection_last)
                ->second.at(index_intersection_current) = intersection_duration;
            cost_map.find(index_intersection_current)
                ->second.at(index_intersection_last) = intersection_duration;
          }
        } else {
          cost_current.at(index_intersection_last) = intersection_duration;
          cost_map.find(index_intersection_last)
              ->second.at(index_intersection_current) = intersection_duration;
          cost_map.find(index_intersection_current)
              ->second.at(index_intersection_last) = intersection_duration;
        }

        /*
        // update the map matrix
        // connection_matrix[index_intersection_current][exit_current] =
        index_intersection_last;
        // connection_matrix[index_intersection_last][exit_last] =
        index_intersection_current;
        // if (cost_matrix[index_intersection_current][index_intersection_last]
        > 0){
        //     int original_duration =
        cost_matrix[index_intersection_current][index_intersection_last];
        //     int current_duration = intersection_duration;
        //     if (current_duration < original_duration) {
        // cost_matrix[index_intersection_current][index_intersection_last] =
        current_duration;
        // cost_matrix[index_intersection_last][index_intersection_current] =
        current_duration;
        //     }
        // } else {
        //     cost_matrix[index_intersection_current][index_intersection_last]
        = intersection_duration;
        //     cost_matrix[index_intersection_last][index_intersection_current]
        = intersection_duration;
        // }
        */
      }

      this->blackboard.direction.set(direction);
      this->blackboard.direction_changed.set(true);

    }
    // if no intersection detected
    else {
      if (this->blackboard.direction_changed.get())
        this->blackboard.direction_changed.set(false);
    }

    bool is_finished = true;
    // check if all intersection finished
    for (const auto& elem : throughtput_intersection) {
      double intersection_x = std::get<0>(elem.first);
      double intersection_y = std::get<1>(elem.first);
      for (int i = 0; i < 4; i += 1) {
        if (elem.second.at(i) == 0) {
          is_finished = false;
        }
      }
    }

    if (is_finished && !throughtput_intersection.empty()) {
      this->blackboard.mapping_finished.set(true);
      this->blackboard.direction_changed.set(false);
      this->blackboard.duration_map.set(cost_map);
      this->blackboard.connection_map.set(connection_map);
      this->blackboard.cost_exits_map.set(cost_intersection);
      std::cout << "Mapping finished." << std::endl;
      break;
    }
  }

  std::cout << "-----" << std::endl;
  std::cout << "intersection result:" << std::endl;
  std::cout << "intersection x,y | exit visit times S-W-N-E" << std::endl;
  for (const auto& elem : throughtput_intersection) {
    int id = index_intersection.find(elem.first)->second;
    double intersection_x = std::get<0>(elem.first);
    double intersection_y = std::get<1>(elem.first);
    int south_exit = std::get<0>(elem.second);
    int left_exit = std::get<1>(elem.second);
    int north_exit = std::get<2>(elem.second);
    int right_exit = std::get<3>(elem.second);
    std::cout << id << " | " << intersection_x << " \t" << intersection_y
              << " \t| " << south_exit << " \t" << left_exit << " \t"
              << north_exit << " \t" << right_exit << std::endl;
  }

  // std::cout << "intersection No. | Duration to next intersection S-W-N-E" <<
  // std::endl; for(const auto& elem : cost_intersection){
  //     int id = elem.first;
  //     int south_exit =  std::get<0>(elem.second);
  //     int left_exit =  std::get<1>(elem.second);
  //     int north_exit =  std::get<2>(elem.second);
  //     int right_exit =  std::get<3>(elem.second);
  //     std::cout << id << " \t| " <<  south_exit << " \t" <<  left_exit << "
  //     \t" <<  north_exit << " \t" <<  right_exit << std::endl;
  // }
  // std::cout << "intersection No. | Connection at Exits S-W-N-E" << std::endl;
  // for(const auto& elem : connection_map){
  //     int id = elem.first;
  //     int south_exit =  std::get<0>(elem.second);
  //     int left_exit =  std::get<1>(elem.second);
  //     int north_exit =  std::get<2>(elem.second);
  //     int right_exit =  std::get<3>(elem.second);
  //     std::cout << id << " \t| " <<  south_exit << " \t" <<  left_exit << "
  //     \t" <<  north_exit << " \t" <<  right_exit << std::endl;
  // }
  // std::cout << "intersection No. | Costs Between Intersection No." <<
  // std::endl; for(const auto& elem : cost_map){
  //     int id = elem.first;
  //     int i0 =  std::get<0>(elem.second);
  //     int i1 =  std::get<1>(elem.second);
  //     int i2 =  std::get<2>(elem.second);
  //     int i3 =  std::get<3>(elem.second);
  //     int i4 =  std::get<4>(elem.second);
  //     int i5 =  std::get<5>(elem.second);
  //     int i6 =  std::get<6>(elem.second);
  //     int i7 =  std::get<7>(elem.second);
  //     int i8 =  std::get<8>(elem.second);
  //     int i9 =  std::get<9>(elem.second);
  //     std::cout << id << " \t| " <<  i0 << " \t" <<  i1 << " \t" <<  i2 << "
  //     \t" <<  i3 << " \t" <<  i4 << " \t" <<  i5 << " \t" <<  i6 << " \t" <<
  //     i7 << " \t" <<  i8 << " \t" <<  i9 << std::endl;
  // }

  /*
  // std::cout << "costs as matrix" << std::endl;
  // for (int i = 0; i < this->mapsize; i++) {
  //     for (int j = 0; j < this->mapsize; j++) {
  //         printf("%d ", cost_matrix[i][j]);
  //     }
  //     printf("\n");
  // }
  // std::cout << "connection as matrix" << std::endl;
  // for (int i = 0; i < this->mapsize; i++) {
  //     for (int j = 0; j < 4; j++) {
  //         printf("%d ", connection_matrix[i][j]);
  //     }
  //     printf("\n");
  // }
  */

  return 0;
}
