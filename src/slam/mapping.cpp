
#include "mapping.hpp"


/**
 * @brief Get number of the exit where the robot is currently
 *
 * @param current_exit Vector, the robot drove last
*/
int get_exit_number(std::tuple<double, double>current_exit){
    double current_exit_x = std::get<0>(current_exit);
    double current_exit_y = std::get<1>(current_exit);
    int exit = -1;
    if (current_exit_x >= current_exit_y){
        if (fabs(current_exit_x) >= fabs(current_exit_y)){
            exit = exit_t::WEST_EXIT;
        } else {
            exit = exit_t::NORTH_EXIT;
        }
    } else {
        if (fabs(current_exit_x) >= fabs(current_exit_y)){
            exit = exit_t::EAST_EXIT;
        } else {
            exit = exit_t::SOUTH_EXIT;
        }
    }
    //std::cout << current_exit_x << " " << current_exit_y << " "<< exit  << std::endl;
    return exit;
}

/**
 * @brief get the exit which will be explored next
 *
 * @param exits Status of all four exits (list of integer, integers are increased every time drive through the exit)
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
    switch(direction) {
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
    //std::cout << "curent exit #" << current_exit << " go to " << text_direction << std::endl;
    return direction;
}

Mapping::Mapping() {
    // No operation
}

Mapping::~Mapping() {
    // No operation
}

int Mapping::exec_thread() {

    // a dictionary to index the intersections
    std::map<std::tuple<double, double> , int> index_intersection;
    // a dictionary of the visit state of intersections' exits 
    std::map<std::tuple<double, double> , std::array<int, 4>> map_intersection;
    // a dictionary of the connection of intersections' exits
    std::map<std::tuple<double, double> , std::array<int, 4>> connection_intersection;
    // a dictionary of the duration of intersections' exits to next intersection
    std::map<std::tuple<double, double> , std::array<int, 4>> duration_intersection;
    
    std::tuple<double, double> intersection_current (0,0);
    std::tuple<double, double> intersection_last (0,0);


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
        "left", "straight", "right", if at intersection*/
    direction_t direction;
    direction_t direction_last;
    this->blackboard.direction_changed.set(false);

    // station detection
    station_t station;

    // map structure initial
    int map_matrix[this->mapsize][this->mapsize] = {};
    std::fill(*map_matrix, *map_matrix + this->mapsize*this->mapsize, 0);
    // start mapping, set finished flag to false
    this->blackboard.mapping_finished.set(false);
    // stations position initial
    this->blackboard.station.set(station_t::STATION_UNKNOWN);
    this->blackboard.stations.set(std::array<Coordinates,4>{Coordinates{0,0,0},Coordinates{0,0,0},Coordinates{0,0,0},Coordinates{0,0,0}});
    
    std::cout << "Mapping started ..." << std::endl;
    while (this->blackboard.mapping_enabled.get()) {
        // direction initial, return unknown if there is no intersection; otherwise return left/right/straight 
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
        if (x == x_last && y == y_last)
            continue;
        //std::cout << "----- current: " << x << " " << y << " last: " << x_last << " " << y_last << std::endl;

        // get the station
        station = this->blackboard.station.get();
        if (station != station_t::STATION_UNKNOWN){
            std::array<Coordinates,4> station_coordinates = this->blackboard.stations.get();
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
            // update the station coordinates
            this->blackboard.stations.set(station_coordinates);
        }


        // get the intersection
        intersection_detected_last = intersection_detected;
        intersection_detected = this->blackboard.intersection_detected.get();
        std::array<bool,3> exits_detected= this->blackboard.exits_detected.get();
        exit_left_detected = exits_detected.at(0);
        exit_middle_detected = exits_detected.at(1);
        exit_right_detected = exits_detected.at(2);

        /* on a rising edge of intersection detected */
        if (intersection_detected && !intersection_detected_last) {
            std::tuple<double, double> current_position (x,y);

            // current_exit: the direction of entering an intersection
            std::tuple<double, double> current_exit (x-x_last, y-y_last);

            // south:0; west:1; north:2; east:3
            exit_last =  (exit_current+direction_last)%4;
            exit_current = (exit_t) get_exit_number(current_exit);

            /* initialise exit information arrays */
            std::array<int, 4> exits_visit_stat {0,0,0,0};

            if (index_intersection.size() > 0){
                intersection_last = intersection_current;
                intersection_time_last = intersection_time;
            }

            intersection_time = std::chrono::system_clock::now();

            /* check if the intersection is in map and update coordinates */
            bool is_recorded_intersection = false;
            for(const auto& elem : map_intersection){
                double intersection_x = std::get<0>(elem.first);
                double intersection_y = std::get<1>(elem.first);
                if ( x >= intersection_x-this->fuzziness &&  x <= intersection_x+this->fuzziness &&
                     y >= intersection_y-this->fuzziness &&  y <= intersection_y+this->fuzziness ) {
                    is_recorded_intersection = true;
                    intersection_current = std::make_tuple(intersection_x, intersection_y);
                    break;
                }
            }
            if (!is_recorded_intersection) {
                intersection_current = current_position;
            }

            // if intersection already visited
            if (is_recorded_intersection) {
                // get the exits stat
                exits_visit_stat = map_intersection.find(intersection_current)->second;
                // comming direction +1
                exits_visit_stat.at(exit_current) += 1;
                // get the next exit
                direction = (direction_t) get_next_direction(exits_visit_stat, exit_current);
                // leaving direction +1
                exits_visit_stat.at((exit_current+direction)%4) += 1;
                //update
                map_intersection.find(intersection_current)->second = exits_visit_stat;
            }

            // if new intersection
            else {
                // mark not-exits at intersection
                if (!exit_left_detected) 
                    exits_visit_stat.at((exit_current+1)%4) = -1;
                if (!exit_middle_detected)
                    exits_visit_stat.at((exit_current+2)%4) = -1;
                if (!exit_right_detected)
                    exits_visit_stat.at((exit_current+3)%4) = -1;

                // comming direction +1
                exits_visit_stat.at((exit_current)%4) += 1;
                // get next exit
                direction = (direction_t) get_next_direction(exits_visit_stat, exit_current);
                // leaving direction +1
                exits_visit_stat.at((exit_current+direction)%4) += 1;
                //update
                map_intersection.insert(std::pair<std::tuple<double, double>,std::array<int, 4>>(current_position,exits_visit_stat));
                index_intersection.insert(std::pair<std::tuple<double, double>, int>(current_position, index_intersection.size()));
                connection_intersection.insert(std::pair<std::tuple<double, double>, std::array<int, 4>>(current_position, exits_visit_stat));
                duration_intersection.insert(std::pair<std::tuple<double, double>, std::array<int, 4>>(current_position, exits_visit_stat));
            }

            // update the map
            if (index_intersection.size() > 1){
                std::array<int, 4> connection_current = connection_intersection.find(intersection_current)->second;
                std::array<int, 4> connection_last = connection_intersection.find(intersection_last)->second;
                std::array<int, 4> durations_current = duration_intersection.find(intersection_current)->second;
                std::array<int, 4> durations_last = duration_intersection.find(intersection_last)->second;
                int index_intersection_current = index_intersection.find(intersection_current)->second;
                int index_intersection_last = index_intersection.find(intersection_last)->second;
                auto intersection_duration = std::chrono::duration_cast<std::chrono::milliseconds>(intersection_time - intersection_time_last).count();

                connection_current.at(exit_current) = index_intersection_last;
                connection_last.at(exit_last) = index_intersection_current;
                durations_current.at(exit_current) = intersection_duration;
                durations_last.at(exit_last) = intersection_duration;
                connection_intersection.find(intersection_current)->second = connection_current;
                connection_intersection.find(intersection_last)->second = connection_last;
                duration_intersection.find(intersection_current)->second = durations_current;
                duration_intersection.find(intersection_last)->second = durations_last;

                //std::cout << index_intersection_last << " " << exit_last << " → " << index_intersection_current << " " << exit_current << std::endl;
                int weight = 1; //todo: change it to the time
                if (map_matrix[index_intersection_current][index_intersection_last] > 0){
                    int original_duration = map_matrix[index_intersection_current][index_intersection_last];
                    int current_duration = intersection_duration;
                    if (current_duration < original_duration) {
                        map_matrix[index_intersection_current][index_intersection_last] = current_duration;
                        map_matrix[index_intersection_last][index_intersection_current] = current_duration;
                    }
                } else {
                    map_matrix[index_intersection_current][index_intersection_last] = intersection_duration;
                    map_matrix[index_intersection_last][index_intersection_current] = intersection_duration;
                }
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
        for(const auto& elem : map_intersection){
            double intersection_x = std::get<0>(elem.first);
            double intersection_y = std::get<1>(elem.first);
            for (int i=0; i<4; i+=1){
                if (elem.second.at(i) == 0 ){
                    is_finished = false;
                }
            }
        }

        if (is_finished && !map_intersection.empty()){
            this->blackboard.mapping_finished.set(true);
            this->blackboard.direction_changed.set(false);
            this->blackboard.connection_map.set(connection_intersection);
            std::cout << "Mapping finished." << std::endl;
            break;
        }

    }

    std::cout << "-----" << std::endl;
    std::cout << "intersection result:" << std::endl;
    std::cout << "intersection x,y | exit visit times S-W-N-E" << std::endl;
    for(const auto& elem : map_intersection){
        int id = index_intersection.find(elem.first)->second;
        double intersection_x = std::get<0>(elem.first);
        double intersection_y = std::get<1>(elem.first);
        int south_exit =  std::get<0>(elem.second);
        int left_exit =  std::get<1>(elem.second);
        int north_exit =  std::get<2>(elem.second);
        int right_exit =  std::get<3>(elem.second);
        std::cout << id << " | " << intersection_x  << " \t" <<  intersection_y << " \t| " <<  south_exit << " \t" <<  left_exit << " \t" <<  north_exit << " \t" <<  right_exit << std::endl;
    }

    std::cout << "intersection x,y | connected to S-W-N-E" << std::endl;
    for(const auto& elem : connection_intersection){
        int id = index_intersection.find(elem.first)->second;
        double intersection_x = std::get<0>(elem.first);
        double intersection_y = std::get<1>(elem.first);
        int south_exit =  std::get<0>(elem.second);
        int left_exit =  std::get<1>(elem.second);
        int north_exit =  std::get<2>(elem.second);
        int right_exit =  std::get<3>(elem.second);
        std::cout << id << " | " << intersection_x  << " \t" <<  intersection_y << " \t| " <<  south_exit << " \t" <<  left_exit << " \t" <<  north_exit << " \t" <<  right_exit << std::endl;
    }
    std::cout << "intersection x,y | Duration to next intersection S-W-N-E" << std::endl;
    for(const auto& elem : duration_intersection){
        int id = index_intersection.find(elem.first)->second;
        double intersection_x = std::get<0>(elem.first);
        double intersection_y = std::get<1>(elem.first);
        int south_exit =  std::get<0>(elem.second);
        int left_exit =  std::get<1>(elem.second);
        int north_exit =  std::get<2>(elem.second);
        int right_exit =  std::get<3>(elem.second);
        std::cout << id << " | " << intersection_x  << " \t" <<  intersection_y << " \t| " <<  south_exit << " \t" <<  left_exit << " \t" <<  north_exit << " \t" <<  right_exit << std::endl;
    }
    std::cout << "intersection connection as matrix" << std::endl;
    for (int i = 0; i < this->mapsize; i++) {
        for (int j = 0; j < this->mapsize; j++) {
            printf("%d ", map_matrix[i][j]);
        }
        printf("\n");
    }

    return 0;
}
