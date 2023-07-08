
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
    std::cout << "cuurent exit #" << current_exit << " go to " << text_direction << std::endl;
    return direction;
}

Mapping::Mapping() {
    // No operation
}

Mapping::~Mapping() {
    // No operation
}

int Mapping::exec_thread() {

    // a dictionary to save all intersections and their exits
    std::map<std::tuple<double, double> , std::array<int, 4>> map_intersection;

    double x = 0;
    double y = 0;
    double x_last = -1;
    double y_last = -1;
    bool intersection_detected = false;
    bool intersection_detected_last;
    bool exit_left_detected;
    bool exit_right_detected;
    bool exit_middle_detected;
    direction_t direction;
    station_t station = station_t::STATION_UNKNOWN;

    this->blackboard.mapping_finished.set(false);
    this->blackboard.stations.set(std::array<Coordinates,4>{Coordinates{0,0,0},Coordinates{0,0,0},Coordinates{0,0,0},Coordinates{0,0,0}});
    std::cout << "Mapping started ..." << std::endl;

    while (this->blackboard.mapping_enabled.get()) {

        direction = direction_t::UNKNOWN;

        // get the current position
        x_last = x;
        y_last = y;
        Coordinates coordinates = this->blackboard.coordinates.get();
        x = coordinates.x;
        y = coordinates.y;
        if (x == x_last && y == y_last)
            continue;
        std::cout << "----- current: " << x << " " << y << " last: " << x_last << " " << y_last << std::endl;

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
        exit_left_detected = this->blackboard.exits_detected.get().at(0);
        exit_middle_detected = this->blackboard.exits_detected.get().at(1);
        exit_right_detected = this->blackboard.exits_detected.get().at(2);

        /* on a rising edge of intersection detected */
        if (intersection_detected && !intersection_detected_last) {
            std::tuple<double, double> current_position (x,y);

            // current_exit: the direction of entering an intersection
            std::tuple<double, double> current_exit (x-x_last, y-y_last);

            // south:0; west:1; north:2; east:3
            auto current_exit_number = (exit_t) get_exit_number(current_exit);

            /* initialise exit information arrays */
            std::array<int, 4> exits_visit_stat {0,0,0,0};
            std::tuple<double, double> current_intersection (0,0);

            /* check if the intersection is in map and update coordinates */
            bool is_recorded_intersection = false;
            for(const auto& elem : map_intersection){
                double intersection_x = std::get<0>(elem.first);
                double intersection_y = std::get<1>(elem.first);
                if ( x >= intersection_x-this->fuzziness &&  x <= intersection_x+this->fuzziness &&
                     y >= intersection_y-this->fuzziness &&  y <= intersection_y+this->fuzziness ) {
                    is_recorded_intersection = true;
                    current_intersection = std::make_tuple(intersection_x, intersection_y);
                    break;
                }
            }


            // if intersection already visited
            if (is_recorded_intersection) {
                // get the exits stat
                exits_visit_stat = map_intersection.find(current_intersection)->second;
                // comming direction +1
                exits_visit_stat.at(current_exit_number) += 1;
                // get the next exit
                direction = (direction_t) get_next_direction(exits_visit_stat, current_exit_number);
                // leaving direction +1
                exits_visit_stat.at((current_exit_number+direction)%4) += 1;
                //update
                map_intersection.find(current_intersection)->second = exits_visit_stat;
            }

            // if new intersection
            else {
                // mark not-exits at intersection
                if (!exit_left_detected) 
                    exits_visit_stat.at((current_exit_number+1)%4) = -1;
                if (!exit_middle_detected)
                    exits_visit_stat.at((current_exit_number+2)%4) = -1;
                if (!exit_right_detected)
                    exits_visit_stat.at((current_exit_number+3)%4) = -1;

                // comming direction +1
                exits_visit_stat.at((current_exit_number)%4) += 1;
                // get next exit
                direction = (direction_t) get_next_direction(exits_visit_stat, current_exit_number);
                // leaving direction +1
                exits_visit_stat.at((current_exit_number+direction)%4) += 1;
                //update
                map_intersection.insert(std::pair<std::tuple<double, double>,std::array<int, 4>>(current_position,exits_visit_stat));
            }

        }

        // update direction to blackboard
        this->blackboard.direction.set(direction);



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
            std::cout << "Mapping finished." << std::endl;
            break;
        }

    }

    std::cout << "-----" << std::endl;
    std::cout << "intersection result:" << std::endl;
    std::cout << "intersection x,y | exit visit times S-W-N-E" << std::endl;
    for(const auto& elem : map_intersection){
        double intersection_x = std::get<0>(elem.first);
        double intersection_y = std::get<1>(elem.first);
        int south_exit =  std::get<0>(elem.second);
        int left_exit =  std::get<1>(elem.second);
        int north_exit =  std::get<2>(elem.second);
        int right_exit =  std::get<3>(elem.second);
        std::cout << intersection_x  << " \t" <<  intersection_y << " \t| " <<  south_exit << " \t" <<  left_exit << " \t" <<  north_exit << " \t" <<  right_exit << std::endl;
    }

    return 0;
}
