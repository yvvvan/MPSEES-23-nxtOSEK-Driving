#ifndef COORDINATES_HPP
#define COORDINATES_HPP

typedef enum Direction {
    LEFT = 1,
    STRAIGHT = 2,
    RIGHT = 3,
    UNKNOWN = 0
} direction_t;

typedef enum Exit {
    LEFT_EXIT = 1,
    MIDDLE_EXIT = 2,
    RIGHT_EXIT = 3,
    CURRENT_EXIT = 0
} exit_t;

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

        double x;
        double y;
        double z;
};

#endif //COORDINATES_HPP