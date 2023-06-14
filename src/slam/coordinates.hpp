#ifndef COORDINATES_HPP
#define COORDINATES_HPP

class Coordinates {
    public:
        /**
         * @brief Construct a new Coordinates object
         * 
         * @param x 
         * @param y 
         */
        Coordinates(double x, double y, double z, double angle);

        /**
         * @brief Destroy the Coordinates object
         * 
         */
        ~Coordinates();

        double x;
        double y;
        double z;
        double angle;
}

#endif //COORDINATES_HPP