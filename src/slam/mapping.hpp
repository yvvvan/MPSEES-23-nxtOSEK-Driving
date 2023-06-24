#ifndef MAPPING_HPP
#define MAPPING_HPP

#include "blackboard/BlackBoard.hpp"

class Mapping {

    public:

        /**
         * @brief Construct a new Mapping object
         *
         * @return Mapping&
         */
        Mapping();

        /**
         * @brief Destroy the Mapping object
         *
         */
        ~Mapping();

        /**
         * @brief Execute the mapping thread
         *
         * @return 0 on success, -1 on failure
         */
        int exec_thread();

    private:

        // the range of the intersection detection
        const double fuzziness = 0.5;
        // the size of the map (the minimum is intersection + station + 1)
        const int mapsize = 10;

        BlackBoard &blackboard = BlackBoard::getInstance();

};


#endif // MAPPING_HPP
