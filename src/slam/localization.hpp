#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include <opencv2/opencv.hpp>
#include <System.h>

#include "blackboard/BlackBoard.hpp"

class Localization {
    public:
        // TODO should this be a singleton?
        /**
         * @brief Construct a new Localization object
         * 
         * @param vocabularyFile path to the ORB_SLAM vocabulary file (ORB_SLAM3/Vocabulary/ORBvoc.txt)
         * @param configFile path to the ORB_SLAM camera config file
         * 
         * @return Localization&
        */
        Localization(std::string vocabularyFile, std::string configFile);

        /**
         * @brief Destroy the Localization object
        */
        ~Localization();

        /**
         * @brief Runs the localization thread
         * 
         * @return int 0 on success, -1 on failure
         */
        int exec_thread();

    private:

        /* ORB SLAM config files */
        std::string vocabularyFile;
        std::string configFile;

        /* ORB SLAM system */
        ORB_SLAM3::System *slam;

        /* Blackboard */
        BlackBoard &blackboard = BlackBoard::getInstance();
};

#endif // LOCALIZATION_HPP