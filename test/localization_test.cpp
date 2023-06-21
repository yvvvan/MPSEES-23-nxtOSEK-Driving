#include <gtest/gtest.h>

#include "slam/localization.hpp"

TEST(LocalizationTest, Constructor) {
    const std::string vocabularyFile = "/home/jakob/Documents/SESE_Projekt/mpsees/lib/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    const std::string configFile = "/home/jakob/Documents/SESE_Projekt/mpsees/src/slam/mono_raspi_cam.yaml";
    Localization localization(vocabularyFile, configFile);
}

/**
 * @brief Run the localization thread, with an example video
*/
TEST(LocalizationTest, TestORBSLAM) {
    // TODO configure this to use the test video -> mock camera or blackboard
    const std::string vocabularyFile = "/home/jakob/Documents/SESE_Projekt/mpsees/lib/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    const std::string configFile = "/home/jakob/Documents/SESE_Projekt/mpsees/src/slam/mono_raspi_cam.yaml";
    Localization localization(vocabularyFile, configFile);
    localization.exec_thread();
}