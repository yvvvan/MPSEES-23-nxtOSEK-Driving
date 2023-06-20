#include <gtest/gtest.h>

#include "slam/localization.hpp"

TEST(LocalizationTest, Constructor) {
    const std::string vocabularyFile = "/home/pi/mpsees-23-nxtosek-driving/libs/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    const std::string configFile = "/home/pi/mpsees-23-nxtosek-driving/src/mono_raspi_cam.yaml";
    Localization localization(vocabularyFile, configFile);
}

/**
 * @brief Run the localization thread, with an example video
*/
TEST(LocalizationTest, TestORBSLAM) {
    // TODO configure this to use the test video -> mock camera or blackboard
    const std::string vocabularyFile = "/home/pi/mpsees-23-nxtosek-driving/libs/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    const std::string configFile = "/home/pi/mpsees-23-nxtosek-driving/src/mono_raspi_cam.yaml";
    Localization localization(vocabularyFile, configFile);
    localization.run();
}