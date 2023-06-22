#include <gtest/gtest.h>
#include <future>
#include <thread>
#include <opencv2/opencv.hpp>

#include "slam/localization.hpp"
#include "blackboard/BlackBoard.hpp"

BlackBoard &l_blackboard = BlackBoard::getInstance();

/**
 * @brief function which can be run as a thread and goes through the video frame by frame
 */
int runVideo(const std::string &videoFile) {
    cv::VideoCapture cap(videoFile);
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    while (l_blackboard.camera_enabled.get()) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            cout << "Empty frame" << endl;
            break;
        }

        /* show frame */
//        cv::imshow("Frame", frame);

        l_blackboard.frame = frame;

        cv::waitKey(200);
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}


/**
 * @brief Test the constructor of the Localization class
*/
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
    const std::string videoFile = "/home/jakob/Documents/SESE_Projekt/mpsees/test/test_video_with_controller.mp4";

//    blackboard.camera_enabled = true;
//    runVideo(videoFile);

    Localization localization(vocabularyFile, configFile);

    l_blackboard.camera_enabled = true;
    l_blackboard.localization_enabled = true;

    auto video_thread = std::async(runVideo, videoFile);

    auto localization_thread = std::async(&Localization::exec_thread, &localization);
//    while (video_thread.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
//        cv::imshow("Frame", blackboard.frame.get());
//    }
    video_thread.wait();
    ASSERT_EQ(video_thread.get(), 0);

    l_blackboard.localization_enabled = false;
    l_blackboard.camera_enabled = false;
    localization_thread.wait();
    ASSERT_EQ(localization_thread.get(), 0);
}