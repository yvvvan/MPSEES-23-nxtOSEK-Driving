#include <gtest/gtest.h>
#include <future>
#include <thread>
#include <opencv2/opencv.hpp>

#include "slam/localization.hpp"
#include "blackboard/BlackBoard.hpp"

#define video

BlackBoard &l_blackboard = BlackBoard::getInstance();

std::string file {__FILE__};
std::string directory {file.substr(0, file.rfind('/'))};

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
//        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
//        cv::Canny(frame, frame, 50, 150);

        l_blackboard.frame.set(frame);

        cv::waitKey(50);
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}

/**
 * @brief function which captures the current frame from the camera and writes it to the blackboard
 */
int runCamera() {
    cv::VideoCapture cap(0);
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

        l_blackboard.frame.set(frame);
    }
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
    const std::string vocabularyFile = directory + "/../lib/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    const std::string configFile = directory + "/testfiles/video_camera_config.yaml";
    const std::string videoFile = directory + "/testfiles/code_output.avi";

//    blackboard.camera_enabled = true;
//    runVideo(videoFile);

    Localization localization(vocabularyFile, configFile);

    l_blackboard.camera_enabled = true;
    l_blackboard.localization_enabled = true;

#ifdef video
    auto video_thread = std::async(runVideo, videoFile);
#else
    auto camera_thread = std::async(runCamera);
#endif

    auto localization_thread = std::async(&Localization::exec_thread, &localization);
//    while (video_thread.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
//        cv::imshow("Frame", blackboard.frame.get());
//    }
#ifdef video
    video_thread.wait();
    ASSERT_EQ(video_thread.get(), 0);
#else
    camera_thread.wait();
    ASSERT_EQ(camera_thread.get(), 0);
#endif

    l_blackboard.localization_enabled = false;
    l_blackboard.camera_enabled = false;
    localization_thread.wait();
    ASSERT_EQ(localization_thread.get(), 0);
}