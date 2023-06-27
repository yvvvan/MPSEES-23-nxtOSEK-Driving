#include "slam/localization.hpp"

#include <gtest/gtest.h>

#include <future>
#include <opencv2/opencv.hpp>
#include <thread>

#include "blackboard/BlackBoard.hpp"
#include "helper.hpp"

#define video

BlackBoard &l_blackboard = BlackBoard::getInstance();

std::string file{__FILE__};
std::string directory{file.substr(0, file.rfind('/'))};

/**
 * @brief function which can be run as a thread and goes through the video frame
 * by frame
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
 * @brief function which captures the current frame from the camera and writes
 * it to the blackboard
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

#define SPEED_ENTRY 0
#define ANGLE_ENTRY 1
#define TIME_ENTRY 2

/**
 * @brief Read the test file for the driving tracking function
 */
int readDrivingTrackingTestFile(const std::string &testFile) {
  /* open test file */
  auto test_file = std::ifstream(testFile);
  if (!test_file.is_open()) {
    std::cout << "Error opening trajectory file" << std::endl;
    return -1;
  }

  std::string line;
  while (std::getline(test_file, line)) {
    auto values = splitStringBySpace(line);

    l_blackboard.speed.set(std::stod(values[SPEED_ENTRY]));
    l_blackboard.angle.set(std::stod(values[ANGLE_ENTRY]));

    std::this_thread::sleep_for(
        std::chrono::milliseconds(std::stol(values[TIME_ENTRY])));
  }
  cout << "Finished reading test file" << endl;
  test_file.close();
  l_blackboard.localization_enabled.set(false);
  return 0;
}

/**
 * @brief Test the constructor of the Localization class
 */
TEST(LocalizationTest, Constructor) {
  const std::string vocabularyFile =
      "/home/jakob/Documents/SESE_Projekt/mpsees/lib/ORB_SLAM3/Vocabulary/"
      "ORBvoc.txt";
  const std::string configFile =
      "/home/jakob/Documents/SESE_Projekt/mpsees/src/slam/mono_raspi_cam.yaml";
  Localization localization(vocabularyFile, configFile);
}

/**
 * @brief Run the localization thread, with an example video
 */
TEST(LocalizationTest, TestORBSLAM) {
  // TODO configure this to use the test video -> mock camera or blackboard
  const std::string vocabularyFile =
      directory + "/../lib/ORB_SLAM3/Vocabulary/ORBvoc.txt";
  const std::string configFile =
      directory + "/testfiles/video_camera_config.yaml";
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

  auto localization_thread =
      std::async(&Localization::exec_thread, &localization);
//    while (video_thread.wait_for(std::chrono::milliseconds(50)) !=
//    std::future_status::ready) {
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

/**
 * @brief Test the driving_tracking function
 */
TEST(LocalizationTest, TestDrivingTracking) {
  const std::string vocabularyFile =
      directory + "/../lib/ORB_SLAM3/Vocabulary/ORBvoc.txt";
  const std::string configFile =
      directory + "/testfiles/video_camera_config.yaml";
  const std::string testFile =
      directory + "/testfiles/driving_tracking_test.txt";

  Localization localization(vocabularyFile, configFile);

  l_blackboard.localization_enabled.set(true);

  auto read_test_file_thread =
      std::thread(readDrivingTrackingTestFile, testFile);

  localization.reset_clock();
  while (l_blackboard.localization_enabled.get()) {
    Coordinates coords = localization.driving_tracking();
    std::cout << std::setprecision(2) << coords.to_string() << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // read_test_file_thread.join();
}
