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

#ifdef USE_ORB_SLAM
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
#endif

#define SPEED_ENTRY 0
#define ANGLE_ENTRY 1
#define INTERSECTION_DETECTED 2
#define TIME_ENTRY 6

/**
 * @brief Test the driving_tracking function
 */
TEST(LocalizationTest, TestDrivingTracking) {
  const std::string testFile =
      directory + "/testfiles/example_logs/simple_loop.txt";
  const std::string logFile = directory + "/testfiles/example_logs/coords.txt";
  std::ofstream logFileStream(logFile);

  /* open test file */
  auto test_file = std::ifstream(testFile);
  if (!test_file.is_open()) {
    std::cout << "Error opening trajectory file" << std::endl;
    FAIL();
  }

  Localization localization;

  l_blackboard.localization_enabled.set(true);

  localization.reset_clock();
  std::queue<double> angles;
  std::queue<double> speeds;
  long old_time = 0;
  long time = 0;
  std::string line;
  while (std::getline(test_file, line)) {
    line.erase(std::remove(line.begin(), line.end(), ','), line.end());
    auto values = splitStringBySpace(line);

    l_blackboard.speed.set(std::stod(values[SPEED_ENTRY]));
    l_blackboard.angle.set(std::stod(values[ANGLE_ENTRY]));

    if (values[INTERSECTION_DETECTED] == "1") {
      l_blackboard.intersection_detected.set(true);
    } else {
      l_blackboard.intersection_detected.set(false);
    }

    long new_time = std::stol(values[TIME_ENTRY]);
    long time_diff = new_time - old_time;
    Coordinates coords = localization.driving_tracking(time_diff);
    time += time_diff;
    old_time = new_time;
    std::cout << "Time: " << time / 1000 << "s | " << new_time << " |"
              << l_blackboard.angle.get() << " " << l_blackboard.speed.get()
              << " " << l_blackboard.intersection_detected.get() << " "
              << coords.to_string() << std::endl;
    logFileStream << setprecision(4) << coords.x << " " << coords.y
                  << std::endl;
  }

  cout << "Finished reading test file" << endl;
  test_file.close();

  logFileStream.close();
  // read_test_file_thread.join();
}

TEST(LocalizationTest, phaseCorrelation) {
  // Open the video file
  std::string videoFile = directory + "/testfiles/simple_loop.avi";
  cv::VideoCapture cap(videoFile);
  if (!cap.isOpened()) {
    std::cout << "Failed to open the video file." << std::endl;
    FAIL();
  }

  cv::Mat frame, prevFrame;
  cv::Mat prevEdges, edges;

  // Read the first frame
  cap >> prevFrame;
  cvtColor(prevFrame, prevEdges, cv::COLOR_BGR2GRAY);
//  Canny(prevEdges, prevEdges, 100, 150);  // Apply Canny edge detection
  prevEdges.convertTo(prevEdges, CV_32FC1);

  // Create coordinate result file
  std::ofstream logFileStream(directory + "/testfiles/example_logs/coords.txt");
  Coordinates coords;

  bool pause = false;
  // Loop through the video frames
  while (true) {
    // Read the current frame
    cap >> frame;
    if (frame.empty()) break;

    // Convert frame to edges
    cvtColor(frame, edges, cv::COLOR_BGR2GRAY);
//    cv::Mat mask;
//    threshold(edges, mask, 200nn, 250, cv::THRESH_BINARY_INV);
//    edges.setTo(cv::Scalar(0, 0 ,0), mask);
//    cv::imshow("Threshhold Frame", edges);
//    Canny(edges, edges, 50, 150);  // Apply Canny edge detection


    cv::imshow("Current Frame", edges);
    cv::imshow("Old Frame", prevEdges);

    // Compute the phase correlation
    edges.convertTo(edges, CV_32FC1);
    cv::Point2d phaseShift = phaseCorrelate(edges, prevEdges);

    // Display the result
    std::cout << "Phase shift: " << phaseShift.x << ", " << phaseShift.y
              << std::endl;

    coords.add_vector(phaseShift.x, phaseShift.y, 0);

    logFileStream << setprecision(4) << coords.x << " " << coords.y
                  << std::endl;

    // Update the previous edges
    edges.copyTo(prevEdges);

    // Exit the loop if 'q' is pressed
//    cv::waitKey(0);  // Press 'p' to pause/unpause

  }

  // Release the video capture and close the windows
  cap.release();
  cv::destroyAllWindows();

  logFileStream.close();
}
