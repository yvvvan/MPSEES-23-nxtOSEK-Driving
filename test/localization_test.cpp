#include "modules/slam/localization.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <future>
#include <opencv2/opencv.hpp>
#include <thread>

#include "communication/internal/BlackBoard.hpp"
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
      std::cout << "Empty frame" << std::endl;
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
      std::cout << "Empty frame" << std::endl;
      break;
    }

    l_blackboard.frame.set(frame);
  }
}

#define SPEED_ENTRY 1
#define ANGLE_ENTRY 0
#define INTERSECTION_DETECTED 2
#define TIME_ENTRY 6

/**
 * @brief Test the driving_tracking function
 */
TEST(LocalizationTest, TestDrivingTracking) {
  const std::string testFile =
      directory + "/testfiles/example_logs/curve_forward.txt";
  const std::string logFile = directory + "/testfiles/example_logs/coords.txt";
  const std::string logAngleFile =
      directory + "/testfiles/example_logs/angles.txt";
  std::ofstream logFileStream(logFile);
  std::ofstream logAngleFileStream(logAngleFile);

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
      l_blackboard.is_intersection.set(true);
    } else {
      l_blackboard.is_intersection.set(false);
    }

    long new_time = std::stol(values[TIME_ENTRY]);
    long time_diff = 1000 / 42;
    Coordinates coords = localization.driving_tracking(time_diff);
    time += time_diff;
    old_time = new_time;
    std::cout << "Time: " << time / 1000 << "s | " << new_time << " |"
              << l_blackboard.angle.get() << " " << l_blackboard.speed.get()
              << " " << l_blackboard.is_intersection.get() << " "
              << coords.to_string() << std::endl;
    logFileStream << std::setprecision(4) << coords.x << " " << coords.y
                  << std::endl;
    logAngleFileStream << std::setprecision(4) << l_blackboard.angle.get()
                       << " " << l_blackboard.speed.get() << " " << time << " "
                       << l_blackboard.is_intersection.get() << std::endl;
  }

  std::cout << "Finished reading test file" << std::endl;
  test_file.close();

  logFileStream.close();
  // read_test_file_thread.join();
}

TEST(LocalizationTest, phaseCorrelation) {
  // Open the video file
  std::string videoFile = directory + "/testfiles/driving1_video.avi";
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
  std::ofstream logShiftFile(directory + "/testfiles/example_logs/shift.txt");
  Coordinates coords;

  bool pause = false;
  long time_counter = 0;
  // Loop through the video frames
  while (true) {
    time_counter++;
    // Read the current frame
    cap >> frame;
    if (frame.empty()) break;

    // Convert frame to edges
    cvtColor(frame, edges, cv::COLOR_BGR2GRAY);
    cv::Mat mask;
    threshold(edges, mask, 200, 250, cv::THRESH_BINARY_INV);
    edges.setTo(cv::Scalar(0, 0, 0), mask);
    //    cv::imshow("Threshhold Frame", edges);
    //    Canny(edges, edges, 50, 150);  // Apply Canny edge detection

    //    prevEdges.convertTo(prevEdges, CV_8UC1);
    //    cv::imshow("Old Frame", prevEdges);
    //    prevEdges.convertTo(prevEdges, CV_32FC1);
    //    cv::imshow("Current Frame", edges);

    // Compute the phase correlation
    edges.convertTo(edges, CV_32FC1);
    cv::Point2d phaseShift = phaseCorrelate(edges, prevEdges);

    // Display the result
    std::cout << "Phase shift: " << phaseShift.x << ", " << phaseShift.y
              << ", Time: " << time_counter / 20 << "s" << std::endl;

    if (phaseShift.x > 2 || phaseShift.y > 2 || phaseShift.x < -2 ||
        phaseShift.y < -2) {
      coords.add_vector(phaseShift.x, phaseShift.y, 0);
    }

    logFileStream << std::setprecision(4) << coords.x << " " << coords.y << " "
                  << time_counter / 20 << std::endl;

    logShiftFile << std::setprecision(4) << phaseShift.x << " " << phaseShift.y
                 << " " << time_counter << std::endl;

    // Update the previous edges
    edges.copyTo(prevEdges);
    edges.release();

    // Exit the loop if 'q' is pressed
    //    cv::waitKey(0);  // Press 'p' to pause/unpause
  }

  // Release the video capture and close the windows
  cap.release();
  cv::destroyAllWindows();

  logFileStream.close();
}
