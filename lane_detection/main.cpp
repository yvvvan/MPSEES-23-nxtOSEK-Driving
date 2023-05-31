#include <iostream>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <vector>

/**
 * image_preprocessing
 *
 * Does the following preprocessing steps:
 *      - grayscaling
 *      - gaussian blurring
 *      - canny algorithm
 *      - masking
 *      - (binary image improvement - optional)
 *
 * @param frame
 * @return
 */
cv::Mat image_preprocessing(cv::Mat frame) {
  //Grayscaling
  cv::Mat grayscaled_frame;
  cv::cvtColor(frame, grayscaled_frame, cv::COLOR_BGR2GRAY);
  //Blurring
  cv::Mat blurred_frame;
  cv::GaussianBlur(grayscaled_frame, blurred_frame, cv::Size(15, 15), 0);
  //Edge Detection using Canny
  cv::Mat canny_frame;
  cv::Canny(blurred_frame, canny_frame, 50, 150);
  //Masking
  cv::Mat masking_frame(frame.size(), CV_8UC1, cv::Scalar(0));
  double mask_th = 0.6;
  int mask_height = static_cast<int>(frame.rows * mask_th);
  cv::Rect roi(0, frame.rows - mask_height, frame.cols, mask_height);
  masking_frame(roi) = 255;
  //Applying the Mask
  cv::Mat result_frame;
  canny_frame.copyTo(result_frame, masking_frame);

  return result_frame;
}

/**
 * line_filtering
 *
 * Find the right and left line on the preprocessed image. Remove unnecessary lines.
 *
 * TODO check for horizontal lines
 *
 * @param hough_lines
 * @return
 */
void line_filtering(cv::Mat preprocessed_frame, cv::Vec4i &leftLane, cv::Vec4i &rightLane, cv::Vec4i &horizontalLine, cv::Vec4i &verticalLine) {
  // perform probabilistic hough transform
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(preprocessed_frame, lines, 1, CV_PI / 180, 50, 30, 10);

  // check for horizontal lines
  // TODO

  // separate left and right lane lines based on their slope
  std::vector<cv::Vec4i> leftLanes, rightLanes;
  for (const cv::Vec4i &line : lines) {
    // calculate slope of line
    float slope = static_cast<float>(line[3] - line[1]) / static_cast<float>(line[2] - line[0]);
    // ignore lines with a small slope degree
    if (std::abs(slope) > 0.5) {
      // separate left and right lane lines
      if (slope < 0)
        leftLanes.push_back(line);
      else
        rightLanes.push_back(line);
    }
  }

  // calculate average line for left and right lane
  if (!leftLanes.empty()) {
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;

    for (const cv::Vec4i &line : leftLanes) {
      x1 += line[0];
      y1 += line[1];
      x2 += line[2];
      y2 += line[3];
    }

    // calculate average of all lines for left lane
    x1 /= leftLanes.size();
    y1 /= leftLanes.size();
    x2 /= leftLanes.size();
    y2 /= leftLanes.size();
    leftLane = cv::Vec4i(x1, y1, x2, y2);
  }

  // calculate average line for left and right lane
  if (!rightLanes.empty()) {
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;

    for (const cv::Vec4i &line : rightLanes) {
      x1 += line[0];
      y1 += line[1];
      x2 += line[2];
      y2 += line[3];
    }

    // calculate average of all lines for right lane
    x1 /= rightLanes.size();
    y1 /= rightLanes.size();
    x2 /= rightLanes.size();
    y2 /= rightLanes.size();
    rightLane = cv::Vec4i(x1, y1, x2, y2);
  }
}

/**
 * check_intersection
 *
 * Check if the car is in an intersection.
 *
 * - build mathematical functions from all lines in the form f = mx + b
 * - check if the lines intersect on the left or right side separately
 *
 * @param leftLane
 * @param rightLane
 * @param horizontalLine
 * @param verticalLine
 * @return
 */
bool check_intersection(cv::Vec4i leftLane, cv::Vec4i rightLane, cv::Vec4i horizontalLines, cv::Vec4i verticalLines) {
  // TODO Fabian
  return false;
}

/**
 * calculate_center
 *
 * Find the center between the right and left line.
 *
 * @param lane_lines
 * @return
 */
std::vector<cv::Vec4i> calculate_center(cv::Vec4i &leftLane, cv::Vec4i &rightLane) {

  std::vector<cv::Vec4i> center_line;
  //Case 1: Both lines detected
  if (leftLane != cv::Vec4i() && rightLane != cv::Vec4i()) {

    // Extract start points of the left and right lines
    cv::Point2f left_start(leftLane[0], leftLane[1]);
    cv::Point2f right_start(rightLane[2], rightLane[3]);
    double mid_start_x = (left_start.x + right_start.x) * 0.5;
    double mid_start_y = (left_start.y + right_start.y) * 0.5;

    // Extract end points of the left and right lines
    cv::Point2f left_end(leftLane[2], leftLane[3]);
    cv::Point2f right_end(rightLane[0], rightLane[1]);
    double mid_end_x = (left_end.x + right_end.x) * 0.5;
    double mid_end_y = (left_end.y + right_end.y) * 0.5;

    //create center between both lines
    cv::Vec4i center(mid_start_x, mid_start_y, mid_end_x, mid_end_y);
    center_line.push_back(center);

  } else if (leftLane != cv::Vec4i()) {

    //PLACEHOLDER FOR CASE 2 (if needed)
    std::cout << "Right line missing." << std::endl;

  } else if (rightLane != cv::Vec4i()) {

    //PLACEHOLDER FOR CASE 2 (if needed)
    std::cout << "Left line missing." << std::endl;

  } else {
    //PLACEHOLDER FOR CASE 3
    std::cout << "Both lines missing." << std::endl;

  }
  //TODO Approximation Handling if needed (CASE 2 & 3)

  return center_line;
}

/**
 * return_function
 *
 * This function communicates with other components. We are not sure what this function will return yet.
 *
 * @return
 */
int return_function() {
  // TODO Chris
  return 0;
}

/**
 * process_image_frame
 *
 * This function will be called for every image frame from the camera sensor or a video file.
 * It will call all necessary functions for processing the image frame.
 *
 * @param frame Image frame from the camera sensor, a video file or a single image
 */
void process_image_frame(cv::Mat frame) {
  // call image preprocessing
  cv::Mat preprocessed_frame = image_preprocessing(frame);

  // filter lines
  cv::Vec4i leftLane;
  cv::Vec4i rightLane;
  cv::Vec4i horizontal_lines;
  cv::Vec4i vertical_lines;
  line_filtering(preprocessed_frame, leftLane, rightLane, horizontal_lines, vertical_lines);

  // check if intersection is detected
  bool is_intersection = check_intersection(leftLane, rightLane, horizontal_lines, vertical_lines);

  // calculate center
  std::vector<cv::Vec4i> lane_lines = calculate_center(leftLane, rightLane);
  // send result to other components using the return_function function
  // TODO complete the return_function function
  return_function();

  // draw lan_lines on frame
  for (int i = 0; i < lane_lines.size(); i++) {
    cv::Vec4i l = lane_lines[i];
    cv::line(frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
  }

  // display processed frame
  cv::imshow("Processed Frame", preprocessed_frame);
  cv::waitKey(0);
}

/**
 * main_loop_camera
 *
 * This main loop will call all necessary functions for processing the image frames from the camera sensor.
 */
void main_loop_camera() {

  // TODO CHRIS
}

/**
 * main_loop_video
 *
 * This main loop will call all necessary functions for processing the image frames from a video.
 *
 * @param video_path Path to the video file
 */
void main_loop_video(const std::string &video_path) {
  // open video file
  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    std::cout << "Error opening video stream or file" << std::endl;
    return;
  }

  // creating an opencv window to display the processed frames
  cv::namedWindow("Processed Frame", cv::WINDOW_NORMAL);

  while (true) {
    // read frame
    cv::Mat frame;
    cap >> frame;

    // if frame is empty, video is over
    if (frame.empty()) {
      break;
    }

    // process image frame and display it
    process_image_frame(frame);

    // exit if ESC is pressed
    if (cv::waitKey(1) == 27) {
      break;
    }
  }
}

/**
 * Main function, parses command line arguments and calls the main loop.
 *
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return 0 if program was executed successfully, 1 otherwise
 */
int main(int argc, char *argv[]) {
  // parse command line arguments
  //      -image for a path to an image (will skip main loop)
  //      -video for a path to a video
  //      no argument for normal operation with camera sensor
  if (strcmp(argv[1], "-image") == 0) {
    // single image will be analyzed, skip main loop
    cv::Mat frame = cv::imread(argv[2]);
    // creating an opencv window to display the processed frames
    cv::namedWindow("Processed Frame", cv::WINDOW_NORMAL);
    // process image frame
    process_image_frame(frame);
  } else if (strcmp(argv[1], "-video") == 0) {
    // video frames will be analyzed
    main_loop_video(argv[2]);
  } else if (argc < 2) {
    // normal operation with camera sensor
    main_loop_camera();
  } else {
    std::cout << "Please provide a valid argument." << std::endl;
    return 1;
  }

  return 0;
}