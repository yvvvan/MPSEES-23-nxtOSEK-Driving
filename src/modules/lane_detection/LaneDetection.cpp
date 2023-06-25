#include "LaneDetection.hpp"

#define VISUALIZE 0
#define WRITE_VIDEO 1

/**
 * image_preprocessing
 *
 * Does the following preprocessing steps:
 *      - gray-scaling
 *      - gaussian blurring
 *      - canny algorithm
 *      - masking
 *      - (binary image improvement - optional)
 *
 * @param frame
 * @return cv::Mat the preprocessed image
 */
void LaneDetection::image_preprocessing() {
  // gray-scaling
  cv::cvtColor(this->frame, this->frame, cv::COLOR_BGR2GRAY);

  // blurring
  cv::GaussianBlur(this->frame, this->frame, cv::Size(13, 13), 0);

  // edge Detection using Canny
  cv::Canny(this->frame, this->frame, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2);

  // masking
  /*cv::Mat masking_frame(frame.size(), CV_8UC1, cv::Scalar(0));
  double mask_th = 0.6;
  int mask_height = static_cast<int>(this->frame.rows * mask_th);
  cv::Rect roi(0, this->frame.rows - mask_height, this->frame.cols, mask_height);
  masking_frame(roi) = 255;

  // applying the Mask
  cv::bitwise_and(this->frame, masking_frame, this->frame);*/
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
void LaneDetection::line_filtering() {
  // perform probabilistic hough transform
  std::vector<cv::Vec4d> lines;
  cv::HoughLinesP(this->frame, lines, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESHOLD, HOUGH_MIN_LINE_LEN, HOUGH_MAX_LINE_GAP);

#if VISUALIZE
  cv::Mat houghImage = cv::Mat::zeros(preprocessed_frame.size(), CV_8UC3);
  for (const cv::Vec4d &line : lines) {
    cv::line(houghImage, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 1);
  }
  cv::imshow("Hough", houghImage);
#endif

  // separate left and right lane lines based on their slope
  std::vector<cv::Vec4d> leftLanes, rightLanes;
  for (const cv::Vec4d &line : lines) {
    // calculate slope of line
    double slope = static_cast<double>(line[3] - line[1]) / static_cast<double>(line[2] - line[0]);
    // ignore lines with a small slope degree
    if (std::abs(slope) > LANE_DETECTION_MIN_SLOPE) {
      // separate left and right lane lines
      //we need to check for both (slope and middle of the image) because if not we would detect a centerline outside the lane while turning in an intersection
      if (slope < 0 && line[2] < IMAGE_MIDDLE)
        leftLanes.push_back(line);
      else if (slope > 0 && line[2] > IMAGE_MIDDLE)
        rightLanes.push_back(line);
    }
  }

  // calculate average line for left and right lane
  if (!leftLanes.empty()) {
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;

    for (const cv::Vec4d &line : leftLanes) {
      x1 += line[0];
      y1 += line[1];
      x2 += line[2];
      y2 += line[3];
    }

    // calculate average of all lines for left lane
    x1 /= static_cast<double>(leftLanes.size());
    y1 /= static_cast<double>(leftLanes.size());
    x2 /= static_cast<double>(leftLanes.size());
    y2 /= static_cast<double>(leftLanes.size());
    leftLane = cv::Vec4d(x1, y1, x2, y2);
    hasLeftLane = true;
  }

  // calculate average line for left and right lane
  if (!rightLanes.empty()) {
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;

    for (const cv::Vec4d &line : rightLanes) {
      x1 += line[0];
      y1 += line[1];
      x2 += line[2];
      y2 += line[3];
    }

    // calculate average of all lines for right lane
    x1 /= static_cast<double>(rightLanes.size());
    y1 /= static_cast<double>(rightLanes.size());
    x2 /= static_cast<double>(rightLanes.size());
    y2 /= static_cast<double>(rightLanes.size());
    rightLane = cv::Vec4d(x2, y2, x1, y1); // change order, so that it's like in a normal coordinate system
    hasRightLane = true;
  }

  // check for horizontal lines
  for (const cv::Vec4d &line : lines) {
    // calculate dx and dy
    double dx = abs(line[0] - line[2]);
    double dy = abs(line[1] - line[3]);

    // ignore lines with a small slope degree
    if (dx > 6 * dy) this->horizontalLines.push_back(line);
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
 * @return bool true if the frame contains an intersection
 */
void LaneDetection::check_intersection() {
  // Calculate the x-coordinate of the center of the image
  double th = IMAGE_MIDDLE / 5;
  std::array<bool, 3> temp_intersections = {false, false, false};
  bool temp_dead_end = false;

  // Loop through each horizontal line
  for (const auto &horizontalLine : this->horizontalLines) {
    // y = m * x + b for the horizontal line
    double m_horizontal = (horizontalLine[1] - horizontalLine[3]) / (horizontalLine[0] - horizontalLine[2]);
    double b_horizontal = horizontalLine[1] - (m_horizontal * horizontalLine[0]);
    // calculate the x-coordinate of the center of the horizontal line
    double hx = (horizontalLine[0] + horizontalLine[2]) / 2;

    // checking for intersections on the left side
    if (horizontalLine[0] < IMAGE_MIDDLE && horizontalLine[2] < IMAGE_MIDDLE) {

      // Calculate the intersection point with the left vertical line
      // m = (y2-y1)/(x2-x1)
      double mLeft = (this->leftLane[1] - this->leftLane[3]) / (this->leftLane[0] - this->leftLane[2]);
      // b = y1 - (m * x1)
      double bLeft = this->leftLane[1] - (mLeft * this->leftLane[0]);
      double intersectionX = -(b_horizontal - bLeft) / (m_horizontal - mLeft);

      // Check if the intersection point is within the left lane segment
      if (intersectionX < IMAGE_MIDDLE && 0 <= intersectionX) {
#if VISUALIZE
        double intersectionY = (mLeft * intersectionX) + bLeft;
        cv::circle(this->original_frame,
                   cv::Point2d(intersectionX, intersectionY),
                   3,
                   cv::Scalar(255, 0, 0),
                   3); //just for testing
#endif

        // there is an intersection on the left side
        temp_intersections[0] = true;

        if (hx > IMAGE_MIDDLE - th && hx < IMAGE_MIDDLE + th && this->rightLane == cv::Vec4d()) {
          // there is an intersection on the left but no right line -> not able to turn left or go ahead, just right is possible
          temp_intersections[0] = false;
          temp_intersections[2] = true;
        } else if (hx > IMAGE_MIDDLE - th && hx < IMAGE_MIDDLE + th && this->leftLane != cv::Vec4d()
            && this->rightLane != cv::Vec4d()) {
          // the line is in the center of the image and there is a left and a right line -> not able to turn right, left or go ahead
          temp_intersections[0] = false;
          this->is_dead_end.push_front(true);
          temp_dead_end = true;
        } else if (horizontalLine[1] > this->leftLane[1] || horizontalLine[3] > this->leftLane[3]) {
          // the line is above the right lane, so it must be possible to go straight ahead
          temp_intersections[1] = true;
        }
      }
    }

    // checking for intersections on the right side
    if (horizontalLine[0] > IMAGE_MIDDLE && horizontalLine[2] > IMAGE_MIDDLE) {
      // Calculate the intersection point with the right vertical line
      double mRight = (this->rightLane[1] - this->rightLane[3]) / (this->rightLane[0] - this->rightLane[2]);
      double bRight = this->rightLane[1] - (mRight * this->rightLane[0]);
      double intersectionX = -(b_horizontal - bRight) / (m_horizontal - mRight);

      // Check if the intersection point is within the right lane segment
      if (intersectionX > IMAGE_MIDDLE && intersectionX <= 2 * IMAGE_MIDDLE) {
#if VISUALIZE
        double intersectionY = (mRight * intersectionX) + bRight;
        cv::circle(this->original_frame,
                   cv::Point2d(intersectionX, intersectionY),
                   3,
                   cv::Scalar(255, 0, 0),
                   3); //just for testing
#endif

        // there is an intersection on the right side
        temp_intersections[2] = true;

        if (hx > IMAGE_MIDDLE - th && hx < IMAGE_MIDDLE + th && this->leftLane == cv::Vec4d()) {
          // there is an intersection on the right but no left line -> not able to turn right or go ahead, just left is possible
          temp_intersections[0] = true;
          temp_intersections[2] = false;
        } else if (hx > IMAGE_MIDDLE - th && hx < IMAGE_MIDDLE + th && this->leftLane != cv::Vec4d()
            && this->rightLane != cv::Vec4d()) {
          // the line is in the center of the image and there is a left and a right line -> not able to turn right, left or go ahead
          temp_intersections[0] = false; // not able to turn left, even if it was detected earlier
          temp_intersections[2] = false;
          this->is_dead_end.push_front(true);
          temp_dead_end = true;
        } else if (horizontalLine[1] > this->rightLane[1] || horizontalLine[3] > this->rightLane[3]) {
          // the line is above the right lane, so it must be possible to go straight ahead
          temp_intersections[1] = true;
        }
      }
    }
  }

  // if dead end is not detected, push false
  if (!temp_dead_end) {
    this->is_dead_end.push_front(false);
  }

  // push to queue, pop if queue is full
  if (this->exits_intersection.size() == QUEUE_SIZE) {
    this->exits_intersection.pop_back();
  }

  this->exits_intersection.push_front(temp_intersections);

  // if is_intersection queue is full, pop the oldest element
  if (this->is_intersection.size() == QUEUE_SIZE) {
    this->is_intersection.pop_back();
  }

  // check if an intersection is visible and push to queue
  if (temp_intersections[0] || temp_intersections[1] || temp_intersections[2]) {
    this->is_intersection.push_front(true);
  } else {
    this->is_intersection.push_front(false);
  }

  // - exits_intersection[0] = left
  // - exits_intersection[1] = straight
  // - exits_intersection[2] = right
}

/**
 * calculate_center
 *
 * Find the center between the right and left line.
 *
 * @return std::vector<cv::Vec4d> the center line
 */
void LaneDetection::calculate_center() {
  bool wasIntersection = this->blackboard.is_intersection.get();

  if (wasIntersection) {
    // currently in an intersection, so we just leave the queue(s) as is
    return;
  }

  // Case 1: Both lines detected
  if ((this->leftLane != cv::Vec4d() && this->rightLane != cv::Vec4d())) {

    // Extract start points of the left and right lines
    cv::Point2d left_start(this->leftLane[0], this->leftLane[1]);
    cv::Point2d right_start(this->rightLane[0], this->rightLane[1]);
    double mid_start_x = (left_start.x + right_start.x) * 0.5;
    double mid_start_y = (left_start.y + right_start.y) * 0.5;

    // Extract end points of the left and right lines
    cv::Point2d left_end(this->leftLane[2], this->leftLane[3]);
    cv::Point2d right_end(this->rightLane[2], this->rightLane[3]);
    double mid_end_x = (left_end.x + right_end.x) * 0.5;
    double mid_end_y = (left_end.y + right_end.y) * 0.5;

    // create center between both lines
    cv::Vec4d center(mid_start_x, mid_start_y, mid_end_x, mid_end_y);
    this->centerLine.push_back(center);

    //double mid_x = (mid_start_x + mid_end_x) * 0.5 - IMAGE_MIDDLE;
    double slope = (mid_end_y - mid_start_y) / (mid_end_x - mid_start_x);

    // if we are in an intersection, we don't want to save the offset
    // if we are not in an intersection, we want to save the offset * 2
    if (!this->is_intersection.front()) {
      // Limit the queue size to 4 elements
      if (this->offset_queue.size() >= QUEUE_SIZE) {
        this->offset_queue.pop_back();
      }
      this->offset_queue.push_front(slope);
    }

  } else if (leftLane != cv::Vec4d()) {
    // double mid_x_right = ((this->leftLane[0] + this->leftLane[2]) * 0.5 - IMAGE_MIDDLE);
    double slope = (this->leftLane[3] - this->leftLane[1]) / (this->leftLane[2] - this->leftLane[0]);

    // if we are in an intersection, we don't want to save the offset
    // if we are not in an intersection, we want to save the offset * 2
    if (!this->is_intersection.front()) {
      // Limit the queue size to 4 elements
      if (this->offset_queue.size() >= QUEUE_SIZE) {
        this->offset_queue.pop_back();
      }
      // Dividing because we want to flatten the slope, not steepen it
      this->offset_queue.push_front(slope / MISSING_LANE_MULTIPLIER);
    }

  } else if (rightLane != cv::Vec4d()) {
    // double mid_x_left = ((this->rightLane[0] + this->rightLane[2]) * 0.5 - IMAGE_MIDDLE);
    double slope = (this->rightLane[3] - this->rightLane[1]) / (this->rightLane[2] - this->rightLane[0]);

    // if we are in an intersection, we don't want to save the offset
    // if we are not in an intersection, we want to save the offset * 2
    if (!this->is_intersection.front()) {
      // Limit the queue size to 4 elements
      if (this->offset_queue.size() >= QUEUE_SIZE) {
        this->offset_queue.pop_back();
      }
      this->offset_queue.push_front(
          slope / MISSING_LANE_MULTIPLIER); //Dividing because we want to flatten the slope, not steepen it
    }
  } else {
    // PLACEHOLDER FOR CASE 4
    // TODO: maybe also a queue, or a simple counter that, when a threshold is reached, reports to the blackboard
    std::cout << "Both lines missing." << std::endl;
  }
}

/*
 * just for drawing lines on the image, just needed for the visualization part in the "process_image"-function
 */
void drawLaneLine(cv::Mat &original_image, const cv::Vec4d &laneLine, const cv::Scalar &color) {
  cv::line(original_image, cv::Point2d(laneLine[0], laneLine[1]), cv::Point2d(laneLine[2], laneLine[3]), color, 2);
}

void drawLaneLines(cv::Mat &original_image, std::vector<cv::Vec4d> &laneLines, const cv::Scalar &color) {
  for (const cv::Vec4d &line : laneLines) {
    cv::line(original_image, cv::Point2d(line[0], line[1]), cv::Point2d(line[2], line[3]), color, 2);
  }
}

/**
 * Analyzes the arrays inside the exits_intersection queue and returns the array that occurs most often.
 *
 * @return std::array<bool, 3> the array that occurs most often
 */
std::array<bool, 3> LaneDetection::find_majority_exits_intersection() {
  std::map<std::array<bool, 3>, int> frequencyMap;

  // Count the frequency of each array
  for (const auto &arr : this->exits_intersection) {
    frequencyMap[arr]++;
  }

  // Find the array with the highest count
  int maxCount = 0;
  std::array<bool, 3> majorityArray{};
  for (const auto &pair : frequencyMap) {
    if (pair.second > maxCount) {
      maxCount = pair.second;
      majorityArray = pair.first;
    }
  }

  return majorityArray;
}

bool LaneDetection::find_majority_bool_deque(std::deque<bool> &boolDeque) {
  std::map<bool, int> frequencyMap;

  // Count the frequency of each element
  for (const auto &element : boolDeque) {
    frequencyMap[element]++;
  }

  // Find the element with the highest count
  int maxCount = 0;
  bool majorityElement = false; // Assuming 'false' as the default majority element
  for (const auto &pair : frequencyMap) {
    if (pair.second > maxCount) {
      maxCount = pair.second;
      majorityElement = pair.first;
    }
  }

  return majorityElement;
}

/**
 * return_function
 *
 * This function communicates with other components. We are not sure what this function will return yet.
 *
 * @return
 */
void LaneDetection::return_function() {
  // write values to blackboard
  // TODO write dead end to blackboard
  double angle = calculate_center_offset_average();

  std::array<bool, 3> debug_result_exits_intersection = find_majority_exits_intersection();

  std::cout
      << "LaneDetection: "
      << "angle: " << angle
      << ",\tisIntersection: " << find_majority_bool_deque(this->is_intersection)
      << ", exits_intersection: " << debug_result_exits_intersection[0] << debug_result_exits_intersection[1]
      << debug_result_exits_intersection[2]
      << ", is_dead_end: " << find_majority_bool_deque(this->is_dead_end)
      << ", hasLeftLane: " << this->hasLeftLane
      << ", hasRightLane: " << this->hasRightLane
      << std::endl;

  int lane_count = 0;
  if (this->hasLeftLane) {
    lane_count++;
  }
  if (this->hasRightLane) {
    lane_count++;
  }

  this->blackboard.lane_count = lane_count;
  this->blackboard.offset_middle_line.set(angle);

  this->blackboard.is_intersection.set(find_majority_bool_deque(this->is_intersection));
  this->blackboard.exits_intersection.set(find_majority_exits_intersection());
  this->blackboard.is_dead_end.set(find_majority_bool_deque(this->is_dead_end));
}

/**
 * process_image_frame
 *
 * This function will be called for every image currentFrame from the camera sensor or a video file.
 * It will call all necessary functions for processing the image currentFrame.
 *
 * @param currentFrame Image currentFrame from the camera sensor, a video file or a single image
 */
void LaneDetection::process_image_frame() {
  /***** Clear all vectors and bools *****/
  // set all vector values to null so that we can check if a line was detected
  this->leftLane = cv::Vec4d();
  this->hasLeftLane = false;
  this->rightLane = cv::Vec4d();
  this->hasRightLane = false;
  this->centerLine.clear();
  this->horizontalLines.clear();

  // if we get a "turning" signal from the blackboard, purge all queues
  if (this->blackboard.has_turned.get()) {
    this->offset_queue = std::deque<double>();
    this->is_intersection = std::deque<bool>();
    this->exits_intersection = std::deque<std::array<bool, 3 >>();
    this->is_dead_end = std::deque<bool>();
  }

  /***** End of Clearing *****/

  // call image preprocessing
  image_preprocessing();

  // call line detection
  line_filtering();

  // check if intersection is detected
  // TODO complete intersection detection
  check_intersection();

  // calculate center
  calculate_center();

#if VISUALIZE
  /* Draw lane lines on the result image*/
  cv::Vec4d
      actual_center(this->original_frame.cols / 2.0, 0, this->original_frame.cols / 2.0, this->original_frame.rows);
  drawLaneLines(this->original_frame, this->centerLine, cv::Scalar(0, 0, 255));
  drawLaneLine(this->original_frame, actual_center, cv::Scalar(255, 0, 0));
  drawLaneLine(this->original_frame, leftLane, cv::Scalar(0, 255, 255));           // yellow
  drawLaneLine(this->original_frame, rightLane, cv::Scalar(255, 255, 0));          // cyan
  drawLaneLines(this->original_frame, this->horizontalLines, cv::Scalar(0, 0, 0)); // black

  cv::imshow("Result with Centerline and actual center of the image", result);
  cv::imshow("Processed Frame", this->frame);
  cv::waitKey(0);
#endif

  // send result to other components using the return_function function
  return_function();
}

/**
 * main_loop_camera
 *
 * This main loop will call all necessary functions for processing the image frames from the camera sensor. The camera
 * sensor is a raspberry pi camera module, which is attached to a raspberry pi 4 running raspbian.
 */
void LaneDetection::main_loop_camera() {
  // connecting to the camera sensor
  cv::VideoCapture capture;

  // open and close again, because for some reason the first
  // time after boot looks extremely different from any time else
  capture.open(0);
  capture.release();

  // actually open
  capture.open(0);

  if (!capture.isOpened()) {
    std::cerr << "Failure during camera initialization" << std::endl;
    return;
  }

  capture.set(cv::CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);
  capture.set(cv::CAP_PROP_FPS, FPS);

  // Check if camera opened successfully
  {
    cv::Mat image;
    capture >> image;

    if (image.empty()) {
      std::cerr << "Failure during camera initialization" << std::endl;
      return;
    }
  }

#if WRITE_VIDEO
  // Create a VideoWriter object to save the video
  cv::VideoWriter writer("/home/pi/code_output.avi",
                         cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                         FPS,
                         cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));

  // Check if the VideoWriter is successfully opened
  if (!writer.isOpened()) {
    std::cerr << "Failed to open output video file" << std::endl;
    return;
  }
#endif

#if VISUALIZE
  // creating an opencv window to display the processed frames
  cv::namedWindow("Processed Frame", cv::WINDOW_NORMAL);
#endif

  // fps counter
  int fps_counter = 0;
  auto start = std::chrono::high_resolution_clock::now();
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed{};

  while (this->blackboard.running.get()) {
    // read frame and store it in the lane detection object
    capture >> this->frame;

#if WRITE_VIDEO
    // write frame to video file
    writer.write(this->frame);
#endif

#if VISUALIZE
    cap >> this->original_frame;
#endif

    // if frame is empty, break loop
    if (this->frame.empty()) {
      break;
    }

    // process image frame and display it
    process_image_frame();

    // update fps counter
    fps_counter++;
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    if (elapsed.count() >= 1) {
      std::cout << "FPS: " << fps_counter << std::endl;
      fps_counter = 0;
      start = std::chrono::high_resolution_clock::now();
    }

#if VISUALIZE
    if (cv::waitKey(1) == 27) {
      break;
    }
#endif
  }

#if WRITE_VIDEO
  std::cout << "releasing writer" << std::endl;
  writer.release();
#endif

  std::cout << "releasing cap" << std::endl;
  capture.release();
}

/**
 * main_loop_video
 *
 * This main loop will call all necessary functions for processing the image frames from a video.
 *
 * @param video_path Path to the video file
 */
void LaneDetection::main_loop_video(const std::string &video_path) {
  // open video file
  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    std::cerr << "Error opening video stream or file" << std::endl;
    return;
  }

#if VISUALIZE
  // creating an opencv window to display the processed frames
  cv::namedWindow("Processed Frame", cv::WINDOW_NORMAL);
#endif

  while (true) {
    // read frame and store it in the lane detection object
    cap >> this->frame;

    // ONLY FOR DEBUGGING - COPY FRAME TO ORIGINAL_FRAME
    cap >> this->original_frame;

    // if frame is empty, video is over
    if (this->frame.empty()) {
      break;
    }

    // process image frame and display it
    process_image_frame();

#if VISUALIZE
    // exit if ESC is pressed
    if (cv::waitKey(1) == 27) {
      break;
    }
#endif
  }
}

void LaneDetection::setup_blackboard_smart_members() {
  // initialize smart members with default values
  this->blackboard.offset_middle_line.set(0);
  this->blackboard.is_intersection.set(false);
  this->blackboard.exits_intersection.set(std::array<bool, 3>{false, false, false});
  this->blackboard.exits_distance_intersection.set(std::array<double, 3>{0.0, 0.0, 0.0});
}

LaneDetection::LaneDetection(LaneDetectionMode mode) {
  // set mode
  this->mode = mode;

  // setup blackboard smart members
  setup_blackboard_smart_members();
}

/**
 * run
 *
 * This function will be called from the main function. It will call the main loop for the specified mode.
 *
 * @param mode Operating mode
 * @param path Path to the video file or image
 */
void LaneDetection::run(const char *path) {
  // call correct main loop depending on the operating mode
  switch (this->mode) {
    case LaneDetectionMode::CAMERA: {
      main_loop_camera();
      break;
    }
    case LaneDetectionMode::VIDEO: {
      main_loop_video(path);
      break;
    }
    case LaneDetectionMode::IMAGE: {
      // single image will be analyzed, skip main loop
      this->frame = cv::imread(path);
#if VISUALIZE
      // creating an opencv window to display the processed frames
      cv::namedWindow("Processed Frame", cv::WINDOW_NORMAL);
#endif
      // process image frame
      process_image_frame();
      break;
    }
  }
}

/**
 * Calculate the average center offset for the last 4 frames.
 * @return
 */
double LaneDetection::calculate_center_offset_average() {
  // 1. calculate the average of the last 4 offsets
  double offset_sum = 0;

  for (const auto &offset : this->offset_queue) {
    offset_sum += offset;
  }

  double avg = offset_sum / 4;
  double angle = std::atan(avg);
  return -angle * 180 / M_PI;
}

LaneDetection::~LaneDetection() {
#if VISUALIZE
  cv::destroyAllWindows();
#endif
}