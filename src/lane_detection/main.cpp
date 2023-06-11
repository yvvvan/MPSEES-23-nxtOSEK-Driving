#include "main.hpp"

#include "../blackboard/BlackBoard.hpp"
#include "../blackboard/BlackBoard.cpp"

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
 * @return cv::Mat the preprocessed image
 */
cv::Mat LaneDetection::image_preprocessing(cv::Mat frame) {
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
void LaneDetection::line_filtering(cv::Mat preprocessed_frame, cv::Vec4i &leftLane, cv::Vec4i &rightLane, std::vector<cv::Vec4i> &horizontalLine) {
  // perform probabilistic hough transform
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(preprocessed_frame, lines, 1, CV_PI / 180, 50, 55, 5);

  /*cv::Mat houghImage = cv::Mat::zeros(preprocessed_frame.size(), CV_8UC3);
  for (const cv::Vec4i &line : lines) {
    cv::line(houghImage, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 1);
  }
  cv::imshow("Hough", houghImage);*/ //just for visualization

  // separate left and right lane lines based on their slope
  std::vector<cv::Vec4i> leftLanes, rightLanes;
  for (const cv::Vec4i &line : lines) {
    // calculate slope of line
    float slope = static_cast<float>(line[3] - line[1]) / static_cast<float>(line[2] - line[0]);
    // ignore lines with a small slope degree
    if (std::abs(slope) > 0.5) {
      // separate left and right lane lines
      //we need to check for both (slope and middle of the image) because if not we would detect a centerline outside the lane while turning in an intersection
      if (slope < 0 && line[2] < preprocessed_frame.cols / 2)
        leftLanes.push_back(line);
      else if (slope > 0 && line[2] > preprocessed_frame.cols / 2)
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
    rightLane = cv::Vec4i(x2, y2, x1, y1); //coordinates the way around, so that is matches the normal coordinate system
  }

  horizontalLine.clear();
  // check for horizontal lines
  for (const cv::Vec4i &line : lines) {
    // calculate dx and dy
    int dx = abs(line[0] - line[2]);
    int dy = abs(line[1] - line[3]);
    // ignore lines with a small slope degree
    if (dx > 6 * dy) horizontalLine.push_back(line);
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
 * @return bool true if the frame contains an intersection
 */
bool LaneDetection::check_intersection(cv::Vec4i leftLane,
                        cv::Vec4i rightLane,
                        std::vector<cv::Vec4i> horizontalLines,
                        cv::Mat orig_image /* orig_image just a placeholder for visualization of common point, might delete later */) {

  // Calculate the x-coordinate of the center of the image
  int imageCenterX =
      orig_image.cols / 2; //TODO find correct one (this is just a placeholder) -> needs to be the exact for every frame
  bool check_left = false;
  bool check_right = false;

  // Loop through each horizontal line
  for (const auto &horizontalLine : horizontalLines) {
    // y = m * x + b for the horizontal line
    float mhorizontal =
        static_cast<float>(horizontalLine[1] - horizontalLine[3]) / (horizontalLine[0] - horizontalLine[2]);
    int bhorizontal = horizontalLine[1] - static_cast<int>(mhorizontal * horizontalLine[0]);

    //std::cout << "y(horizontal) = " << mhorizontal << " * x + " << bhorizontal << std::endl; //just for testing

    // checking for intersections on the left side
    if (horizontalLine[0] < imageCenterX && horizontalLine[2] < imageCenterX) {

      // Calculate the intersection point with the left vertical line
      float mLeft = static_cast<float>(leftLane[1] - leftLane[3]) / (leftLane[0] - leftLane[2]); //m = (y2-y1)/(x2-x1)
      int bLeft = leftLane[1] - static_cast<int>(mLeft * leftLane[0]); // b = y1 - (m * x1)
      int intersectionX = -static_cast<int>((bhorizontal - bLeft) / (mhorizontal - mLeft));

      //int intersectionY = static_cast<int>(mLeft * intersectionX) + bLeft; //just for visualization, maybe needed if returning a point
      //std::cout << "y(left) = " << mLeft << " * x + " << bLeft << std::endl; //just for testing

      // Check if the intersection point is within the left lane segment
      if (intersectionX < imageCenterX && 0 <= intersectionX) {

        //std::cout << "X_left:" << intersectionX << " Y_left:" << intersectionY << std::endl; //just for testing
        //cv::circle(orig_image, cv::Point(intersectionX, intersectionY), 3, cv::Scalar(255, 0, 0), 3); //just for testing

        check_left = true;
      }
    }

    // checking for intersections on the right side
    if (horizontalLine[0] > imageCenterX && horizontalLine[2] > imageCenterX) {

      // Calculate the intersection point with the right vertical line
      float mRight = static_cast<float>(rightLane[1] - rightLane[3]) / (rightLane[0] - rightLane[2]);
      int bRight = rightLane[1] - static_cast<int>(mRight * rightLane[0]);
      int intersectionX = -static_cast<int>((bhorizontal - bRight) / (mhorizontal - mRight));

      //int intersectionY = static_cast<int>(mRight * intersectionX) + bRight; //just for visualization, maybe needed if returning a point
      //std::cout << "y(right) = " << mRight << " * x + " << bRight << std::endl;

      // Check if the intersection point is within the right lane segment
      if (intersectionX > imageCenterX && intersectionX <= 2 * imageCenterX) {

        //std::cout << "X_right:" << intersectionX << " Y_right:" << intersectionY << std::endl; //just for testing
        //cv::circle(orig_image, cv::Point(intersectionX, intersectionY), 3, cv::Scalar(255, 0, 0), 3); //just for testing

        check_right = true;
      }
    }
  }
  // TODO calculate the average common point
  if (check_left && check_right) {
    std::cout << "Intersection on both sides detected" << std::endl;
    return true;
  } else if (check_left) {
    std::cout << "Intersection on the left detected" << std::endl;
    return true;
  } else if (check_right) {
    std::cout << "Intersection on the right detected" << std::endl;
    return true;
  } else
    return false;
}

/**
 * calculate_center
 *
 * Find the center between the right and left line.
 *
 * @param lane_lines
 * @return std::vector<cv::Vec4i> the center line
 */
std::vector<cv::Vec4i> LaneDetection::calculate_center(cv::Vec4i &leftLane, cv::Vec4i &rightLane) {

  std::vector<cv::Vec4i> center_line;
  //Case 1: Both lines detected
  if (leftLane != cv::Vec4i() && rightLane != cv::Vec4i()) {

    // Extract start points of the left and right lines
    cv::Point2f left_start(leftLane[0], leftLane[1]);
    cv::Point2f right_start(rightLane[0], rightLane[1]);
    double mid_start_x = (left_start.x + right_start.x) * 0.5;
    double mid_start_y = (left_start.y + right_start.y) * 0.5;

    // Extract end points of the left and right lines
    cv::Point2f left_end(leftLane[2], leftLane[3]);
    cv::Point2f right_end(rightLane[2], rightLane[3]);
    double mid_end_x = (left_end.x + right_end.x) * 0.5;
    double mid_end_y = (left_end.y + right_end.y) * 0.5;

    //create center between both lines
    cv::Vec4i center(mid_start_x, mid_start_y, mid_end_x, mid_end_y);
    center_line.push_back(center);

    std::cout << "Centerline calculated." << std::endl;

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

/*void drawLaneLine(cv::Mat &original_image, const cv::Vec4i &laneLine, const cv::Scalar &color) {
  cv::line(original_image, cv::Point(laneLine[0], laneLine[1]), cv::Point(laneLine[2], laneLine[3]), color, 2);
}
void drawLaneLines(cv::Mat &original_image, std::vector<cv::Vec4i> &laneLines, const cv::Scalar &color) {
  for (const cv::Vec4i &line : laneLines) {
    cv::line(original_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), color, 2);
  }
} */ // just for drawing lines on the image, just needed for the visualization part in the "process_image"-function

/**
 * return_function
 *
 * This function communicates with other components. We are not sure what this function will return yet.
 *
 * @return
 */
int LaneDetection::return_function() {
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
void LaneDetection::process_image_frame(cv::Mat frame) {
  // call image preprocessing
  cv::Mat preprocessed_frame = image_preprocessing(frame);

  // filter lines
  cv::Vec4i leftLane;
  cv::Vec4i rightLane;
  std::vector<cv::Vec4i> horizontal_lines;
  line_filtering(preprocessed_frame, leftLane, rightLane, horizontal_lines);

  // check if intersection is detected
  // TODO complete intersection detection
  bool is_intersection = check_intersection(leftLane, rightLane, horizontal_lines, frame);

  // calculate center
  std::vector<cv::Vec4i> lane_lines = calculate_center(leftLane, rightLane);

  // Draw lane lines on the result image
  /*cv::Mat result = frame.clone();
  cv::Vec4i actual_center(result.cols / 2, 0, result.cols / 2, result.rows);
  drawLaneLines(result, lane_lines, cv::Scalar(0, 0, 255));
  drawLaneLine(result, actual_center, cv::Scalar(255, 0, 0));
  drawLaneLine(result, leftLane, cv::Scalar(0, 255, 255)); //gelb
  drawLaneLine(result, rightLane, cv::Scalar(255, 255, 0)); //türkis
  drawLaneLines(result, horizontal_lines, cv::Scalar(0, 0, 0)); //schwarz
  cv::imshow("Result with Centerline and actual center of the image", result);
  cv::imshow("Processed Frame", preprocessed_frame);
  cv::waitKey(0);*/ //just for visualization

  // send result to other components using the return_function function
  // TODO complete the return_function function
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
    cv::VideoCapture cap(0);
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

        // if frame is empty, break loop
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

void LaneDetection::setup_blackboard_smart_members() {
    // get pointer to blackboard
    BlackBoard *blackboard = &BlackBoard::getInstance();

    // initialize smart members with default values
    blackboard->offset_middle_line.set(0);
    blackboard->is_intersection.set(false);
    blackboard->distance_intersection.set(0.0);
    blackboard->exits_intersection.set(std::array<bool, 3>{false, false, false});
    blackboard->exits_distance_intersection.set(std::array<double, 3>{0.0, 0.0, 0.0});
}

LaneDetection::LaneDetection(LaneDetectionMode mode) {
    // set mode
    this->mode = mode;

    // initialize average_offset_array with 0.0
    for (int i = 0; i < average_offset_array.size(); i++) {
        this->average_offset_array[i] = 0.0;
    }

    // setup blackboard smart members
    setup_blackboard_smart_members();
}

/**
 * ~LaneDetection
 *
 * Destructor
 */
LaneDetection::~LaneDetection() {
    // Nothing to do here
}

/**
 * run
 *
 * This function will be called from the main function. It will call the main loop for the specified mode.
 *
 * @param mode Operating mode
 * @param path Path to the video file or image
 */
void LaneDetection::run(LaneDetectionMode mode, char *path) {
    switch (mode) {
        case LaneDetectionMode::CAMERA:
            main_loop_camera();
            break;
        case LaneDetectionMode::VIDEO:
            main_loop_video(path);
            break;
        case LaneDetectionMode::IMAGE:
            // single image will be analyzed, skip main loop
            cv::Mat frame = cv::imread(path);
            // creating an opencv window to display the processed frames
            cv::namedWindow("Processed Frame", cv::WINDOW_NORMAL);
            // process image frame
            process_image_frame(frame);
            break;
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
    // create variable that holds the operating mode for the lane detection and the path for the image/video
    LaneDetectionMode mode;
    char *path = nullptr;

  // parse command line arguments
  //      -image for a path to an image (will skip main loop)
  //      -video for a path to a video
  //      no argument for normal operation with camera sensor
  if (strcmp(argv[1], "-image") == 0) {
        mode = LaneDetectionMode::IMAGE;
        path = argv[2];
  } else if (strcmp(argv[1], "-video") == 0) {
        mode = LaneDetectionMode::VIDEO;
        path = argv[2];
  } else if (argc < 2) {
        mode = LaneDetectionMode::CAMERA;
  } else {
    std::cout << "Please provide a valid argument." << std::endl;
    return 1;
  }

    // create lane detection object and run it
    LaneDetection lane_detection = LaneDetection(mode);
    lane_detection.run(mode, path);

  return 0;
}