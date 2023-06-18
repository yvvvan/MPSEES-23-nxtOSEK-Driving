#include "LaneDetection.hpp"

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
void LaneDetection::image_preprocessing() {
    // grayscaling
    cv::cvtColor(this->frame, this->frame, cv::COLOR_BGR2GRAY);

    // blurring
    cv::GaussianBlur(this->frame, this->frame, cv::Size(15, 15), 0);

    // edge Detection using Canny
    cv::Canny(this->frame, this->frame, 50, 150);

    // masking
    cv::Mat masking_frame(frame.size(), CV_8UC1, cv::Scalar(0));
    double mask_th = 0.6;
    int mask_height = static_cast<int>(this->frame.rows * mask_th);
    cv::Rect roi(0, this->frame.rows - mask_height, this->frame.cols, mask_height);
    masking_frame(roi) = 255;

    // applying the Mask
    cv::bitwise_and(this->frame, masking_frame, this->frame);
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
    std::vector<cv::Vec4f> lines;
    cv::HoughLinesP(this->frame, lines, 1, CV_PI / 180, 50, 55, 5);

    /* UNCOMMENT FOR VISUALIZATION
    cv::Mat houghImage = cv::Mat::zeros(preprocessed_frame.size(), CV_8UC3);
    for (const cv::Vec4f &line : lines) {
      cv::line(houghImage, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 1);
    }
    cv::imshow("Hough", houghImage);
    */

    // separate left and right lane lines based on their slope
    std::vector<cv::Vec4f> leftLanes, rightLanes;
    for (const cv::Vec4f &line: lines) {
        // calculate slope of line
        float slope = static_cast<float>(line[3] - line[1]) / static_cast<float>(line[2] - line[0]);
        // ignore lines with a small slope degree
        if (std::abs(slope) > 0.5) {
            // separate left and right lane lines
            //we need to check for both (slope and middle of the image) because if not we would detect a centerline outside the lane while turning in an intersection
            if (slope < 0 && line[2] < static_cast<float>(this->frame.cols / 2.0))
                leftLanes.push_back(line);
            else if (slope > 0 && line[2] > static_cast<float>(this->frame.cols / 2.0))
                rightLanes.push_back(line);
        }
    }

    // calculate average line for left and right lane
    if (!leftLanes.empty()) {
        float x1 = 0, y1 = 0, x2 = 0, y2 = 0;

        for (const cv::Vec4f &line: leftLanes) {
            x1 += line[0];
            y1 += line[1];
            x2 += line[2];
            y2 += line[3];
        }

        // calculate average of all lines for left lane
        x1 /= static_cast<float>(leftLanes.size());
        y1 /= static_cast<float>(leftLanes.size());
        x2 /= static_cast<float>(leftLanes.size());
        y2 /= static_cast<float>(leftLanes.size());
        leftLane = cv::Vec4f(x1, y1, x2, y2);
    }

    // calculate average line for left and right lane
    if (!rightLanes.empty()) {
        float x1 = 0, y1 = 0, x2 = 0, y2 = 0;

        for (const cv::Vec4f &line: rightLanes) {
            x1 += line[0];
            y1 += line[1];
            x2 += line[2];
            y2 += line[3];
        }

        // calculate average of all lines for right lane
        x1 /= static_cast<float>(rightLanes.size());
        y1 /= static_cast<float>(rightLanes.size());
        x2 /= static_cast<float>(rightLanes.size());
        y2 /= static_cast<float>(rightLanes.size());
        rightLane = cv::Vec4f(x2, y2, x1, y1); // change order, so that it's like in a normal coordinate system
    }

    // check for horizontal lines
    for (const cv::Vec4f &line: lines) {
        // calculate dx and dy
        float dx = abs(line[0] - line[2]);
        float dy = abs(line[1] - line[3]);

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
    // TODO add average value of intersections (maybe if there are 2 intersections in 3 frames, then we are in an intersection)
    // Calculate the x-coordinate of the center of the image
    float imageCenterX = this->frame.cols / 2.0;
    float th = imageCenterX * 0.2;

    // Loop through each horizontal line
    for (const auto &horizontalLine: this->horizontalLines) {
        // y = m * x + b for the horizontal line
        float mhorizontal = (horizontalLine[1] - horizontalLine[3]) / (horizontalLine[0] - horizontalLine[2]);
        float bhorizontal = horizontalLine[1] - (mhorizontal * horizontalLine[0]);
        // calculate the x-coordinate of the center of the horizontal line
        float hx = (horizontalLine[0] + horizontalLine[2]) / 2;

        //std::cout << "y(horizontal) = " << mhorizontal << " * x + " << bhorizontal << std::endl; //just for testing

        // checking for intersections on the left side
        if (horizontalLine[0] < imageCenterX && horizontalLine[2] < imageCenterX) {

            // Calculate the intersection point with the left vertical line
            float mLeft = (this->leftLane[1] - this->leftLane[3]) / (this->leftLane[0] - this->leftLane[2]); //m = (y2-y1)/(x2-x1)
            float bLeft = this->leftLane[1] - (mLeft * this->leftLane[0]); // b = y1 - (m * x1)
            float intersectionX = -(bhorizontal - bLeft) / (mhorizontal - mLeft);

            float intersectionY = (mLeft * intersectionX) + bLeft; //just for visualization, maybe needed if returning a point
            //std::cout << "y(left) = " << mLeft << " * x + " << bLeft << std::endl; //just for testing

            // Check if the intersection point is within the left lane segment
            if (intersectionX < imageCenterX && 0 <= intersectionX) {

                //std::cout << "X_left:" << intersectionX << " Y_left:" << intersectionY << std::endl; //just for testing
                cv::circle(this->original_frame, cv::Point2f(intersectionX, intersectionY), 3, cv::Scalar(255, 0, 0), 3); //just for testing

                // there is an intersection on the left side
                this->exits_intersection[0] = true;

                if (hx > imageCenterX - th && hx < imageCenterX + th && this->rightLane == cv::Vec4f()) {
                    // there is an intersection on the left but no right line -> not able to turn left or go ahead, just right is possible
                    this->exits_intersection[0] = false;
                    this->exits_intersection[2] = true;
                } else if (hx > imageCenterX - th && hx < imageCenterX + th && this->leftLane != cv::Vec4f() && this->rightLane != cv::Vec4f()) {
                    // the line is in the center of the image and there is a left and a right line -> not able to turn right, left or go ahead
                    this->exits_intersection[0] = false;
                    this->is_dead_end = true;
                } else if (horizontalLine[1] > this->leftLane[1] || horizontalLine[3] > this->leftLane[3]) {
                    // the line is above the right lane, so it must be possible to go straight ahead
                    this->exits_intersection[1] = true;
                }
            }
        }

        // checking for intersections on the right side
        if (horizontalLine[0] > imageCenterX && horizontalLine[2] > imageCenterX) {

            // Calculate the intersection point with the right vertical line
            float mRight = (this->rightLane[1] - this->rightLane[3]) / (this->rightLane[0] - this->rightLane[2]);
            float bRight = this->rightLane[1] - (mRight * this->rightLane[0]);
            float intersectionX = -(bhorizontal - bRight) / (mhorizontal - mRight);

            float intersectionY = (mRight * intersectionX) + bRight; //just for visualization, maybe needed if returning a point
            //std::cout << "y(right) = " << mRight << " * x + " << bRight << std::endl;

            // Check if the intersection point is within the right lane segment
            if (intersectionX > imageCenterX && intersectionX <= 2 * imageCenterX) {

                //std::cout << "X_right:" << intersectionX << " Y_right:" << intersectionY << std::endl; //just for testing
                cv::circle(this->original_frame, cv::Point2f(intersectionX, intersectionY), 3, cv::Scalar(255, 0, 0), 3); //just for testing

                // there is an intersection on the right side
                this->exits_intersection[2] = true;

                if (hx > imageCenterX - th && hx < imageCenterX + th && this->leftLane == cv::Vec4f()) {
                    // there is an intersection on the right but no left line -> not able to turn right or go ahead, just left is possible
                    this->exits_intersection[0] = true;
                    this->exits_intersection[2] = false;
                } else if (hx > imageCenterX - th && hx < imageCenterX + th && this->leftLane != cv::Vec4f() && this->rightLane != cv::Vec4f()) {
                    // the line is in the center of the image and there is a left and a right line -> not able to turn right, left or go ahead
                    this->exits_intersection[0] = false; // not able to turn left, even if it was detected earlier
                    this->exits_intersection[2] = false;
                    this->is_dead_end = true;
                } else if (horizontalLine[1] > this->rightLane[1] || horizontalLine[3] > this->rightLane[3]) {
                    // the line is above the right lane, so it must be possible to go straight ahead
                    this->exits_intersection[1] = true;
                }
            }
        }
    }

    // - exits_intersection[0] = left
    // - exits_intersection[1] = straight
    // - exits_intersection[2] = right

    /* just for debugging*/
    if(this->is_dead_end)
        std::cout << "Dead end detected" << std::endl;
    else {
        if (this->exits_intersection[0])
            std::cout << "Intersection on the left detected" << std::endl;
        if (this->exits_intersection[1])
            std::cout << "Intersection straight ahead detected" << std::endl;
        if (this->exits_intersection[2])
            std::cout << "Intersection on the right detected" << std::endl;
        if(!this->exits_intersection[0] && !this->exits_intersection[1] && !this->exits_intersection[2])
            std::cout << "No intersection detected" << std::endl;
    }
}

/**
 * calculate_center
 *
 * Find the center between the right and left line.
 *
 * @return std::vector<cv::Vec4f> the center line
 */
void LaneDetection::calculate_center() {

    //Case 1: Both lines detected
    if (this->leftLane != cv::Vec4f() && this->rightLane != cv::Vec4f()) {

        // Extract start points of the left and right lines
        cv::Point2f left_start(this->leftLane[0], this->leftLane[1]);
        cv::Point2f right_start(this->rightLane[0], this->rightLane[1]);
        float mid_start_x = (left_start.x + right_start.x) * 0.5;
        float mid_start_y = (left_start.y + right_start.y) * 0.5;

        // Extract end points of the left and right lines
        cv::Point2f left_end(this->leftLane[2], this->leftLane[3]);
        cv::Point2f right_end(this->rightLane[2], this->rightLane[3]);
        float mid_end_x = (left_end.x + right_end.x) * 0.5;
        float mid_end_y = (left_end.y + right_end.y) * 0.5;

        //create center between both lines
        cv::Vec4f center(mid_start_x, mid_start_y, mid_end_x, mid_end_y);
        this->centerLine.push_back(center);

        std::cout << "Centerline calculated." << std::endl;

    } else if (leftLane != cv::Vec4f()) {

        //PLACEHOLDER FOR CASE 2 (if needed)
        std::cout << "Right line missing." << std::endl;

    } else if (rightLane != cv::Vec4f()) {

        //PLACEHOLDER FOR CASE 2 (if needed)
        std::cout << "Left line missing." << std::endl;

    } else {
        //PLACEHOLDER FOR CASE 3
        std::cout << "Both lines missing." << std::endl;

    }
    //TODO Approximation Handling if needed (CASE 2 & 3)

}

/* UNCOMMENT FOR VISUALIZATION
 * just for drawing lines on the image, just needed for the visualization part in the "process_image"-function
 */
void drawLaneLine(cv::Mat &original_image, const cv::Vec4f &laneLine, const cv::Scalar &color) {
  cv::line(original_image, cv::Point(laneLine[0], laneLine[1]), cv::Point(laneLine[2], laneLine[3]), color, 2);
}

void drawLaneLines(cv::Mat &original_image, std::vector<cv::Vec4f> &laneLines, const cv::Scalar &color) {
  for (const cv::Vec4f &line : laneLines) {
    cv::line(original_image, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), color, 2);
  }
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
    this->blackboard->offset_middle_line.set(calculate_center_offset_average());
    this->blackboard->is_intersection.set(this->is_intersection);
    this->blackboard->exits_intersection.set(this->exits_intersection);
    this->blackboard->exits_distance_intersection.set(this->exits_distance_intersection);
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
    this->leftLane = cv::Vec4f();
    this->rightLane = cv::Vec4f();
    this->centerLine.clear();
    this->horizontalLines.clear();

    // set dead_end to false
    this->is_dead_end = false;

    // set all bools in exits_intersection to false
    this->exits_intersection.fill(false);

    // set all distances in exits_distance_intersection to 0
    this->exits_distance_intersection.fill(0);

    // set is_intersection to false
    this->is_intersection = false;
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

    /* UNCOMMENT FOR VISUALIZATION
     * Draw lane lines on the result image*/

    cv::Mat result = this->original_frame.clone();
    cv::Vec4f actual_center(result.cols / 2, 0, result.cols / 2, result.rows);
    drawLaneLines(result, this->centerLine, cv::Scalar(0, 0, 255));
    drawLaneLine(result, actual_center, cv::Scalar(255, 0, 0));
    drawLaneLine(result, leftLane, cv::Scalar(0, 255, 255)); //gelb
    drawLaneLine(result, rightLane, cv::Scalar(255, 255, 0)); //türkis
    drawLaneLines(result, this->horizontalLines, cv::Scalar(0, 0, 0)); //schwarz
    cv::imshow("Result with Centerline and actual center of the image", result);
    cv::imshow("Processed Frame", this->frame);
    cv::waitKey(0);


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
        // read frame and store it in the lane detection object
        cap >> this->frame;

        // ONLY FOR DEBUGGING - COPY FRAME TO ORIGINAL_FRAME
        cap >> this->original_frame;

        // if frame is empty, break loop
        if (this->frame.empty()) {
            break;
        }

        // process image frame and display it
        process_image_frame();

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

        // exit if ESC is pressed
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
}

void LaneDetection::setup_blackboard_smart_members() {
    // get pointer to blackboard
    this->blackboard = &BlackBoard::getInstance();

    // initialize smart members with default values
    this->blackboard->offset_middle_line.set(0);
    this->blackboard->is_intersection.set(false);
    this->blackboard->exits_intersection.set(std::array<bool, 3>{false, false, false});
    this->blackboard->exits_distance_intersection.set(std::array<double, 3>{0.0, 0.0, 0.0});
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
        case LaneDetectionMode::CAMERA:
            main_loop_camera();
            break;
        case LaneDetectionMode::VIDEO:
            main_loop_video(path);
            break;
        case LaneDetectionMode::IMAGE:
            // single image will be analyzed, skip main loop
            this->frame = cv::imread(path);
            // creating an opencv window to display the processed frames
            cv::namedWindow("Processed Frame", cv::WINDOW_NORMAL);
            // process image frame
            process_image_frame();
            break;
    }
}

/**
 * Calculate the average center offset for the last 4 frames.
 * @return
 */
float LaneDetection::calculate_center_offset_average() {
    return 0;
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
    lane_detection.run(path);

    return 0;
}