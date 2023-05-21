#include <iostream>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <vector>


/**
 * image_preprocessing
 *
 * Does the following preprocessing steps:
 *      - grayscaling
 *      - masking
 *      - gaussian blurring
 *      - canny algorithm
 *      - (binary image improvement - optional)
 *
 * @param frame
 * @return
 */
cv::Mat image_preprocessing(cv::Mat frame) {

    // TODO FABIAN

    cv::Mat grayscaled_frame;
    cv::cvtColor(frame, grayscaled_frame,
                 cv::COLOR_BGR2GRAY);    //convert original frame into a grayscaled image, save result in previous created variable
    cv::Mat masking_frame;  //Frame to store mask overlay
    //PLACEHOLDER FOR MASKING
    cv::Mat masked_frame;
    grayscaled_frame.copyTo(masked_frame, masking_frame);    //apply the mask on the grayscaled image
    cv::Mat blurred_frame;
    cv::GaussianBlur(masked_frame, blurred_frame, cv::Size(5, 5),
                     0);   //applying blur, where size 5x5 and 0 are just placeholders for now
    cv::Mat result_frame;
    cv::Canny(blurred_frame, result_frame, 50,
              150);   //applying Canny filter, where th1 and th2 are just placeholders for now

    return result_frame;
}

/**
 * line_filtering
 *
 * Find the right and left line on the preprocessed image. Remove unnecessary lines.
 *
 * @param hough_lines
 * @return
 */
void line_filtering(cv::Mat preprocessed_frame, cv::Vec4i& leftLane, cv::Vec4i& rightLane) {
    // perform probabilistic hough transform
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(preprocessed_frame, lines, 1, CV_PI / 180, 50, 30, 10);

    // separate left and right lane lines based on their slope
    std::vector<cv::Vec4i> leftLanes, rightLanes;
    for (const cv::Vec4i &line: lines) {
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

        for (const cv::Vec4i& line : leftLanes) {
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

        for (const cv::Vec4i& line : rightLanes) {
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
 * calculate_center
 *
 * Find the center between the right and left line.
 *
 * @param lane_lines
 * @return
 */
std::vector<cv::Vec4i> calculate_center(cv::Vec4i &leftLane, cv::Vec4i &rightLane) {

    // TODO FABIAN

    std::vector<cv::Vec4i> placeholder;
    return placeholder;
}

/**
 * return_function
 *
 * This function communicates with other components. We are not sure what this function will return yet.
 *
 * @return
 */
int return_function() {

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
    line_filtering(preprocessed_frame, leftLane, rightLane);

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
