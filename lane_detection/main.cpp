#include <iostream>
#include <string.h>
#include <opencv2/opencv.hpp>


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

    cv::Mat placeholder;
    return placeholder;
}

/**
 * line_filtering
 *
 * Find the right and left line on the preprocessed image. Remove unnecessary lines.
 *
 * @param hough_lines
 * @return
 */
std::vector<cv::Vec4i> line_filtering(std::vector<cv::Vec4i> hough_lines) {

    // TODO CHRIS

    std::vector<cv::Vec4i> placeholder;
    return placeholder;
}

/**
 * calculate_center
 *
 * Find the center between the right and left line.
 *
 * @param lane_lines
 * @return
 */
std::vector<cv::Vec4i> calculate_center(std::vector<cv::Vec4i> lane_lines) {

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
void main_loop_video(const std::string& video_path) {
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

        // call image preprocessing
        cv::Mat preprocessed_frame = image_preprocessing(frame);

        // filter lines
        std::vector<cv::Vec4i> hough_lines = line_filtering(preprocessed_frame);

        // calculate center
        std::vector<cv::Vec4i> lane_lines = calculate_center(hough_lines);

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

        // exit if ESC is pressed
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
}

int main(int argc, char *argv[]) {
    // parse command line arguments
    //      -image for a path to an image (will skip main loop)
    //      -video for a path to a video
    //      no argument for normal operation with camera sensor
    if (strcmp(argv[1], "-image") == 0) {
        // single image will be analyzed, skip main loop
        // TODO
    } else if (strcmp( argv[1], "-video") == 0) {
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
