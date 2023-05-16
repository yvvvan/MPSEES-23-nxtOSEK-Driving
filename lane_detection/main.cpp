#include <iostream>
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
 * main_loop
 *
 * This main loop will call all necessary functions for processing the image frames from the camera sensor.
 */
void main_loop() {

    // TODO CHRIS
}

int main() {
    // Call main image processing loop
    main_loop();

    return 0;
}
