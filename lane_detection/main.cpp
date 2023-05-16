#include <iostream>
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
    cv::cvtColor(frame,grayscaled_frame,cv::COLOR_BGR2GRAY);    //convert original frame into a grayscaled image, save result in previous created variable
    cv::Mat masking_frame;  //Frame to store mask overlay
    //PLACEHOLDER FOR MASKING
    cv::Mat masked_frame;
    grayscaled_frame.copyTo(masked_frame,masking_frame);    //apply the mask on the grayscaled image
    cv::Mat blurred_frame;
    cv::GaussianBlur(masked_frame,blurred_frame,cv::Size(5,5),0);   //applying blur, where size 5x5 and 0 are just placeholders for now
    cv::Mat result_frame;
    cv::Canny(blurred_frame,result_frame,50,150);   //applying Canny filter, where th1 and th2 are just placeholders for now

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
