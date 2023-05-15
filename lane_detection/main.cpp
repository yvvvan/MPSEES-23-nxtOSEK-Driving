#include <iostream>
#include <opencv2/opencv.hpp>

Mat image_preprocessing(Mat frame) {
    /**
     * Does the following preprocessing steps:
     *      - grayscaling
     *      - masking
     *      - gaussian blurring
     *      - canny algorithm
     *      - (binary image improvement - optional)
     */

    // TODO FABIAN

    return NULL;
}

vector<Vec4i> line_filtering(vector<Vec4i> hough_lines) {
    /**
     * Find the right and left line on the preprocessed image. Remove unnecessary lines.
     */

    // TODO CHRIS

    return NULL;
}

vector<Vec4i> calculate_center(vector<Vec4i> lane_lines) {
    /**
     * Find the center between the right and left line.
     */

    // TODO FABIAN

    return NULL;
}

int return_function() {
    /**
     * This function communicates with other components. We are not sure what this function will return yet.
     */

    return 0;
}

void main_loop() {
    /**
     * This main loop will call all necessary functions for processing the image frames from the camera sensor.
     */

    // TODO CHRIS
}

int main() {
    // Call main image processing loop
    main_loop();

    return 0;
}
