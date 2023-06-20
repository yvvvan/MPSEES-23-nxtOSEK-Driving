#include <stdio.h>
#include <opencv2/opencv.hpp>

#include "camera.hpp"

PiCamera::PiCamera(int framerate) {
    /* Open the camera */
    std::string connstr = "libcamerasrc ! video/x-raw,framerate=" + std::to_string(framerate) + "/1 ! appsink";
    this->cap.open(connstr, cv::CAP_GSTREAMER);

    /* Check if camera opened successfully */
    if(!this->cap.isOpened()) {
        std::cerr << "Error opening camera" << std::endl;
    }
}

PiCamera::~PiCamera() {
    if (this->cap.isOpened()) {
        this->cap.release();
    }
}

cv::Mat PiCamera::getFrame() {

    /* check if camera is open */
    if (!this->cap.isOpened()) {
        std::cerr << "Camera not opened" << std::endl;
        return cv::Mat();
    }

    /* get current frame from camera */
    cv::Mat frame;
    this->cap.read(frame);
    return frame;
}

int PiCamera::run() {

    if (!this->cap.isOpened()) {
        std::cerr << "Camera not opened" << std::endl;
        return -1;
    }

    /* Loop, which analyses the incoming camera frames */
    while(this->blackboard.camera_enabled == true) {
        cv::Mat frame = this->getFrame();

        /* If the frame was read successfully */
        if (!frame.empty()) {

            /* Write the frame to the blackboard */
            this->blackboard.frame = frame;
            // TODO copy frame?

        } else {
            std::cerr << "No frame received" << std::endl;
            /* If the frame was not read successfully, exit the loop */
            return -1;
        }
    }

    return 0;
}
