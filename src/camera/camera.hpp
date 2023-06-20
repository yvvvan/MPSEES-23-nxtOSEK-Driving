#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <opencv2/opencv.hpp>

#include "blackboard/BlackBoard.hpp"

class PiCamera {
    public:
        /**
         * @brief Create a new camera object, which connects to the
         *        camera via OpenCV and GStreamer
        */
        PiCamera(int framerate);

        /**
         * @brief Destroy the Camera object
        */
        ~PiCamera();

        /**
         * Run the camera thread
        */
        int run();

    private:

        /**
         * @brief Get the current frame from the camera
         * 
         * @return cv::Mat 
        */
        cv::Mat getFrame();

        /* OpenCV camera object */
        cv::VideoCapture cap;

        /* Blackboard */
        BlackBoard &blackboard = BlackBoard::getInstance();

}

#endif // CAMERA_HPP