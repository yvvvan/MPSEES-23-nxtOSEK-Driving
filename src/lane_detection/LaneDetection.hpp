#ifndef LANE_DETECTION_MAIN_HPP
#define LANE_DETECTION_MAIN_HPP

#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <vector>

#include "../blackboard/BlackBoard.hpp"
#include "../blackboard/BlackBoard.cpp"

/**
 * @brief The mode to run in.
 */
enum LaneDetectionMode {
    IMAGE,
    VIDEO,
    CAMERA
};

/**
 * @brief The lane detection class.
 */
class LaneDetection {
public:
    // the mode to run in
    LaneDetectionMode mode;

    /**
     * @brief Construct a new Lane Detection object.
     *
     * @param mode The mode to run in.
     */
    explicit LaneDetection(LaneDetectionMode mode);

    /**
     * @brief Destroy the Lane Detection object.
     *
     */
    ~LaneDetection() = default;

    /**
     * @brief Runs the lane detection.
     *
     * @param path The path to the file (image or video).
     */
    void run(const char *path);

private:
    // blackboard
    BlackBoard *blackboard;

    // frame
    cv::Mat frame;
    cv::Mat original_frame; // only for debugging, remove in final version

    // right and left lane
    cv::Vec4f leftLane;
    cv::Vec4f rightLane;

    // important lines
    std::vector<cv::Vec4f> horizontalLines;
    std::vector<cv::Vec4f> centerLine;

    // intersection booleans and distance
    bool is_intersection;                                   // is there an intersection
    std::array<bool, 3> exits_intersection;                 // is there an exit on the left, middle, right
    std::array<double, 3> exits_distance_intersection;      // distance to the exit on the left, middle, right

    // queue for the last 4 offsets
    std::queue<double> offset_queue;

    /**
     * @brief Find the right and left line on the preprocessed image. Remove unnecessary lines.
     */
    void line_filtering();

    /**
     * @brief Preprocesses the image frame.
     */
    void image_preprocessing();

    /**
     * @brief Checks if intersection exists in the frame.
     */
    void check_intersection();

    /**
     * @brief Calculates the offset to the middle line.
     */
    void calculate_center();

    /**
     * @brief Writes values to the blackboard.
     */
    void return_function();

    /**
     * @brief Runs the lane detection on a single image.
     */
    void process_image_frame();

    /**
     * @brief Runs the lane detection on the camera stream.
     */
    void main_loop_camera();

    /**
     * @brief Runs the lane detection on a video.
     *
     * @param video_path the path to the video
     */
    void main_loop_video(const std::string &video_path);

    /**
     * @brief Sets up the blackboard members.
     */
    void setup_blackboard_smart_members();

    /**
     * @brief Calculates the average of the last 4 offsets.
     */
    float calculate_center_offset_average();
};

#endif //LANE_DETECTION_MAIN_HPP
