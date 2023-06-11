#ifndef LANE_DETECTION_MAIN_HPP
#define LANE_DETECTION_MAIN_HPP

#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief the mode to run in
 */
enum LaneDetectionMode {
    IMAGE,
    VIDEO,
    CAMERA
};

/**
 * @brief the lane detection class
 */
class LaneDetection {
public:
    // the mode to run in
    LaneDetectionMode mode;

    /**
     * @brief Construct a new Lane Detection object
     *
     * @param mode the mode to run in
     */
    explicit LaneDetection(LaneDetectionMode mode);

    /**
     * @brief Destroy the Lane Detection object
     *
     */
    ~LaneDetection();

    /**
     * @brief runs the lane detection
     *
     * @param mode the mode to run in
     * @param path the path to the video or image
     */
    void run(LaneDetectionMode mode, char *path);

private:
    // holds the average offset of the last 4 frames
    std::array<double, 4> average_offset_array{};

    /**
     * @brief find the right and left line on the preprocessed image. Remove unnecessary lines.
     *
     * @param preprocessed_frame the preprocessed image
     * @param leftLane left lane
     * @param rightLane right lane
     * @param horizontalLine horizontal line
     * @param verticalLine vertical line
     */
    void
    line_filtering(cv::Mat preprocessed_frame, cv::Vec4i &leftLane, cv::Vec4i &rightLane,
                   std::vector<cv::Vec4i> &horizontalLine);

    /**
     * @brief preprocesses the image frame
     * @param frame the image frame
     * @return cv::Mat the preprocessed image frame
     */
    cv::Mat image_preprocessing(cv::Mat frame);

    /**
     * @brief checks if intersection exists in the frame
     * @param leftLane the left lane
     * @param rightLane the right lane
     * @param horizontalLines the horizontal lines
     * @param verticalLines the vertical lines
     * @return bool true if intersection exists
     */
    bool check_intersection(cv::Vec4i leftLane, cv::Vec4i rightLane, std::vector<cv::Vec4i> horizontalLines,
                            cv::Mat orig_image);

    /**
     * @brief calculates the offset to the middle line
     *
     * @param leftLane the left lane
     * @param rightLane the right lane
     * @return std::vector<cv::Vec4i> the center line
     */
    static
    std::vector<cv::Vec4i> calculate_center(cv::Vec4i &leftLane, cv::Vec4i &rightLane);

    int return_function();

    /**
     * @brief runs the lane detection on a single image
     *
     * @param frame the image frame
     */
    void process_image_frame(cv::Mat frame);

    /**
     * @brief runs the lane detection on the camera stream
     */
    void main_loop_camera();

    /**
     * @brief runs the lane detection on a video
     *
     * @param video_path the path to the video
     */
    void main_loop_video(const std::string &video_path);

    /**
     * @brief sets up the blackboard members
     */
    void setup_blackboard_smart_members();
};

#endif //LANE_DETECTION_MAIN_HPP
