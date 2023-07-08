#ifndef LANE_DETECTION_MAIN_HPP
#define LANE_DETECTION_MAIN_HPP

#include <vector>
#include <cstring>
#include <iostream>
#include <deque>

#include <opencv2/opencv.hpp>

#include "communication/internal/BlackBoard.hpp"

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
  BlackBoard &blackboard{BlackBoard::getInstance()};

  // frame
  cv::Mat frame;
  cv::Mat original_frame; // only for debugging, remove in final version

  // right and left lane
  cv::Vec4d leftLane;
  cv::Vec4d rightLane;

  bool hasLeftLane{};
  bool hasRightLane{};

  // important lines
  std::vector<cv::Vec4d> horizontalLines;
  std::vector<cv::Vec4d> centerLine;

  // boolean for dead end
  std::deque<bool> is_dead_end;

  // intersection booleans and distance
  std::deque<bool> lower_intersections;                 // is there an intersection in the lower half
  std::deque<bool> is_intersection;                     // is there an intersection
  std::deque<std::array<bool, 3>> exits_intersection;   // is there an exit on the left, middle, right

  // queue for the last 4 offsets
  std::deque<double> offset_queue;

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
  double calculate_center_offset_average();

  std::array<bool, 3> find_majority_exits_intersection();

  static bool find_majority_bool_deque(std::deque<bool> &boolDeque);
};

#endif //LANE_DETECTION_MAIN_HPP