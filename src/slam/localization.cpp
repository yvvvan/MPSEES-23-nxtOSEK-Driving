/**
 * This file should contain the mapping class for the SESE driving projekt.
 * Currently it only contains a test script to test ORB_SLAM on the raspberry
 * pi.
 */
#include "localization.hpp"


#include <signal.h>
#include <unistd.h>

#include <ctime>
#include <opencv2/opencv.hpp>

#define MAXNOTRECEIVEDFRAMES 100

#ifdef USE_ORB_SLAM
Localization::Localization(std::string vocabularyFile, std::string configFile) {
  this->vocabularyFile = vocabularyFile;
  this->configFile = configFile;

  /* create slam instance */
  this->slam = new ORB_SLAM3::System(vocabularyFile, configFile,
                                     ORB_SLAM3::System::MONOCULAR, false);
}
#endif
Localization::Localization() {
  this->driving_direction = {1, 0};
  this->accum_angle = 0;
}

Localization::~Localization() {
#ifdef USE_ORB_SLAM
  /* Stop ORB SLAM */
  if (slam->GetTrackingState() !=
      ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY) {
    slam->Shutdown();
  }

  /* Delete ORB SLAM */
  if (this->slam != nullptr) {
    delete this->slam;
  }
#endif
}

#ifdef USE_ORB_SLAM
int Localization::exec_thread() {
  int notReceivedFrames = 0;

  /* Loop, which analyses the incoming camera frames */
  while (this->blackboard.localization_enabled == true &&
         this->blackboard.camera_enabled == true) {
    auto frame = this->blackboard.frame.get().clone();

    /* If the frame was read successfully */
    if (!frame.empty()) {
      // Get the current system time as a timestamp
      std::time_t timestamp = std::time(nullptr);

      Sophus::SE3f camera_pose = slam->TrackMonocular(frame, (double)timestamp);
      cv::imshow("Frame", frame);

      Eigen::Quaternionf q = camera_pose.unit_quaternion();
      Eigen::Vector3f twb = camera_pose.translation();
      // TODO translate to Coordinates Class correctly
      this->blackboard.coordinates = Coordinates(twb(0), twb(1), twb(2));

      std::cout << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x()
                << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;

    } else {
      notReceivedFrames++;

      if (notReceivedFrames > MAXNOTRECEIVEDFRAMES) {
        std::cerr << "No frame received for " << MAXNOTRECEIVEDFRAMES
                  << " frames, exiting..." << std::endl;
        return -1;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    frame.release();
  }

  return 0;
}
#endif

void Localization::reset_clock() {
  this->time = std::chrono::system_clock::now();
}

void Localization::handle_intersection(double angle, long time_difference) {
  if (!intersection) accum_time = 0;

  if (!blackboard.intersection_detected.get()) {
    accum_time += time_difference;
  }

  if (accum_time > INTERSECTION_SEC_RANGE * 1000) {
    accum_time = 0;
    intersection = false;
    intersection_driven = false;
    return;
  }

  if (intersection_driven) {
    return;
  }

  accum_angle += angle;
  if (accum_angle >= 70) {
    accum_angle = 0;
    this->adjust_driving_direction(90);
    intersection_driven = true;

  } else if (accum_angle <= -70) {
    accum_angle = 0;
    this->adjust_driving_direction(-90);
    intersection_driven = true;
  }
}

double calc_sin(double angle) {
  return std::round(sin(angle * M_PI / 180) * 100) / 100;
}

double calc_cos(double angle) {
  return std::round(cos(angle * M_PI / 180) * 100) / 100;
}

Coordinates Localization::driving_tracking(long time_difference) {
  //  auto curr_time = std::chrono::system_clock::now();
  //  long time_difference =
  //  std::chrono::duration_cast<std::chrono::milliseconds>(
  //                             curr_time - this->time)
  //                             .count();
  //  this->time = curr_time;
  double speed = this->blackboard.speed.get();
  double angle = this->blackboard.angle.get();
  Coordinates coordinates = this->blackboard.coordinates.get();

  /* TODO find appropriate time difference */
  if (time_difference < 10 || (speed == 0 && angle == 0)) {
    return coordinates;
  }

  /* calculate new driving direction */
  //  if (angle >= 90 || angle <= -90) {
  //    angle = 90;
  //  }
  angle = angle * (double)time_difference / 1000 * speed * -1;
  if (this->blackboard.intersection_detected.get() || this->intersection) {
    this->intersection = true;
    this->handle_intersection(angle, time_difference);
  } else {
    this->adjust_driving_direction(angle);
  }

  // TODO caclulate correct multiplier
  double mult = speed * (double)time_difference / 1000;
  coordinates.add_vector(this->driving_direction.at(0) * mult,
                         this->driving_direction.at(1) * mult, 0);
  std::cout << "Angle: " << angle << "; "
            << "X Forward: " << this->driving_direction.at(0) * mult << "; "
            << "Y Forward: " << this->driving_direction.at(1) * mult << "; "
            << mult << " " << time_difference << std::endl;
  this->blackboard.coordinates.set(coordinates);
  return coordinates;
}

void Localization::adjust_driving_direction(double new_angle) {
  double x = this->driving_direction.at(0) * calc_cos(new_angle) -
             this->driving_direction.at(1) * calc_sin(new_angle);
  double y = this->driving_direction.at(0) * calc_sin(new_angle) +
             this->driving_direction.at(1) * calc_cos(new_angle);
  this->driving_direction.at(0) = std::round(x * 100) / 100;
  this->driving_direction.at(1) = std::round(y * 100) / 100;
}
