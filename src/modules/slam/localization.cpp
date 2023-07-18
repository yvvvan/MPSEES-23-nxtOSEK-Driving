/**
 * This file should contain the mapping class for the SESE driving projekt.
 * Currently it only contains a test script to test ORB_SLAM on the raspberry
 * pi.
 */
#include "localization.hpp"

#include <algorithm>
#include <opencv2/opencv.hpp>

Localization::Localization() {
  this->driving_direction = {1, 0};
  this->accum_angle = 0;
}

Localization::~Localization() {}

void Localization::reset_clock() {
  this->time = std::chrono::system_clock::now();
}

void Localization::handle_intersection(double angle, long time_difference) {
  if (!intersection) accum_time = 0;

  if (!blackboard.is_intersection.get()) {
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

Coordinates Localization::track_driving_params(long time_difference) {
  double speed = this->blackboard.speed.get();
  double angle = this->blackboard.angle.get();
  Coordinates coordinates = this->blackboard.coordinates.get();

  /* TODO find appropriate time difference */
  if (time_difference < 10 || (speed == 0 && angle == 0)) {
    return coordinates;
  }

  /* calculate new driving direction */
  angle = angle * (double)time_difference / 1000 * speed * -1;
  if (this->blackboard.is_intersection.get() || this->intersection) {
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

int get_index_of(std::array<int, 4> array, int value) {
  for (int i = 0; i < array.size(); i++) {
    if (array.at(i) == value) {
      return i;
    }
  }
  return -1;
}
