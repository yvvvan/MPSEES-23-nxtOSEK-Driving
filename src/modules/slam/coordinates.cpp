#include "coordinates.hpp"

Coordinates::Coordinates(double x, double y, double z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

Coordinates::Coordinates() {
  this->x = 0;
  this->y = 0;
  this->z = 0;
}

Coordinates::~Coordinates() {}

void Coordinates::add_vector(double x, double y, double z) {
  this->x += x;
  this->y += y;
  this->z += z;
}

std::string Coordinates::to_string() {
  std::string str = "(" + std::to_string(this->x) + ", " +
                    std::to_string(this->y) + ", " + std::to_string(this->z) +
                    ")";
  return str;
}