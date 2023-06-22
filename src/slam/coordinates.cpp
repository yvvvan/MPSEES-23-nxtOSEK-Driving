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

Coordinates::~Coordinates() {
}
