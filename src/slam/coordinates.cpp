#include "coordinates.hpp"

Coordinates::Coordinates(double x, double y, double z, double angle) {
  this->x = x;
  this->y = y;
  this->z = z;
  this->angle = angle;
}

Coordinates::~Coordinates() {
}
