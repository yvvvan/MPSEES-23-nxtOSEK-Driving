#include "Timer.hpp"

Timer::Timer() {
  start();
}

Timer::Timer(int _startTimeMS) {
  this->time = std::chrono::system_clock::now() - std::chrono::milliseconds(_startTimeMS);
}

void Timer::start() {
  this->time = std::chrono::system_clock::now();
}

double Timer::getTimeS() {
  std::chrono::duration<double> diff = std::chrono::system_clock::now() - this->time;
  return diff.count();
}

double Timer::getTimeMS() { return getTimeS() * 1000; }

void Timer::reset() { start(); }
