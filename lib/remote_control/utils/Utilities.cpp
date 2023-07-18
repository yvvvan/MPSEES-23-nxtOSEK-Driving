#include "Utilities.hpp"

#include <array>
#include <memory>
#include <thread>
#include <iostream>
#include <algorithm>

std::string Utilities::exec(std::string const &cmd, bool async) {
  auto run = [cmd]() {
    std::cout << cmd << std::endl;
    std::array<char, 128> buffer{};
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
      if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
        result += buffer.data();
    }
    return result;
  };
  if (async) {
    std::thread(run).detach();
    return "";
  }
  return run();
}

int16_t Utilities::read16LE(unsigned char const *const buf, int offset) {
  return static_cast<int16_t>((buf[offset + 0]) | (buf[offset + 1] << 8));
}

void Utilities::ltrim(std::string &s) {
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
    return !std::isspace(ch);
  }));
}

void Utilities::rtrim(std::string &s) {
  s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
    return !std::isspace(ch);
  }).base(), s.end());
}

void Utilities::trim(std::string &s) {
  rtrim(s);
  ltrim(s);
}