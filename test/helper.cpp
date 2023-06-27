#include "helper.hpp"

std::vector<std::string> splitStringBySpace(const std::string &input) {
  std::vector<std::string> tokens;
  std::istringstream iss(input);
  std::string token;

  while (iss >> token) {
    tokens.push_back(token);
  }

  return tokens;
}
