#include "helper.hpp"

#include <algorithm>
#include <string>

std::vector<std::string> splitStringBySpace(const std::string &input) {
  std::vector<std::string> tokens;
  std::istringstream iss(input);
  std::string token;

  while (iss >> token) {
    tokens.push_back(token);
  }

  return tokens;
}

std::vector<std::string> splitStringByChar(const std::string &input,
                                           const char &c) {
  std::replace(const_cast<std::string &>(input).begin(),
               const_cast<std::string &>(input).end(), c, ' ');
  return splitStringBySpace(input);
}
