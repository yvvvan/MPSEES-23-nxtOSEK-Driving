#include "globals.hpp"

#include "buildhat++/BuildHat.hpp"


int main() {
  auto &hat = BuildHat::getInstance();

  if (hat.isReady()) {
    hat.serial_write_line("version");
    hat.serial_read_line(true);
  }

  return 0;
}
