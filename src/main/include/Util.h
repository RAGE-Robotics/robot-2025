#pragma once

#include <cmath>

class Util {
public:
  // Squares the value while keeping the sign
  static double exp(double value) {
    if (value >= 0) {
      return std::pow(value, 2);
    }

    return -std::pow(value, 2);
  }
};
