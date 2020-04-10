#ifndef WRAP_TO_PI_HPP
#define WRAP_TO_PI_HPP

#include <math.h>

static double wrapToPi(double angle) {
  if (angle >= 0) {
    return fmod(angle + M_PI, 2 * M_PI) - M_PI;
  } else {
    return fmod(angle - M_PI, 2 * M_PI) + M_PI;
  }
}

#endif
