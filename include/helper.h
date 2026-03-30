#ifndef HELPER_H
#define HELPER_H

#include <cmath>
#include <algorithm>

#include "constants.h"
namespace HybridAStar {
namespace Helper {
static inline float normalizeHeadingDeg(float t) {
  int a = (int)t;
  a = a % 360;

  if (a < 0) {
    a += 360;
  }

  return (float)a;
}

static inline float normalizeHeadingRad(float t) {
  if (t < 0) {
    t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }

  return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}

static inline float toDeg(float t) {
  return normalizeHeadingRad(t) * 180.f / M_PI;
}
}
}

#endif // HELPER_H
