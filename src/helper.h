#ifndef HELPER
#define HELPER

#include <cmath>

namespace helper {

// set theta to a value (0,360]
inline float normalizeHeading(float t) {
  if ((int)t <= 0 || (int)t >= 360) {
    if (t < -0.1) {
      t += 360.f;
    } else if ((int)t >= 360) {
      t -= 360.f;
    } else {
      t =  0;
    }
  }

  return t;
}

inline float normalizeHeadingRad(float t) {
  if (t < 0) {
    t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }

  return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}

inline float toDeg(float t) {
  return normalizeHeadingRad(t) * 180.f / M_PI ;
}

inline float toRad(float t) {
  return normalizeHeadingRad(t / 180.f * M_PI);
}

}

#endif // HELPER

