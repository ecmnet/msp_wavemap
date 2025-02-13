#ifndef WAVEMAP_CORE_UTILS_MATH_ANGLE_NORMALIZATION_H_
#define WAVEMAP_CORE_UTILS_MATH_ANGLE_NORMALIZATION_H_

#include "wavemap/core/common.h"

namespace wavemap::angle_math {
inline FloatingPoint normalize(FloatingPoint angle) {
  return std::remainder(angle, kPi);
}

inline FloatingPoint normalize_near(FloatingPoint angle) {
  if (angle < -kPi) {
    angle += kTwoPi;
  } else if (kPi < angle) {
    angle -= kTwoPi;
  }
  return angle;
}
}  // namespace wavemap::angle_math

#endif  // WAVEMAP_CORE_UTILS_MATH_ANGLE_NORMALIZATION_H_
