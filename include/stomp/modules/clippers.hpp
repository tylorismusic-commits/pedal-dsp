// pedal-dsp — antiderivative clippers
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

namespace stomp::modules {

// First-order ADAA tanh clipper. Input is already gain-staged.
struct AdaaTanh {
  float x1 = 0.0f;
  float f1 = 0.0f;

  static float antiderivative(float x) noexcept {
    x = std::min(std::max(x, -20.0f), 20.0f);
    return std::log(std::cosh(x));
  }
  void reset() noexcept { x1 = 0.0f; f1 = antiderivative(0.0f); }
  inline float process(float x) noexcept {
    const float f = antiderivative(x);
    const float dx = x - x1;
    float y;
    if (std::fabs(dx) < 1.0e-5f) {
      y = std::tanh(0.5f * (x + x1));
    } else {
      y = (f - f1) / dx;
    }
    x1 = x;
    f1 = f;
    return y;
  }
};

// First-order ADAA hard clip.
struct AdaaHard {
  float x1 = 0.0f;
  float f1 = 0.0f;

  static float antiderivative(float x) noexcept {
    const float a = std::fabs(x);
    if (a <= 1.0f) return 0.5f * x * x;
    return a - 0.5f;
  }
  void reset() noexcept { x1 = 0.0f; f1 = 0.0f; }
  inline float process(float x) noexcept {
    const float f = antiderivative(x);
    const float dx = x - x1;
    float y;
    if (std::fabs(dx) < 1.0e-5f) {
      y = std::min(std::max(0.5f * (x + x1), -1.0f), 1.0f);
    } else {
      y = (f - f1) / dx;
    }
    x1 = x;
    f1 = f;
    return y;
  }
};

} // namespace stomp::modules
