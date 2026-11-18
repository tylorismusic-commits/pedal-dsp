// pedal-dsp — monophonic octave pedal
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>

namespace stomp::modules {

namespace octavedetail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
} // namespace octavedetail

// Flip-flop sub-octave plus full-wave-rectified octave up. Monophonic, like
// the classic analog octave pedals it is modelled on.
struct OctaveControls {
  float down = 0.55f;  // sub-octave level
  float up = 0.00f;    // octave-up level
  float tone = 0.50f;
  float mix = 1.00f;
};

class Octave {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    envCoeff_ = 1.0f - std::exp(-1.0f / (0.006f * sr_));
    reset();
    dirty_ = true;
  }
  void setControls(const OctaveControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    const float in = 0.5f * (l + r);
    env_ += (std::fabs(in) - env_) * envCoeff_;

    if (in > kHysteresis) {
      high_ = true;
    } else if (in < -kHysteresis) {
      high_ = false;
    }
    if (high_ && !lastHigh_) subState_ = -subState_;
    lastHigh_ = high_;

    const float gate = std::min(env_ * 3.0f, 1.0f);
    const float sub = subState_ * gate * down_;
    const float rectified = dc_.process(std::fabs(in) * 1.6f);
    const float wet = tone_.process(sub + rectified * up_ * 2.0f *
                                             (env_ > 0.001f ? 1.0f : 0.0f));

    l += (wet - l) * mix_;
    r += (wet - r) * mix_;
  }
  void reset() noexcept {
    env_ = 0.0f;
    high_ = false;
    lastHigh_ = false;
    subState_ = 1.0f;
    tone_.reset();
    dc_.reset();
  }
  int latencySamples() const noexcept { return 0; }

private:
  static constexpr float kHysteresis = 0.012f;
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    down_ = octavedetail::clamp01(c_.down);
    up_ = octavedetail::clamp01(c_.up);
    mix_ = octavedetail::clamp01(c_.mix);
    const float hz =
        300.0f * std::pow(12.0f, octavedetail::clamp01(c_.tone));
    tone_.lowPass(hz, 0.72f, sr_);
    dc_.highPass(28.0f, 0.7f, sr_);
  }

  float sr_ = 48000.0f;
  float envCoeff_ = 0.003f;
  bool dirty_ = true;
  OctaveControls c_;
  hdsp::Biquad tone_;
  hdsp::Biquad dc_;
  float env_ = 0.0f;
  bool high_ = false;
  bool lastHigh_ = false;
  float subState_ = 1.0f;
  float down_ = 0.55f;
  float up_ = 0.0f;
  float mix_ = 1.0f;
};

} // namespace stomp::modules
