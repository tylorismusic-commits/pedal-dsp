// pedal-dsp — clean boost
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>

namespace stomp::modules {

struct BoostControls {
  float gain = 0.0f;   // 0..1 -> 0..+24 dB
  float tone = 0.5f;   // dark .. bright
  float level = 0.5f;  // -12..+12 dB
};

class Boost {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
    dirty_ = true;
  }

  void setControls(const BoostControls& c) noexcept {
    controls_ = c;
    dirty_ = true;
  }

  inline void process(float& l, float& r) noexcept {
    update();
    l = highL_.process(lowL_.process(l * inputGain_)) * outputGain_;
    r = highR_.process(lowR_.process(r * inputGain_)) * outputGain_;
  }

  void reset() noexcept {
    lowL_.reset();
    lowR_.reset();
    highL_.reset();
    highR_.reset();
  }

  int latencySamples() const noexcept { return 0; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    inputGain_ = std::pow(10.0f, (24.0f * clamp01(controls_.gain)) * 0.05f);
    outputGain_ =
        std::pow(10.0f, (-12.0f + 24.0f * clamp01(controls_.level)) * 0.05f);
    const float tilt = (clamp01(controls_.tone) - 0.5f) * 12.0f;
    lowL_.lowShelf(300.0f, -tilt * 0.5f, 0.6f, sr_);
    lowR_.lowShelf(300.0f, -tilt * 0.5f, 0.6f, sr_);
    highL_.highShelf(2500.0f, tilt * 0.5f, 0.6f, sr_);
    highR_.highShelf(2500.0f, tilt * 0.5f, 0.6f, sr_);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  BoostControls controls_;
  float inputGain_ = 1.0f;
  float outputGain_ = 1.0f;
  hdsp::Biquad lowL_, lowR_, highL_, highR_;
};

} // namespace stomp::modules
