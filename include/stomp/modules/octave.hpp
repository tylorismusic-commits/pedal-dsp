// pedal-dsp — monophonic octave pedal
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <hdsp/biquad.hpp>

#include "pitch_tracker.hpp"

namespace stomp::modules {

namespace octavedetail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
} // namespace octavedetail

// Flip-flop sub-octave plus full-wave-rectified octave up. Monophonic, like
// the classic analog octave pedals it is modelled on.
//
// A bare flip-flop divides whatever zero crossings it sees, so harmonics make
// it toggle twice per cycle and the note jumps an octave.  A tracked low-pass
// in front of the divider, set an interval above the detected fundamental,
// keeps it seeing one crossing per cycle — the same trick the analog pedals
// use with a fixed input filter, but following the note.
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
    tracker_.prepare(sr_, 60.0f, 500.0f);
    lastPitchUpdate_ = 0;
    cutoff_ = 600.0f;
    preLp_.lowPass(cutoff_, 0.707f, sr_);
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

    tracker_.process(in);
    if (tracker_.updates() != lastPitchUpdate_) {
      lastPitchUpdate_ = tracker_.updates();
      if (tracker_.valid() && tracker_.confidence() > 0.4f) {
        // An interval above the fundamental: high enough to keep the attack,
        // low enough to remove the harmonics that double the divider's rate.
        const float target = std::min(std::max(1.6f * tracker_.hz(), 90.0f), 700.0f);
        cutoff_ += (target - cutoff_) * 0.25f;
        preLp_.lowPass(cutoff_, 0.707f, sr_);
      }
    }

    // Only the fundamental reaches the divider; harmonics would toggle it an
    // extra time and jump the note up an octave.
    const float filtered = preLp_.process(in);
    const float hyst = std::max(kHysteresis, env_ * 0.25f);
    if (filtered > hyst) {
      high_ = true;
    } else if (filtered < -hyst) {
      high_ = false;
    }
    if (high_ && !lastHigh_) subState_ = -subState_;
    lastHigh_ = high_;

    const float gate = std::min(env_ * 3.0f, 1.0f);
    const float sub = subState_ * gate * down_;
    const float rectified = dc_.process(std::fabs(filtered) * 1.6f);
    const float wet = tone_.process(sub + rectified * up_ * 2.0f * gate);

    l += (wet - l) * mix_;
    r += (wet - r) * mix_;
  }
  void reset() noexcept {
    env_ = 0.0f;
    tracker_.reset();
    lastPitchUpdate_ = 0;
    subState_ = 1.0f;
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
  hdsp::Biquad preLp_;
  PitchTracker tracker_;
  unsigned lastPitchUpdate_ = 0;
  float cutoff_ = 600.0f;
  float env_ = 0.0f;
  bool high_ = false;
  bool lastHigh_ = false;
  float subState_ = 1.0f;
  float down_ = 0.55f;
  float up_ = 0.0f;
  float mix_ = 1.0f;
};

} // namespace stomp::modules
