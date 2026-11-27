// pedal-dsp — time-domain pitch pedals
// SPDX-License-Identifier: GPL-3.0-or-later
//
// SOLA splicing, the family the Whammy and Drop belong to.  One read head runs
// through the delay line at the shifted rate; when it drifts too close to (or
// too far from) the write head it jumps by about one period, choosing the jump
// that best correlates with the audio it was already playing, and crossfades
// over a few milliseconds.  Because the splice is phase-aligned the pitch is
// constant instead of the beating/hunting a windowed crossfade produces.
//
// The Whammy tracks the input period for that jump (monophonic, like the
// pedal).  The Drop uses a fixed splice spacing and a band cap, which keeps it
// polyphonic.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <hdsp/biquad.hpp>

#include "pitch_tracker.hpp"
#include "sola.hpp"

namespace stomp::modules {

namespace whammydetail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
constexpr float kPi = 3.14159265358979323846f;
} // namespace whammydetail

// Whammy: monophonic, period-tracked splices, two octaves either way.
struct WhammyControls {
  float pitch = 0.5f;  // 0.5 = unison, 0 = two octaves down, 1 = two up
  float glide = 0.18f;
  float tone = 0.60f;
  float mix = 1.00f;
};

class Whammy {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    shifter_.prepare(sr_, 34.0f, 4.0f, 4.0f);
    shifter_.setSpliceSamples(sr_ * 0.008f);
    tracker_.prepare(sr_, 40.0f, 500.0f);
    toneL_.lowPass(6000.0f, 0.72f, sr_);
    toneR_.lowPass(6000.0f, 0.72f, sr_);
    reset();
    dirty_ = true;
  }
  void setControls(const WhammyControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  // Bypassed: run at unison (same latency) and restore the knobs on engage.
  void setBypassed(bool bypassed) noexcept {
    if (bypassed_ == bypassed) return;
    bypassed_ = bypassed;
    dirty_ = true;
  }
  // mixOverride >= 0 forces the dry/wet balance, which is how a bypassed slot
  // keeps its latency while passing the signal through unchanged.
  inline void process(float& l, float& r) noexcept { process(l, r, -1.0f); }
  inline void process(float& l, float& r, float mixOverride) noexcept {
    update();
    tracker_.process(0.5f * (l + r));
    updatePeriod();

    const float dryL = shifter_.dryLeft();
    const float dryR = shifter_.dryRight();
    float wetL = l;
    float wetR = r;
    shifter_.process(wetL, wetR);
    wetL = toneL_.process(wetL);
    wetR = toneR_.process(wetR);
    const float mix = mixOverride >= 0.0f ? mixOverride : mix_;
    l = dryL + (wetL - dryL) * mix;
    r = dryR + (wetR - dryR) * mix;
  }
  void reset() noexcept {
    tracker_.reset();
    shifter_.reset();
    toneL_.reset();
    toneR_.reset();
  }
  int latencySamples() const noexcept { return shifter_.latencySamples(); }

private:
  // The period comes from the shared YIN tracker; see pitch_tracker.hpp.
  void updatePeriod() noexcept {
    if (!tracker_.valid()) return;
    const float period = tracker_.periodSamples();
    shifter_.setSpliceSamples(period);
    shifter_.setPeriodSamples(period);
  }

  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    const float octaves = bypassed_
                              ? 0.0f
                              : (whammydetail::clamp01(c_.pitch) - 0.5f) * 4.0f;
    const float glideMs = 5.0f + 295.0f * whammydetail::clamp01(c_.glide);
    shifter_.setRatio(std::pow(2.0f, octaves), 0.001f * glideMs);
    mix_ = bypassed_ ? 1.0f : whammydetail::clamp01(c_.mix);
    const float hz = 400.0f * std::pow(24.0f, whammydetail::clamp01(c_.tone));
    toneL_.lowPass(hz, 0.72f, sr_);
    toneR_.lowPass(hz, 0.72f, sr_);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  float mix_ = 1.0f;
  bool bypassed_ = false;
  WhammyControls c_;
  SolaShifter shifter_;
  PitchTracker tracker_;
  hdsp::Biquad toneL_, toneR_;
};

} // namespace stomp::modules
