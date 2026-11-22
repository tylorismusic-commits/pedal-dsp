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
    history_.assign(kWindow, 0.0f);
    historyPos_ = 0;
    hopCounter_ = 0;
    period_ = 0.0f;
    toneL_.lowPass(6000.0f, 0.72f, sr_);
    toneR_.lowPass(6000.0f, 0.72f, sr_);
    reset();
    dirty_ = true;
  }
  void setControls(const WhammyControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  // mixOverride >= 0 forces the dry/wet balance, which is how a bypassed slot
  // keeps its latency while passing the signal through unchanged.
  inline void process(float& l, float& r) noexcept { process(l, r, -1.0f); }
  inline void process(float& l, float& r, float mixOverride) noexcept {
    update();
    const float in = 0.5f * (l + r);
    history_[static_cast<size_t>(historyPos_)] = in;
    historyPos_ = (historyPos_ + 1) % kWindow;
    if (++hopCounter_ >= kHop) {
      hopCounter_ = 0;
      detectPeriod();
      if (period_ > 0.0f) shifter_.setSpliceSamples(period_);
    }

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
    std::fill(history_.begin(), history_.end(), 0.0f);
    historyPos_ = 0;
    hopCounter_ = 0;
    period_ = 0.0f;
    shifter_.reset();
    toneL_.reset();
    toneR_.reset();
  }
  int latencySamples() const noexcept { return shifter_.latencySamples(); }

private:
  static constexpr int kWindow = 1024;
  static constexpr int kHop = 256;

  void detectPeriod() noexcept {
    double energy = 0.0;
    for (int i = 0; i < kWindow; ++i) {
      energy += static_cast<double>(history_[static_cast<size_t>(i)]) *
                history_[static_cast<size_t>(i)];
    }
    if (energy < 1.0e-6) {
      period_ = 0.0f;
      return;
    }
    double best = 0.0;
    int bestLag = 0;
    for (int lag = 96; lag <= 686; ++lag) {
      double corr = 0.0, tail = 0.0;
      for (int i = 0; i + lag < kWindow; ++i) {
        const double a = history_[static_cast<size_t>(i)];
        const double b = history_[static_cast<size_t>(i + lag)];
        corr += a * b;
        tail += b * b;
      }
      if (tail <= 1.0e-9) continue;
      const double norm = corr / std::sqrt(energy * tail);
      if (norm > best) {
        best = norm;
        bestLag = lag;
      }
    }
    if (best > 0.55 && bestLag > 0) {
      const float p = static_cast<float>(bestLag);
      period_ = period_ > 0.0f ? period_ + (p - period_) * 0.5f : p;
    } else {
      period_ = 0.0f;
    }
  }

  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    const float octaves = (whammydetail::clamp01(c_.pitch) - 0.5f) * 4.0f;
    const float glideMs = 5.0f + 295.0f * whammydetail::clamp01(c_.glide);
    shifter_.setRatio(std::pow(2.0f, octaves), 0.001f * glideMs);
    mix_ = whammydetail::clamp01(c_.mix);
    const float hz = 400.0f * std::pow(24.0f, whammydetail::clamp01(c_.tone));
    toneL_.lowPass(hz, 0.72f, sr_);
    toneR_.lowPass(hz, 0.72f, sr_);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  float mix_ = 1.0f;
  float period_ = 0.0f;
  WhammyControls c_;
  SolaShifter shifter_;
  std::vector<float> history_;
  int historyPos_ = 0;
  int hopCounter_ = 0;
  hdsp::Biquad toneL_, toneR_;
};

} // namespace stomp::modules
