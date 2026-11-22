// pedal-dsp — pitch pedal engine
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <memory>

#include <hdsp/biquad.hpp>

#include "sola.hpp"

namespace stomp::modules {

namespace pitchdetail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
} // namespace pitchdetail

// Pitch engine backed by the vendored Signalsmith Stretch shifter.
class PitchEngine {
public:
  PitchEngine();
  ~PitchEngine();
  PitchEngine(const PitchEngine&) = delete;
  PitchEngine& operator=(const PitchEngine&) = delete;

  // block/interval select the analysis window; 0 keeps the 4096/512 default.
  void prepare(float sr, bool preserveFormants = true, int block = 0,
               int interval = 0) noexcept;
  void setRatio(float ratio, float glideSeconds) noexcept;
  void setTone(float tone) noexcept;
  void setLowCut(float lowCut) noexcept;
  void clearLowCut() noexcept;
  void process(float& l, float& r, float mix, float wetScale = 1.0f) noexcept;
  void reset() noexcept;
  int latencySamples() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct PitchDropControls {
  float pitch = 0.0f;   // mode-dependent pitch control
  int mode = 0;         // 0 up, 1 down, 2 drop, 3 harmony, 4 detune
  float glide = 0.2f;
  float tone = 0.5f;
  float mix = 1.0f;
};

class PitchDrop {
public:
  void prepare(float sr) noexcept {
    engine_.prepare(sr, false);
    engine_.clearLowCut();
    engine_.setTone(1.0f);
    reset();
    dirty_ = true;
  }
  void setControls(const PitchDropControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    process(l, r, c_.mix);
  }
  inline void process(float& l, float& r, float mix) noexcept {
    update();
    engine_.process(l, r, mix >= 0.0f ? mix : c_.mix);
  }
  void reset() noexcept { engine_.reset(); }
  int latencySamples() const noexcept { return engine_.latencySamples(); }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    const float v = pitchdetail::clamp01(c_.pitch);
    float semis = 0.0f;
    switch (std::clamp(c_.mode, 0, 4)) {
      case 0: semis = 24.0f * v; break;                   // Whammy Up
      case 1: semis = -24.0f * v; break;                  // Whammy Down
      case 2: semis = -std::round(24.0f * v); break;      // Drop
      case 3: semis = std::round(-24.0f + 48.0f * v) + 7.0f; break; // Harmony
      default: semis = 0.02f + 0.28f * v; break;          // Detune (cents)
    }
    engine_.setRatio(std::pow(2.0f, semis / 12.0f),
                     0.01f + 0.5f * pitchdetail::clamp01(c_.glide));
    engine_.setTone(c_.tone);
  }
  PitchEngine engine_;
  PitchDropControls c_;
  bool dirty_ = true;
};

// DigiTech Drop-style transposer.  Time-domain like the Whammy but polyphonic:
// the splice spacing is fixed instead of tracked, and the band is capped at
// 10 kHz — the trade the pedal makes to stay quick, and why it sounds darker.
struct DropControls {
  float semitones = 0.0f;  // 0 = unison, 1 = one octave down
  float tone = 0.5f;
  float mix = 1.0f;
};

class Drop {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    shifter_.prepare(sr_, 34.0f, 4.0f, 4.0f);
    shifter_.setSpliceSamples(sr_ * 0.012f);
    lowL_.lowPass(10000.0f, 0.7f, sr_);
    lowR_.lowPass(10000.0f, 0.7f, sr_);
    reset();
    dirty_ = true;
  }
  void setControls(const DropControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept { process(l, r, -1.0f); }
  // mixOverride >= 0 forces the dry/wet balance, so a bypassed slot can keep
  // its latency and still pass the signal through unshifted.
  inline void process(float& l, float& r, float mixOverride) noexcept {
    update();
    const float mix = mixOverride >= 0.0f ? mixOverride : mix_;
    const float dryL = shifter_.dryLeft();
    const float dryR = shifter_.dryRight();
    float wetL = lowL_.process(l);
    float wetR = lowR_.process(r);
    shifter_.process(wetL, wetR);
    l = dryL + (wetL - dryL) * mix;
    r = dryR + (wetR - dryR) * mix;
  }
  void reset() noexcept {
    shifter_.reset();
    lowL_.reset();
    lowR_.reset();
  }
  int latencySamples() const noexcept { return shifter_.latencySamples(); }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    // The pedal has discrete steps, so the DSP snaps too: a continuous knob
    // would leave the ratio between semitones.
    const float semis =
        -std::round(12.0f * pitchdetail::clamp01(c_.semitones));
    shifter_.setRatio(std::pow(2.0f, semis / 12.0f), 0.03f);
    mix_ = pitchdetail::clamp01(c_.mix);
  }
  SolaShifter shifter_;
  hdsp::Biquad lowL_, lowR_;
  DropControls c_;
  float sr_ = 48000.0f;
  float mix_ = 1.0f;
  bool dirty_ = true;
};

struct RetuneControls {
  float semitones = 0.5f;  // -12..+12
  float cents = 0.5f;      // -50..+50
  float lowCut = 0.5f;     // 40..300 Hz
  float mix = 1.0f;
};

class Retune {
public:
  void prepare(float sr) noexcept {
    engine_.prepare(sr, true);
    engine_.setLowCut(0.5f);
    engine_.setTone(1.0f);
    reset();
    dirty_ = true;
  }
  void setControls(const RetuneControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    process(l, r, c_.mix);
  }
  inline void process(float& l, float& r, float mix) noexcept {
    update();
    engine_.process(l, r, mix >= 0.0f ? mix : c_.mix);
  }
  void reset() noexcept { engine_.reset(); }
  int latencySamples() const noexcept { return engine_.latencySamples(); }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    const int semis = std::clamp(
        static_cast<int>(std::lround(pitchdetail::clamp01(c_.semitones) * 24.0f)) - 12,
        -12, 12);
    const float cents = -50.0f + 100.0f * pitchdetail::clamp01(c_.cents);
    const float total = static_cast<float>(semis) + cents / 100.0f;
    engine_.setRatio(std::pow(2.0f, total / 12.0f), 0.005f);
    engine_.setLowCut(c_.lowCut);
  }
  PitchEngine engine_;
  RetuneControls c_;
  bool dirty_ = true;
};

} // namespace stomp::modules
