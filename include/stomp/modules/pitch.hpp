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
// it tracks the lowest strong period so the splice lands a whole number of
// periods away, which is what keeps low notes from beating, and it caps the
// band at 10 kHz — the trade the pedal makes to stay quick.
struct DropControls {
  float semitones = 0.0f;  // 0 = unison, 1 = one octave down
  float tone = 0.5f;
  float mix = 1.0f;
};

class Drop {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    shifter_.prepare(sr_, 45.0f, 3.0f, 4.0f);
    shifter_.setSpliceSamples(sr_ * 0.012f);
    decim_ = std::max(1, static_cast<int>(std::lround(sr_ / 12000.0f)));
    analysisRate_ = sr_ / static_cast<float>(decim_);
    history_.assign(kWindow, 0.0f);
    historyPos_ = 0;
    hopCounter_ = 0;
    decimCounter_ = 0;
    period_ = 0.0f;
    lowL_.lowPass(10000.0f, 0.7f, sr_);
    lowR_.lowPass(10000.0f, 0.7f, sr_);
    reset();
    dirty_ = true;
  }
  void setControls(const DropControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  // A bypassed pitch pedal cannot leave the chain without changing latency, so
  // it runs at unison instead: the same delay, no shift.  Engaging restores the
  // knobs, which are still applied while bypassed.
  void setBypassed(bool bypassed) noexcept {
    if (bypassed_ == bypassed) return;
    bypassed_ = bypassed;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept { process(l, r, -1.0f); }
  // mixOverride >= 0 forces the dry/wet balance, so a bypassed slot can keep
  // its latency and still pass the signal through unshifted.
  inline void process(float& l, float& r, float mixOverride) noexcept {
    update();
    const float mix = mixOverride >= 0.0f ? mixOverride : mix_;

    if (++decimCounter_ >= decim_) {
      decimCounter_ = 0;
      history_[static_cast<size_t>(historyPos_)] = 0.5f * (l + r);
      historyPos_ = (historyPos_ + 1) % kWindow;
      if (++hopCounter_ >= kHop) {
        hopCounter_ = 0;
        detectPeriod();
      }
    }

    const float dryL = shifter_.dryLeft();  // delayed to the middle of the range
    const float dryR = shifter_.dryRight();
    float wetL = lowL_.process(l);
    float wetR = lowR_.process(r);
    shifter_.process(wetL, wetR);
    l = dryL + (wetL - dryL) * mix;
    r = dryR + (wetR - dryR) * mix;
  }
  void reset() noexcept {
    std::fill(history_.begin(), history_.end(), 0.0f);
    historyPos_ = 0;
    hopCounter_ = 0;
    period_ = 0.0f;
    shifter_.reset();
    lowL_.reset();
    lowR_.reset();
  }
  int latencySamples() const noexcept { return shifter_.latencySamples(); }

private:
  static constexpr int kWindow = 1024;  // decimated analysis window
  static constexpr int kHop = 256;

  // YIN (cumulative mean normalised difference) on a ~12 kHz decimated copy.
  // Plain autocorrelation locks onto a harmonic when the fundamental is weak,
  // which for a low guitar note means splicing half a period apart and
  // cancelling the fundamental.  YIN's CMND curve avoids that.
  void detectPeriod() noexcept {
    float w[kWindow];
    for (int i = 0; i < kWindow; ++i) {
      w[i] = history_[static_cast<size_t>((historyPos_ + i) % kWindow)];
    }
    const int minLag = std::max(2, static_cast<int>(analysisRate_ / 420.0f));
    const int maxLag = std::min(kWindow / 2, static_cast<int>(analysisRate_ / 40.0f));
    if (maxLag <= minLag + 2) return;

    scratch_.assign(static_cast<size_t>(maxLag + 1), 0.0f);
    double energy = 0.0;
    for (int i = 0; i < kWindow; ++i) energy += static_cast<double>(w[i]) * w[i];
    if (energy < 1.0e-6) return;

    for (int lag = 1; lag <= maxLag; ++lag) {
      double diff = 0.0;
      for (int i = 0; i + lag < kWindow; ++i) {
        const double d = static_cast<double>(w[i]) - w[i + lag];
        diff += d * d;
      }
      scratch_[static_cast<size_t>(lag)] = static_cast<float>(diff);
    }

    double running = 0.0;
    scratch_[0] = 1.0f;
    for (int lag = 1; lag <= maxLag; ++lag) {
      running += scratch_[static_cast<size_t>(lag)];
      scratch_[static_cast<size_t>(lag)] =
          static_cast<float>(running > 0.0
                                 ? scratch_[static_cast<size_t>(lag)] * lag / running
                                 : 1.0);
    }

    int chosen = 0;
    for (int lag = minLag; lag < maxLag; ++lag) {
      const float v = scratch_[static_cast<size_t>(lag)];
      if (v < 0.18f && v <= scratch_[static_cast<size_t>(lag - 1)] &&
          v <= scratch_[static_cast<size_t>(lag + 1)]) {
        chosen = lag;
        break;
      }
    }
    if (!chosen) {
      int best = minLag;
      for (int lag = minLag; lag <= maxLag; ++lag) {
        if (scratch_[static_cast<size_t>(lag)] <
            scratch_[static_cast<size_t>(best)]) {
          best = lag;
        }
      }
      if (scratch_[static_cast<size_t>(best)] < 0.35f) chosen = best;
    }
    if (!chosen) return;  // keep the last period

    // Parabolic refinement around the dip.
    float refined = static_cast<float>(chosen);
    if (chosen > 1 && chosen < maxLag) {
      const float a = scratch_[static_cast<size_t>(chosen - 1)];
      const float b = scratch_[static_cast<size_t>(chosen)];
      const float c = scratch_[static_cast<size_t>(chosen + 1)];
      const float denom = a - 2.0f * b + c;
      if (std::fabs(denom) > 1.0e-9f) {
        const float delta = 0.5f * (a - c) / denom;
        refined += std::max(-0.5f, std::min(0.5f, delta));
      }
    }

    const float period = refined * static_cast<float>(decim_);
    period_ = period_ > 0.0f ? period_ + (period - period_) * 0.5f : period;
    shifter_.setPeriodSamples(period_);
    // Two periods for the splice: fewer joins, and the jump still fits the
    // delay range.
    shifter_.setSpliceSamples(2.0f * period_);
  }

  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    // The pedal has discrete steps, so the DSP snaps too: a continuous knob
    // would leave the ratio between semitones.
    const float semis =
        bypassed_ ? 0.0f
                  : -std::round(12.0f * pitchdetail::clamp01(c_.semitones));
    shifter_.setRatio(std::pow(2.0f, semis / 12.0f), 0.03f);
    mix_ = bypassed_ ? 1.0f : pitchdetail::clamp01(c_.mix);
  }

  SolaShifter shifter_;
  hdsp::Biquad lowL_, lowR_;
  DropControls c_;
  std::vector<float> history_;
  std::vector<float> scratch_;
  int historyPos_ = 0;
  int hopCounter_ = 0;
  int decim_ = 4;
  int decimCounter_ = 0;
  float analysisRate_ = 12000.0f;
  float period_ = 0.0f;
  float sr_ = 48000.0f;
  float mix_ = 1.0f;
  bool bypassed_ = false;
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
