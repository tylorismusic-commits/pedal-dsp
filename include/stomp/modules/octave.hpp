// pedal-dsp — monophonic octave pedal
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <hdsp/biquad.hpp>

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

namespace octavedetail {
// Autocorrelation window and lag range: 1024 samples covers the low E with
// enough overlap, and 70-500 Hz is where a guitar octave pedal lives.
constexpr int kWindow = 1024;
constexpr int kHop = 512;
constexpr int kMinLag = 96;   // 500 Hz at 48 kHz
constexpr int kMaxLag = 686;  // 70 Hz
} // namespace octavedetail

class Octave {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    envCoeff_ = 1.0f - std::exp(-1.0f / (0.006f * sr_));
    history_.assign(octavedetail::kWindow, 0.0f);
    historyPos_ = 0;
    hopCounter_ = 0;
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

    history_[static_cast<size_t>(historyPos_)] = in;
    historyPos_ = (historyPos_ + 1) % octavedetail::kWindow;
    if (++hopCounter_ >= octavedetail::kHop) {
      hopCounter_ = 0;
      detectPitch();
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
    if (!history_.empty()) std::fill(history_.begin(), history_.end(), 0.0f);
    historyPos_ = 0;
    hopCounter_ = 0;
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

  // Normalised autocorrelation over the tracked window.  The largest lag with
  // a near-best score wins, which avoids locking an octave high on a signal
  // with strong second-harmonic content.
  void detectPitch() noexcept {
    const int n = octavedetail::kWindow;
    // Chronological copy: a linear pass over the ring buffer puts the wrap in
    // the middle of the window and destroys the long-lag match.
    float window[octavedetail::kWindow];
    for (int i = 0; i < n; ++i) {
      window[i] = history_[static_cast<size_t>((historyPos_ + i) % n)];
    }
    double energy = 0.0;
    for (int i = 0; i < n; ++i) {
      energy += static_cast<double>(window[i]) * window[i];
    }
    if (energy < 1.0e-6) {
      return;
    }

    double best = 0.0;
    int bestLag = 0;
    for (int lag = octavedetail::kMinLag; lag <= octavedetail::kMaxLag; ++lag) {
      // Both energies from the overlapped range: normalising against the
      // whole window penalises long lags and locks an octave high.
      double corr = 0.0, ea = 0.0, eb = 0.0;
      for (int i = 0; i + lag < n; ++i) {
        const double a = window[i];
        const double b = window[i + lag];
        corr += a * b;
        ea += a * a;
        eb += b * b;
      }
      if (ea <= 1.0e-9 || eb <= 1.0e-9) continue;
      const double norm = corr / std::sqrt(ea * eb);
      if (norm > best) {
        best = norm;
        bestLag = lag;
      }
    }
    if (best < 0.55 || bestLag <= 0) return;

    // Prefer the lowest plausible fundamental: keep stepping down while a
    // multiple of the lag still correlates nearly as well.
    int chosen = bestLag;
    for (int mult = 2; mult <= 3; ++mult) {
      const int lag = bestLag * mult;
      if (lag > octavedetail::kMaxLag) break;
      double corr = 0.0, ea = 0.0, eb = 0.0;
      for (int i = 0; i + lag < n; ++i) {
        const double a = window[i];
        const double b = window[i + lag];
        corr += a * b;
        ea += a * a;
        eb += b * b;
      }
      if (ea > 1.0e-9 && eb > 1.0e-9 &&
          corr / std::sqrt(ea * eb) > 0.82 * best) {
        chosen = lag;
      }
    }

    const float f0 = sr_ / static_cast<float>(chosen);
    // An interval above the fundamental: high enough to keep the attack, low
    // enough to remove the harmonics that double the divider's rate.
    const float target = std::min(std::max(1.6f * f0, 90.0f), 700.0f);
    cutoff_ += (target - cutoff_) * 0.25f;
    preLp_.lowPass(cutoff_, 0.707f, sr_);
  }
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
  std::vector<float> history_;
  int historyPos_ = 0;
  int hopCounter_ = 0;
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
