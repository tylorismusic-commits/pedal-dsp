// pedal-dsp — time-domain Whammy
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The Whammy's family of algorithm: two playback taps reading a delay line at
// the shifted rate, crossfaded with complementary Hann windows.  No spectral
// analysis, so attacks stay intact and latency is one grain; chords warble
// because nothing is pitch-tracked, which is the pedal's character.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <hdsp/biquad.hpp>

namespace stomp::modules {

namespace whammydetail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
constexpr float kPi = 3.14159265358979323846f;
} // namespace whammydetail

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
    grain_ = std::max(64, static_cast<int>(std::lround(sr_ * 0.032f)));
    baseDelay_ = std::max(1, static_cast<int>(std::lround(sr_ * 0.002f)));
    size_ = 1;
    while (size_ < grain_ * 4 + baseDelay_) size_ <<= 1;
    mask_ = size_ - 1;
    history_.assign(kWindow, 0.0f);
    historyPos_ = 0;
    hopCounter_ = 0;
    offsetPhase_ = 0.5f;
    period_ = 0.0f;
    bufL_.assign(static_cast<size_t>(size_), 0.0f);
    bufR_.assign(static_cast<size_t>(size_), 0.0f);
    write_ = 0;
    phase_ = 0.0f;
    ratio_ = 1.0f;
    target_ = 1.0f;
    glideCoeff_ = 1.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const WhammyControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    ratio_ += (target_ - ratio_) * glideCoeff_;

    bufL_[static_cast<size_t>(write_)] = l;
    bufR_[static_cast<size_t>(write_)] = r;

    // Track the input period so the two taps are offset by a whole number of
    // periods and add coherently.  Without this the taps cancel whenever the
    // offset happens to land half a period apart, which sounds like ring
    // modulation instead of a pitch shift.
    history_[static_cast<size_t>(historyPos_)] = l;
    historyPos_ = (historyPos_ + 1) % kWindow;
    if (++hopCounter_ >= kHop) {
      hopCounter_ = 0;
      detectPeriod();
    }

    // Delay drifts at (1 - ratio) samples per sample; wrapping it moves the
    // crossfade to the other tap.
    phase_ += (1.0f - ratio_) / static_cast<float>(grain_);
    if (phase_ >= 1.0f) {
      phase_ -= 1.0f;
      offsetPhase_ = coherentOffset();
    } else if (phase_ < 0.0f) {
      phase_ += 1.0f;
      offsetPhase_ = coherentOffset();
    }

    float phaseB = phase_ + offsetPhase_;
    if (phaseB >= 1.0f) phaseB -= 1.0f;

    const float delayA = static_cast<float>(baseDelay_) + phase_ * static_cast<float>(grain_);
    const float delayB = static_cast<float>(baseDelay_) + phaseB * static_cast<float>(grain_);
    const float w = 0.5f - 0.5f * std::cos(2.0f * whammydetail::kPi * phase_);

    const float aL = tap(bufL_, delayA), bL = tap(bufL_, delayB);
    const float aR = tap(bufR_, delayA), bR = tap(bufR_, delayB);
    float wetL = aL * w + bL * (1.0f - w);
    float wetR = aR * w + bR * (1.0f - w);

    wetL = toneL_.process(wetL);
    wetR = toneR_.process(wetR);

    // Dry for the blend, delayed to the average wet delay so the mix does not
    // comb against the taps.
    const float dryDelay =
        static_cast<float>(baseDelay_) + 0.5f * static_cast<float>(grain_);
    const float dryL = tap(bufL_, dryDelay);
    const float dryR = tap(bufR_, dryDelay);

    l = dryL + (wetL - dryL) * mix_;
    r = dryR + (wetR - dryR) * mix_;

    write_ = (write_ + 1) & mask_;
  }
  void reset() noexcept {
    std::fill(history_.begin(), history_.end(), 0.0f);
    historyPos_ = 0;
    hopCounter_ = 0;
    period_ = 0.0f;
    offsetPhase_ = 0.5f;
    std::fill(bufL_.begin(), bufL_.end(), 0.0f);
    std::fill(bufR_.begin(), bufR_.end(), 0.0f);
    toneL_.reset();
    toneR_.reset();
    write_ = 0;
    phase_ = 0.0f;
    ratio_ = target_;
  }
  // Worst-case tap delay, which is what the host has to compensate.
  int latencySamples() const noexcept { return baseDelay_ + grain_; }

private:
  static constexpr int kWindow = 1024;
  static constexpr int kHop = 512;

  // Autocorrelation over 70-500 Hz, the range a guitar whammy cares about.
  void detectPeriod() noexcept {
    const int n = kWindow;
    double energy = 0.0;
    for (int i = 0; i < n; ++i) {
      energy += history_[static_cast<size_t>(i)] * history_[static_cast<size_t>(i)];
    }
    if (energy < 1.0e-6) {
      period_ = 0.0f;
      return;
    }
    double best = 0.0;
    int bestLag = 0;
    for (int lag = 96; lag <= 686; ++lag) {
      double corr = 0.0, tail = 0.0;
      for (int i = 0; i + lag < n; ++i) {
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
    if (best > 0.6 && bestLag > 0) {
      period_ += (static_cast<float>(bestLag) - period_) * 0.35f;
    } else {
      period_ = 0.0f;
    }
  }

  // Offset for the second tap: as close to half a grain as a whole number of
  // input periods allows.
  float coherentOffset() const noexcept {
    if (period_ <= 0.0f) return 0.5f;
    const float half = 0.5f * static_cast<float>(grain_);
    const float periods = std::max(1.0f, std::round(half / period_));
    const float samples = std::min(periods * period_,
                                   static_cast<float>(grain_) - period_);
    return std::max(samples, period_) / static_cast<float>(grain_);
  }

  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    const float p = whammydetail::clamp01(c_.pitch);
    const float octaves = (p - 0.5f) * 4.0f;  // -2 .. +2
    target_ = std::pow(2.0f, octaves);
    const float glideMs = 5.0f + 195.0f * whammydetail::clamp01(c_.glide);
    glideCoeff_ = 1.0f - std::exp(-1.0f / (0.001f * glideMs * sr_));
    mix_ = whammydetail::clamp01(c_.mix);
    const float hz =
        400.0f * std::pow(24.0f, whammydetail::clamp01(c_.tone));
    toneL_.lowPass(hz, 0.72f, sr_);
    toneR_.lowPass(hz, 0.72f, sr_);
  }

  inline float tap(const std::vector<float>& buf, float delay) const noexcept {
    float pos = static_cast<float>(write_) - delay;
    if (pos < 0.0f) pos += static_cast<float>(size_);
    int i0 = static_cast<int>(pos) & mask_;
    const float frac = pos - std::floor(pos);
    const int i1 = (i0 + 1) & mask_;
    return buf[static_cast<size_t>(i0)] * (1.0f - frac) +
           buf[static_cast<size_t>(i1)] * frac;
  }

  float sr_ = 48000.0f;
  int grain_ = 1536;
  int baseDelay_ = 96;
  int size_ = 8192;
  int mask_ = 8191;
  int write_ = 0;
  float phase_ = 0.0f;
  float offsetPhase_ = 0.5f;
  float period_ = 0.0f;
  std::vector<float> history_;
  int historyPos_ = 0;
  int hopCounter_ = 0;
  float ratio_ = 1.0f;
  float target_ = 1.0f;
  float glideCoeff_ = 1.0f;
  float mix_ = 1.0f;
  bool dirty_ = true;
  WhammyControls c_;
  std::vector<float> bufL_, bufR_;
  hdsp::Biquad toneL_, toneR_;
};

} // namespace stomp::modules
