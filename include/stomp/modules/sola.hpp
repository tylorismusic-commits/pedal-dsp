// pedal-dsp — SOLA splice engine
// SPDX-License-Identifier: GPL-3.0-or-later
//
// One read head runs through a delay line at the shifted rate.  When it drifts
// too close to (or too far from) the write head it jumps by about one period,
// choosing the jump that best correlates with the audio it was already
// playing, and crossfades over a few milliseconds.  Phase-aligned splices keep
// the pitch constant instead of the beating a windowed crossfade produces.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace stomp::modules {

// Shared engine.  Read positions are kept in samples and identical for both
// channels, so the stereo image survives.
class SolaShifter {
public:
  void prepare(float sr, float maxDelayMs, float minDelayMs,
               float fadeMs) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    maxDelay_ = static_cast<int>(std::lround(sr_ * 0.001f * maxDelayMs));
    minDelay_ = static_cast<int>(std::lround(sr_ * 0.001f * minDelayMs));
    fade_ = std::max(8, static_cast<int>(std::lround(sr_ * 0.001f * fadeMs)));
    size_ = 1;
    while (size_ < maxDelay_ * 4 + fade_ * 4) size_ <<= 1;
    mask_ = size_ - 1;
    bufL_.assign(static_cast<size_t>(size_), 0.0f);
    bufR_.assign(static_cast<size_t>(size_), 0.0f);
    reset();
  }

  void setRatio(float ratio, float glideSeconds) noexcept {
    target_ = std::max(0.125f, std::min(8.0f, ratio));
    glideCoeff_ =
        glideSeconds <= 0.0f
            ? 1.0f
            : 1.0f - std::exp(-1.0f / (glideSeconds * sr_));
  }

  // Splice spacing.  The Whammy feeds the tracked period; the Drop feeds a
  // fixed value and does not track.
  void setSpliceSamples(float samples) noexcept {
    splice_ = std::max(16.0f, samples);
  }

  // When the caller knows the input period, the jump is quantised to a whole
  // number of periods so the splice is phase aligned by construction.
  void setPeriodSamples(float samples) noexcept {
    period_ = samples > 0.0f ? samples : 0.0f;
  }

  void reset() noexcept {
    std::fill(bufL_.begin(), bufL_.end(), 0.0f);
    std::fill(bufR_.begin(), bufR_.end(), 0.0f);
    write_ = 0;
    read_ = -static_cast<double>(maxDelay_);
    oldRead_ = read_;
    ratio_ = target_;
    fadeLeft_ = 0;
    primed_ = false;
  }

  inline void process(float& l, float& r) noexcept {
    bufL_[static_cast<size_t>(write_)] = l;
    bufR_[static_cast<size_t>(write_)] = r;

    if (!primed_) {
      // Let the delay line fill before reading it.
      read_ -= 1.0;
      write_ = (write_ + 1) & mask_;
      if (static_cast<int>(std::lround(delaySamples())) >= maxDelay_) {
        primed_ = true;
      }
      l = 0.0f;
      r = 0.0f;
      return;
    }

    ratio_ += (target_ - ratio_) * glideCoeff_;

    if (fadeLeft_ > 0) {
      oldRead_ += ratio_;
      read_ += ratio_;
      const float a =
          1.0f - static_cast<float>(fadeLeft_) / static_cast<float>(fade_);
      const float xL = tap(bufL_, oldRead_) * (1.0f - a) + tap(bufL_, read_) * a;
      const float xR = tap(bufR_, oldRead_) * (1.0f - a) + tap(bufR_, read_) * a;
      --fadeLeft_;
      l = xL;
      r = xR;
    } else {
      read_ += ratio_;
      l = tap(bufL_, read_);
      r = tap(bufR_, read_);
      const double d = delaySamples();
      if (d < static_cast<double>(minDelay_) ||
          d > static_cast<double>(maxDelay_)) {
        splice();
      }
    }

    write_ = (write_ + 1) & mask_;
  }

  int latencySamples() const noexcept { return maxDelay_ + fade_; }

  // Input delayed to the middle of the read range, for the dry side of a mix.
  float dryLeft() const noexcept {
    return tap(bufL_, static_cast<double>(write_) -
                          0.5 * static_cast<double>(minDelay_ + maxDelay_));
  }
  float dryRight() const noexcept {
    return tap(bufR_, static_cast<double>(write_) -
                          0.5 * static_cast<double>(minDelay_ + maxDelay_));
  }

private:
  inline double delaySamples() const noexcept {
    double d = static_cast<double>(write_) - read_;
    while (d < 0.0) d += static_cast<double>(size_);
    while (d >= static_cast<double>(size_)) d -= static_cast<double>(size_);
    return d;
  }

  inline float tap(const std::vector<float>& buf, double pos) const noexcept {
    double p = std::fmod(pos, static_cast<double>(size_));
    if (p < 0.0) p += static_cast<double>(size_);
    const int i0 = static_cast<int>(p) & mask_;
    const float frac = static_cast<float>(p - std::floor(p));
    const int i1 = (i0 + 1) & mask_;
    return buf[static_cast<size_t>(i0)] * (1.0f - frac) +
           buf[static_cast<size_t>(i1)] * frac;
  }

  // Find the jump that best continues what we were playing.  The comparison
  // windows end at the read position, i.e. they look *backwards*: correlating
  // forwards would run past the write head into stale audio from the previous
  // lap of the ring, which is why the offset used to come out arbitrary.
  void splice() noexcept {
    const double d = delaySamples();
    const bool tooClose = d < static_cast<double>(minDelay_);
    const int base = std::max(16, static_cast<int>(std::lround(splice_)));
    const int lo = std::max(16, base * 3 / 4);
    const int hi = std::max(lo + 1, base * 5 / 4);
    const int win = std::max(64, std::min(base, 1024));

    const double refStart = read_ - static_cast<double>(win);
    double refEnergy = 0.0;
    for (int k = 0; k < win; ++k) {
      const float v = tap(bufL_, refStart + k);
      refEnergy += static_cast<double>(v) * v;
    }

    double bestScore = -2.0;
    int bestOffset = base;
    if (refEnergy > 1.0e-9) {
      for (int off = lo; off <= hi; ++off) {
        const double cand = tooClose ? read_ - off : read_ + off;
        const double candStart = cand - static_cast<double>(win);
        double corr = 0.0, energy = 0.0;
        for (int k = 0; k < win; ++k) {
          const float a = tap(bufL_, refStart + k);
          const float b = tap(bufL_, candStart + k);
          corr += static_cast<double>(a) * b;
          energy += static_cast<double>(b) * b;
        }
        if (energy <= 1.0e-12) continue;
        const double score = corr / std::sqrt(refEnergy * energy);
        if (score > bestScore) {
          bestScore = score;
          bestOffset = off;
        }
      }
    }
    if (period_ >= 16.0f) {
      // Use exactly one or more whole periods: a correlation maximum can sit a
      // few samples off on a real string, and even a couple of samples per
      // splice is enough to make low notes beat.
      const int p = static_cast<int>(std::lround(period_));
      const int mult = std::max(1, static_cast<int>(std::lround(splice_ / static_cast<float>(p))));
      bestOffset = std::max(16, mult * p);
    } else if (bestScore < -1.0) {
      return;  // nothing usable yet: keep playing
    }

    oldRead_ = read_;
    read_ = tooClose ? read_ - bestOffset : read_ + bestOffset;
    fadeLeft_ = fade_;
  }

  float sr_ = 48000.0f;
  int size_ = 8192;
  int mask_ = 8191;
  int write_ = 0;
  double read_ = 0.0;
  double oldRead_ = 0.0;
  int minDelay_ = 192;
  int maxDelay_ = 1920;
  int fade_ = 192;
  int fadeLeft_ = 0;
  bool primed_ = false;
  float ratio_ = 1.0f;
  float target_ = 1.0f;
  float glideCoeff_ = 1.0f;
  float splice_ = 576.0f;
  float period_ = 0.0f;
  std::vector<float> bufL_, bufR_;
};

} // namespace stomp::modules
