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
    ref_.assign(1024, 0.0f);
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
      // Raised-cosine fade: a linear one has a step in slope at both ends,
      // which is a click on a bright note.
      const float x =
          1.0f - static_cast<float>(fadeLeft_) / static_cast<float>(fade_);
      const float a = 0.5f - 0.5f * std::cos(3.14159265f * x);
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

#ifdef STOMP_SOLA_DEBUG
  // Test hooks: the splice log is what tells us whether a jump actually landed
  // on the period, and the fade gain lets a test separate the two heads.
  unsigned splices() const noexcept { return splices_; }
  double lastJump() const noexcept { return lastJump_; }
  double readPosition() const noexcept { return read_; }
  float fadeGain() const noexcept {
    if (fadeLeft_ <= 0) return -1.0f;
    const float x =
        1.0f - static_cast<float>(fadeLeft_) / static_cast<float>(fade_);
    return 0.5f - 0.5f * std::cos(3.14159265f * x);
  }
#endif

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

  // Find the jump that best continues what we were already playing.  The
  // comparison windows end at the read position, i.e. they look *backwards*:
  // correlating forwards would run past the write head into stale audio from
  // the previous lap of the ring.
  //
  // The jump is not forced to a whole number of tracked periods.  A real note
  // has vibrato and a decaying spectrum, so the tracker's estimate -- an
  // average over its analysis window -- can be several samples out, and the
  // phase error lands in the crossfade as a flam: at a low note that is what
  // makes a dropped note sound like two notes.  The period only centres the
  // search; the correlation of the actual waveform picks the offset, refined
  // to a fraction of a sample so there is no step left at the splice.
  void splice() noexcept {
    const double d = delaySamples();
    const bool tooClose = d < static_cast<double>(minDelay_);
    const double room = tooClose ? static_cast<double>(maxDelay_) - d
                                 : d - static_cast<double>(minDelay_);
    const double period = static_cast<double>(period_);
    double center = std::max(16.0, static_cast<double>(std::lround(splice_)));
    if (period >= 16.0) {
      // Two periods mean the fewest joins, but only when both fit in the
      // delay range: a note below ~48 Hz otherwise re-splices on the very
      // next sample and the head jumps back and forth.
      int mult = static_cast<int>(std::floor((room - 16.0) / period));
      mult = std::max(1, std::min(2, mult));
      center = static_cast<double>(mult) * period;
      if (center > room - 16.0) {
        // The period alone is longer than the window can hold: centre on the
        // room we have and let the correlation choose (still far better than
        // jumping by a fraction of a period blindly).
        center = room * 0.5;
      }
    }
    const double search = period >= 16.0
                              ? std::max(4.0, 0.08 * period)
                              : std::max(16.0, 0.25 * center);
    const double lo = std::max(16.0, center - search);
    const double hi = std::min(room - 8.0, center + search);
    if (hi <= lo + 1.0) return;  // no room to move: keep playing

    const int win = static_cast<int>(
        std::max(64.0, std::min(1024.0, period >= 16.0 ? period : center)));
    const double refStart = read_ - static_cast<double>(win);
    // Copy the reference once.  Tapping it inside the offset loop would
    // multiply a modular floor by the search length -- hundreds of thousands
    // of libm calls on the audio thread at every splice.
    double refEnergy = 0.0;
    for (int k = 0; k < win; ++k) {
      const float v = tap(bufL_, refStart + k);
      ref_[static_cast<size_t>(k)] = v;
      refEnergy += static_cast<double>(v) * v;
    }
    if (refEnergy <= 1.0e-9) return;

    auto scoreAt = [&](double off) {
      const double cand = tooClose ? read_ - off : read_ + off;
      double p = std::fmod(cand - static_cast<double>(win),
                           static_cast<double>(size_));
      if (p < 0.0) p += static_cast<double>(size_);
      const int i0 = static_cast<int>(p) & mask_;
      const float frac = static_cast<float>(p - std::floor(p));
      const float g0 = 1.0f - frac;
      double corr = 0.0, energy = 0.0;
      for (int k = 0; k < win; ++k) {
        const float b = bufL_[static_cast<size_t>((i0 + k) & mask_)] * g0 +
                        bufL_[static_cast<size_t>((i0 + k + 1) & mask_)] * frac;
        corr += static_cast<double>(ref_[static_cast<size_t>(k)]) * b;
        energy += static_cast<double>(b) * b;
      }
      return energy > 1.0e-12 ? corr / std::sqrt(refEnergy * energy) : -2.0;
    };

    // Coarse whole-sample scan, then a fine pass around the winner: the
    // correlation peak is smooth, so this costs a quarter of a flat scan and
    // still leaves no phase step at the splice.
    double bestScore = -2.0;
    double bestOffset = center;
    for (double off = lo; off <= hi; off += 1.0) {
      const double score = scoreAt(off);
      if (score > bestScore) {
        bestScore = score;
        bestOffset = off;
      }
    }
    if (bestScore < -1.0) return;  // nothing usable yet: keep playing
    constexpr double kFine = 0.05;
    const double fineLo = std::max(lo, bestOffset - 1.0);
    const double fineHi = std::min(hi, bestOffset + 1.0);
    for (double off = fineLo; off <= fineHi; off += kFine) {
      const double score = scoreAt(off);
      if (score > bestScore) {
        bestScore = score;
        bestOffset = off;
      }
    }

    // Parabolic refinement of the correlation peak.
    const double s0 = scoreAt(bestOffset - kFine);
    const double s2 = scoreAt(bestOffset + kFine);
    const double denom = s0 - 2.0 * bestScore + s2;
    if (std::fabs(denom) > 1.0e-9) {
      const double delta = 0.5 * (s0 - s2) / denom;
      bestOffset += std::max(-1.0, std::min(1.0, delta)) * kFine;
      bestOffset = std::max(lo, std::min(hi, bestOffset));
    }

    oldRead_ = read_;
    read_ = tooClose ? read_ - bestOffset : read_ + bestOffset;
    fadeLeft_ = fade_;
#ifdef STOMP_SOLA_DEBUG
    lastJump_ = bestOffset;
    lastClose_ = tooClose;
    ++splices_;
#endif
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
  std::vector<float> ref_;
#ifdef STOMP_SOLA_DEBUG
  unsigned splices_ = 0;
  double lastJump_ = 0.0;
  bool lastClose_ = false;
#endif
};

} // namespace stomp::modules
