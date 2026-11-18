// pedal-dsp — monophonic tuner
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace stomp::modules {

// YIN-style monophonic pitch detector for a tuner display.
class Tuner {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    decim_ = std::max(1, static_cast<int>(std::lround(sr_ / 12000.0f)));
    analysisRate_ = sr_ / static_cast<float>(decim_);
    hop_ = std::max(1, static_cast<int>(sr_ * 0.08f));
    ring_.assign(8192, 0.0f);
    write_ = 0;
    counter_ = 0;
    resetResult();
  }

  inline void process(float l, float r) noexcept {
    ring_[static_cast<size_t>(write_)] = 0.5f * (l + r);
    if (++write_ >= static_cast<int>(ring_.size())) write_ = 0;
    if (++counter_ >= hop_) {
      counter_ = 0;
      analyze();
    }
  }

  float frequency() const noexcept { return hz_; }
  float confidence() const noexcept { return confidence_; }
  int midiNote() const noexcept { return note_; }
  float cents() const noexcept { return cents_; }

  void reset() noexcept {
    std::fill(ring_.begin(), ring_.end(), 0.0f);
    write_ = 0;
    counter_ = 0;
    resetResult();
  }

private:
  static constexpr int kWindow = 1024;

  void resetResult() noexcept {
    hz_ = 0.0f;
    confidence_ = 0.0f;
    note_ = -1;
    cents_ = 0.0f;
  }

  void analyze() noexcept {
    const int n = kWindow;
    if (static_cast<int>(ring_.size()) < n * decim_ + 4) return;
    const int start =
        (write_ - n * decim_ + static_cast<int>(ring_.size())) %
        static_cast<int>(ring_.size());
    float x[kWindow];
    double mean = 0.0;
    for (int k = 0; k < n; ++k) {
      const int base =
          (start + k * decim_) % static_cast<int>(ring_.size());
      float sum = 0.0f;
      for (int j = 0; j < decim_; ++j) {
        sum += ring_[static_cast<size_t>(
            (base + j) % static_cast<int>(ring_.size()))];
      }
      x[k] = sum / static_cast<float>(decim_);
      mean += x[k];
    }
    mean /= n;
    double energy = 0.0;
    for (int k = 0; k < n; ++k) {
      x[k] -= static_cast<float>(mean);
      energy += double(x[k]) * x[k];
    }
    const double rms = std::sqrt(energy / n);
    if (rms < 2.0e-4) {
      resetResult();
      return;
    }

    const int minLag = std::max(2, static_cast<int>(analysisRate_ / 1400.0f));
    const int maxLag = std::min(
        n / 2, static_cast<int>(analysisRate_ / 55.0f) + 1);
    const int compare = n - maxLag;
    if (maxLag <= minLag || compare < 32) {
      resetResult();
      return;
    }

    std::vector<float> diff(static_cast<size_t>(maxLag + 1), 0.0f);
    for (int tau = 1; tau <= maxLag; ++tau) {
      double sum = 0.0;
      for (int i = 0; i < compare; ++i) {
        const double d = double(x[i]) - x[i + tau];
        sum += d * d;
      }
      diff[static_cast<size_t>(tau)] = static_cast<float>(sum);
    }

    // Cumulative mean normalised difference (YIN).
    std::vector<float> cmnd(static_cast<size_t>(maxLag + 1), 1.0f);
    double running = 0.0;
    for (int tau = 1; tau <= maxLag; ++tau) {
      running += diff[static_cast<size_t>(tau)];
      cmnd[static_cast<size_t>(tau)] =
          running > 1.0e-12
              ? static_cast<float>(diff[static_cast<size_t>(tau)] *
                                   tau / running)
              : 1.0f;
    }

    constexpr float kThreshold = 0.16f;
    int best = -1;
    for (int tau = minLag; tau <= maxLag; ++tau) {
      const float value = cmnd[static_cast<size_t>(tau)];
      if (value < kThreshold &&
          value < cmnd[static_cast<size_t>(tau - 1)] &&
          value <= cmnd[static_cast<size_t>(
                       std::min(tau + 1, maxLag))]) {
        best = tau;
        break;
      }
    }
    if (best < 0) {
      float lowest = 1.0e9f;
      for (int tau = minLag; tau <= maxLag; ++tau) {
        const float value = cmnd[static_cast<size_t>(tau)];
        if (value < lowest) {
          lowest = value;
          best = tau;
        }
      }
    }
    if (best < minLag || best > maxLag) {
      resetResult();
      return;
    }

    // Parabolic refinement around the chosen lag.
    float refined = static_cast<float>(best);
    if (best > 1 && best < maxLag) {
      const float a = cmnd[static_cast<size_t>(best - 1)];
      const float b = cmnd[static_cast<size_t>(best)];
      const float c = cmnd[static_cast<size_t>(best + 1)];
      const float denom = a - 2.0f * b + c;
      if (std::fabs(denom) > 1.0e-9f) {
        refined += 0.5f * (a - c) / denom;
      }
    }
    const float conf = 1.0f - cmnd[static_cast<size_t>(best)];
    if (refined < 2.0f || conf < 0.5f) {
      resetResult();
      return;
    }

    float hz = analysisRate_ / refined;
    if (hz < 55.0f || hz > 1400.0f) {
      resetResult();
      return;
    }
    if (hz_ > 0.0f && std::fabs(hz - hz_) / hz_ < 0.04f) {
      hz = 0.5f * (hz + hz_);  // light smoothing on a stable note
    }
    hz_ = hz;
    confidence_ = std::min(std::max(conf, 0.0f), 1.0f);
    const float midi = 69.0f + 12.0f * std::log2(hz / 440.0f);
    note_ = static_cast<int>(std::lround(midi));
    cents_ = (midi - static_cast<float>(note_)) * 100.0f;
  }

  float sr_ = 48000.0f;
  float analysisRate_ = 12000.0f;
  int decim_ = 4;
  int hop_ = 3840;
  int counter_ = 0;
  std::vector<float> ring_;
  int write_ = 0;
  float hz_ = 0.0f;
  float confidence_ = 0.0f;
  int note_ = -1;
  float cents_ = 0.0f;
};

} // namespace stomp::modules
