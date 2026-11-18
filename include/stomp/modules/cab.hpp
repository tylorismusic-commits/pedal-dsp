// pedal-dsp — cabinet impulse-response pedal
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <hdsp/biquad.hpp>
#include <hdsp/db.hpp>

namespace stomp::modules {

namespace cabdetail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
} // namespace cabdetail

struct CabControls {
  float lowCut = 0.18f;   // 20..500 Hz
  float highCut = 0.55f;  // 500..20 kHz
  float level = 0.50f;    // -12..+12 dB
  float mix = 1.00f;
};

// Direct FIR cabinet convolution. The impulse is owned by the host-side IR
// library and must outlive this object.
class Cabinet {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    historyL_.assign(kMaxTaps, 0.0f);
    historyR_.assign(kMaxTaps, 0.0f);
    write_ = 0;
    reset();
    dirty_ = true;
  }

  void setIR(const float* h, int taps) noexcept {
    ir_ = h;
    taps_ = h ? std::min(std::max(taps, 0), kMaxTaps) : 0;
    reset();
  }

  void setControls(const CabControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }

  inline void process(float& l, float& r) noexcept {
    update();
    float yl = l;
    float yr = r;
    if (ir_ && taps_ > 0) {
      historyL_[static_cast<size_t>(write_)] = l;
      historyR_[static_cast<size_t>(write_)] = r;
      float cl = 0.0f;
      float cr = 0.0f;
      int idx = write_;
      for (int k = 0; k < taps_; ++k) {
        const float h = ir_[static_cast<size_t>(k)];
        cl += h * historyL_[static_cast<size_t>(idx)];
        cr += h * historyR_[static_cast<size_t>(idx)];
        idx = (idx - 1) & (kMaxTaps - 1);
      }
      write_ = (write_ + 1) & (kMaxTaps - 1);
      yl = cl;
      yr = cr;
    }
    yl = hpL_.process(yl);
    yr = hpR_.process(yr);
    yl = lpL_.process(yl) * level_;
    yr = lpR_.process(yr) * level_;
    l += (yl - l) * mix_;
    r += (yr - r) * mix_;
  }

  void reset() noexcept {
    std::fill(historyL_.begin(), historyL_.end(), 0.0f);
    std::fill(historyR_.begin(), historyR_.end(), 0.0f);
    write_ = 0;
    hpL_.reset(); hpR_.reset();
    lpL_.reset(); lpR_.reset();
  }

  int latencySamples() const noexcept { return 0; }

private:
  static constexpr int kMaxTaps = 4096;
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    const float hpHz = 20.0f * std::pow(25.0f, cabdetail::clamp01(c_.lowCut));
    const float lpHz =
        500.0f * std::pow(40.0f, cabdetail::clamp01(c_.highCut));
    hpL_.highPass(hpHz, 0.7f, sr_);
    hpR_.highPass(hpHz, 0.7f, sr_);
    lpL_.lowPass(std::min(lpHz, sr_ * 0.45f), 0.7f, sr_);
    lpR_.lowPass(std::min(lpHz, sr_ * 0.45f), 0.7f, sr_);
    level_ = hdsp::dbToGain(-12.0f + 24.0f * cabdetail::clamp01(c_.level));
    mix_ = cabdetail::clamp01(c_.mix);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  CabControls c_;
  const float* ir_ = nullptr;
  int taps_ = 0;
  std::vector<float> historyL_, historyR_;
  int write_ = 0;
  hdsp::Biquad hpL_, hpR_, lpL_, lpR_;
  float level_ = 1.0f;
  float mix_ = 1.0f;
};

} // namespace stomp::modules
