// pedal-dsp — cabinet impulse-response pedal
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

#include <fftw3.h>
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

// Zero-latency-except-one-block partitioned convolution. 256-sample blocks,
// 4096 taps, overlap-add. The impulse is owned by the host IR library.
class Cabinet {
public:
  static constexpr int kBlock = 256;
  static constexpr int kFft = 512;
  static constexpr int kBins = kFft / 2 + 1;
  static constexpr int kParts = 16;
  static constexpr int kMaxTaps = kBlock * kParts;

  Cabinet() {
    timeL_.resize(kFft);
    timeR_.resize(kFft);
    freqL_.resize(kBins);
    freqR_.resize(kBins);
    yL_.resize(kBins);
    yR_.resize(kBins);
    outL_.resize(kFft);
    outR_.resize(kFft);
    irFreq_.resize(static_cast<size_t>(kParts) * kBins);
    fdlL_.resize(static_cast<size_t>(kParts) * kBins);
    fdlR_.resize(static_cast<size_t>(kParts) * kBins);
    inL_.resize(kBlock);
    inR_.resize(kBlock);
    fifoL_.resize(kBlock);
    fifoR_.resize(kBlock);
    overlapL_.resize(kBlock);
    overlapR_.resize(kBlock);
  }

  ~Cabinet() {
    if (fwdL_) fftwf_destroy_plan(fwdL_);
    if (fwdR_) fftwf_destroy_plan(fwdR_);
    if (invL_) fftwf_destroy_plan(invL_);
    if (invR_) fftwf_destroy_plan(invR_);
  }

  Cabinet(const Cabinet&) = delete;
  Cabinet& operator=(const Cabinet&) = delete;

  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    if (!fwdL_) {
      fwdL_ = fftwf_plan_dft_r2c_1d(
          kFft, timeL_.data(),
          reinterpret_cast<fftwf_complex*>(freqL_.data()), FFTW_ESTIMATE);
      fwdR_ = fftwf_plan_dft_r2c_1d(
          kFft, timeR_.data(),
          reinterpret_cast<fftwf_complex*>(freqR_.data()), FFTW_ESTIMATE);
      invL_ = fftwf_plan_dft_c2r_1d(
          kFft, reinterpret_cast<fftwf_complex*>(yL_.data()), outL_.data(),
          FFTW_ESTIMATE);
      invR_ = fftwf_plan_dft_c2r_1d(
          kFft, reinterpret_cast<fftwf_complex*>(yR_.data()), outR_.data(),
          FFTW_ESTIMATE);
    }
    reset();
    dirty_ = true;
  }

  void setIR(const float* h, int taps) noexcept {
    taps_ = h ? std::min(std::max(taps, 0), kMaxTaps) : 0;
    if (fwdL_ && taps_ > 0) {
      for (int p = 0; p < kParts; ++p) {
        std::fill(timeL_.begin(), timeL_.end(), 0.0f);
        const int start = p * kBlock;
        const int count = std::min(kBlock, std::max(0, taps_ - start));
        for (int i = 0; i < count; ++i) timeL_[static_cast<size_t>(i)] = h[start + i];
        fftwf_execute(fwdL_);
        std::copy(freqL_.begin(), freqL_.end(),
                  irFreq_.begin() + static_cast<size_t>(p) * kBins);
      }
    } else {
      std::fill(irFreq_.begin(), irFreq_.end(), std::complex<float>(0.0f, 0.0f));
    }
    reset();
  }

  void setControls(const CabControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }

  inline void process(float& l, float& r) noexcept {
    update();
    inL_[static_cast<size_t>(inCount_)] = l;
    inR_[static_cast<size_t>(inCount_)] = r;
    if (++inCount_ >= kBlock) {
      inCount_ = 0;
      processBlock();
      outCount_ = kBlock;
      outPos_ = 0;
    }
    float wet = 0.0f;
    if (outCount_ > 0) {
      wet = fifoL_[static_cast<size_t>(outPos_)];
      const float wetR = fifoR_[static_cast<size_t>(outPos_)];
      ++outPos_;
      --outCount_;
      wet = hpL_.process(wet);
      wet = lpL_.process(wet) * level_;
      const float wr = hpR_.process(wetR);
      const float wr2 = lpR_.process(wr) * level_;
      l += (wet - l) * mix_;
      r += (wr2 - r) * mix_;
      return;
    }
    l = 0.0f;
    r = 0.0f;
  }

  void reset() noexcept {
    std::fill(fdlL_.begin(), fdlL_.end(), std::complex<float>(0.0f, 0.0f));
    std::fill(fdlR_.begin(), fdlR_.end(), std::complex<float>(0.0f, 0.0f));
    std::fill(overlapL_.begin(), overlapL_.end(), 0.0f);
    std::fill(overlapR_.begin(), overlapR_.end(), 0.0f);
    std::fill(fifoL_.begin(), fifoL_.end(), 0.0f);
    std::fill(fifoR_.begin(), fifoR_.end(), 0.0f);
    std::fill(inL_.begin(), inL_.end(), 0.0f);
    std::fill(inR_.begin(), inR_.end(), 0.0f);
    inCount_ = 0;
    outCount_ = 0;
    outPos_ = 0;
    fdlPos_ = 0;
    hpL_.reset(); hpR_.reset(); lpL_.reset(); lpR_.reset();
  }

  int latencySamples() const noexcept { return kBlock; }

private:
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

  void processBlock() noexcept {
    for (int i = 0; i < kBlock; ++i) {
      timeL_[static_cast<size_t>(i)] = inL_[static_cast<size_t>(i)];
      timeR_[static_cast<size_t>(i)] = inR_[static_cast<size_t>(i)];
      timeL_[static_cast<size_t>(kBlock + i)] = 0.0f;
      timeR_[static_cast<size_t>(kBlock + i)] = 0.0f;
    }
    fftwf_execute(fwdL_);
    fftwf_execute(fwdR_);
    std::copy(freqL_.begin(), freqL_.end(),
              fdlL_.begin() + static_cast<size_t>(fdlPos_) * kBins);
    std::copy(freqR_.begin(), freqR_.end(),
              fdlR_.begin() + static_cast<size_t>(fdlPos_) * kBins);
    std::fill(yL_.begin(), yL_.end(), std::complex<float>(0.0f, 0.0f));
    std::fill(yR_.begin(), yR_.end(), std::complex<float>(0.0f, 0.0f));
    for (int p = 0; p < kParts; ++p) {
      const int idx = (fdlPos_ - p + kParts) % kParts;
      const std::complex<float>* ir =
          irFreq_.data() + static_cast<size_t>(p) * kBins;
      const std::complex<float>* xl =
          fdlL_.data() + static_cast<size_t>(idx) * kBins;
      const std::complex<float>* xr =
          fdlR_.data() + static_cast<size_t>(idx) * kBins;
      for (int b = 0; b < kBins; ++b) {
        yL_[static_cast<size_t>(b)] += xl[b] * ir[b];
        yR_[static_cast<size_t>(b)] += xr[b] * ir[b];
      }
    }
    fftwf_execute(invL_);
    fftwf_execute(invR_);
    constexpr float norm = 1.0f / static_cast<float>(kFft);
    for (int i = 0; i < kBlock; ++i) {
      fifoL_[static_cast<size_t>(i)] =
          outL_[static_cast<size_t>(i)] * norm + overlapL_[static_cast<size_t>(i)];
      fifoR_[static_cast<size_t>(i)] =
          outR_[static_cast<size_t>(i)] * norm + overlapR_[static_cast<size_t>(i)];
      overlapL_[static_cast<size_t>(i)] =
          outL_[static_cast<size_t>(kBlock + i)] * norm;
      overlapR_[static_cast<size_t>(i)] =
          outR_[static_cast<size_t>(kBlock + i)] * norm;
    }
    fdlPos_ = (fdlPos_ + 1) % kParts;
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  CabControls c_;
  int taps_ = 0;
  fftwf_plan fwdL_ = nullptr, fwdR_ = nullptr, invL_ = nullptr, invR_ = nullptr;
  std::vector<float> timeL_, timeR_;
  std::vector<std::complex<float>> freqL_, freqR_, yL_, yR_;
  std::vector<float> outL_, outR_;
  std::vector<std::complex<float>> irFreq_;
  std::vector<std::complex<float>> fdlL_, fdlR_;
  std::vector<float> inL_, inR_, fifoL_, fifoR_, overlapL_, overlapR_;
  int inCount_ = 0;
  int outCount_ = 0;
  int outPos_ = 0;
  int fdlPos_ = 0;
  hdsp::Biquad hpL_, hpR_, lpL_, lpR_;
  float level_ = 1.0f;
  float mix_ = 1.0f;
};

} // namespace stomp::modules
