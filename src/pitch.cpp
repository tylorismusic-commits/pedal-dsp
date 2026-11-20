// pedal-dsp — Signalsmith Stretch pitch engine
// SPDX-License-Identifier: GPL-3.0-or-later
//
// 1024-sample block / 256-sample interval gives 512 samples of input latency
// plus 512 of output latency: ~21 ms at 48 kHz.  That is roughly a third of the
// previous Rubber Band live shifter while still handling chords and full notes.
#include "stomp/modules/pitch.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <hdsp/biquad.hpp>

#include "signalsmith-stretch.h"

namespace stomp::modules {

struct PitchEngine::Impl {
  static constexpr int kBlock = 1024;
  static constexpr int kInterval = 256;

  std::unique_ptr<signalsmith::stretch::SignalsmithStretch<float>> stretch;
  std::vector<float> inL, inR, outL, outR;
  float* inPtrs[2] = {};
  float* outPtrs[2] = {};
  int inPos = 0;
  std::vector<float> fifoL, fifoR;
  size_t fifoSize = 1, fifoMask = 0, fifoRead = 0, fifoWrite = 0, fifoCount = 0;
  std::vector<float> dryL, dryR;
  size_t drySize = 1, dryMask = 0, dryWrite = 0;
  int latency = 1;
  float sr = 48000.0f;
  float ratio = 1.0f, target = 1.0f, glideCoeff = 1.0f;
  bool preserveFormants = false;
  float toneG = 0.5f;
  float toneStateL = 0.0f, toneStateR = 0.0f;
  bool lowCutOn = false;
  hdsp::Biquad lowHpL, lowHpR;
  float mixCurrent = 0.0f;
  float mixCoeff = 0.01f;

  void rebuild(float sampleRate, bool preserve) {
    sr = sampleRate > 0.0f ? sampleRate : 48000.0f;
    preserveFormants = preserve;
    stretch = std::make_unique<signalsmith::stretch::SignalsmithStretch<float>>();
    stretch->configure(2, kBlock, kInterval, false);
    stretch->setTransposeFactor(ratio);
    stretch->setFormantFactor(1.0f, preserveFormants);
    latency = stretch->inputLatency() + stretch->outputLatency();
    inL.assign(kInterval, 0.0f);
    inR.assign(kInterval, 0.0f);
    outL.assign(kInterval, 0.0f);
    outR.assign(kInterval, 0.0f);
    inPtrs[0] = inL.data();
    inPtrs[1] = inR.data();
    outPtrs[0] = outL.data();
    outPtrs[1] = outR.data();
    inPos = 0;
    fifoSize = 1;
    const size_t need = static_cast<size_t>(latency + kInterval * 4 + 8);
    while (fifoSize < need) fifoSize <<= 1;
    fifoMask = fifoSize - 1;
    fifoL.assign(fifoSize, 0.0f);
    fifoR.assign(fifoSize, 0.0f);
    fifoRead = fifoWrite = fifoCount = 0;
    drySize = fifoSize;
    dryMask = fifoMask;
    dryL.assign(drySize, 0.0f);
    dryR.assign(drySize, 0.0f);
    dryWrite = 0;
    mixCurrent = 0.0f;
    mixCoeff = 1.0f - std::exp(-1.0f / (0.005f * sr));
    lowHpL.reset();
    lowHpR.reset();
  }

  void setFilter(hdsp::Biquad& f, bool high, float hz) {
    if (high) {
      f.highPass(hz, 0.7f, sr);
    } else {
      f.lowPass(hz, 0.7f, sr);
    }
  }

  inline float tone(float x, float& state) {
    const float v = (x - state) * toneG;
    const float y = v + state;
    state = y + v;
    if (std::fabs(state) < 1.0e-20f) state = 0.0f;
    return y;
  }

  inline void process(float& l, float& r, float mix, float wetScale) {
    if (!stretch) return;
    ratio += (target - ratio) * glideCoeff;

    const float inputL = l, inputR = r;
    dryL[dryWrite & dryMask] = inputL;
    dryR[dryWrite & dryMask] = inputR;
    ++dryWrite;
    const size_t d = std::min<size_t>(static_cast<size_t>(latency),
                                      drySize - 1);
    const float dryDelayedL = dryL[(dryWrite - d) & dryMask];
    const float dryDelayedR = dryR[(dryWrite - d) & dryMask];

    inL[static_cast<size_t>(inPos)] = inputL;
    inR[static_cast<size_t>(inPos)] = inputR;
    if (++inPos >= kInterval) {
      inPos = 0;
      stretch->setTransposeFactor(ratio);
      stretch->setFormantFactor(1.0f, preserveFormants);
      stretch->process(inPtrs, kInterval, outPtrs, kInterval);
      for (int i = 0; i < kInterval; ++i) {
        fifoL[fifoWrite & fifoMask] = outL[static_cast<size_t>(i)];
        fifoR[fifoWrite & fifoMask] = outR[static_cast<size_t>(i)];
        ++fifoWrite;
        fifoCount = std::min(fifoCount + 1, fifoSize);
      }
    }

    float wetL = 0.0f;
    float wetR = 0.0f;
    if (fifoCount > 0) {
      wetL = fifoL[fifoRead & fifoMask];
      wetR = fifoR[fifoRead & fifoMask];
      ++fifoRead;
      --fifoCount;
    }
    wetL = tone(wetL, toneStateL);
    wetR = tone(wetR, toneStateR);
    if (lowCutOn) {
      wetL = lowHpL.process(wetL);
      wetR = lowHpR.process(wetR);
    }
    wetL *= wetScale;
    wetR *= wetScale;
    const float targetMix = pitchdetail::clamp01(mix);
    mixCurrent += (targetMix - mixCurrent) * mixCoeff;
    l = dryDelayedL + (wetL - dryDelayedL) * mixCurrent;
    r = dryDelayedR + (wetR - dryDelayedR) * mixCurrent;
  }
};

PitchEngine::PitchEngine() : impl_(std::make_unique<Impl>()) {}
PitchEngine::~PitchEngine() = default;

void PitchEngine::prepare(float sr, bool preserveFormants) noexcept {
  impl_->rebuild(sr, preserveFormants);
  impl_->ratio = impl_->target;
  impl_->glideCoeff = 1.0f;
}

void PitchEngine::setRatio(float ratio, float glideSeconds) noexcept {
  impl_->target = std::max(0.125f, std::min(8.0f, ratio));
  impl_->glideCoeff =
      1.0f - std::exp(-1.0f / (std::max(glideSeconds, 0.005f) * impl_->sr));
}

void PitchEngine::setTone(float toneValue) noexcept {
  const float hz =
      1200.0f * std::pow(6.0f, pitchdetail::clamp01(toneValue));
  const float t = std::tan(3.14159265358979323846f *
                           std::min(std::max(hz, 10.0f), impl_->sr * 0.49f) /
                           impl_->sr);
  impl_->toneG = t / (1.0f + t);
}

void PitchEngine::setLowCut(float lowCut) noexcept {
  const float hz = 40.0f + 260.0f * pitchdetail::clamp01(lowCut);
  impl_->setFilter(impl_->lowHpL, true, hz);
  impl_->setFilter(impl_->lowHpR, true, hz);
  impl_->lowCutOn = true;
}

void PitchEngine::clearLowCut() noexcept { impl_->lowCutOn = false; }

void PitchEngine::process(float& l, float& r, float mix, float wetScale) noexcept {
  impl_->process(l, r, mix, wetScale);
}

void PitchEngine::reset() noexcept {
  if (impl_->stretch) impl_->stretch->reset();
  std::fill(impl_->fifoL.begin(), impl_->fifoL.end(), 0.0f);
  std::fill(impl_->fifoR.begin(), impl_->fifoR.end(), 0.0f);
  std::fill(impl_->dryL.begin(), impl_->dryL.end(), 0.0f);
  std::fill(impl_->dryR.begin(), impl_->dryR.end(), 0.0f);
  impl_->inPos = 0;
  impl_->fifoRead = impl_->fifoWrite = impl_->fifoCount = 0;
  impl_->dryWrite = 0;
  impl_->mixCurrent = 0.0f;
  impl_->toneStateL = impl_->toneStateR = 0.0f;
  impl_->lowHpL.reset();
  impl_->lowHpR.reset();
  impl_->ratio = impl_->target;
}

int PitchEngine::latencySamples() const noexcept { return impl_->latency; }

} // namespace stomp::modules
