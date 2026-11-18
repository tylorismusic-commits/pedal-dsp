// pedal-dsp — Rubber Band Live pitch engine
// SPDX-License-Identifier: GPL-3.0-or-later
#include "stomp/modules/pitch.hpp"

#include <rubberband/RubberBandLiveShifter.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <hdsp/biquad.hpp>

namespace stomp::modules {

struct PitchEngine::Impl {
  std::unique_ptr<RubberBand::RubberBandLiveShifter> shifter;
  size_t block = 0;
  std::vector<float> inL, inR, outL, outR;
  float* inPtrs[2] = {};
  float* outPtrs[2] = {};
  size_t inPos = 0;
  size_t outPos = 0;
  float sr = 48000.0f;
  float ratio = 1.0f, target = 1.0f, glideCoeff = 1.0f;
  float toneG = 0.5f;
  float toneStateL = 0.0f, toneStateR = 0.0f;
  bool lowCutOn = false;
  hdsp::Biquad lowHpL, lowHpR;
  int latency = 0;

  void rebuild(float sampleRate) {
    sr = sampleRate > 0.0f ? sampleRate : 48000.0f;
    shifter = std::make_unique<RubberBand::RubberBandLiveShifter>(
        static_cast<size_t>(sr), 2,
        RubberBand::RubberBandLiveShifter::OptionWindowShort |
            RubberBand::RubberBandLiveShifter::OptionFormantPreserved |
            RubberBand::RubberBandLiveShifter::OptionChannelsTogether);
    shifter->setPitchScale(ratio);
    block = shifter->getBlockSize();
    inL.assign(block, 0.0f);
    inR.assign(block, 0.0f);
    outL.assign(block, 0.0f);
    outR.assign(block, 0.0f);
    inPtrs[0] = inL.data();
    inPtrs[1] = inR.data();
    outPtrs[0] = outL.data();
    outPtrs[1] = outR.data();
    inPos = block;
    outPos = block;
    latency = static_cast<int>(shifter->getStartDelay() + block);
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

  inline void process(float& l, float& r, float mix) {
    if (!shifter || block == 0) return;
    ratio += (target - ratio) * glideCoeff;
    const float dryL = l, dryR = r;

    if (inPos >= block) {
      shifter->setPitchScale(ratio);
      shifter->shift(inPtrs, outPtrs);
      inPos = 0;
      outPos = 0;
    }
    inL[inPos] = l;
    inR[inPos] = r;
    ++inPos;

    float wetL = 0.0f, wetR = 0.0f;
    if (outPos < block) {
      wetL = outL[outPos];
      wetR = outR[outPos];
      ++outPos;
    }
    wetL = tone(wetL, toneStateL);
    wetR = tone(wetR, toneStateR);
    if (lowCutOn) {
      wetL = lowHpL.process(wetL);
      wetR = lowHpR.process(wetR);
    }
    const float m = pitchdetail::clamp01(mix);
    l = dryL + (wetL - dryL) * m;
    r = dryR + (wetR - dryR) * m;
  }
};

PitchEngine::PitchEngine() : impl_(std::make_unique<Impl>()) {}
PitchEngine::~PitchEngine() = default;

void PitchEngine::prepare(float sr) noexcept {
  impl_->rebuild(sr);
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

void PitchEngine::process(float& l, float& r, float mix) noexcept {
  impl_->process(l, r, mix);
}

void PitchEngine::reset() noexcept {
  if (impl_->shifter) impl_->shifter->reset();
  impl_->inPos = impl_->block;
  impl_->outPos = impl_->block;
  impl_->toneStateL = impl_->toneStateR = 0.0f;
  impl_->lowHpL.reset();
  impl_->lowHpR.reset();
  impl_->ratio = impl_->target;
}

int PitchEngine::latencySamples() const noexcept { return impl_->latency; }

} // namespace stomp::modules
