// pedal-dsp — polyphonic pitch engine
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Spectrally processed transposition (Signalsmith Stretch, MIT).
//
// The window and hop are fixed for a given sample rate, so changing Pitch,
// Glide or any other control only changes the phase advance — it cannot rebuild
// the analysis grid or click.  A 4096-sample window (85 ms at 48 kHz) is the
// smallest size that keeps a guitar chord coherent: measured on a
// 196/247/294 Hz triad it puts ~99% of the output energy on the three shifted
// partials, while 2048 smears each partial into +/-20 Hz sidebands, which is
// the "hunting" sound.
//
// No formant processing is applied.  A transpose should move the whole
// spectrum, exactly like down-tuning the instrument.
#include "stomp/modules/pitch.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <hdsp/biquad.hpp>

#include "signalsmith-stretch.h"

namespace stomp::modules {

namespace {
constexpr int kBlock = 4096;    // analysis/synthesis window
constexpr int kInterval = 512;  // hop, 8x overlap
} // namespace

struct PitchEngine::Impl {
  std::unique_ptr<signalsmith::stretch::SignalsmithStretch<float>> stretch;
  std::vector<float> inL, inR, outL, outR;
  float* inPtrs[2] = {};
  float* outPtrs[2] = {};
  int inPos = 0;
  int outPos = 0;

  // Latency-matched dry path and mix ramp.
  std::vector<float> dryL, dryR;
  size_t dryMask = 0;
  int64_t dryWrite = 0;
  int latency = 1;
  float sr = 48000.0f;

  float ratio = 1.0f, target = 1.0f, glideCoeff = 1.0f;
  float toneG = 0.5f;
  float toneStateL = 0.0f, toneStateR = 0.0f;
  bool lowCutOn = false;
  hdsp::Biquad lowHpL, lowHpR;
  float mixCurrent = 0.0f;
  float mixCoeff = 0.01f;

  void rebuild(float sampleRate) {
    sr = sampleRate > 0.0f ? sampleRate : 48000.0f;
    stretch = std::make_unique<signalsmith::stretch::SignalsmithStretch<float>>();
    stretch->configure(2, kBlock, kInterval, false);
    stretch->setTransposeFactor(ratio);
    stretch->setFormantFactor(1.0f, false);
    // The engine reports the window; the block pipeline adds one hop because a
    // frame is drained over the following interval.  Measured 4608 at 48 kHz.
    latency =
        stretch->inputLatency() + stretch->outputLatency() + kInterval;

    inL.assign(kInterval, 0.0f);
    inR.assign(kInterval, 0.0f);
    outL.assign(kInterval, 0.0f);
    outR.assign(kInterval, 0.0f);
    inPtrs[0] = inL.data();
    inPtrs[1] = inR.data();
    outPtrs[0] = outL.data();
    outPtrs[1] = outR.data();
    inPos = 0;
    outPos = 0;

    size_t dry = 1;
    while (dry < static_cast<size_t>(latency) + 8192) dry <<= 1;
    dryL.assign(dry, 0.0f);
    dryR.assign(dry, 0.0f);
    dryMask = dry - 1;
    dryWrite = 0;

    mixCurrent = 0.0f;
    mixCoeff = 1.0f - std::exp(-1.0f / (0.005f * sr));
    lowHpL.reset();
    lowHpR.reset();
    toneStateL = toneStateR = 0.0f;
    ratio = target;
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

    // Dry path, delayed by the pipeline so the mix never moves the timing.
    dryL[static_cast<size_t>(dryWrite) & dryMask] = l;
    dryR[static_cast<size_t>(dryWrite) & dryMask] = r;
    ++dryWrite;
    const size_t d = static_cast<size_t>(latency);
    const float dryOutL =
        dryL[static_cast<size_t>(dryWrite - static_cast<int64_t>(d)) & dryMask];
    const float dryOutR =
        dryR[static_cast<size_t>(dryWrite - static_cast<int64_t>(d)) & dryMask];

    inL[static_cast<size_t>(inPos)] = l;
    inR[static_cast<size_t>(inPos)] = r;
    if (++inPos >= kInterval) {
      inPos = 0;
      // Fixed window and hop: this only changes the phase advance, so a control
      // sweep cannot re-grid or click the output.
      stretch->setTransposeFactor(ratio);
      stretch->process(inPtrs, kInterval, outPtrs, kInterval);
      outPos = 0;
    }

    float wetL = outL[static_cast<size_t>(outPos)];
    float wetR = outR[static_cast<size_t>(outPos)];
    if (++outPos >= kInterval) outPos = 0;

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
    l = dryOutL + (wetL - dryOutL) * mixCurrent;
    r = dryOutR + (wetR - dryOutR) * mixCurrent;
  }
};

PitchEngine::PitchEngine() : impl_(std::make_unique<Impl>()) {}
PitchEngine::~PitchEngine() = default;

void PitchEngine::prepare(float sr, bool /*preserveFormants*/) noexcept {
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
  const float hz = 1200.0f * std::pow(6.0f, pitchdetail::clamp01(toneValue));
  const float t = std::tan(3.14159265358979323846f *
                           std::min(std::max(hz, 10.0f), impl_->sr * 0.49f) /
                           impl_->sr);
  impl_->toneG = t / (1.0f + t);
}

void PitchEngine::setLowCut(float lowCut) noexcept {
  const float hz = 40.0f + 260.0f * pitchdetail::clamp01(lowCut);
  impl_->lowHpL.highPass(hz, 0.7f, impl_->sr);
  impl_->lowHpR.highPass(hz, 0.7f, impl_->sr);
  impl_->lowCutOn = true;
}

void PitchEngine::clearLowCut() noexcept { impl_->lowCutOn = false; }

void PitchEngine::process(float& l, float& r, float mix,
                          float wetScale) noexcept {
  impl_->process(l, r, mix, wetScale);
}

void PitchEngine::reset() noexcept {
  if (impl_->stretch) impl_->stretch->reset();
  std::fill(impl_->dryL.begin(), impl_->dryL.end(), 0.0f);
  std::fill(impl_->dryR.begin(), impl_->dryR.end(), 0.0f);
  std::fill(impl_->inL.begin(), impl_->inL.end(), 0.0f);
  std::fill(impl_->inR.begin(), impl_->inR.end(), 0.0f);
  std::fill(impl_->outL.begin(), impl_->outL.end(), 0.0f);
  std::fill(impl_->outR.begin(), impl_->outR.end(), 0.0f);
  impl_->inPos = 0;
  impl_->outPos = 0;
  impl_->dryWrite = 0;
  impl_->mixCurrent = 0.0f;
  impl_->toneStateL = impl_->toneStateR = 0.0f;
  impl_->lowHpL.reset();
  impl_->lowHpR.reset();
  impl_->ratio = impl_->target;
}

int PitchEngine::latencySamples() const noexcept { return impl_->latency; }

} // namespace stomp::modules
