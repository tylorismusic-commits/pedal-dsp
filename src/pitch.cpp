// pedal-dsp — low-latency waveform transposer
// SPDX-License-Identifier: GPL-3.0-or-later
//
// WSOLA (waveform-similarity overlap-add) transposition.
//
// The input is resampled by the pitch ratio (anti-aliased above unity) and then
// stretched back to the original length with correlation-aligned overlap-add
// grains.  Every grain is a literal copy of the input waveform, so a chord
// keeps its exact intervals and a sustained note has no pitch drift and no
// phase-vocoder "hunt".  Grain starts are nudged to wherever the waveform best
// continues the previous grain, which is what keeps the crossfades coherent
// instead of comb-filtering.
//
// Latency is 20 ms at any ratio: the analysis window is sized in output time
// and the resampling rate is folded into the buffer lead.
#include "stomp/modules/pitch.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <hdsp/biquad.hpp>

namespace stomp::modules {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr int kHannSize = 4096;

constexpr double kWindowSeconds = 0.016;
constexpr double kLeadSeconds = 0.020;
constexpr int kResampleLead = 8;

inline float flushDenormal(float x) noexcept {
  return std::fabs(x) < 1.0e-20f ? 0.0f : x;
}
} // namespace

struct PitchEngine::Impl {
  float sr = 48000.0f;
  float ratio = 1.0f, target = 1.0f, glideCoeff = 1.0f;
  int latency = 1;

  // -------------------------------------------- stage A: input + resampler
  static constexpr size_t kInSize = 8192;
  static constexpr size_t kInMask = kInSize - 1;
  std::vector<float> inL, inR;
  int64_t nIn = 0;
  hdsp::Biquad aa[3][2];
  float aaCut = 0.0f;

  static constexpr size_t kYSize = 16384;
  static constexpr size_t kYMask = kYSize - 1;
  std::vector<float> y1L, y1R;
  // Low-passed mono copy used for grain alignment: correlating the fundamental
  // keeps a whole chord coherent instead of locking onto one loud partial.
  std::vector<float> yMon;
  hdsp::Biquad alignLp[2];
  int64_t mY = 0;
  double accA = 0.0;

  // ------------------------------------------------- WSOLA grains / output
  static constexpr size_t kOutSize = 16384;
  static constexpr size_t kOutMask = kOutSize - 1;
  std::vector<float> outL, outR, wsum;
  int64_t nOut = 0;
  int64_t nextFrame = 0;
  double bNominal = 0.0;
  int64_t bPrev = 0;
  std::vector<float> hann;

  // --------------------------------------------------------- conditioning
  float toneG = 0.5f;
  float toneStateL = 0.0f, toneStateR = 0.0f;
  bool lowCutOn = false;
  hdsp::Biquad lowHpL, lowHpR;

  static constexpr size_t kDrySize = 8192;
  static constexpr size_t kDryMask = kDrySize - 1;
  std::vector<float> dryL, dryR;
  int64_t dryWrite = 0;
  float mixCurrent = 0.0f;
  float mixCoeff = 0.001f;

  void rebuild(float sampleRate) {
    sr = sampleRate > 0.0f ? sampleRate : 48000.0f;
    inL.assign(kInSize, 0.0f);
    inR.assign(kInSize, 0.0f);
    y1L.assign(kYSize, 0.0f);
    y1R.assign(kYSize, 0.0f);
    yMon.assign(kYSize, 0.0f);
    outL.assign(kOutSize, 0.0f);
    outR.assign(kOutSize, 0.0f);
    wsum.assign(kOutSize, 0.0f);
    dryL.assign(kDrySize, 0.0f);
    dryR.assign(kDrySize, 0.0f);

    hann.resize(kHannSize + 1);
    for (int i = 0; i <= kHannSize; ++i) {
      hann[static_cast<size_t>(i)] =
          0.5f - 0.5f * std::cos(2.0f * kPi * static_cast<float>(i) /
                                 static_cast<float>(kHannSize));
    }

    latency = static_cast<int>(std::lround(kLeadSeconds * sr)) + kResampleLead;
    mixCoeff = 1.0f - std::exp(-1.0f / (0.005f * sr));
    reset();
  }

  void reset() {
    std::fill(inL.begin(), inL.end(), 0.0f);
    std::fill(inR.begin(), inR.end(), 0.0f);
    std::fill(y1L.begin(), y1L.end(), 0.0f);
    std::fill(y1R.begin(), y1R.end(), 0.0f);
    std::fill(yMon.begin(), yMon.end(), 0.0f);
    std::fill(outL.begin(), outL.end(), 0.0f);
    std::fill(outR.begin(), outR.end(), 0.0f);
    std::fill(wsum.begin(), wsum.end(), 0.0f);
    std::fill(dryL.begin(), dryL.end(), 0.0f);
    std::fill(dryR.begin(), dryR.end(), 0.0f);
    nIn = 0;
    mY = 0;
    accA = 0.0;
    nOut = 0;
    nextFrame = 0;
    bNominal = 0.0;
    bPrev = 0;
    dryWrite = 0;
    mixCurrent = 0.0f;
    toneStateL = toneStateR = 0.0f;
    for (int s = 0; s < 3; ++s) {
      aa[s][0].reset();
      aa[s][1].reset();
    }
    lowHpL.reset();
    lowHpR.reset();
    alignLp[0].lowPass(420.0f, 0.7071f, sr);
    alignLp[1].lowPass(420.0f, 0.7071f, sr);
    aaCut = 0.0f;
    ratio = target;
    // One analysis hop of look-ahead so the similarity search can move either
    // way; this is where the 20 ms pipeline lead comes from.
    bNominal = static_cast<double>(std::max(1, windowLen() / 4));
    bPrev = static_cast<int64_t>(std::llround(bNominal));
  }

  // ------------------------------------------------------------ anti-alias
  void updateAntiAlias() {
    const float r = std::max(ratio, 0.05f);
    const float cut = std::min(0.40f * sr / std::max(r, 1.0f), 0.45f * sr);
    if (aaCut > 0.0f && std::fabs(cut - aaCut) < aaCut * 0.01f) return;
    aaCut = cut;
    static constexpr float kQ[3] = {0.51763809f, 0.70710678f, 1.93185165f};
    for (int s = 0; s < 3; ++s) {
      aa[s][0].lowPass(cut, kQ[s], sr);
      aa[s][1].lowPass(cut, kQ[s], sr);
    }
  }

  inline float antiAlias(float x, int ch) {
    x = aa[0][ch].process(x);
    x = aa[1][ch].process(x);
    x = aa[2][ch].process(x);
    return flushDenormal(x);
  }

  // ------------------------------------------------------------- resampler
  inline float readCubic(const std::vector<float>& buf, size_t mask, double p) {
    const double fp = std::floor(p);
    const int64_t i = static_cast<int64_t>(fp);
    const float f = static_cast<float>(p - fp);
    const float ym1 = buf[static_cast<size_t>(i - 1) & mask];
    const float y0 = buf[static_cast<size_t>(i) & mask];
    const float y1 = buf[static_cast<size_t>(i + 1) & mask];
    const float y2 = buf[static_cast<size_t>(i + 2) & mask];
    const float a = -0.5f * ym1 + 1.5f * y0 - 1.5f * y1 + 0.5f * y2;
    const float b = ym1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    const float c = -0.5f * ym1 + 0.5f * y1;
    return ((a * f + b) * f + c) * f + y0;
  }

  // ----------------------------------------------------------- WSOLA grains
  int windowLen() const {
    double w = kWindowSeconds * static_cast<double>(sr) /
               static_cast<double>(std::max(ratio, 0.05f));
    if (w < 160.0) w = 160.0;
    if (w > 8192.0) w = 8192.0;
    return static_cast<int>(std::lround(w));
  }

  inline float correlation(int64_t a, int64_t b, int n) const {
    float acc = 0.0f;
    for (int i = 0; i < n; ++i) {
      const size_t ia = static_cast<size_t>(a + i) & kYMask;
      const size_t ib = static_cast<size_t>(b + i) & kYMask;
      acc += yMon[ia] * yMon[ib];
    }
    return acc;
  }

  void writeFrame(int64_t start, int w) {
    for (int i = 0; i < w; ++i) {
      const float win =
          hann[static_cast<size_t>(static_cast<int64_t>(i) * kHannSize / w)];
      const size_t is = static_cast<size_t>(start + i) & kYMask;
      const size_t io = static_cast<size_t>(nextFrame + i) & kOutMask;
      outL[io] += y1L[is] * win;
      outR[io] += y1R[is] * win;
      wsum[io] += win;
    }
  }

  void emitFrames() {
    for (int guard = 0; guard < 64; ++guard) {
      const int w = windowLen();
      const int hs = std::max(1, w / 4);
      const int64_t maxStart = mY - w;
      if (maxStart < 0) return;

      const int64_t nominal = static_cast<int64_t>(std::llround(bNominal));
      if (nominal > maxStart) return;

      int64_t lo = nominal - hs;
      int64_t hi = nominal + hs;
      if (lo < 0) lo = 0;
      if (hi > maxStart) hi = maxStart;
      if (lo > hi) return;

      // Reference: where the previous grain would have continued if it had
      // simply kept playing.
      const int64_t ref = bPrev + hs;
      int ov = hs * 2;
      if (ref + ov > mY) ov = static_cast<int>(mY - ref);
      if (ov < 16) ov = 16;

      int64_t best = lo;
      float bestScore = -1.0e30f;
      for (int64_t s = lo; s <= hi; ++s) {
        float score = correlation(s, ref, ov);
        score -= 2.0e-3f * static_cast<float>(std::llabs(s - nominal));
        if (score > bestScore) {
          bestScore = score;
          best = s;
        }
      }

      writeFrame(best, w);
      bPrev = best;
      // The nominal grid carries the time base; deriving it from the chosen
      // grain would let the read pointer stall or run backwards.
      bNominal += static_cast<double>(hs) / static_cast<double>(ratio);
      nextFrame += hs;
    }
  }

  // ------------------------------------------------------------- per-sample
  inline float tone(float x, float& state) {
    const float v = (x - state) * toneG;
    const float y = v + state;
    state = y + v;
    if (std::fabs(state) < 1.0e-20f) state = 0.0f;
    return y;
  }

  inline void process(float& l, float& r, float mix, float wetScale) {
    ratio += (target - ratio) * glideCoeff;
    updateAntiAlias();

    const float dryInL = l, dryInR = r;
    dryL[static_cast<size_t>(dryWrite) & kDryMask] = dryInL;
    dryR[static_cast<size_t>(dryWrite) & kDryMask] = dryInR;
    ++dryWrite;
    const size_t d =
        std::min<size_t>(static_cast<size_t>(latency), kDryMask);
    const float dryOutL =
        dryL[static_cast<size_t>(dryWrite - static_cast<int64_t>(d)) & kDryMask];
    const float dryOutR =
        dryR[static_cast<size_t>(dryWrite - static_cast<int64_t>(d)) & kDryMask];

    inL[static_cast<size_t>(nIn) & kInMask] = antiAlias(l, 0);
    inR[static_cast<size_t>(nIn) & kInMask] = antiAlias(r, 1);
    ++nIn;

    accA += 1.0 / static_cast<double>(std::max(ratio, 0.05f));
    while (accA >= 1.0) {
      accA -= 1.0;
      double p = static_cast<double>(mY) * static_cast<double>(ratio) -
                 static_cast<double>(kResampleLead);
      const double hiP = static_cast<double>(nIn - 3);
      if (p > hiP) p = hiP;
      if (p < 0.0) p = 0.0;
      const size_t iy = static_cast<size_t>(mY) & kYMask;
      y1L[iy] = readCubic(inL, kInMask, p);
      y1R[iy] = readCubic(inR, kInMask, p);
      float mono = 0.5f * (y1L[iy] + y1R[iy]);
      mono = alignLp[0].process(mono);
      yMon[iy] = flushDenormal(alignLp[1].process(mono));
      ++mY;
    }

    emitFrames();

    float wetL = 0.0f, wetR = 0.0f;
    if (nOut < nextFrame) {
      const size_t k = static_cast<size_t>(nOut) & kOutMask;
      const float norm = 1.0f / std::max(wsum[k], 1.0e-4f);
      wetL = outL[k] * norm;
      wetR = outR[k] * norm;
      outL[k] = 0.0f;
      outR[k] = 0.0f;
      wsum[k] = 0.0f;
      ++nOut;
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
  const float t = std::tan(kPi * std::min(std::max(hz, 10.0f),
                                          impl_->sr * 0.49f) /
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

void PitchEngine::reset() noexcept { impl_->reset(); }

int PitchEngine::latencySamples() const noexcept { return impl_->latency; }

} // namespace stomp::modules
