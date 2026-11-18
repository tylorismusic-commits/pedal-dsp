// pedal-dsp — modulation pedals
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <hdsp/biquad.hpp>

namespace stomp::modules {

namespace modulationdetail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
inline float lfoRate(float control) noexcept {
  return 0.05f * std::pow(200.0f, clamp01(control));
}

// Fractional-delay circular buffer shared by the modulation pedals.
struct ModDelay {
  std::vector<float> buf;
  int size = 0;
  int write = 0;

  void prepare(int maxSamples) {
    size = std::max(8, maxSamples);
    buf.assign(static_cast<size_t>(size), 0.0f);
    write = 0;
  }
  void reset() noexcept {
    std::fill(buf.begin(), buf.end(), 0.0f);
    write = 0;
  }
  inline void push(float x) noexcept {
    buf[static_cast<size_t>(write)] = x;
    if (++write >= size) write = 0;
  }
  inline float read(float delaySamples) const noexcept {
    float pos = static_cast<float>(write) - delaySamples;
    while (pos < 0.0f) pos += static_cast<float>(size);
    int i0 = static_cast<int>(pos);
    if (i0 >= size) i0 -= size;
    const float frac = pos - static_cast<float>(i0);
    int i1 = i0 + 1;
    if (i1 >= size) i1 = 0;
    return buf[static_cast<size_t>(i0)] * (1.0f - frac) +
           buf[static_cast<size_t>(i1)] * frac;
  }
};
} // namespace modulationdetail

// Analog-style stereo chorus.
struct ChorusControls {
  float rate = 0.35f;
  float depth = 0.50f;
  float width = 0.60f;
  float mix = 0.50f;
};

class Chorus {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    const int maxSamples = static_cast<int>(sr_ * 0.035f) + 4;
    for (auto& d : delay_) d.prepare(maxSamples);
    reset();
    dirty_ = true;
  }
  void setControls(const ChorusControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    phase_ += 2.0f * kPi * rateHz_ / sr_;
    if (phase_ >= 2.0f * kPi) phase_ -= 2.0f * kPi;

    const float modL = 0.5f + 0.5f * std::sin(phase_);
    const float modR = 0.5f + 0.5f * std::sin(phase_ + width_ * kPi);
    const float baseSamples = 0.008f * sr_;
    const float depthSamples = depthMs_ * 0.001f * sr_;
    const float wetL = delay_[0].read(baseSamples + depthSamples * modL);
    const float wetR = delay_[1].read(baseSamples + depthSamples * modR);

    delay_[0].push(l);
    delay_[1].push(r);

    l += (wetL - l) * mix_;
    r += (wetR - r) * mix_;
  }
  void reset() noexcept {
    for (auto& d : delay_) d.reset();
    phase_ = 0.0f;
  }
  int latencySamples() const noexcept { return 0; }

private:
  static constexpr float kPi = 3.14159265358979323846f;
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    rateHz_ = modulationdetail::lfoRate(c_.rate);
    depthMs_ = 0.5f + 7.0f * modulationdetail::clamp01(c_.depth);
    width_ = modulationdetail::clamp01(c_.width);
    mix_ = modulationdetail::clamp01(c_.mix);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  ChorusControls c_;
  modulationdetail::ModDelay delay_[2];
  float phase_ = 0.0f;
  float rateHz_ = 0.35f;
  float depthMs_ = 4.0f;
  float width_ = 0.6f;
  float mix_ = 0.5f;
};

// Feedback flanger with a bipolar feedback control.
struct FlangerControls {
  float rate = 0.30f;
  float depth = 0.55f;
  float feedback = 0.60f;
  float mix = 0.55f;
};

class Flanger {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    const int maxSamples = static_cast<int>(sr_ * 0.012f) + 4;
    for (auto& d : delay_) d.prepare(maxSamples);
    reset();
    dirty_ = true;
  }
  void setControls(const FlangerControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    phase_ += 2.0f * kPi * rateHz_ / sr_;
    if (phase_ >= 2.0f * kPi) phase_ -= 2.0f * kPi;

    const float mod = 0.5f + 0.5f * std::sin(phase_);
    const float baseSamples = 0.0006f * sr_;
    const float depthSamples = depthMs_ * 0.001f * sr_;
    const float wetL = delay_[0].read(baseSamples + depthSamples * mod);
    const float wetR = delay_[1].read(baseSamples + depthSamples * mod);

    delay_[0].push(l + wetL * feedback_);
    delay_[1].push(r + wetR * feedback_);

    l += wetL * mix_;
    r += wetR * mix_;
  }
  void reset() noexcept {
    for (auto& d : delay_) d.reset();
    phase_ = 0.0f;
  }
  int latencySamples() const noexcept { return 0; }

private:
  static constexpr float kPi = 3.14159265358979323846f;
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    rateHz_ = modulationdetail::lfoRate(c_.rate);
    depthMs_ = 0.2f + 5.0f * modulationdetail::clamp01(c_.depth);
    feedback_ = (modulationdetail::clamp01(c_.feedback) - 0.5f) * 1.7f;
    mix_ = modulationdetail::clamp01(c_.mix);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  FlangerControls c_;
  modulationdetail::ModDelay delay_[2];
  float phase_ = 0.0f;
  float rateHz_ = 0.3f;
  float depthMs_ = 3.0f;
  float feedback_ = 0.2f;
  float mix_ = 0.55f;
};

// Six-stage all-pass phaser.
struct PhaserControls {
  float rate = 0.30f;
  float depth = 0.60f;
  float feedback = 0.50f;
  float mix = 0.50f;
};

class Phaser {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const PhaserControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    phase_ += 2.0f * kPi * rateHz_ / sr_;
    if (phase_ >= 2.0f * kPi) phase_ -= 2.0f * kPi;

    const float lfo = 0.5f + 0.5f * std::sin(phase_);
    const float hz = 250.0f * std::pow(10.0f, lfo * depth_ * 0.95f);
    const float t = std::tan(kPi * std::min(std::max(hz, 30.0f), sr_ * 0.45f) /
                             sr_);
    const float a = (t - 1.0f) / (t + 1.0f);

    const float wetL = processCh(st_[0], l, a);
    const float wetR = processCh(st_[1], r, a);
    l += wetL * mix_;
    r += wetR * mix_;
  }
  void reset() noexcept {
    for (auto& ch : st_) {
      for (auto& s : ch) s = 0.0f;
    }
    phase_ = 0.0f;
  }
  int latencySamples() const noexcept { return 0; }

private:
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr int kStages = 6;
  inline float processCh(float* state, float x, float a) noexcept {
    float y = x + feedback_ * state[kStages - 1];
    for (int i = 0; i < kStages; ++i) {
      const float out = -a * y + state[i];
      state[i] = y + a * out;
      y = out;
    }
    return y;
  }
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    rateHz_ = modulationdetail::lfoRate(c_.rate);
    depth_ = modulationdetail::clamp01(c_.depth);
    feedback_ = 0.8f * modulationdetail::clamp01(c_.feedback);
    mix_ = modulationdetail::clamp01(c_.mix);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  PhaserControls c_;
  float st_[2][kStages] = {};
  float phase_ = 0.0f;
  float rateHz_ = 0.3f;
  float depth_ = 0.6f;
  float feedback_ = 0.4f;
  float mix_ = 0.5f;
};

// Bias/optical tremolo with a sine-to-square shape control.
struct TremoloControls {
  float rate = 0.40f;
  float depth = 0.60f;
  float shape = 0.20f;
  float mix = 1.00f;
};

class Tremolo {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const TremoloControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    phase_ += 2.0f * kPi * rateHz_ / sr_;
    if (phase_ >= 2.0f * kPi) phase_ -= 2.0f * kPi;

    const float sine = std::sin(phase_);
    const float square = sine >= 0.0f ? 1.0f : -1.0f;
    const float wave = sine * (1.0f - shape_) + square * shape_;
    const float gain = 1.0f - depth_ * 0.5f * (1.0f + wave);
    l += (l * gain - l) * mix_;
    r += (r * gain - r) * mix_;
  }
  void reset() noexcept { phase_ = 0.0f; }
  int latencySamples() const noexcept { return 0; }

private:
  static constexpr float kPi = 3.14159265358979323846f;
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    rateHz_ = 0.05f * std::pow(200.0f, modulationdetail::clamp01(c_.rate));
    depth_ = modulationdetail::clamp01(c_.depth);
    shape_ = modulationdetail::clamp01(c_.shape);
    mix_ = modulationdetail::clamp01(c_.mix);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  TremoloControls c_;
  float phase_ = 0.0f;
  float rateHz_ = 0.4f;
  float depth_ = 0.6f;
  float shape_ = 0.2f;
  float mix_ = 1.0f;
};


// Bass chorus: modulation stays above the low band so the fundamental keeps
// its weight.
struct BassChorusControls {
  float rate = 0.30f;
  float depth = 0.50f;
  float lowCut = 0.30f;
  float mix = 0.45f;
};

class BassChorus {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    const int maxSamples = static_cast<int>(sr_ * 0.035f) + 4;
    for (auto& d : delay_) d.prepare(maxSamples);
    reset();
    dirty_ = true;
  }
  void setControls(const BassChorusControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    phase_ += 2.0f * kPi * rateHz_ / sr_;
    if (phase_ >= 2.0f * kPi) phase_ -= 2.0f * kPi;

    const float modL = 0.5f + 0.5f * std::sin(phase_);
    const float modR = 0.5f + 0.5f * std::sin(phase_ + kPi * 0.5f);
    const float baseSamples = 0.008f * sr_;
    const float depthSamples = depthMs_ * 0.001f * sr_;
    const float wetL = delay_[0].read(baseSamples + depthSamples * modL);
    const float wetR = delay_[1].read(baseSamples + depthSamples * modR);

    delay_[0].push(l);
    delay_[1].push(r);

    // Split the wet signal: keep modulated highs, pass the clean lows.
    const float modHiL = lp_[0].process(wetL);
    const float modHiR = lp_[1].process(wetR);
    const float cleanLoL = lp_[2].process(l);
    const float cleanLoR = lp_[3].process(r);
    const float chorusL = modHiL + cleanLoL;
    const float chorusR = modHiR + cleanLoR;

    l += (chorusL - l) * mix_;
    r += (chorusR - r) * mix_;
  }
  void reset() noexcept {
    for (auto& d : delay_) d.reset();
    for (auto& f : lp_) f.reset();
    phase_ = 0.0f;
  }
  int latencySamples() const noexcept { return 0; }

private:
  static constexpr float kPi = 3.14159265358979323846f;
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    rateHz_ = modulationdetail::lfoRate(c_.rate);
    depthMs_ = 0.5f + 6.0f * modulationdetail::clamp01(c_.depth);
    mix_ = modulationdetail::clamp01(c_.mix);
    const float hz = 60.0f * std::pow(12.0f, modulationdetail::clamp01(c_.lowCut));
    lp_[0].highPass(hz, 0.7f, sr_);
    lp_[1].highPass(hz, 0.7f, sr_);
    lp_[2].lowPass(hz, 0.7f, sr_);
    lp_[3].lowPass(hz, 0.7f, sr_);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  BassChorusControls c_;
  modulationdetail::ModDelay delay_[2];
  hdsp::Biquad lp_[4];
  float phase_ = 0.0f;
  float rateHz_ = 0.3f;
  float depthMs_ = 3.0f;
  float mix_ = 0.45f;
};

// Pitch vibrato from a fully modulated delay.
struct VibratoControls {
  float rate = 0.45f;
  float depth = 0.50f;
  float mix = 1.00f;
};

class Vibrato {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    const int maxSamples = static_cast<int>(sr_ * 0.015f) + 4;
    for (auto& d : delay_) d.prepare(maxSamples);
    reset();
    dirty_ = true;
  }
  void setControls(const VibratoControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    phase_ += 2.0f * kPi * rateHz_ / sr_;
    if (phase_ >= 2.0f * kPi) phase_ -= 2.0f * kPi;

    const float mod = 0.5f + 0.5f * std::sin(phase_);
    const float delaySamples =
        (0.004f + depthMs_ * 0.001f * mod) * sr_;
    const float wetL = delay_[0].read(delaySamples);
    const float wetR = delay_[1].read(delaySamples);

    delay_[0].push(l);
    delay_[1].push(r);

    l += (wetL - l) * mix_;
    r += (wetR - r) * mix_;
  }
  void reset() noexcept {
    for (auto& d : delay_) d.reset();
    phase_ = 0.0f;
  }
  int latencySamples() const noexcept { return 0; }

private:
  static constexpr float kPi = 3.14159265358979323846f;
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    rateHz_ = modulationdetail::lfoRate(c_.rate);
    depthMs_ = 0.15f + 3.5f * modulationdetail::clamp01(c_.depth);
    mix_ = modulationdetail::clamp01(c_.mix);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  VibratoControls c_;
  modulationdetail::ModDelay delay_[2];
  float phase_ = 0.0f;
  float rateHz_ = 0.45f;
  float depthMs_ = 1.9f;
  float mix_ = 1.0f;
};

} // namespace stomp::modules
