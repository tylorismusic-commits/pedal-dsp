// pedal-dsp — compressor, gate and envelope-filter pedals
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/db.hpp>

namespace stomp::modules {

namespace dyndetail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
inline float coeffForMs(float ms, float sr) noexcept {
  return 1.0f - std::exp(-1.0f / (std::max(ms, 0.01f) * 0.001f * sr));
}
} // namespace dyndetail

// 1176-style FET compressor pedal.
struct FetCompControls {
  float input = 0.35f;
  float ratio = 0.45f;
  float attack = 0.15f;
  float release = 0.30f;
  float output = 0.50f;
};

class FetComp {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const FetCompControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    const float detector = std::max(std::fabs(l), std::fabs(r)) * drive_;
    const float envCoeff = detector > env_ ? attackCoeff_ : releaseCoeff_;
    env_ += (detector - env_) * envCoeff;

    const float overDb = hdsp::gainToDb(env_) - kThresholdDb;
    float gainDb = 0.0f;
    if (overDb > 0.0f) {
      gainDb = -overDb * (1.0f - 1.0f / ratio_);
    }
    const float gain = hdsp::dbToGain(gainDb + outputDb_);
    l *= gain;
    r *= gain;
    if (!std::isfinite(env_)) env_ = 0.0f;
  }
  void reset() noexcept { env_ = 0.0f; }
  int latencySamples() const noexcept { return 0; }

private:
  static constexpr float kThresholdDb = -18.0f;
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    drive_ = hdsp::dbToGain(-12.0f + 36.0f * dyndetail::clamp01(c_.input));
    ratio_ = 2.0f + 18.0f * dyndetail::clamp01(c_.ratio);
    const float attackMs = 0.1f + 9.9f * dyndetail::clamp01(c_.attack);
    const float releaseMs =
        100.0f * std::pow(12.0f, dyndetail::clamp01(c_.release));
    attackCoeff_ = dyndetail::coeffForMs(attackMs, sr_);
    releaseCoeff_ = dyndetail::coeffForMs(releaseMs, sr_);
    outputDb_ = -12.0f + 24.0f * dyndetail::clamp01(c_.output);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  FetCompControls c_;
  float env_ = 0.0f;
  float drive_ = 1.0f;
  float ratio_ = 8.0f;
  float attackCoeff_ = 0.1f;
  float releaseCoeff_ = 0.01f;
  float outputDb_ = 0.0f;
};

// Optical compressor pedal with a fixed gentle ratio.
struct OptoCompControls {
  float peak = 0.50f;
  float gain = 0.50f;
  float mix = 1.00f;
};

class OptoComp {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    attackCoeff_ = dyndetail::coeffForMs(8.0f, sr_);
    releaseCoeff_ = dyndetail::coeffForMs(220.0f, sr_);
    reset();
  }
  void setControls(const OptoCompControls& c) noexcept {
    c_ = c;
    gainDb_ = -12.0f + 24.0f * dyndetail::clamp01(c.gain);
    mix_ = dyndetail::clamp01(c.mix);
    drive_ = hdsp::dbToGain(-6.0f + 30.0f * dyndetail::clamp01(c.peak));
    driveComp_ = 1.0f / std::sqrt(drive_);
  }
  inline void process(float& l, float& r) noexcept {
    const float detector = std::max(std::fabs(l), std::fabs(r)) * drive_;
    const float envCoeff = detector > env_ ? attackCoeff_ : releaseCoeff_;
    env_ += (detector - env_) * envCoeff;

    const float overDb = hdsp::gainToDb(env_) - kThresholdDb;
    float gainDb = 0.0f;
    if (overDb > 0.0f) {
      gainDb = -overDb * (1.0f - 1.0f / kRatio);
    }
    const float gain = hdsp::dbToGain(gainDb + gainDb_) * driveComp_;
    const float wetL = l * gain;
    const float wetR = r * gain;
    l += (wetL - l) * mix_;
    r += (wetR - r) * mix_;
  }
  void reset() noexcept { env_ = 0.0f; }
  int latencySamples() const noexcept { return 0; }

private:
  static constexpr float kThresholdDb = -24.0f;
  static constexpr float kRatio = 3.0f;
  float sr_ = 48000.0f;
  OptoCompControls c_;
  float env_ = 0.0f;
  float drive_ = 1.0f;
  float driveComp_ = 0.707f;
  float gainDb_ = 0.0f;
  float mix_ = 1.0f;
  float attackCoeff_ = 0.1f;
  float releaseCoeff_ = 0.01f;
};

// Envelope-controlled band-pass auto-wah.
struct EnvFilterControls {
  float sens = 0.55f;
  float q = 0.40f;
  float range = 0.55f;
  float blend = 0.60f;
};

class EnvFilter {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    attackCoeff_ = dyndetail::coeffForMs(2.0f, sr_);
    releaseCoeff_ = dyndetail::coeffForMs(90.0f, sr_);
    reset();
    dirty_ = true;
  }
  void setControls(const EnvFilterControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    const float detector = 0.5f * (std::fabs(l) + std::fabs(r)) * sensGain_;
    const float envCoeff = detector > env_ ? attackCoeff_ : releaseCoeff_;
    env_ += (detector - env_) * envCoeff;

    const float octaves = rangeOct_ * dyndetail::clamp01(env_);
    const float hz = std::min(std::max(baseHz_ * std::exp2(octaves), 40.0f),
                              sr_ * 0.45f);
    const float g = std::tan(kPi * hz / sr_);
    const float k = 1.0f / q_;

    const float bpL = svf_[0].bandpass(l, g, k);
    const float bpR = svf_[1].bandpass(r, g, k);
    l += (bpL * 2.0f - l) * blend_;
    r += (bpR * 2.0f - r) * blend_;
  }
  void reset() noexcept {
    env_ = 0.0f;
    svf_[0].reset();
    svf_[1].reset();
  }
  int latencySamples() const noexcept { return 0; }

private:
  static constexpr float kPi = 3.14159265358979323846f;
  struct Svf {
    float ic1 = 0.0f, ic2 = 0.0f;
    void reset() noexcept { ic1 = ic2 = 0.0f; }
    inline float bandpass(float x, float g, float k) noexcept {
      const float a1 = 1.0f / (1.0f + g * (g + k));
      const float a2 = g * a1;
      const float a3 = g * a2;
      const float v3 = x - ic2;
      const float v1 = a1 * ic1 + a2 * v3;
      const float v2 = ic2 + a2 * ic1 + a3 * v3;
      ic1 = 2.0f * v1 - ic1;
      ic2 = 2.0f * v2 - ic2;
      return v1;
    }
  };
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    sensGain_ = 2.0f + 28.0f * dyndetail::clamp01(c_.sens);
    q_ = 0.3f * std::pow(20.0f, dyndetail::clamp01(c_.q));
    rangeOct_ = 0.5f + 4.5f * dyndetail::clamp01(c_.range);
    blend_ = dyndetail::clamp01(c_.blend);
    baseHz_ = 250.0f;
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  EnvFilterControls c_;
  Svf svf_[2];
  float env_ = 0.0f;
  float sensGain_ = 10.0f;
  float q_ = 5.0f;
  float rangeOct_ = 3.0f;
  float blend_ = 0.6f;
  float baseHz_ = 250.0f;
  float attackCoeff_ = 0.1f;
  float releaseCoeff_ = 0.01f;
};

// Stereo gate shared by the Noise Gate and Tight Gate pedals.
struct GatePedalControls {
  float threshold = 0.50f;
  float decay = 0.35f;
  float range = 0.60f;
  float link = 1.00f;
};

class GatePedal {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
  }
  void setControls(const GatePedalControls& c, bool tight) noexcept {
    thresholdGain_ =
        hdsp::dbToGain(-60.0f + 60.0f * dyndetail::clamp01(c.threshold));
    resetGain_ =
        hdsp::dbToGain(-80.0f * dyndetail::clamp01(c.range));
    const float decayMs = tight
                              ? 1.0f + 199.0f * dyndetail::clamp01(c.decay)
                              : 10.0f + 990.0f * dyndetail::clamp01(c.decay);
    openCoeff_ = dyndetail::coeffForMs(0.05f, sr_);
    closeCoeff_ = dyndetail::coeffForMs(decayMs, sr_);
    link_ = dyndetail::clamp01(c.link);
  }
  inline void process(float& l, float& r) noexcept {
    const float detL = std::fabs(l);
    const float detR = std::fabs(r);
    const float shared = std::max(detL, detR);
    const float useL = shared * link_ + detL * (1.0f - link_);
    const float useR = shared * link_ + detR * (1.0f - link_);
    const float targetL = useL > thresholdGain_ ? 1.0f : resetGain_;
    const float targetR = useR > thresholdGain_ ? 1.0f : resetGain_;
    gain_[0] += (targetL - gain_[0]) *
                (targetL > gain_[0] ? openCoeff_ : closeCoeff_);
    gain_[1] += (targetR - gain_[1]) *
                (targetR > gain_[1] ? openCoeff_ : closeCoeff_);
    l *= gain_[0];
    r *= gain_[1];
  }
  void reset() noexcept {
    gain_[0] = gain_[1] = 1.0f;
  }
  int latencySamples() const noexcept { return 0; }

private:
  float sr_ = 48000.0f;
  float thresholdGain_ = 0.01f;
  float resetGain_ = 0.001f;
  float openCoeff_ = 0.5f;
  float closeCoeff_ = 0.001f;
  float link_ = 1.0f;
  float gain_[2] = {1.0f, 1.0f};
};

class NoiseGate {
public:
  void prepare(float sr) noexcept { core_.prepare(sr); }
  void setControls(const GatePedalControls& c) noexcept {
    core_.setControls(c, false);
  }
  inline void process(float& l, float& r) noexcept { core_.process(l, r); }
  void reset() noexcept { core_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  GatePedal core_;
};

class TightGate {
public:
  void prepare(float sr) noexcept { core_.prepare(sr); }
  void setControls(const GatePedalControls& c) noexcept {
    core_.setControls(c, true);
  }
  inline void process(float& l, float& r) noexcept { core_.process(l, r); }
  void reset() noexcept { core_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  GatePedal core_;
};

} // namespace stomp::modules
