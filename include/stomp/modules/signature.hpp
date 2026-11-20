// pedal-dsp — signature performance pedals
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Precision Drive (tight high-gain boost with an input gate), Wah and Laser.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <hdsp/biquad.hpp>
#include <hdsp/db.hpp>

#include "clippers.hpp"
#include "pitch.hpp"

namespace stomp::modules {

namespace sigdetail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
inline float coeffForMs(float ms, float sr) noexcept {
  return 1.0f - std::exp(-1.0f / (std::max(ms, 0.01f) * 0.001f * sr));
}
} // namespace sigdetail

// Horizon Devices-style Precision Drive: a tight overdrive with a built-in
// input gate. Attack raises the pre-clip high-pass, tightening the low end.
struct PrecisionDriveControls {
  float gate = 0.35f;
  float drive = 0.45f;
  float attack = 0.70f;
  float level = 0.50f;
};

class PrecisionDrive {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    gateAttack_ = sigdetail::coeffForMs(1.0f, sr_);
    gateRelease_ = sigdetail::coeffForMs(60.0f, sr_);
    gainAttack_ = sigdetail::coeffForMs(0.2f, sr_);
    gainRelease_ = sigdetail::coeffForMs(40.0f, sr_);
    reset();
    dirty_ = true;
  }
  void setControls(const PrecisionDriveControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    float gateGain = 1.0f;
    if (gateOn_) {
      const float detector = std::max(std::fabs(l), std::fabs(r));
      gateEnv_ += (detector - gateEnv_) *
                  (detector > gateEnv_ ? gateAttack_ : gateRelease_);
      const float target = gateEnv_ > gateThreshold_ ? 1.0f : gateFloor_;
      gateGain_ += (target - gateGain_) *
                   (target > gateGain_ ? gainAttack_ : gainRelease_);
      gateGain = gateGain_;
    } else {
      gateGain_ = 1.0f;
    }
    grDb_ = -hdsp::gainToDb(std::max(gateGain, 1.0e-5f));

    float inL = hpL_.process(l) * gateGain;
    float inR = hpR_.process(r) * gateGain;
    inL = midL_.process(inL);
    inR = midR_.process(inR);

    const float drive = driveGain_;
    inL = dcL_.process(clip_[0].process(inL * drive)) * comp_;
    inR = dcR_.process(clip_[1].process(inR * drive)) * comp_;

    l = outLpL_.process(inL) * level_;
    r = outLpR_.process(inR) * level_;
    if (!std::isfinite(l)) l = 0.0f;
    if (!std::isfinite(r)) r = 0.0f;
  }
  void reset() noexcept {
    gateEnv_ = 0.0f;
    gateGain_ = 1.0f;
    grDb_ = 0.0f;
    hpL_.reset(); hpR_.reset();
    midL_.reset(); midR_.reset();
    dcL_.reset(); dcR_.reset();
    outLpL_.reset(); outLpR_.reset();
    clip_[0].reset(); clip_[1].reset();
  }
  int latencySamples() const noexcept { return 0; }
  float gainReductionDb() const noexcept { return grDb_; }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    gateOn_ = c_.gate > 0.01f;
    gateThreshold_ = hdsp::dbToGain(-60.0f + 30.0f * sigdetail::clamp01(c_.gate));
    gateFloor_ = hdsp::dbToGain(-80.0f);
    const float hpHz =
        55.0f * std::pow(4.7f, sigdetail::clamp01(c_.attack));
    hpL_.highPass(hpHz, 0.75f, sr_);
    hpR_.highPass(hpHz, 0.75f, sr_);
    midL_.peak(720.0f, 4.5f, 0.9f, sr_);
    midR_.peak(720.0f, 4.5f, 0.9f, sr_);
    driveGain_ = hdsp::dbToGain(2.0f + 30.0f * sigdetail::clamp01(c_.drive));
    comp_ = 1.0f / std::sqrt(driveGain_);
    level_ = hdsp::dbToGain(-12.0f + 24.0f * sigdetail::clamp01(c_.level));
    outLpL_.lowPass(9000.0f, 0.7f, sr_);
    outLpR_.lowPass(9000.0f, 0.7f, sr_);
    dcL_.highPass(18.0f, 0.7f, sr_);
    dcR_.highPass(18.0f, 0.7f, sr_);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  PrecisionDriveControls c_;
  hdsp::Biquad hpL_, hpR_, midL_, midR_, dcL_, dcR_, outLpL_, outLpR_;
  AdaaTanh clip_[2];
  float gateEnv_ = 0.0f;
  float gateGain_ = 1.0f;
  float grDb_ = 0.0f;
  float gateThreshold_ = 0.001f;
  float gateFloor_ = 0.0001f;
  float gateAttack_ = 0.1f;
  float gateRelease_ = 0.001f;
  float gainAttack_ = 0.1f;
  float gainRelease_ = 0.001f;
  float driveGain_ = 1.0f;
  float comp_ = 1.0f;
  float level_ = 1.0f;
  bool gateOn_ = true;
};

// Classic band-pass wah with an automatable pedal position.
struct WahControls {
  float pedal = 0.35f;
  float q = 0.55f;
  float range = 0.60f;
  float mix = 1.00f;
};

class Wah {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const WahControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    const float octaves = 1.8f + 1.2f * range_;
    const float hz = std::min(
        std::max(300.0f * std::exp2(pedal_ * octaves), 80.0f), sr_ * 0.45f);
    const float g = std::tan(kPi * hz / sr_);
    const float k = 1.0f / q_;
    const float wetL = svf_[0].bandpass(l, g, k) * 2.0f;
    const float wetR = svf_[1].bandpass(r, g, k) * 2.0f;
    const float mix = mix_;
    l += (wetL - l) * mix;
    r += (wetR - r) * mix;
  }
  void reset() noexcept {
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
    pedal_ = sigdetail::clamp01(c_.pedal);
    q_ = 1.5f + 10.5f * sigdetail::clamp01(c_.q);
    range_ = sigdetail::clamp01(c_.range);
    mix_ = sigdetail::clamp01(c_.mix);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  WahControls c_;
  Svf svf_[2];
  float pedal_ = 0.35f;
  float q_ = 6.0f;
  float range_ = 0.6f;
  float mix_ = 1.0f;
};

// Laser: expression-swept pitch shifter with ring modulation and an envelope
// follower that can move the pedal by itself.
struct LaserControls {
  float pitch = 0.50f;  // PitchSemis: -24..+24
  float ring = 0.35f;
  float sens = 0.45f;
  float pedal = 0.50f;
  float mix = 1.00f;
};

class Laser {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    engine_.prepare(sr_, false);
    engine_.setTone(1.0f);
    engine_.clearLowCut();
    envAttack_ = sigdetail::coeffForMs(5.0f, sr_);
    envRelease_ = sigdetail::coeffForMs(120.0f, sr_);
    reset();
    dirty_ = true;
  }
  void setControls(const LaserControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    process(l, r, -1.0f);
  }
  inline void process(float& l, float& r, float mixOverride) noexcept {
    update();
    const float detector = std::max(std::fabs(l), std::fabs(r));
    env_ += (detector - env_) *
            (detector > env_ ? envAttack_ : envRelease_);

    if (--ratioCount_ <= 0) {
      ratioCount_ = 32;
      const float autoSweep = env_ * sens_ * 24.0f;
      const float manual = (pedal_ - 0.5f) * 24.0f;
      const float semis = pitchSemis_ + manual + autoSweep;
      engine_.setRatio(std::pow(2.0f, semis / 12.0f), 0.03f);
      ringPhase_ += 0.0f;
    }
    ringPhase_ += 2.0f * kPi * ringHz_ / sr_;
    if (ringPhase_ >= 2.0f * kPi) ringPhase_ -= 2.0f * kPi;
    const float ringSig = std::sin(ringPhase_);

    const float carrier = 1.0f - ring_ + ring_ * ringSig;
    const float mix = mixOverride >= 0.0f ? mixOverride : mix_;
    engine_.process(l, r, mix, carrier);
  }
  void reset() noexcept {
    engine_.reset();
    env_ = 0.0f;
    ringPhase_ = 0.0f;
    ratioCount_ = 0;
  }
  int latencySamples() const noexcept { return engine_.latencySamples(); }

private:
  static constexpr float kPi = 3.14159265358979323846f;
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    const float v = sigdetail::clamp01(c_.pitch);
    pitchSemis_ = std::round(-24.0f + 48.0f * v);
    ring_ = sigdetail::clamp01(c_.ring);
    sens_ = sigdetail::clamp01(c_.sens);
    pedal_ = sigdetail::clamp01(c_.pedal);
    mix_ = sigdetail::clamp01(c_.mix);
    const float hz = 80.0f * std::exp2(pedal_ * 4.0f + pitchSemis_ * 0.05f);
    ringHz_ = std::min(std::max(hz, 20.0f), sr_ * 0.45f);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  LaserControls c_;
  PitchEngine engine_;
  float env_ = 0.0f;
  float envAttack_ = 0.1f;
  float envRelease_ = 0.001f;
  float ring_ = 0.35f;
  float sens_ = 0.45f;
  float pedal_ = 0.5f;
  float mix_ = 1.0f;
  float pitchSemis_ = 0.0f;
  float ringHz_ = 100.0f;
  float ringPhase_ = 0.0f;
  int ratioCount_ = 0;
};

} // namespace stomp::modules
