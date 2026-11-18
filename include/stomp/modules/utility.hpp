// pedal-dsp — EQ, lo-fi and tape-saturation pedals
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>
#include <hdsp/db.hpp>
#include <hdsp/modules/crush.hpp>

#include "clippers.hpp"

namespace stomp::modules {

namespace utildetail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
} // namespace utildetail

// Three-band tone stack used by the guitar and bass EQ pedals.
struct Eq3Controls {
  float low = 0.50f;
  float mid = 0.50f;
  float high = 0.50f;
  float level = 0.50f;
};

class Eq3 {
public:
  void prepare(float sr, float lowHz, float midHz, float highHz) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    lowHz_ = lowHz;
    midHz_ = midHz;
    highHz_ = highHz;
    reset();
    dirty_ = true;
  }
  void setControls(const Eq3Controls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    l = high_.process(mid_.process(low_.process(l))) * level_;
    r = highR_.process(midR_.process(lowR_.process(r))) * level_;
  }
  void reset() noexcept {
    low_.reset();
    mid_.reset();
    high_.reset();
    lowR_.reset();
    midR_.reset();
    highR_.reset();
  }
  int latencySamples() const noexcept { return 0; }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    const float lowDb = (utildetail::clamp01(c_.low) - 0.5f) * 30.0f;
    const float midDb = (utildetail::clamp01(c_.mid) - 0.5f) * 24.0f;
    const float highDb = (utildetail::clamp01(c_.high) - 0.5f) * 30.0f;
    low_.lowShelf(lowHz_, lowDb, 0.7f, sr_);
    mid_.peak(midHz_, midDb, 0.8f, sr_);
    high_.highShelf(highHz_, highDb, 0.7f, sr_);
    lowR_.lowShelf(lowHz_, lowDb, 0.7f, sr_);
    midR_.peak(midHz_, midDb, 0.8f, sr_);
    highR_.highShelf(highHz_, highDb, 0.7f, sr_);
    level_ = hdsp::dbToGain(-12.0f + 24.0f * utildetail::clamp01(c_.level));
  }

  float sr_ = 48000.0f;
  float lowHz_ = 120.0f;
  float midHz_ = 800.0f;
  float highHz_ = 4000.0f;
  bool dirty_ = true;
  Eq3Controls c_;
  hdsp::Biquad low_, mid_, high_, lowR_, midR_, highR_;
  float level_ = 1.0f;
};

class BassEq {
public:
  void prepare(float sr) noexcept { eq_.prepare(sr, 80.0f, 700.0f, 3000.0f); }
  void setControls(const Eq3Controls& c) noexcept { eq_.setControls(c); }
  inline void process(float& l, float& r) noexcept { eq_.process(l, r); }
  void reset() noexcept { eq_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  Eq3 eq_;
};

// Sample/bit-reduction pedal.
struct LoFiControls {
  float bits = 0.65f;
  float rate = 0.70f;
  float tone = 0.60f;
  float mix = 1.00f;
};

class LoFi {
public:
  void prepare(float sr) noexcept { crush_.prepare(sr); }
  void setControls(const LoFiControls& c) noexcept {
    hdsp::modules::CrushControls cc;
    cc.bits = c.bits;
    cc.rate = c.rate;
    cc.tone = c.tone;
    cc.mix = c.mix;
    crush_.setControls(cc);
  }
  inline void process(float& l, float& r) noexcept { crush_.process(l, r); }
  void reset() noexcept { crush_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  hdsp::modules::Crush crush_;
};

// Pedal tape saturation with a post low-pass.
struct TapeSatControls {
  float drive = 0.50f;
  float tone = 0.50f;
  float mix = 0.70f;
};

class TapeSat {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const TapeSatControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    const float wetL = tone_.process(clip_[0].process(l * drive_)) * comp_;
    const float wetR = toneR_.process(clip_[1].process(r * drive_)) * comp_;
    l += (wetL - l) * mix_;
    r += (wetR - r) * mix_;
  }
  void reset() noexcept {
    tone_.reset();
    toneR_.reset();
    clip_[0].reset();
    clip_[1].reset();
  }
  int latencySamples() const noexcept { return 0; }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    drive_ = hdsp::dbToGain(3.0f + 21.0f * utildetail::clamp01(c_.drive));
    comp_ = 1.0f / std::sqrt(drive_);
    const float hz = 1000.0f * std::pow(8.0f, utildetail::clamp01(c_.tone));
    tone_.lowPass(hz, 0.72f, sr_);
    toneR_.lowPass(hz, 0.72f, sr_);
    mix_ = utildetail::clamp01(c_.mix);
  }

  float sr_ = 48000.0f;
  bool dirty_ = true;
  TapeSatControls c_;
  AdaaTanh clip_[2];
  hdsp::Biquad tone_, toneR_;
  float drive_ = 1.0f;
  float comp_ = 1.0f;
  float mix_ = 0.7f;
};

} // namespace stomp::modules
