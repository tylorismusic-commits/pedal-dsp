// pedal-dsp — delay and reverb pedals
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>

#include <hdsp/modules/space/delay.hpp>
#include <hdsp/modules/space/reverb.hpp>

namespace stomp::modules {

namespace delaydetail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
} // namespace delaydetail

// Bucket-brigade-style analog delay.
struct AnalogDelayControls {
  float time = 0.40f;      // 20..1000 ms
  float feedback = 0.40f;  // %
  float tone = 0.50f;      // %
  float mix = 0.30f;       // %
};

class AnalogDelay {
public:
  void prepare(float sr) noexcept {
    delay_.prepare(sr, 1100.0f);
    dirty_ = true;
  }
  void setControls(const AnalogDelayControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    delay_.process(l, r);
  }
  void reset() noexcept { delay_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    hdsp::space::Controls c;
    c.delayOn = true;
    c.delayMode = 2;  // Analog.
    c.delaySync = false;
    c.delayTimeMs = 20.0f + 980.0f * delaydetail::clamp01(c_.time);
    c.delayFeedback = 100.0f * delaydetail::clamp01(c_.feedback);
    c.delayTone = 100.0f * delaydetail::clamp01(c_.tone);
    c.delayMix = 100.0f * delaydetail::clamp01(c_.mix);
    c.delayWidth = 50.0f;
    c.delayDuck = 0.0f;
    c.delayMod = 15.0f;
    delay_.setControls(c, 120.0f);
  }

  hdsp::space::Delay delay_;
  AnalogDelayControls c_;
  bool dirty_ = true;
};

// Spring reverb.
struct SpringReverbControls {
  float decay = 0.40f;  // 0.3..12 s
  float tone = 0.50f;   // %
  float mix = 0.30f;    // %
};

class SpringReverb {
public:
  void prepare(float sr) noexcept {
    reverb_.prepare(sr);
    dirty_ = true;
  }
  void setControls(const SpringReverbControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    reverb_.process(l, r);
  }
  void reset() noexcept { reverb_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    hdsp::space::Controls c;
    c.reverbOn = true;
    c.reverbMode = 3;  // Spring.
    c.reverbDecay = 0.3f + 11.7f * delaydetail::clamp01(c_.decay);
    c.reverbTone = 100.0f * delaydetail::clamp01(c_.tone);
    c.reverbMix = 100.0f * delaydetail::clamp01(c_.mix);
    c.reverbWidth = 70.0f;
    c.reverbPreDelay = 0.0f;
    reverb_.setControls(c);
  }

  hdsp::space::Reverb reverb_;
  SpringReverbControls c_;
  bool dirty_ = true;
};

} // namespace stomp::modules
