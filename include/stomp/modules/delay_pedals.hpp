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

class DelayCore {
public:
  void prepare(float sr, float maxMs) noexcept {
    delay_.prepare(sr, maxMs);
  }
  void configure(int mode, float time, float feedback, float tone, float mix,
                 float mod) noexcept {
    hdsp::space::Controls c;
    c.delayOn = true;
    c.delayMode = mode;
    c.delaySync = false;
    c.delayTimeMs = 20.0f + 980.0f * clamp01(time);
    c.delayFeedback = 100.0f * clamp01(feedback);
    c.delayTone = 100.0f * clamp01(tone);
    c.delayMix = 100.0f * clamp01(mix);
    c.delayMod = 100.0f * clamp01(mod);
    c.delayWidth = 50.0f;
    c.delayDuck = 0.0f;
    delay_.setControls(c, 120.0f);
  }
  inline void process(float& l, float& r) noexcept { delay_.process(l, r); }
  void reset() noexcept { delay_.reset(); }

private:
  hdsp::space::Delay delay_;
};

class ReverbCore {
public:
  void prepare(float sr) noexcept { reverb_.prepare(sr); }
  void configure(int mode, float decay, float tone, float mix) noexcept {
    hdsp::space::Controls c;
    c.reverbOn = true;
    c.reverbMode = mode;
    c.reverbDecay = 0.3f + 11.7f * clamp01(decay);
    c.reverbTone = 100.0f * clamp01(tone);
    c.reverbMix = 100.0f * clamp01(mix);
    c.reverbWidth = 70.0f;
    c.reverbPreDelay = 0.0f;
    reverb_.setControls(c);
  }
  inline void process(float& l, float& r) noexcept { reverb_.process(l, r); }
  void reset() noexcept { reverb_.reset(); }

private:
  hdsp::space::Reverb reverb_;
};
} // namespace delaydetail

// Bucket-brigade-style analog delay.
struct AnalogDelayControls {
  float time = 0.40f;
  float feedback = 0.40f;
  float tone = 0.50f;
  float mix = 0.30f;
};

class AnalogDelay {
public:
  void prepare(float sr) noexcept {
    core_.prepare(sr, 1100.0f);
    dirty_ = true;
  }
  void setControls(const AnalogDelayControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    core_.process(l, r);
  }
  void reset() noexcept { core_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    core_.configure(2, c_.time, c_.feedback, c_.tone, c_.mix, 0.15f);
  }
  delaydetail::DelayCore core_;
  AnalogDelayControls c_;
  bool dirty_ = true;
};

// Clean digital delay.
struct DigitalDelayControls {
  float time = 0.40f;
  float feedback = 0.40f;
  float tone = 0.60f;
  float mix = 0.30f;
};

class DigitalDelay {
public:
  void prepare(float sr) noexcept {
    core_.prepare(sr, 1100.0f);
    dirty_ = true;
  }
  void setControls(const DigitalDelayControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    core_.process(l, r);
  }
  void reset() noexcept { core_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    core_.configure(0, c_.time, c_.feedback, c_.tone, c_.mix, 0.0f);
  }
  delaydetail::DelayCore core_;
  DigitalDelayControls c_;
  bool dirty_ = true;
};

// Ping-pong delay.
struct PingPongControls {
  float time = 0.35f;
  float feedback = 0.40f;
  float tone = 0.60f;
  float mix = 0.30f;
};

class PingPongDelay {
public:
  void prepare(float sr) noexcept {
    core_.prepare(sr, 1100.0f);
    dirty_ = true;
  }
  void setControls(const PingPongControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    core_.process(l, r);
  }
  void reset() noexcept { core_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    core_.configure(3, c_.time, c_.feedback, c_.tone, c_.mix, 0.0f);
  }
  delaydetail::DelayCore core_;
  PingPongControls c_;
  bool dirty_ = true;
};

// Tape echo with wow/flutter modulation.
struct TapeEchoControls {
  float time = 0.38f;
  float feedback = 0.42f;
  float wow = 0.20f;
  float tone = 0.40f;
  float mix = 0.30f;
};

class TapeEcho {
public:
  void prepare(float sr) noexcept {
    core_.prepare(sr, 1100.0f);
    dirty_ = true;
  }
  void setControls(const TapeEchoControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    core_.process(l, r);
  }
  void reset() noexcept { core_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    core_.configure(1, c_.time, c_.feedback, c_.tone, c_.mix, c_.wow);
  }
  delaydetail::DelayCore core_;
  TapeEchoControls c_;
  bool dirty_ = true;
};

// Spring reverb.
struct SpringReverbControls {
  float decay = 0.40f;
  float tone = 0.50f;
  float mix = 0.30f;
};

class SpringReverb {
public:
  void prepare(float sr) noexcept {
    core_.prepare(sr);
    dirty_ = true;
  }
  void setControls(const SpringReverbControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    core_.process(l, r);
  }
  void reset() noexcept { core_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    core_.configure(3, c_.decay, c_.tone, c_.mix);
  }
  delaydetail::ReverbCore core_;
  SpringReverbControls c_;
  bool dirty_ = true;
};

// Plate reverb.
struct PlateReverbControls {
  float decay = 0.40f;
  float tone = 0.55f;
  float mix = 0.30f;
};

class PlateReverb {
public:
  void prepare(float sr) noexcept {
    core_.prepare(sr);
    dirty_ = true;
  }
  void setControls(const PlateReverbControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    core_.process(l, r);
  }
  void reset() noexcept { core_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    core_.configure(0, c_.decay, c_.tone, c_.mix);
  }
  delaydetail::ReverbCore core_;
  PlateReverbControls c_;
  bool dirty_ = true;
};

// Room reverb.
struct RoomReverbControls {
  float decay = 0.30f;
  float tone = 0.50f;
  float mix = 0.30f;
};

class RoomReverb {
public:
  void prepare(float sr) noexcept {
    core_.prepare(sr);
    dirty_ = true;
  }
  void setControls(const RoomReverbControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    core_.process(l, r);
  }
  void reset() noexcept { core_.reset(); }
  int latencySamples() const noexcept { return 0; }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    core_.configure(1, c_.decay, c_.tone, c_.mix);
  }
  delaydetail::ReverbCore core_;
  RoomReverbControls c_;
  bool dirty_ = true;
};

} // namespace stomp::modules
