// pedal-dsp — drive, distortion and fuzz pedals
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>

#include <hdsp/biquad.hpp>

#include "clippers.hpp"

namespace stomp::modules {

namespace detail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
inline float dbToGain(float db) noexcept {
  return std::pow(10.0f, db * 0.05f);
}
inline float onePoleGain(float hz, float sr) noexcept {
  const float t = std::tan(3.14159265358979323846f *
                           std::min(std::max(hz, 10.0f), sr * 0.49f) / sr);
  return t / (1.0f + t);
}
struct OnePoleLP {
  float state = 0.0f;
  void reset() noexcept { state = 0.0f; }
  inline float process(float x, float g) noexcept {
    const float v = (x - state) * g;
    const float y = v + state;
    state = y + v;
    if (std::fabs(state) < 1.0e-20f) state = 0.0f;
    return y;
  }
};
} // namespace detail

// Rangemaster-style treble booster.
struct TrebleBoostControls {
  float boost = 0.5f;
  float range = 0.5f;
  float level = 0.5f;
};
class TrebleBoost {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const TrebleBoostControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    l = processCh(ch_[0], l);
    r = processCh(ch_[1], r);
  }
  void reset() noexcept {
    for (auto& ch : ch_) ch.reset();
  }
  int latencySamples() const noexcept { return 0; }
private:
  struct Channel {
    hdsp::Biquad hpf, shelf;
    AdaaTanh clip;
    detail::OnePoleLP lp;
    void reset() noexcept { hpf.reset(); shelf.reset(); clip.reset(); lp.reset(); }
  };
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    const float gain = detail::dbToGain(4.0f + 20.0f * detail::clamp01(c_.boost));
    const float range = 1200.0f * std::pow(3.5f, detail::clamp01(c_.range));
    const float level = detail::dbToGain(-12.0f + 18.0f * detail::clamp01(c_.level));
    const float lpG = detail::onePoleGain(9000.0f, sr_);
    for (auto& ch : ch_) {
      ch.hpf.highPass(70.0f, 0.7f, sr_);
      ch.shelf.highShelf(range, 12.0f * detail::clamp01(c_.boost), 0.55f, sr_);
    }
    gain_ = gain;
    level_ = level;
    lpG_ = lpG;
  }
  inline float processCh(Channel& ch, float x) noexcept {
    x = ch.hpf.process(x);
    x = ch.shelf.process(x) * gain_;
    x = ch.clip.process(x);
    x = ch.lp.process(x, lpG_);
    return x * level_;
  }
  float sr_ = 48000.0f;
  bool dirty_ = true;
  TrebleBoostControls c_;
  float gain_ = 1.0f, level_ = 1.0f, lpG_ = 0.5f;
  Channel ch_[2];
};

// Transparent soft-clip overdrive.
struct OverdriveControls {
  float drive = 0.4f;
  float tone = 0.5f;
  float level = 0.5f;
  float mix = 1.0f;
};
class Overdrive {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const OverdriveControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    const float dryL = l, dryR = r;
    l = processCh(ch_[0], l);
    r = processCh(ch_[1], r);
    l = dryL + (l - dryL) * mix_;
    r = dryR + (r - dryR) * mix_;
  }
  void reset() noexcept { for (auto& ch : ch_) ch.reset(); }
  int latencySamples() const noexcept { return 0; }
private:
  struct Channel {
    hdsp::Biquad hpf, tone;
    AdaaTanh clip;
    void reset() noexcept { hpf.reset(); tone.reset(); clip.reset(); }
  };
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    driveGain_ = detail::dbToGain(4.0f + 30.0f * detail::clamp01(c_.drive));
    level_ = detail::dbToGain(-12.0f + 18.0f * detail::clamp01(c_.level));
    mix_ = detail::clamp01(c_.mix);
    const float toneHz = 800.0f * std::pow(10.0f, detail::clamp01(c_.tone));
    for (auto& ch : ch_) {
      ch.hpf.highPass(50.0f, 0.7f, sr_);
      ch.tone.lowPass(toneHz, 0.7f, sr_);
    }
  }
  inline float processCh(Channel& ch, float x) noexcept {
    x = ch.hpf.process(x);
    x = ch.clip.process(x * driveGain_);
    x = ch.tone.process(x);
    return x * level_;
  }
  float sr_ = 48000.0f;
  bool dirty_ = true;
  OverdriveControls c_;
  float driveGain_ = 1.0f, level_ = 1.0f, mix_ = 1.0f;
  Channel ch_[2];
};

// Low-gain blues overdrive with asymmetric diode flavour.
struct BluesDriveControls {
  float drive = 0.35f;
  float tone = 0.5f;
  float level = 0.5f;
};
class BluesDrive {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const BluesDriveControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    l = processCh(ch_[0], l);
    r = processCh(ch_[1], r);
  }
  void reset() noexcept { for (auto& ch : ch_) ch.reset(); }
  int latencySamples() const noexcept { return 0; }
private:
  struct Channel {
    hdsp::Biquad hpf, tone;
    AdaaTanh clip;
    void reset() noexcept { hpf.reset(); tone.reset(); clip.reset(); }
  };
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    gain_ = detail::dbToGain(3.0f + 18.0f * detail::clamp01(c_.drive));
    level_ = detail::dbToGain(-12.0f + 18.0f * detail::clamp01(c_.level));
    const float toneHz = 1200.0f * std::pow(6.0f, detail::clamp01(c_.tone));
    for (auto& ch : ch_) {
      ch.hpf.highPass(70.0f, 0.7f, sr_);
      ch.tone.lowPass(toneHz, 0.65f, sr_);
    }
  }
  inline float processCh(Channel& ch, float x) noexcept {
    x = ch.hpf.process(x);
    float y = ch.clip.process(x * gain_);
    y = 0.82f * y + 0.18f * y * y;
    return ch.tone.process(y) * level_;
  }
  float sr_ = 48000.0f;
  bool dirty_ = true;
  BluesDriveControls c_;
  float gain_ = 1.0f, level_ = 1.0f;
  Channel ch_[2];
};

// Hard-clip distortion.
struct DistortionControls {
  float dist = 0.5f;
  float filter = 0.5f;
  float level = 0.5f;
};
class Distortion {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const DistortionControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    l = processCh(ch_[0], l);
    r = processCh(ch_[1], r);
  }
  void reset() noexcept { for (auto& ch : ch_) ch.reset(); }
  int latencySamples() const noexcept { return 0; }
private:
  struct Channel {
    hdsp::Biquad hpf, tone;
    AdaaHard clip;
    void reset() noexcept { hpf.reset(); tone.reset(); clip.reset(); }
  };
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    gain_ = detail::dbToGain(6.0f + 36.0f * detail::clamp01(c_.dist));
    level_ = detail::dbToGain(-15.0f + 18.0f * detail::clamp01(c_.level));
    const float f = 900.0f * std::pow(9.0f, detail::clamp01(c_.filter));
    for (auto& ch : ch_) {
      ch.hpf.highPass(40.0f, 0.7f, sr_);
      ch.tone.lowPass(f, 0.7f, sr_);
    }
  }
  inline float processCh(Channel& ch, float x) noexcept {
    x = ch.hpf.process(x);
    x = ch.clip.process(x * gain_);
    return ch.tone.process(x) * level_;
  }
  float sr_ = 48000.0f;
  bool dirty_ = true;
  DistortionControls c_;
  float gain_ = 1.0f, level_ = 1.0f;
  Channel ch_[2];
};

// Rat-style hard clipping with a darker filter.
struct RatControls {
  float dist = 0.5f;
  float filter = 0.5f;
  float level = 0.5f;
};
class Rat {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const RatControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    l = processCh(ch_[0], l);
    r = processCh(ch_[1], r);
  }
  void reset() noexcept { for (auto& ch : ch_) ch.reset(); }
  int latencySamples() const noexcept { return 0; }
private:
  struct Channel {
    hdsp::Biquad hpf, tone, lowMid;
    AdaaHard clip;
    void reset() noexcept { hpf.reset(); tone.reset(); lowMid.reset(); clip.reset(); }
  };
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    gain_ = detail::dbToGain(8.0f + 38.0f * detail::clamp01(c_.dist));
    level_ = detail::dbToGain(-15.0f + 18.0f * detail::clamp01(c_.level));
    const float f = 500.0f * std::pow(10.0f, detail::clamp01(c_.filter));
    for (auto& ch : ch_) {
      ch.hpf.highPass(45.0f, 0.7f, sr_);
      ch.tone.lowPass(f, 0.72f, sr_);
      ch.lowMid.peak(900.0f, 3.0f, 0.8f, sr_);
    }
  }
  inline float processCh(Channel& ch, float x) noexcept {
    x = ch.hpf.process(x);
    x = ch.clip.process(x * gain_);
    x = ch.lowMid.process(x);
    return ch.tone.process(x) * level_;
  }
  float sr_ = 48000.0f;
  bool dirty_ = true;
  RatControls c_;
  float gain_ = 1.0f, level_ = 1.0f;
  Channel ch_[2];
};

// Big-Muff-style two-stage fuzz.
struct BigFuzzControls {
  float sustain = 0.5f;
  float tone = 0.5f;
  float level = 0.5f;
};
class BigFuzz {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const BigFuzzControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    l = processCh(ch_[0], l);
    r = processCh(ch_[1], r);
  }
  void reset() noexcept { for (auto& ch : ch_) ch.reset(); }
  int latencySamples() const noexcept { return 0; }
private:
  struct Channel {
    hdsp::Biquad hpf, toneLP, toneHP, mid;
    AdaaTanh clip1, clip2;
    detail::OnePoleLP lp;
    void reset() noexcept {
      hpf.reset(); toneLP.reset(); toneHP.reset(); mid.reset();
      clip1.reset(); clip2.reset(); lp.reset();
    }
  };
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    gain_ = detail::dbToGain(10.0f + 34.0f * detail::clamp01(c_.sustain));
    level_ = detail::dbToGain(-18.0f + 18.0f * detail::clamp01(c_.level));
    tone_ = detail::clamp01(c_.tone);
    for (auto& ch : ch_) {
      ch.hpf.highPass(80.0f, 0.7f, sr_);
      ch.toneLP.lowPass(1000.0f, 0.7f, sr_);
      ch.toneHP.highPass(900.0f, 0.7f, sr_);
      ch.mid.peak(700.0f, -4.0f, 0.9f, sr_);
    }
    lpG_ = detail::onePoleGain(7000.0f, sr_);
  }
  inline float processCh(Channel& ch, float x) noexcept {
    x = ch.hpf.process(x);
    x = ch.clip1.process(x * gain_);
    x = ch.clip2.process(x * 2.2f);
    const float lp = ch.toneLP.process(x);
    const float hp = ch.toneHP.process(x);
    x = lp + (hp - lp) * tone_;
    x = ch.mid.process(x);
    x = ch.lp.process(x, lpG_);
    return x * level_;
  }
  float sr_ = 48000.0f;
  bool dirty_ = true;
  BigFuzzControls c_;
  float gain_ = 1.0f, level_ = 1.0f, tone_ = 0.5f, lpG_ = 0.5f;
  Channel ch_[2];
};

// Fuzz Face-style germanium fuzz with bias control.
struct FuzzFaceControls {
  float fuzz = 0.6f;
  float bias = 0.5f;
  float volume = 0.5f;
};
class FuzzFace {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const FuzzFaceControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    l = processCh(ch_[0], l);
    r = processCh(ch_[1], r);
  }
  void reset() noexcept { for (auto& ch : ch_) ch.reset(); }
  int latencySamples() const noexcept { return 0; }
private:
  struct Channel {
    hdsp::Biquad hpf, tone;
    AdaaTanh clip;
    void reset() noexcept { hpf.reset(); tone.reset(); clip.reset(); }
  };
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    gain_ = detail::dbToGain(6.0f + 30.0f * detail::clamp01(c_.fuzz));
    bias_ = (detail::clamp01(c_.bias) - 0.5f) * 0.9f;
    level_ = detail::dbToGain(-12.0f + 15.0f * detail::clamp01(c_.volume));
    for (auto& ch : ch_) {
      ch.hpf.highPass(60.0f, 0.7f, sr_);
      ch.tone.lowPass(5500.0f, 0.7f, sr_);
    }
  }
  inline float processCh(Channel& ch, float x) noexcept {
    x = ch.hpf.process(x);
    x = x * gain_ + bias_;
    x = ch.clip.process(x) - std::tanh(bias_);
    x = ch.tone.process(x);
    return x * level_;
  }
  float sr_ = 48000.0f;
  bool dirty_ = true;
  FuzzFaceControls c_;
  float gain_ = 1.0f, bias_ = 0.0f, level_ = 1.0f;
  Channel ch_[2];
};


// Tone Bender-style silicon fuzz.
struct ToneBenderControls {
  float attack = 0.6f;
  float tone = 0.5f;
  float level = 0.5f;
};
class ToneBender {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const ToneBenderControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    l = processCh(ch_[0], l);
    r = processCh(ch_[1], r);
  }
  void reset() noexcept { for (auto& ch : ch_) ch.reset(); }
  int latencySamples() const noexcept { return 0; }
private:
  struct Channel {
    hdsp::Biquad hpf, tone, mid;
    AdaaTanh clip1, clip2;
    void reset() noexcept {
      hpf.reset(); tone.reset(); mid.reset(); clip1.reset(); clip2.reset();
    }
  };
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    gain_ = detail::dbToGain(5.0f + 32.0f * detail::clamp01(c_.attack));
    level_ = detail::dbToGain(-14.0f + 18.0f * detail::clamp01(c_.level));
    const float f = 1200.0f * std::pow(7.0f, detail::clamp01(c_.tone));
    for (auto& ch : ch_) {
      ch.hpf.highPass(60.0f, 0.7f, sr_);
      ch.tone.lowPass(f, 0.72f, sr_);
      ch.mid.peak(1200.0f, 3.0f, 0.8f, sr_);
    }
  }
  inline float processCh(Channel& ch, float x) noexcept {
    x = ch.hpf.process(x);
    x = ch.clip1.process(x * gain_);
    x = ch.clip2.process(x * 1.8f);
    x = ch.mid.process(x);
    return ch.tone.process(x) * level_;
  }
  float sr_ = 48000.0f;
  bool dirty_ = true;
  ToneBenderControls c_;
  float gain_ = 1.0f, level_ = 1.0f;
  Channel ch_[2];
};

// Bass overdrive with a preserved clean low band.
struct BassDriveControls {
  float drive = 0.45f;
  float blend = 0.8f;
  float tone = 0.5f;
  float level = 0.5f;
};
class BassDrive {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const BassDriveControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    l = processCh(ch_[0], l);
    r = processCh(ch_[1], r);
  }
  void reset() noexcept { for (auto& ch : ch_) ch.reset(); }
  int latencySamples() const noexcept { return 0; }
private:
  struct Channel {
    hdsp::Biquad lowSplit, highSplit, tone;
    AdaaTanh clip;
    void reset() noexcept { lowSplit.reset(); highSplit.reset(); tone.reset(); clip.reset(); }
  };
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    gain_ = detail::dbToGain(4.0f + 28.0f * detail::clamp01(c_.drive));
    level_ = detail::dbToGain(-12.0f + 18.0f * detail::clamp01(c_.level));
    blend_ = detail::clamp01(c_.blend);
    const float f = 1000.0f * std::pow(7.0f, detail::clamp01(c_.tone));
    for (auto& ch : ch_) {
      ch.lowSplit.lowPass(150.0f, 0.7f, sr_);
      ch.highSplit.highPass(150.0f, 0.7f, sr_);
      ch.tone.lowPass(f, 0.7f, sr_);
    }
  }
  inline float processCh(Channel& ch, float x) noexcept {
    const float low = ch.lowSplit.process(x);
    const float high = ch.highSplit.process(x);
    const float wet = ch.tone.process(ch.clip.process(high * gain_));
    return (low + high * (1.0f - blend_) + wet * blend_) * level_;
  }
  float sr_ = 48000.0f;
  bool dirty_ = true;
  BassDriveControls c_;
  float gain_ = 1.0f, level_ = 1.0f, blend_ = 0.8f;
  Channel ch_[2];
};

// Bass fuzz with a preserved clean low band.
struct BassFuzzControls {
  float fuzz = 0.6f;
  float blend = 0.8f;
  float tone = 0.5f;
  float level = 0.5f;
};
class BassFuzz {
public:
  void prepare(float sr) noexcept {
    sr_ = sr > 0.0f ? sr : 48000.0f;
    reset();
    dirty_ = true;
  }
  void setControls(const BassFuzzControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    l = processCh(ch_[0], l);
    r = processCh(ch_[1], r);
  }
  void reset() noexcept { for (auto& ch : ch_) ch.reset(); }
  int latencySamples() const noexcept { return 0; }
private:
  struct Channel {
    hdsp::Biquad lowSplit, highSplit, tone;
    AdaaTanh clip1, clip2;
    void reset() noexcept {
      lowSplit.reset(); highSplit.reset(); tone.reset();
      clip1.reset(); clip2.reset();
    }
  };
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    gain_ = detail::dbToGain(8.0f + 30.0f * detail::clamp01(c_.fuzz));
    level_ = detail::dbToGain(-14.0f + 18.0f * detail::clamp01(c_.level));
    blend_ = detail::clamp01(c_.blend);
    const float f = 900.0f * std::pow(6.0f, detail::clamp01(c_.tone));
    for (auto& ch : ch_) {
      ch.lowSplit.lowPass(120.0f, 0.7f, sr_);
      ch.highSplit.highPass(120.0f, 0.7f, sr_);
      ch.tone.lowPass(f, 0.7f, sr_);
    }
  }
  inline float processCh(Channel& ch, float x) noexcept {
    const float low = ch.lowSplit.process(x);
    const float high = ch.highSplit.process(x);
    float wet = ch.clip1.process(high * gain_);
    wet = ch.clip2.process(wet * 2.0f);
    wet = ch.tone.process(wet);
    return (low + high * (1.0f - blend_) + wet * blend_) * level_;
  }
  float sr_ = 48000.0f;
  bool dirty_ = true;
  BassFuzzControls c_;
  float gain_ = 1.0f, level_ = 1.0f, blend_ = 0.8f;
  Channel ch_[2];
};

}

