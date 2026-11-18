// pedal-dsp — retuned TS9/808 overdrive
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Port of the retuned 808 from Amp Suite, itself based on Guitarix's ts9sim
// (GPL-2.0-or-later), with the TS-9 tone range centred at 400–1000 Hz.
#pragma once

#include <algorithm>
#include <cmath>

namespace stomp::modules {

double drive808Nonlinear(double x) noexcept;

struct Drive808Controls {
  float drive = 0.28f;
  float tone = 0.50f;
  float level = 0.55f;
};

class Drive808 {
public:
  void prepare(float sampleRate) noexcept {
    sr_ = sampleRate > 0.0f ? sampleRate : 48000.0f;
    reset();
    dirty_ = true;
  }

  void setControls(const Drive808Controls& c) noexcept {
    controls_ = c;
    dirty_ = true;
  }

  inline void process(float& l, float& r) noexcept {
    update();
    l = ch_[0].process(l, b0_, b1_, a1_, toneG_, level_);
    r = ch_[1].process(r, b0_, b1_, a1_, toneG_, level_);
  }

  void reset() noexcept {
    for (auto& ch : ch_) ch.reset();
  }

  int latencySamples() const noexcept { return 0; }

private:
  static float clamp01(float x) noexcept {
    return std::min(std::max(x, 0.0f), 1.0f);
  }

  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    constexpr float R1 = 4700.0f;
    const float R2 = 51000.0f + 500000.0f * clamp01(controls_.drive);
    constexpr float C = 0.047e-6f;
    const float a1 = (R1 + R2) * C * 2.0f * sr_;
    const float a2 = R1 * C * 2.0f * sr_;
    const float den = 1.0f + a2;
    b0_ = (1.0f + a1) / den;
    b1_ = (1.0f - a1) / den;
    a1_ = (1.0f - a2) / den;

    const float toneHz =
        400.0f * std::pow(2.5f, clamp01(controls_.tone));
    const float t = std::tan(3.14159265358979323846f * toneHz / sr_);
    toneG_ = t / (1.0f + t);
    level_ =
        std::pow(10.0f, (-12.0f + 20.0f * clamp01(controls_.level)) * 0.05f);
  }

  struct Channel {
    float xPrev = 0.0f;
    float x2Prev = 0.0f;
    float toneState = 0.0f;

    void reset() noexcept {
      xPrev = x2Prev = toneState = 0.0f;
    }

    inline float process(float x, float b0, float b1, float a1, float toneG,
                         float level) noexcept {
      const float x2 = b0 * x + b1 * xPrev - a1 * x2Prev;
      xPrev = x;
      x2Prev = x2;
      const float clipped = static_cast<float>(drive808Nonlinear(x2 - x));
      float y = x - clipped;
      const float v = (y - toneState) * toneG;
      const float toneOut = v + toneState;
      toneState = toneOut + v;
      if (std::fabs(toneState) < 1.0e-20f) toneState = 0.0f;
      return toneOut * level;
    }
  };

  float sr_ = 48000.0f;
  bool dirty_ = true;
  Drive808Controls controls_;
  float b0_ = 1.0f;
  float b1_ = 0.0f;
  float a1_ = 0.0f;
  float toneG_ = 0.0f;
  float level_ = 1.0f;
  Channel ch_[2];
};

} // namespace stomp::modules
