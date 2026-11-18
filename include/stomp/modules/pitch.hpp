// pedal-dsp — pitch pedal engine
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <memory>

namespace stomp::modules {

namespace pitchdetail {
inline float clamp01(float x) noexcept {
  return std::min(std::max(x, 0.0f), 1.0f);
}
} // namespace pitchdetail

// Pitch engine backed by Rubber Band Live's low-delay shifter.
class PitchEngine {
public:
  PitchEngine();
  ~PitchEngine();
  PitchEngine(const PitchEngine&) = delete;
  PitchEngine& operator=(const PitchEngine&) = delete;

  void prepare(float sr) noexcept;
  void setRatio(float ratio, float glideSeconds) noexcept;
  void setTone(float tone) noexcept;
  void setLowCut(float lowCut) noexcept;
  void clearLowCut() noexcept;
  void process(float& l, float& r, float mix) noexcept;
  void reset() noexcept;
  int latencySamples() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct PitchDropControls {
  float pitch = 0.5f;   // -24..+24 semitones
  int mode = 0;         // 0 up, 1 down, 2 drop, 3 harmony, 4 detune
  float glide = 0.2f;
  float tone = 0.5f;
  float mix = 1.0f;
};

class PitchDrop {
public:
  void prepare(float sr) noexcept {
    engine_.prepare(sr);
    engine_.clearLowCut();
    reset();
    dirty_ = true;
  }
  void setControls(const PitchDropControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    engine_.process(l, r, c_.mix);
  }
  void reset() noexcept { engine_.reset(); }
  int latencySamples() const noexcept { return engine_.latencySamples(); }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    float semis = -24.0f + 48.0f * pitchdetail::clamp01(c_.pitch);
    switch (std::clamp(c_.mode, 0, 4)) {
      case 0: semis = std::max(0.0f, semis); break;          // Whammy Up
      case 1: semis = std::min(0.0f, semis); break;          // Whammy Down
      case 2: semis = -std::fabs(std::round(semis)); break;  // Drop
      case 3: semis = semis + 7.0f; break;                   // Harmony
      default: semis = 0.12f; break;                         // Detune
    }
    engine_.setRatio(std::pow(2.0f, semis / 12.0f),
                     0.01f + 0.5f * pitchdetail::clamp01(c_.glide));
    engine_.setTone(c_.tone);
  }
  PitchEngine engine_;
  PitchDropControls c_;
  bool dirty_ = true;
};

struct RetuneControls {
  float semitones = 0.5f;  // -12..+12
  float cents = 0.5f;      // -50..+50
  float lowCut = 0.5f;     // 40..300 Hz
  float mix = 1.0f;
};

class Retune {
public:
  void prepare(float sr) noexcept {
    engine_.prepare(sr);
    engine_.setLowCut(0.5f);
    engine_.setTone(1.0f);
    reset();
    dirty_ = true;
  }
  void setControls(const RetuneControls& c) noexcept {
    c_ = c;
    dirty_ = true;
  }
  inline void process(float& l, float& r) noexcept {
    update();
    engine_.process(l, r, c_.mix);
  }
  void reset() noexcept { engine_.reset(); }
  int latencySamples() const noexcept { return engine_.latencySamples(); }

private:
  void update() noexcept {
    if (!dirty_) return;
    dirty_ = false;
    const int semis = std::clamp(
        static_cast<int>(std::lround(pitchdetail::clamp01(c_.semitones) * 24.0f)) - 12,
        -12, 12);
    const float cents = -50.0f + 100.0f * pitchdetail::clamp01(c_.cents);
    const float total = static_cast<float>(semis) + cents / 100.0f;
    engine_.setRatio(std::pow(2.0f, total / 12.0f), 0.005f);
    engine_.clearLowCut();
  }
  PitchEngine engine_;
  RetuneControls c_;
  bool dirty_ = true;
};

} // namespace stomp::modules
