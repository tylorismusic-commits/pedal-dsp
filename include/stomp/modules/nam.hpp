// pedal-dsp — NAM amp pedal (hard-limited to two instances)
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <memory>
#include <string>

namespace stomp::modules {

// Mono NAM model wrapper. All methods are real-time safe once load() has
// completed, apart from the deliberate load on model selection.
class NamAmp {
public:
  NamAmp();
  ~NamAmp();
  NamAmp(const NamAmp&) = delete;
  NamAmp& operator=(const NamAmp&) = delete;

  void prepare(float sampleRate, int maxBlock) noexcept;
  bool load(const std::string& path) noexcept;
  void clear() noexcept;
  bool loaded() const noexcept;
  const std::string& path() const noexcept;
  const char* error() const noexcept;
  float expectedSampleRate() const noexcept;
  int latencySamples() const noexcept;
  void process(float* mono, int frames) noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct NamAmpControls {
  float gain = 0.50f;
  float bass = 0.50f;
  float mid = 0.50f;
  float treble = 0.50f;
  float level = 0.50f;
  float gate = 0.35f;  // 0 = off; higher = more aggressive
};

// Amp pedal: input trim -> NAM model -> 3-band EQ -> output level.
class NamAmpPedal {
public:
  NamAmpPedal();
  ~NamAmpPedal();
  NamAmpPedal(const NamAmpPedal&) = delete;
  NamAmpPedal& operator=(const NamAmpPedal&) = delete;

  void prepare(float sampleRate) noexcept;
  void setControls(const NamAmpControls& c) noexcept;
  bool loadModel(const std::string& path) noexcept;
  void clearModel() noexcept;
  bool modelLoaded() const noexcept;
  const char* error() const noexcept;
  void process(float& l, float& r) noexcept;
  void reset() noexcept;
  int latencySamples() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace stomp::modules
