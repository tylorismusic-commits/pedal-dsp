// pedal-dsp — NAM runtime bridge
// SPDX-License-Identifier: GPL-3.0-or-later
#include "stomp/modules/nam.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <vector>

#include <hdsp/biquad.hpp>
#include <hdsp/db.hpp>

#include "get_dsp.h"
#include "container.h"
#include "convnet.h"
#include "linear.h"
#include "lstm.h"
#include "sequential.h"
#include "wavenet/model.h"

namespace stomp::modules {

namespace {
// Static registrations live in translation units with no referenced symbols,
// so a static archive would drop them. Referencing each parser forces the
// linker to keep every supported architecture.
[[gnu::used]] const void* const kNamArchAnchors[] = {
    reinterpret_cast<const void*>(&nam::wavenet::create_config),
    reinterpret_cast<const void*>(&nam::convnet::create_config),
    reinterpret_cast<const void*>(&nam::lstm::create_config),
    reinterpret_cast<const void*>(&nam::linear::create_config),
    reinterpret_cast<const void*>(&nam::container::create_config),
    reinterpret_cast<const void*>(&nam::sequential::create_config)};

constexpr int kMaxBlock = 4096;
constexpr int kMaxModelBytes = 32 * 1024 * 1024;

class LinearResampler {
public:
  void configure(double inRate, double outRate) noexcept {
    ratio_ = (inRate > 0.0 && outRate > 0.0) ? inRate / outRate : 1.0;
    pos_ = 0.0;
    prev_ = 0.0f;
    down_ = inRate > outRate * 1.001;
    if (down_) {
      const double cutoff = std::min(outRate * 0.45, inRate * 0.49);
      lp_.lowPass(static_cast<float>(cutoff), 0.7f,
                  static_cast<float>(inRate));
    } else {
      lp_.reset();
    }
    filtered_.assign(static_cast<size_t>(kMaxBlock + 8), 0.0f);
  }
  int process(const float* in, int n, float* out, int maxOut) noexcept {
    if (!in || !out || n <= 0 || maxOut <= 0) return 0;
    const float* src = in;
    if (down_) {
      const int count = std::min(n, static_cast<int>(filtered_.size()));
      for (int i = 0; i < count; ++i) filtered_[static_cast<size_t>(i)] =
          lp_.process(in[i]);
      src = filtered_.data();
    }
    int k = 0;
    double p = pos_;
    while (p < static_cast<double>(n) && k < maxOut) {
      const int i = static_cast<int>(p);
      const float frac = static_cast<float>(p - static_cast<double>(i));
      const float b = src[i];
      const float a = i == 0 ? prev_ : src[i - 1];
      out[k++] = a + (b - a) * frac;
      p += ratio_;
    }
    prev_ = src[n - 1];
    pos_ = p - static_cast<double>(n);
    if (pos_ < 0.0) pos_ = 0.0;
    return k;
  }

private:
  double ratio_ = 1.0;
  double pos_ = 0.0;
  float prev_ = 0.0f;
  bool down_ = false;
  hdsp::Biquad lp_;
  std::vector<float> filtered_;
};
} // namespace

struct NamAmp::Impl {
  std::unique_ptr<nam::DSP> dsp;
  std::string path;
  std::string error;
  float sampleRate = 48000.0f;
  float expected = 48000.0f;
  int maxBlock = kMaxBlock;
  int modelMaxBlock = kMaxBlock;
  bool resample = false;
  bool arbitrary = false;
  double hostToModelRatio = 1.0;
  LinearResampler hostToModel;
  LinearResampler modelToHost;
  std::vector<float> monoIn;
  std::vector<float> modelIn;
  std::vector<float> modelOut;
  std::vector<float> monoOut;

  void allocate() noexcept {
    const size_t maxModel = static_cast<size_t>(
        std::ceil(static_cast<double>(maxBlock) *
                  std::max(1.0, hostToModelRatio) + 8.0));
    monoIn.assign(static_cast<size_t>(maxBlock + 8), 0.0f);
    modelIn.assign(maxModel, 0.0f);
    modelOut.assign(maxModel, 0.0f);
    monoOut.assign(static_cast<size_t>(maxBlock + 8), 0.0f);
    modelMaxBlock = static_cast<int>(maxModel);
  }

  void configure() noexcept {
    if (!dsp) return;
    arbitrary = dsp->SupportsArbitrarySampleRate();
    expected = static_cast<float>(dsp->GetExpectedSampleRate());
    if (expected <= 0.0f) expected = sampleRate;
    resample = !arbitrary && std::fabs(expected - sampleRate) > 0.5f;
    hostToModelRatio =
        static_cast<double>(sampleRate) / static_cast<double>(expected);
    if (resample) {
      hostToModel.configure(sampleRate, expected);
      modelToHost.configure(expected, sampleRate);
    } else {
      hostToModel.configure(expected, expected);
      modelToHost.configure(expected, expected);
    }
    allocate();
    const double modelRate = resample ? expected : sampleRate;
    try {
      dsp->Reset(modelRate, modelMaxBlock);
    } catch (...) {
      error = "Model reset failed";
      dsp.reset();
      path.clear();
    }
  }

  void processBlock(float* monoSamples, int n) noexcept {
    if (!dsp || n <= 0) return;
    int offset = 0;
    while (offset < n) {
      const int count = std::min(kMaxBlock, n - offset);
      const float* in = monoSamples + offset;
      int modelFrames = count;
      const float* modelInput = in;
      if (resample) {
        modelFrames = hostToModel.process(in, count, modelIn.data(),
                                          modelMaxBlock);
        modelInput = modelIn.data();
      } else {
        std::copy(in, in + count, modelIn.begin());
        modelInput = modelIn.data();
      }
      if (modelFrames <= 0) {
        std::fill(monoSamples + offset, monoSamples + offset + count, 0.0f);
        offset += count;
        continue;
      }
      float* ip[1] = {const_cast<float*>(modelInput)};
      float* op[1] = {modelOut.data()};
      dsp->process(ip, op, modelFrames);
      if (resample) {
        const int hostFrames = modelToHost.process(
            modelOut.data(), modelFrames, monoOut.data(), count);
        for (int i = 0; i < hostFrames; ++i) monoSamples[offset + i] =
            monoOut[static_cast<size_t>(i)];
        const float tail = hostFrames > 0 ? monoOut[static_cast<size_t>(hostFrames - 1)] : 0.0f;
        for (int i = hostFrames; i < count; ++i) monoSamples[offset + i] = tail;
      } else {
        for (int i = 0; i < count; ++i) monoSamples[offset + i] =
            modelOut[static_cast<size_t>(i)];
      }
      offset += count;
    }
  }
};

NamAmp::NamAmp() : impl_(std::make_unique<Impl>()) {}
NamAmp::~NamAmp() = default;

void NamAmp::prepare(float sampleRate, int maxBlock) noexcept {
  impl_->sampleRate = sampleRate > 0.0f ? sampleRate : 48000.0f;
  impl_->maxBlock = std::max(64, std::min(maxBlock, kMaxBlock));
  impl_->configure();
}

bool NamAmp::load(const std::string& path) noexcept {
  if (path == impl_->path && impl_->dsp) return true;
  impl_->error.clear();
  impl_->dsp.reset();
  impl_->path.clear();
  if (path.empty()) return false;
  std::error_code ec;
  const auto size = std::filesystem::file_size(path, ec);
  if (ec || size == 0 || size > static_cast<uintmax_t>(kMaxModelBytes)) {
    impl_->error = "Model file missing or too large";
    return false;
  }
  try {
    auto dsp = nam::get_dsp(std::filesystem::path(path));
    if (!dsp) {
      impl_->error = "Unsupported or invalid NAM model";
      return false;
    }
    impl_->dsp = std::move(dsp);
    impl_->path = path;
    impl_->configure();
    return impl_->dsp != nullptr;
  } catch (const std::exception& e) {
    impl_->error = std::string("NAM: ") + e.what();
    impl_->dsp.reset();
    impl_->path.clear();
    return false;
  } catch (...) {
    impl_->error = "NAM model failed to load";
    impl_->dsp.reset();
    impl_->path.clear();
    return false;
  }
}

void NamAmp::clear() noexcept {
  impl_->dsp.reset();
  impl_->path.clear();
  impl_->error.clear();
}

bool NamAmp::loaded() const noexcept { return impl_->dsp != nullptr; }
const std::string& NamAmp::path() const noexcept { return impl_->path; }
const char* NamAmp::error() const noexcept { return impl_->error.c_str(); }
float NamAmp::expectedSampleRate() const noexcept { return impl_->expected; }
int NamAmp::latencySamples() const noexcept { return 0; }

void NamAmp::process(float* mono, int frames) noexcept {
  if (!impl_->dsp || !mono || frames <= 0) return;
  try {
    impl_->processBlock(mono, frames);
  } catch (...) {
    impl_->error = "NAM processing failed";
    impl_->dsp.reset();
    impl_->path.clear();
  }
}

struct NamAmpPedal::Impl {
  static constexpr int kBlock = 64;
  NamAmp model;
  NamAmpControls c;
  hdsp::Biquad bass;
  hdsp::Biquad mid;
  hdsp::Biquad treble;
  hdsp::Biquad dc;
  bool dirty = true;
  float sampleRate = 48000.0f;
  float gain = 1.0f;
  float level = 1.0f;
  std::vector<float> blockIn;
  std::vector<float> blockOut;
  std::vector<float> mono;
  int inCount = 0;
  int outCount = 0;
  int outPos = 0;

  Impl() {
    blockIn.assign(kBlock, 0.0f);
    blockOut.assign(kBlock, 0.0f);
    mono.assign(kBlock, 0.0f);
  }

  void update() noexcept {
    if (!dirty) return;
    dirty = false;
    const float lowDb = (c.bass - 0.5f) * 30.0f;
    const float midDb = (c.mid - 0.5f) * 30.0f;
    const float highDb = (c.treble - 0.5f) * 30.0f;
    bass.lowShelf(100.0f, lowDb, 0.7f, sampleRate);
    mid.peak(700.0f, midDb, 0.8f, sampleRate);
    treble.highShelf(3000.0f, highDb, 0.7f, sampleRate);
    dc.highPass(15.0f, 0.7f, sampleRate);
    gain = hdsp::dbToGain(-12.0f + 36.0f * c.gain);
    level = hdsp::dbToGain(-12.0f + 24.0f * c.level);
  }

  void resetBuffers() noexcept {
    inCount = 0;
    outCount = 0;
    outPos = 0;
    std::fill(blockIn.begin(), blockIn.end(), 0.0f);
    std::fill(blockOut.begin(), blockOut.end(), 0.0f);
  }
};

NamAmpPedal::NamAmpPedal() : impl_(std::make_unique<Impl>()) {}
NamAmpPedal::~NamAmpPedal() = default;

void NamAmpPedal::prepare(float sampleRate) noexcept {
  impl_->sampleRate = sampleRate > 0.0f ? sampleRate : 48000.0f;
  impl_->bass.reset();
  impl_->mid.reset();
  impl_->treble.reset();
  impl_->dc.reset();
  impl_->resetBuffers();
  impl_->dirty = true;
  impl_->model.prepare(impl_->sampleRate, Impl::kBlock);
}

void NamAmpPedal::setControls(const NamAmpControls& c) noexcept {
  impl_->c = c;
  impl_->dirty = true;
}

bool NamAmpPedal::loadModel(const std::string& path) noexcept {
  const bool ok = impl_->model.load(path);
  impl_->resetBuffers();
  return ok;
}
void NamAmpPedal::clearModel() noexcept {
  impl_->model.clear();
  impl_->resetBuffers();
}
bool NamAmpPedal::modelLoaded() const noexcept { return impl_->model.loaded(); }
const char* NamAmpPedal::error() const noexcept { return impl_->model.error(); }
int NamAmpPedal::latencySamples() const noexcept {
  return Impl::kBlock + impl_->model.latencySamples();
}

void NamAmpPedal::reset() noexcept {
  impl_->bass.reset();
  impl_->mid.reset();
  impl_->treble.reset();
  impl_->dc.reset();
  impl_->resetBuffers();
  impl_->model.prepare(impl_->sampleRate, Impl::kBlock);
}

void NamAmpPedal::process(float& l, float& r) noexcept {
  impl_->update();
  const float dry = 0.5f * (l + r);
  impl_->blockIn[static_cast<size_t>(impl_->inCount++)] =
      dry * impl_->gain;
  if (impl_->inCount >= Impl::kBlock) {
    impl_->inCount = 0;
    std::copy(impl_->blockIn.begin(), impl_->blockIn.end(),
              impl_->mono.begin());
    if (impl_->model.loaded()) impl_->model.process(impl_->mono.data(),
                                                    Impl::kBlock);
    for (int i = 0; i < Impl::kBlock; ++i) {
      float wet = impl_->mono[static_cast<size_t>(i)];
      wet = impl_->dc.process(
          impl_->treble.process(impl_->mid.process(impl_->bass.process(wet))));
      wet *= impl_->level;
      impl_->blockOut[static_cast<size_t>(i)] = std::isfinite(wet) ? wet : 0.0f;
    }
    impl_->outPos = 0;
    impl_->outCount = Impl::kBlock;
  }
  float wet = 0.0f;
  if (impl_->outCount > 0) {
    wet = impl_->blockOut[static_cast<size_t>(impl_->outPos++)];
    --impl_->outCount;
  }
  l = wet;
  r = wet;
}

} // namespace stomp::modules
