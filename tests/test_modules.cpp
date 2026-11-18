// pedal-dsp — module tests
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include <stomp/modules/modules.hpp>

namespace {
int failures = 0;
void check(bool ok, const char* name) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++failures;
}
constexpr float kSr = 48000.0f;
constexpr int kN = 24000;

std::vector<float> tone(float hz, float amp = 0.1f) {
  std::vector<float> x(kN);
  for (int i = 0; i < kN; ++i) {
    x[i] = amp * std::sin(2.0f * 3.14159265f * hz * i / kSr);
  }
  return x;
}
double rms(const std::vector<float>& x, int a, int b) {
  double s = 0.0;
  for (int i = a; i < b; ++i) s += double(x[i]) * x[i];
  return std::sqrt(s / std::max(1, b - a));
}
bool finite(const std::vector<float>& x) {
  for (float v : x)
    if (!std::isfinite(v)) return false;
  return true;
}
} // namespace

int main() {
  {
    stomp::modules::Boost boost;
    boost.prepare(kSr);
    stomp::modules::BoostControls c;
    c.gain = 1.0f;
    boost.setControls(c);
    auto x = tone(1000.0f);
    const double before = rms(x, 4800, kN);
    for (int i = 0; i < kN; ++i) {
      float l = x[i], r = x[i];
      boost.process(l, r);
      x[i] = l;
    }
    check(finite(x), "Boost stays finite");
    check(rms(x, 4800, kN) > before * 3.0, "Boost adds clean gain");
  }
  {
    stomp::modules::Drive808 drive;
    drive.prepare(kSr);
    stomp::modules::Drive808Controls c;
    c.drive = 0.75f;
    c.tone = 0.4f;
    drive.setControls(c);
    auto x = tone(220.0f, 0.4f);
    for (int i = 0; i < kN; ++i) {
      float l = x[i], r = x[i];
      drive.process(l, r);
      x[i] = l;
    }
    check(finite(x), "808 stays finite");
    check(rms(x, 4800, kN) > 0.01, "808 produces output");
  }
  {
    stomp::modules::TrebleBoost treble;
    treble.prepare(kSr);
    stomp::modules::TrebleBoostControls c;
    treble.setControls(c);
    auto x = tone(2000.0f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; treble.process(l,r); x[i]=l; }
    check(finite(x), "Treble Boost stays finite");
  }
  {
    stomp::modules::Overdrive od;
    od.prepare(kSr);
    stomp::modules::OverdriveControls c;
    od.setControls(c);
    auto x = tone(440.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; od.process(l,r); x[i]=l; }
    check(finite(x), "Overdrive stays finite");
  }
  {
    stomp::modules::BluesDrive bd;
    bd.prepare(kSr);
    stomp::modules::BluesDriveControls c;
    bd.setControls(c);
    auto x = tone(440.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; bd.process(l,r); x[i]=l; }
    check(finite(x), "Blues Drive stays finite");
  }
  {
    stomp::modules::Distortion dist;
    dist.prepare(kSr);
    stomp::modules::DistortionControls c;
    dist.setControls(c);
    auto x = tone(440.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; dist.process(l,r); x[i]=l; }
    check(finite(x), "Distortion stays finite");
  }
  {
    stomp::modules::Rat rat;
    rat.prepare(kSr);
    stomp::modules::RatControls c;
    rat.setControls(c);
    auto x = tone(440.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; rat.process(l,r); x[i]=l; }
    check(finite(x), "Rat stays finite");
  }
  {
    stomp::modules::BigFuzz fuzz;
    fuzz.prepare(kSr);
    stomp::modules::BigFuzzControls c;
    fuzz.setControls(c);
    auto x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; fuzz.process(l,r); x[i]=l; }
    check(finite(x), "Big Fuzz stays finite");
  }
  {
    stomp::modules::FuzzFace fuzz;
    fuzz.prepare(kSr);
    stomp::modules::FuzzFaceControls c;
    fuzz.setControls(c);
    auto x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; fuzz.process(l,r); x[i]=l; }
    check(finite(x), "Fuzz Face stays finite");
  }
  {
    stomp::modules::ToneBender bender;
    bender.prepare(kSr);
    stomp::modules::ToneBenderControls c;
    bender.setControls(c);
    auto x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; bender.process(l,r); x[i]=l; }
    check(finite(x), "Tone Bender stays finite");
  }
  {
    stomp::modules::BassDrive bd;
    bd.prepare(kSr);
    stomp::modules::BassDriveControls c;
    bd.setControls(c);
    auto x = tone(80.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; bd.process(l,r); x[i]=l; }
    check(finite(x), "Bass Drive stays finite");
  }
  {
    stomp::modules::BassFuzz bf;
    bf.prepare(kSr);
    stomp::modules::BassFuzzControls c;
    bf.setControls(c);
    auto x = tone(80.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; bf.process(l,r); x[i]=l; }
    check(finite(x), "Bass Fuzz stays finite");
  }
  for (float sr : {48000.0f, 96000.0f}) {
    stomp::modules::Drive808 drive;
    drive.prepare(sr);
    stomp::modules::Drive808Controls c;
    c.drive = 1.0f;
    drive.setControls(c);
    bool ok = true;
    float phase = 0.0f;
    for (int i = 0; i < 24000; ++i) {
      const float in = 0.3f * std::sin(phase);
      phase += 2.0f * 3.14159265f * 440.0f / sr;
      float l = in, r = in;
      drive.process(l, r);
      if (!std::isfinite(l) || !std::isfinite(r)) ok = false;
    }
    char label[64];
    std::snprintf(label, sizeof(label), "808 finite at %.0f Hz", sr);
    check(ok, label);
  }
  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
