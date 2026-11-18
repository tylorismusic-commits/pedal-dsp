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
  {
    stomp::modules::PitchDrop pitch;
    pitch.prepare(kSr);
    stomp::modules::PitchDropControls c;
    c.pitch = 0.75f;  // +12 semitones
    c.mode = 0;
    c.glide = 0.0f;
    c.mix = 1.0f;
    pitch.setControls(c);
    int crossings = 0;
    float prev = 0.0f;
    for (int i = 0; i < kN; ++i) {
      float l = 0.2f * std::sin(2.0f * 3.14159265f * 220.0f * i / kSr);
      float r = l;
      pitch.process(l, r);
      if (i > 4800) {
        if ((prev <= 0.0f && l > 0.0f) || (prev >= 0.0f && l < 0.0f)) {
          ++crossings;
        }
        prev = l;
      }
    }
    const double hz = crossings * 0.5 / ((kN - 4800) / double(kSr));
    check(std::fabs(hz - 440.0) < 25.0, "Pitch Drop shifts +12 semitones");
    check(pitch.latencySamples() > 0, "Pitch Drop reports latency");
  }
  {
    stomp::modules::Retune retune;
    retune.prepare(kSr);
    stomp::modules::RetuneControls c;
    c.semitones = 1.0f;  // +12
    c.cents = 0.5f;
    c.lowCut = 0.0f;
    c.mix = 1.0f;
    retune.setControls(c);
    int crossings = 0;
    float prev = 0.0f;
    for (int i = 0; i < kN; ++i) {
      float l = 0.2f * std::sin(2.0f * 3.14159265f * 220.0f * i / kSr);
      float r = l;
      retune.process(l, r);
      if (i > 4800) {
        if ((prev <= 0.0f && l > 0.0f) || (prev >= 0.0f && l < 0.0f)) {
          ++crossings;
        }
        prev = l;
      }
    }
    const double hz = crossings * 0.5 / ((kN - 4800) / double(kSr));
    std::printf("DBG retune hz %.2f\n", hz);
    check(std::fabs(hz - 440.0) < 25.0, "Retune shifts +12 semitones");
  }
  {
    stomp::modules::Chorus chorus;
    chorus.prepare(kSr);
    stomp::modules::ChorusControls c;
    chorus.setControls(c);
    auto x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; chorus.process(l,r); x[i]=l; }
    check(finite(x), "Chorus stays finite");
    stomp::modules::Flanger flanger;
    flanger.prepare(kSr);
    stomp::modules::FlangerControls fc;
    flanger.setControls(fc);
    x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; flanger.process(l,r); x[i]=l; }
    check(finite(x), "Flanger stays finite");
    stomp::modules::Phaser phaser;
    phaser.prepare(kSr);
    stomp::modules::PhaserControls pc;
    phaser.setControls(pc);
    x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; phaser.process(l,r); x[i]=l; }
    check(finite(x), "Phaser stays finite");
    stomp::modules::Tremolo tremolo;
    tremolo.prepare(kSr);
    stomp::modules::TremoloControls tc;
    tc.depth = 1.0f;
    tc.shape = 0.0f;
    tc.rate = 0.9f;
    tremolo.setControls(tc);
    x = tone(220.0f, 0.3f);
    double drySum = 0.0, wetSum = 0.0;
    for (int i = 0; i < kN; ++i) {
      float l=x[i], r=x[i];
      tremolo.process(l,r);
      if (i > kN / 2) { drySum += double(x[i]) * x[i]; wetSum += double(l) * l; }
    }
    check(finite(x), "Tremolo stays finite");
    check(wetSum < drySum * 0.9, "Tremolo modulates level");
  }
  {
    stomp::modules::AnalogDelay delay;
    delay.prepare(kSr);
    stomp::modules::AnalogDelayControls c;
    delay.setControls(c);
    auto x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; delay.process(l,r); x[i]=l; }
    check(finite(x), "Analog Delay stays finite");
    stomp::modules::SpringReverb reverb;
    reverb.prepare(kSr);
    stomp::modules::SpringReverbControls rc;
    reverb.setControls(rc);
    x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; reverb.process(l,r); x[i]=l; }
    check(finite(x), "Spring Reverb stays finite");
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
