// pedal-dsp — module tests
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
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
double crossingHz(const std::vector<float>& x, int a, int b) {
  int crossings = 0;
  float prev = 0.0f;
  for (int i = a; i < b; ++i) {
    if ((prev <= 0.0f && x[i] > 0.0f) || (prev >= 0.0f && x[i] < 0.0f)) {
      ++crossings;
    }
    prev = x[i];
  }
  return crossings * 0.5 / ((b - a) / double(kSr));
}
double windowRms(const std::vector<float>& x, int a, int w) {
  double s = 0.0;
  for (int i = a; i < a + w && i < static_cast<int>(x.size()); ++i) {
    s += double(x[i]) * x[i];
  }
  return std::sqrt(s / w);
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
    c.pitch = 0.5f;  // +12 semitones in Whammy Up mode
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
    check(pitch.latencySamples() <= 1280, "Pitch Drop latency is 1280 samples or less");
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
    check(retune.latencySamples() <= 1280, "Retune latency is 1280 samples or less");
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
  {
    stomp::modules::BassChorus bc;
    bc.prepare(kSr);
    stomp::modules::BassChorusControls bcc;
    bc.setControls(bcc);
    auto x = tone(80.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; bc.process(l,r); x[i]=l; }
    check(finite(x), "Bass Chorus stays finite");
    stomp::modules::Vibrato vib;
    vib.prepare(kSr);
    stomp::modules::VibratoControls vc;
    vib.setControls(vc);
    x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; vib.process(l,r); x[i]=l; }
    check(finite(x), "Vibrato stays finite");
    stomp::modules::Octave oct;
    oct.prepare(kSr);
    stomp::modules::OctaveControls oc;
    oc.down = 0.8f;
    oct.setControls(oc);
    x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; oct.process(l,r); x[i]=l; }
    check(finite(x), "Octave stays finite");
  }
  {
    stomp::modules::DigitalDelay dd;
    dd.prepare(kSr);
    stomp::modules::DigitalDelayControls dc;
    dd.setControls(dc);
    auto x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; dd.process(l,r); x[i]=l; }
    check(finite(x), "Digital Delay stays finite");
    stomp::modules::PingPongDelay pp;
    pp.prepare(kSr);
    stomp::modules::PingPongControls pc;
    pp.setControls(pc);
    x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; pp.process(l,r); x[i]=l; }
    check(finite(x), "Ping-Pong Delay stays finite");
    stomp::modules::TapeEcho te;
    te.prepare(kSr);
    stomp::modules::TapeEchoControls tc;
    te.setControls(tc);
    x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; te.process(l,r); x[i]=l; }
    check(finite(x), "Tape Echo stays finite");
    stomp::modules::PlateReverb pr;
    pr.prepare(kSr);
    stomp::modules::PlateReverbControls prc;
    pr.setControls(prc);
    x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; pr.process(l,r); x[i]=l; }
    check(finite(x), "Plate Reverb stays finite");
    stomp::modules::RoomReverb rr;
    rr.prepare(kSr);
    stomp::modules::RoomReverbControls rrc;
    rr.setControls(rrc);
    x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; rr.process(l,r); x[i]=l; }
    check(finite(x), "Room Reverb stays finite");
  }
  {
    stomp::modules::FetComp fet;
    fet.prepare(kSr);
    stomp::modules::FetCompControls fc;
    fc.input = 1.0f;
    fet.setControls(fc);
    auto x = tone(220.0f, 0.4f);
    double before = 0.0, after = 0.0;
    for (int i = 0; i < kN; ++i) {
      float l=x[i], r=x[i]; fet.process(l,r);
      if (i > kN / 2) { before += double(x[i]) * x[i]; after += double(l) * l; }
      x[i]=l;
    }
    check(finite(x), "FET Comp stays finite");
    check(after < before, "FET Comp reduces level");
    stomp::modules::OptoComp opto;
    opto.prepare(kSr);
    stomp::modules::OptoCompControls oc;
    oc.peak = 0.9f;
    opto.setControls(oc);
    x = tone(220.0f, 0.4f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; opto.process(l,r); x[i]=l; }
    check(finite(x), "Opto Comp stays finite");
    stomp::modules::EnvFilter wah;
    wah.prepare(kSr);
    stomp::modules::EnvFilterControls wc;
    wah.setControls(wc);
    x = tone(220.0f, 0.4f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; wah.process(l,r); x[i]=l; }
    check(finite(x), "Envelope Filter stays finite");
    stomp::modules::AutoWah autowah;
    autowah.prepare(kSr);
    stomp::modules::AutoWahControls awc;
    awc.sens = 0.8f;
    awc.q = 0.8f;
    awc.range = 0.8f;
    awc.mix = 1.0f;
    autowah.setControls(awc);
    x = tone(110.0f, 0.5f);
    for (int i = 0; i < kN; ++i) {
      float l=x[i], r=x[i]; autowah.process(l,r); x[i]=l;
    }
    check(finite(x), "Auto-Wah stays finite");
    auto autowahGain = [&](float amp) {
      stomp::modules::AutoWah w;
      w.prepare(kSr);
      stomp::modules::AutoWahControls wc;
      wc.sens = 0.8f;
      wc.q = 0.7f;
      wc.range = 0.9f;
      wc.mix = 1.0f;
      w.setControls(wc);
      std::vector<float> y = tone(6000.0f, amp);
      for (int i = 0; i < kN; ++i) {
        float l = y[i], r = y[i];
        w.process(l, r);
        y[i] = l;
      }
      double out = 0.0;
      for (int i = kN / 2; i < kN; ++i) out += double(y[i]) * y[i];
      return std::sqrt(out / (kN / 2)) / std::max(1.0e-9f, amp);
    };
    const float awLow = autowahGain(0.002f);
    const float awHigh = autowahGain(0.4f);
    std::printf("AutoWah gain low %.4f high %.4f\n", awLow, awHigh);
    check(awHigh > awLow * 1.5, "Auto-Wah opens with input level");
    stomp::modules::NoiseGate gate;
    gate.prepare(kSr);
    stomp::modules::GatePedalControls gc;
    gc.threshold = 0.9f;
    gate.setControls(gc);
    x = tone(220.0f, 0.4f);
    double gIn = 0.0, gOut = 0.0;
    for (int i = 0; i < kN; ++i) {
      float l=x[i], r=x[i]; gate.process(l,r);
      if (i > kN / 2) { gIn += double(x[i]) * x[i]; gOut += double(l) * l; }
      x[i]=l;
    }
    check(finite(x), "Noise Gate stays finite");
    check(gOut < gIn * 0.5, "Noise Gate closes");
    stomp::modules::TightGate tight;
    tight.prepare(kSr);
    tight.setControls(gc);
    x = tone(220.0f, 0.4f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; tight.process(l,r); x[i]=l; }
    check(finite(x), "Tight Gate stays finite");
    stomp::modules::Eq3 eq;
    eq.prepare(kSr, 120.0f, 800.0f, 4000.0f);
    stomp::modules::Eq3Controls ec;
    ec.low = 0.9f;
    eq.setControls(ec);
    x = tone(80.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; eq.process(l,r); x[i]=l; }
    check(finite(x), "3-Band EQ stays finite");
    stomp::modules::BassEq beq;
    beq.prepare(kSr);
    beq.setControls(ec);
    x = tone(80.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; beq.process(l,r); x[i]=l; }
    check(finite(x), "Bass EQ stays finite");
    stomp::modules::LoFi lofi;
    lofi.prepare(kSr);
    stomp::modules::LoFiControls lc;
    lofi.setControls(lc);
    x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; lofi.process(l,r); x[i]=l; }
    check(finite(x), "Lo-Fi stays finite");
    stomp::modules::TapeSat tape;
    tape.prepare(kSr);
    stomp::modules::TapeSatControls tc;
    tape.setControls(tc);
    x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; tape.process(l,r); x[i]=l; }
    check(finite(x), "Tape Sat stays finite");
  }
  {
    // Octave tracking, one octave down and up.
    auto octaveHz = [](bool up) {
      stomp::modules::Octave oct;
      oct.prepare(kSr);
      stomp::modules::OctaveControls c;
      c.down = up ? 0.0f : 0.9f;
      c.up = up ? 0.9f : 0.0f;
      c.tone = 0.9f;
      c.mix = 1.0f;
      oct.setControls(c);
      std::vector<float> y(48000);
      for (int i = 0; i < 48000; ++i) {
        float l = 0.2f * std::sin(2.0f * 3.14159265f * 220.0f * i / kSr);
        float r = l;
        oct.process(l, r);
        y[i] = l;
      }
      return crossingHz(y, 24000, 48000);
    };
    check(std::fabs(octaveHz(false) - 110.0) < 4.0,
          "Octave down tracks one octave");
    check(std::fabs(octaveHz(true) - 440.0) < 8.0,
          "Octave up tracks one octave");

    // EQ gain calibration.
    stomp::modules::Eq3 eq;
    eq.prepare(kSr, 120.0f, 800.0f, 4000.0f);
    stomp::modules::Eq3Controls ec;
    ec.mid = 1.0f;
    eq.setControls(ec);
    std::vector<float> in(24000), out(24000);
    for (int i = 0; i < 24000; ++i) {
      in[i] = 0.1f * std::sin(2.0f * 3.14159265f * 800.0f * i / kSr);
      float l = in[i], r = in[i];
      eq.process(l, r);
      out[i] = l;
    }
    const double midDb =
        20.0 * std::log10(rms(out, 12000, 24000) / rms(in, 12000, 24000));
    check(std::fabs(midDb - 15.0) < 1.5, "3-Band EQ mid gain is 15 dB");

    // Delay time accuracy.
    stomp::modules::DigitalDelay delay;
    delay.prepare(kSr);
    stomp::modules::DigitalDelayControls dc;
    dc.time = (400.0f - 20.0f) / 980.0f;
    dc.feedback = 0.0f;
    dc.tone = 1.0f;
    dc.mix = 1.0f;
    delay.setControls(dc);
    std::vector<float> echo(kSr, 0.0f);
    for (int i = 0; i < kSr; ++i) {
      float l = (i == 0) ? 0.5f : 0.0f;
      float r = l;
      delay.process(l, r);
      echo[i] = l;
    }
    int peak = 0;
    for (int i = 1; i < kSr; ++i) {
      if (std::fabs(echo[i]) > std::fabs(echo[peak])) peak = i;
    }
    const double echoMs = peak * 1000.0 / kSr;
    check(std::fabs(echoMs - 400.0) < 2.0, "Digital Delay echo time is 400 ms");

    // Gate must not chop a steady tone when the threshold sits near its peak.
    stomp::modules::GatePedal gate;
    gate.prepare(kSr);
    stomp::modules::GatePedalControls gc;
    gc.threshold = 0.72f;
    gc.decay = 0.1f;
    gc.range = 0.8f;
    gc.link = 1.0f;
    gate.setControls(gc, false);
    std::vector<float> gated(48000);
    for (int i = 0; i < 48000; ++i) {
      float l = 0.2f * std::sin(2.0f * 3.14159265f * 220.0f * i / kSr);
      float r = l;
      gate.process(l, r);
      gated[i] = l;
    }
    const double heldRms = rms(gated, 24000, 48000);
    check(heldRms > 0.2 / std::sqrt(2.0) * 0.95,
          "Gate holds a steady tone without chopping");

    // FET compressor should reduce dynamic range.
    auto fetOut = [](float amp) {
      stomp::modules::FetComp f;
      f.prepare(kSr);
      stomp::modules::FetCompControls c;
      c.input = 0.6f;
      c.ratio = 0.6f;
      c.attack = 0.5f;
      c.release = 0.4f;
      c.output = 0.5f;
      f.setControls(c);
      std::vector<float> y(24000);
      for (int i = 0; i < 24000; ++i) {
        float l = amp * std::sin(2.0f * 3.14159265f * 220.0f * i / kSr);
        float r = l;
        f.process(l, r);
        y[i] = l;
      }
      return rms(y, 12000, 24000);
    };
    const double ratio = fetOut(0.5f) / std::max(1.0e-9, fetOut(0.05f));
    check(ratio < 6.0, "FET Comp reduces dynamic range");

    // Plate reverb decay calibration (display 2.05 s at 0.15).
    stomp::modules::PlateReverb rv;
    rv.prepare(kSr);
    stomp::modules::PlateReverbControls rc;
    rc.decay = 0.15f;
    rc.tone = 0.5f;
    rc.mix = 1.0f;
    rv.setControls(rc);
    const int rn = kSr * 3;
    std::vector<float> tail(rn);
    for (int i = 0; i < rn; ++i) {
      float l = 0.0f, r = 0.0f;
      if (i < kSr / 10) {
        l = r = 0.3f * std::sin(2.0f * 3.14159265f * 300.0f * i / kSr);
      }
      rv.process(l, r);
      tail[i] = l;
    }
    const int win = 2400;
    double refPeak = 0.0;
    int peakAt = 0;
    for (int i = kSr / 10; i < kSr * 2; i += win) {
      const double r = windowRms(tail, i, win);
      if (r > refPeak) {
        refPeak = r;
        peakAt = i;
      }
    }
    int rt = -1;
    for (int i = peakAt; i < rn - win; i += win) {
      if (windowRms(tail, i, win) < refPeak * 0.001) {
        rt = i;
        break;
      }
    }
    const double rt60 = rt > 0 ? (rt - peakAt) / double(kSr) : -1.0;
    check(rt60 > 1.5 && rt60 < 2.6, "Plate reverb decay is calibrated");
  }
  {
    stomp::modules::Tuner tuner;
    tuner.prepare(kSr);
    for (int i = 0; i < kSr; ++i) {
      const float v = 0.2f * std::sin(2.0f * 3.14159265f * 440.0f * i / kSr);
      tuner.process(v, v);
    }
    check(std::fabs(tuner.frequency() - 440.0f) < 1.5f,
          "Tuner detects 440 Hz");
    check(tuner.midiNote() == 69, "Tuner reports A4");
    check(std::fabs(tuner.cents()) < 5.0f, "Tuner cents are in tune");
    for (int i = 0; i < kSr; ++i) {
      const float v = 0.2f * std::sin(2.0f * 3.14159265f * 110.0f * i / kSr);
      tuner.process(v, v);
    }
    check(std::fabs(tuner.frequency() - 110.0f) < 1.0f,
          "Tuner detects 110 Hz");
    check(tuner.midiNote() == 45, "Tuner reports A2");
    for (int i = 0; i < kSr; ++i) tuner.process(0.0f, 0.0f);
    check(tuner.confidence() < 0.1f, "Tuner releases on silence");
  }
  {
    stomp::modules::PrecisionDrive pd;
    pd.prepare(kSr);
    stomp::modules::PrecisionDriveControls pc;
    pc.gate = 0.0f;
    pc.drive = 0.2f;
    pc.attack = 0.0f;
    pd.setControls(pc);
    auto loose = tone(80.0f, 0.2f);
    for (int i = 0; i < kN; ++i) { float l=loose[i], r=l; pd.process(l,r); loose[i]=l; }
    pc.attack = 1.0f;
    pd.setControls(pc);
    auto tight = tone(80.0f, 0.2f);
    for (int i = 0; i < kN; ++i) { float l=tight[i], r=l; pd.process(l,r); tight[i]=l; }
    check(finite(tight), "Precision Drive stays finite");
    check(rms(tight, kN / 2, kN) < rms(loose, kN / 2, kN) * 0.7,
          "Precision Drive Attack tightens the low end");

    stomp::modules::PrecisionDrive gatePd;
    gatePd.prepare(kSr);
    stomp::modules::PrecisionDriveControls gc;
    gc.gate = 0.9f;
    gc.drive = 0.2f;
    gc.attack = 0.5f;
    gatePd.setControls(gc);
    auto quiet = tone(220.0f, 0.004f);
    for (int i = 0; i < kN; ++i) { float l=quiet[i], r=l; gatePd.process(l,r); quiet[i]=l; }
    check(rms(quiet, kN / 2, kN) < 0.004 / std::sqrt(2.0) * 0.3,
          "Precision Drive gate closes on low-level signal");
    check(gatePd.gainReductionDb() > 6.0, "Precision Drive reports gate GR");

    stomp::modules::Wah wah;
    wah.prepare(kSr);
    stomp::modules::WahControls wc;
    wc.pedal = 0.0f;
    wc.q = 0.6f;
    wc.range = 0.5f;
    wah.setControls(wc);
    auto lowWah = tone(700.0f, 0.2f);
    for (int i = 0; i < kN; ++i) { float l=lowWah[i], r=l; wah.process(l,r); lowWah[i]=l; }
    wc.pedal = 0.5f;
    wah.setControls(wc);
    auto midWah = tone(700.0f, 0.2f);
    for (int i = 0; i < kN; ++i) { float l=midWah[i], r=l; wah.process(l,r); midWah[i]=l; }
    check(finite(midWah), "Wah stays finite");
    check(rms(midWah, kN / 2, kN) > rms(lowWah, kN / 2, kN) * 1.5,
          "Wah pedal sweeps the band-pass");

    stomp::modules::Laser laser;
    laser.prepare(kSr);
    stomp::modules::LaserControls lc;
    lc.pitch = 0.75f;  // +12 st
    lc.ring = 0.0f;
    lc.sens = 0.0f;
    lc.pedal = 0.5f;
    lc.mix = 1.0f;
    laser.setControls(lc);
    std::vector<float> out(96000);
    for (int i = 0; i < 96000; ++i) {
      float l = 0.2f * std::sin(2.0f * 3.14159265f * 220.0f * i / kSr);
      float r = l;
      laser.process(l, r);
      out[i] = l;
    }
    check(finite(out), "Laser stays finite");
    check(std::fabs(crossingHz(out, 48000, 96000) - 440.0) < 20.0,
          "Laser shifts pitch");
    check(laser.latencySamples() > 0, "Laser reports latency");
  }
  {
    std::vector<float> ir(64, 0.0f);
    ir[0] = 1.0f;
    stomp::modules::Cabinet cab;
    cab.prepare(kSr);
    cab.setIR(ir.data(), static_cast<int>(ir.size()));
    stomp::modules::CabControls cc;
    cc.lowCut = 0.0f;
    cc.highCut = 1.0f;
    cab.setControls(cc);
    auto x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; cab.process(l,r); x[i]=l; }
    check(finite(x), "Cab stays finite");
    check(cab.latencySamples() == 256, "Cab reports one block of latency");
    check(rms(x, 4800, kN) > 0.01, "Cab passes audio");
    cab.setIR(nullptr, 0);
    x = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=x[i], r=x[i]; cab.process(l,r); x[i]=l; }
    check(finite(x), "Cab without IR stays finite");

    // An IR selected before prepare must survive the lazy preparation.
    stomp::modules::Cabinet late;
    late.setIR(ir.data(), static_cast<int>(ir.size()));
    late.prepare(kSr);
    auto z = tone(220.0f, 0.3f);
    for (int i = 0; i < kN; ++i) { float l=z[i], r=z[i]; late.process(l,r); z[i]=l; }
    check(rms(z, 4800, kN) > 0.01,
          "Cab keeps an IR selected before prepare");
  }
#ifdef NAM_TEST_MODEL
  {
    stomp::modules::NamAmpPedal nam;
    nam.prepare(kSr);
    stomp::modules::NamAmpControls nc;
    nam.setControls(nc);
    const bool loaded = nam.loadModel(NAM_TEST_MODEL);
    check(loaded, "NAM model loads");
    check(nam.modelLoaded(), "NAM model reports loaded");
    check(nam.latencySamples() >= 64, "NAM reports block latency");
    auto x = tone(220.0f, 0.2f);
    for (int i = 0; i < kN; ++i) {
      float l = x[i], r = x[i];
      nam.process(l, r);
      x[i] = l;
    }
    check(finite(x), "NAM stays finite at 48 kHz");
    check(rms(x, 4800, kN) > 1.0e-4, "NAM produces audio at 48 kHz");

    stomp::modules::NamAmpPedal nam96;
    nam96.prepare(96000.0f);
    nam96.setControls(nc);
    nam96.loadModel(NAM_TEST_MODEL);
    std::vector<float> y(48000);
    for (int i = 0; i < 48000; ++i) {
      float l = 0.2f * std::sin(2.0f * 3.14159265f * 220.0f * i / 96000.0f);
      float r = l;
      nam96.process(l, r);
      y[i] = l;
    }
    check(finite(y), "NAM stays finite at 96 kHz");
    check(rms(y, 9600, 48000) > 1.0e-4, "NAM resamples at 96 kHz");

    stomp::modules::NamAmpPedal gated;
    gated.prepare(kSr);
    stomp::modules::NamAmpControls gc;
    gc.gate = 0.9f;
    gated.setControls(gc);
    gated.loadModel(NAM_TEST_MODEL);
    std::vector<float> quiet(48000);
    for (int i = 0; i < 48000; ++i) {
      float l = 0.001f * std::sin(2.0f * 3.14159265f * 220.0f * i / kSr);
      float r = l;
      gated.process(l, r);
      quiet[i] = l;
    }
    check(rms(quiet, 24000, 48000) < 1.0e-5,
          "NAM gate mutes the model noise floor");

    {
      namespace fs = std::filesystem;
      const fs::path dir = fs::temp_directory_path() / "pedal-nam-test";
      std::error_code ec;
      fs::remove_all(dir, ec);
      fs::create_directories(dir, ec);
      const fs::path malformed = dir / "bad.nam";
      {
        std::ofstream out(malformed);
        out << "this is not a NAM model";
      }
      stomp::modules::NamAmpPedal broken;
      broken.prepare(kSr);
      check(!broken.loadModel(malformed.string()),
            "malformed NAM model is rejected");
      float bl = 0.1f, br = 0.1f;
      broken.process(bl, br);
      check(std::isfinite(bl), "malformed NAM passes safely");
      const fs::path huge = dir / "huge.nam";
      {
        std::ofstream out(huge);
        out << "{}";
      }
      fs::resize_file(huge, 40u * 1024u * 1024u, ec);
      stomp::modules::NamAmpPedal oversized;
      oversized.prepare(kSr);
      check(!oversized.loadModel(huge.string()),
            "oversized NAM model is rejected");
      fs::remove_all(dir, ec);
    }

    stomp::modules::NamAmpPedal bad;
    bad.prepare(kSr);
    check(!bad.loadModel("/definitely/not/a/model.nam"),
          "missing NAM model is rejected");
    float l = 0.2f, r = 0.2f;
    bad.process(l, r);
    check(std::isfinite(l), "missing NAM model passes safely");
  }
#endif
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
  // Retune's Low Cut must actually remove the low end.
  {
    stomp::modules::Retune retune;
    retune.prepare(kSr);
    stomp::modules::RetuneControls c;
    c.semitones = 0.5f;  // no shift
    c.cents = 0.5f;
    c.mix = 1.0f;
    c.lowCut = 1.0f;     // 300 Hz
    retune.setControls(c);
    double cut = 0.0;
    for (int i = 0; i < kN; ++i) {
      float l = 0.2f * std::sin(2.0f * 3.14159265f * 80.0f * i / kSr);
      float r = l;
      retune.process(l, r);
      if (i > 4800) cut += static_cast<double>(l) * l;
    }
    stomp::modules::Retune flat;
    flat.prepare(kSr);
    c.lowCut = 0.0f;
    flat.setControls(c);
    double full = 0.0;
    for (int i = 0; i < kN; ++i) {
      float l = 0.2f * std::sin(2.0f * 3.14159265f * 80.0f * i / kSr);
      float r = l;
      flat.process(l, r);
      if (i > 4800) full += static_cast<double>(l) * l;
    }
    std::printf("retune low cut energy %.5f vs %.5f\n", cut, full);
    check(cut < full * 0.5, "Retune low cut removes the low end");
  }
  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
