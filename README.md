# pedal-dsp

Shared guitar and bass pedal DSP for Pedal Suite.

- **Boost** — clean FET-style boost with tilt tone and output level.
- **Drive808** — retuned TS9/808 overdrive ported from Amp Suite.

The library depends on hearth-dsp for shared primitives and keeps
pedal-specific DSP out of the other Suite plugins.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

GPL-3.0-or-later. See `THIRD_PARTY_NOTICES.md`.


Dependencies: FFTW3F, hearth-dsp, and the vendored Signalsmith Stretch headers.
