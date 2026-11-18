# Neural Amp Modeler core

Vendored from https://github.com/sdatkinson/NeuralAmpModelerCore
(revision d3189ed482ded880c128bf312941266c4fa5ceec), MIT licence.

- `NAM/` — model architectures and loader.
- `eigen/` — Eigen 3.4 headers (MPL2; see the COPYING files).
- `json.hpp` — nlohmann/json single header (MIT).
- `test_wavenet.nam` — small test model from the upstream example models.

Eigen and nlohmann/json are used unmodified. The NAM sources are used
unmodified apart from building with `NAM_SAMPLE_FLOAT`.
