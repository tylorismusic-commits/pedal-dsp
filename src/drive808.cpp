// pedal-dsp — Guitarix TS9 nonlinear table bridge
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cmath>

#define __rt_data
#include "ts9nonlin.h"
#undef __rt_data

#include "stomp/modules/drive808.hpp"

namespace stomp::modules {

double drive808Nonlinear(double x) noexcept { return ts9nonlin(x); }

} // namespace stomp::modules
