// SPDX-License-Identifier: Apache-2.0
// Hardware-independent, monotonic energy accounting for the experimental F1Q mode.
#pragma once
#include <stdint.h>
#include <math.h>

struct EnergyLimit {
  double joules = 0;
  uint32_t started = 0, lastSample = 0;
  float previousW = 0;
  bool running = false, locked = false;
  const char *reason = "idle";
  void start(uint32_t now, float watts) {
    if (locked || running) return;
    started = lastSample = now; previousW = fmaxf(0, watts); joules = 0;
    running = true; reason = "running";
  }
  void stop(const char *why) { running = false; locked = true; reason = why; }
  void sample(uint32_t now, float watts) {
    if (!running || !isfinite(watts)) return;
    watts = fmaxf(0, watts);
    joules += (double(previousW) + watts) * 0.5 * uint32_t(now-lastSample) / 1000.0;
    previousW = watts; lastSample = now;
  }
  void check(uint32_t now, float budgetJ, uint32_t maxMs, bool healthy) {
    if (!running) return;
    if (!healthy || uint32_t(now-lastSample) > 30) { stop("sensor/stale"); return; }
    if (uint32_t(now-started) >= maxMs) { stop("time"); return; }
    // Include the interval since the last conversion in the cutoff decision.
    if (joules + previousW * uint32_t(now-lastSample)/1000.0 >= budgetJ) stop("energy");
  }
};
