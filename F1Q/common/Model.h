// SPDX-License-Identifier: Apache-2.0
// F1Q extension, 2026. Original VM-T1 author: Мстислав.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>

namespace f1q {
constexpr uint32_t MAGIC = 0x31514656;
constexpr uint8_t VERSION = 1, CHANNEL = 6;
enum Kind : uint8_t { CONFIG = 1, TELEMETRY = 2, DT = 3 };
enum State : uint8_t { IDLE, ARMED, MOTOR, GLIDE, DEThermalized, FAULT };
enum Reason : uint8_t { NONE, TIME, ENERGY, REMOTE_DT, LOCAL_DT, SENSOR, OVERCURRENT, UNDERVOLTAGE, FLIGHT_TIME };
#pragma pack(push, 1)
struct Config {
  uint32_t revision = 1;
  float shuntOhm = 0.001f; // EXAMPLE ONLY: must be measured and confirmed
  float energyJ = 0;       // 0 disables energy limit; time limit remains mandatory
  float maxCurrentA = 30;
  float minVoltageV = 9;
  uint32_t motorMs = 25000;
  uint32_t flightMs = 180000;
  uint16_t escOffUs = 1000, escRunUs = 1300;
  uint8_t shuntConfirmed = 0;
  uint8_t servoEnabled = 1; // bit mask for S/R/W
  uint16_t idleUs[3] = {1500,1500,1500};
  uint16_t motorUs[3] = {1500,1500,1500};
  uint16_t glideUs[3] = {1500,1500,1500};
  uint16_t dtUs[3] = {2000,1500,1500};
};
struct Telemetry {
  uint32_t appliedRevision = 0, flightMs = 0, fixAgeMs = UINT32_MAX;
  float voltageV = 0, currentA = 0, energyJ = 0;
  double latitude = 0, longitude = 0;
  uint8_t state = IDLE, reason = NONE, sensorOk = 0, gpsValid = 0, satellites = 0;
  Config config;
};
struct Packet {
  uint32_t magic = MAGIC;
  uint8_t version = VERSION, kind = 0;
  uint16_t bytes = sizeof(Packet);
  uint32_t sequence = 0;
  Config config;
  Telemetry telemetry;
};
#pragma pack(pop)
static_assert(sizeof(Packet) <= 250, "ESP-NOW v1 payload limit");
inline bool valid(const Config &c) {
  if (!c.revision || !isfinite(c.shuntOhm) || c.shuntOhm < 0.00005f || c.shuntOhm > 0.1f ||
      !isfinite(c.energyJ) || c.energyJ < 0 || c.energyJ > 100000 ||
      !isfinite(c.maxCurrentA) || c.maxCurrentA <= 0 || c.maxCurrentA > 200 ||
      c.maxCurrentA * c.shuntOhm > 0.075f ||
      !isfinite(c.minVoltageV) || c.minVoltageV < 2 || c.minVoltageV > 35 ||
      c.motorMs < 100 || c.motorMs > 120000 || c.flightMs < c.motorMs || c.flightMs > 3600000 ||
      c.escOffUs < 900 || c.escOffUs > 1200 || c.escRunUs <= c.escOffUs || c.escRunUs > 2000 ||
      c.shuntConfirmed > 1 || c.servoEnabled > 7) return false;
  for (int i=0;i<3;i++)
    if (c.idleUs[i]<500 || c.idleUs[i]>2400 || c.motorUs[i]<500 || c.motorUs[i]>2400 ||
        c.glideUs[i]<500 || c.glideUs[i]>2400 || c.dtUs[i]<500 || c.dtUs[i]>2400) return false;
  return true;
}
// Wrap-aware ordering, assuming fewer than 2^31 packets between observations.
inline bool newer(uint32_t a, uint32_t b) { return int32_t(a-b)>0; }
inline double integrate(double joules, float previousW, float watts, uint32_t dtMs) {
  return joules + (fmaxf(0, previousW)+fmaxf(0, watts))*0.5*dtMs/1000.0;
}
}
