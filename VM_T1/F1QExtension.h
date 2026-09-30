// SPDX-License-Identifier: Apache-2.0
// Experimental F1Q controller; no EDIC approval. See docs/F1Q.md.
#pragma once
#include "FlightSensors.h"
#include "EnergyLimit.h"

FlightSensors flightSensors;
EnergyLimit energyLimit;
Servo motorEsc;
struct F1QSettings {
  bool enabled = false;
  float massG = 300, joulesPerGram = 3, reserveJ = 20;
  uint32_t motorMs = 29000;
  uint16_t runUs = 1500;
} f1q;
bool extensionReady = false, armOpenSeen = false, flightArmed = false;
bool flightStartEvent = false;
uint32_t startHeldAt = 0, latestSequence = 0;

float allowedEnergyJ() { return fminf(f1q.massG,550.0f)*f1q.joulesPerGram; }
bool flightBusy() { return flightArmed || energyLimit.running; }
void motorStop(const char *why) {
  // Drop PWM to idle BEFORE any bookkeeping or I2C transaction.
  motorEsc.writeMicroseconds(1000);
  energyLimit.stop(why); flightArmed = false;
}
void beginExtension() {
  pinMode(PIN_ARM, INPUT_PULLUP); pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_ESC, OUTPUT); digitalWrite(PIN_ESC, LOW);
  motorEsc.setPeriodHertz(50); motorEsc.attach(PIN_ESC,1000,2000);
  motorEsc.writeMicroseconds(1000);
  Preferences p; p.begin("c3-r010",true);
  f1q.enabled = p.getBool("enabled",false);
  f1q.massG = p.getFloat("mass",300); f1q.joulesPerGram = p.getFloat("jpg",3);
  f1q.reserveJ = p.getFloat("reserve",20); f1q.motorMs = p.getUInt("ms",29000);
  f1q.runUs = p.getUShort("pwm",1500);
  flightSensors.shuntOhms = p.getFloat("shunt",0.01f);
  flightSensors.currentScale = p.getFloat("gain",1);
  flightSensors.currentOffsetA = p.getFloat("offset",0);
  p.end();
  // Reject corrupt/stale persistent settings, never silently arm with them.
  if (!isfinite(f1q.massG) || f1q.massG < 50 || f1q.massG > 5000 ||
      !(f1q.joulesPerGram == 2 || f1q.joulesPerGram == 3) ||
      !isfinite(f1q.reserveJ) || f1q.reserveJ < 5 || f1q.reserveJ >= allowedEnergyJ() ||
      f1q.motorMs < 100 || f1q.motorMs > (f1q.joulesPerGram == 2 ? 19500u : 29500u) ||
      f1q.runUs < 1000 || f1q.runUs > 2000 ||
      !isfinite(flightSensors.shuntOhms) || flightSensors.shuntOhms < 0.009f || flightSensors.shuntOhms > 0.011f ||
      !isfinite(flightSensors.currentScale) || flightSensors.currentScale < 0.5f || flightSensors.currentScale > 1.5f ||
      !isfinite(flightSensors.currentOffsetA) || fabsf(flightSensors.currentOffsetA)>0.1f) {
    f1q.enabled = false; motorStop("invalid configuration");
    flightSensors.shuntOhms=0.01f;flightSensors.currentScale=1;flightSensors.currentOffsetA=0;
  }
  flightSensors.begin(); extensionReady = true;
}

void serviceExtension() {
  if (!extensionReady) return;
  uint32_t now = millis();
  if (energyLimit.running) {
    if (digitalRead(PIN_ARM) != LOW || rcDtPressed() || digitalRead(PIN_OFF_BUTTON) == LOW) {
      motorStop("abort");
    } else {
      energyLimit.check(now, allowedEnergyJ()-f1q.reserveJ, f1q.motorMs, flightSensors.fresh(now));
      if (!energyLimit.running) motorStop(energyLimit.reason);
    }
  }
  // I2C transactions use a bounded timeout; check the cutoff again immediately afterwards.
  flightSensors.poll();
  now = millis();
  if (energyLimit.running && latestSequence != flightSensors.powerSequence) {
    latestSequence = flightSensors.powerSequence;
    energyLimit.sample(flightSensors.powerAt,flightSensors.power);
  }
  if (energyLimit.running) {
    energyLimit.check(now,allowedEnergyJ()-f1q.reserveJ,f1q.motorMs,flightSensors.fresh(now));
    if (!energyLimit.running) motorStop(energyLimit.reason);
    return;
  }
  if (!f1q.enabled || energyLimit.locked) return;
  if (digitalRead(PIN_ARM) != LOW) {
    armOpenSeen = true; flightArmed = false; startHeldAt = 0; return;
  }
  if (!armOpenSeen || !flightSensors.fresh(now) || rcDtPressed() ||
      digitalRead(PIN_OFF_BUTTON)==LOW || flightSensors.current > 1.0f) {
    if (flightArmed) motorStop("arming fault");
    startHeldAt = 0; return;
  }
  if (digitalRead(PIN_START) == LOW) {
    if (!startHeldAt) startHeldAt = now;
    if (uint32_t(now-startHeldAt)>=1000) {
      if (!flightArmed) flightSensors.zeroAltitude();
      flightArmed = true;
    }
  } else {
    if (flightArmed) {
      flightArmed = false; flightStartEvent = true;
      latestSequence = flightSensors.powerSequence;
      energyLimit.start(now,flightSensors.power);
      motorEsc.writeMicroseconds(f1q.runUs);
    }
    startHeldAt = 0;
  }
}

// Existing melodies and button waits must keep servicing the power limiter.
void servicedDelay(uint32_t ms) {
  const uint32_t start = millis();
  do { serviceExtension(); delay(1); } while (uint32_t(millis()-start)<ms);
}

void handleF1QConfig() {
  if (digitalRead(PIN_ARM)==LOW || flightBusy()) { server.send(409,"text/plain","Remove ARM jumper first"); return; }
  JsonDocument d;
  if (deserializeJson(d,server.arg("plain"))) { server.send(400,"text/plain","Bad JSON"); return; }
  F1QSettings n;
  n.enabled=d["enabled"]|false; n.massG=d["massG"]|300.0f;
  n.joulesPerGram=d["joulesPerGram"]|3.0f; n.reserveJ=d["reserveJ"]|20.0f;
  n.motorMs=d["motorMs"]|29000u; n.runUs=d["runUs"]|1500;
  const float shunt=d["shuntOhms"]|0.01f, gain=d["currentScale"]|1.0f, offset=d["currentOffsetA"]|0.0f;
  if (!isfinite(n.massG)||n.massG<50||n.massG>5000||!(n.joulesPerGram==2||n.joulesPerGram==3)||
      !isfinite(n.reserveJ)||n.reserveJ<5||n.reserveJ>=fminf(n.massG,550)*n.joulesPerGram||
      n.motorMs<100||n.motorMs>(n.joulesPerGram==2?19500u:29500u)||n.runUs<1000||n.runUs>2000||
      !isfinite(shunt)||shunt<0.009f||shunt>0.011f||!isfinite(gain)||gain<0.5f||gain>1.5f||
      !isfinite(offset)||fabsf(offset)>0.1f) { server.send(400,"text/plain","Invalid limits/calibration"); return; }
  Preferences p; p.begin("c3-r010",false);
  p.putBool("enabled",n.enabled); p.putFloat("mass",n.massG); p.putFloat("jpg",n.joulesPerGram);
  p.putFloat("reserve",n.reserveJ); p.putUInt("ms",n.motorMs); p.putUShort("pwm",n.runUs);
  p.putFloat("shunt",shunt); p.putFloat("gain",gain); p.putFloat("offset",offset); p.end();
  server.send(200,"application/json","{\"ok\":true,\"restartRequired\":true}");
  // Apply only at next power-on; saving never unlocks a finished flight.
}

void addSensorStatus(JsonDocument &doc) {
  doc["ina226"]=flightSensors.fresh(millis()); doc["bmp280"]=flightSensors.altitudeValid;
  if(flightSensors.fresh(millis())) {doc["voltageV"]=flightSensors.voltage;doc["currentA"]=flightSensors.current;doc["powerW"]=flightSensors.power;}
  if(flightSensors.altitudeValid) {doc["heightM"]=flightSensors.altitude;doc["maxHeightM"]=flightSensors.maxAltitude;}
  doc["energyJ"]=energyLimit.joules; doc["energyLimitJ"]=allowedEnergyJ()-f1q.reserveJ;
  doc["f1qEnabled"]=f1q.enabled;doc["motorState"]=energyLimit.reason;doc["armed"]=flightArmed;
  doc["motorLocked"]=energyLimit.locked;
}
