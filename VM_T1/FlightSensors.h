// SPDX-License-Identifier: Apache-2.0
// VM-T1 sensor extension, 2026-09-25.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <math.h>

class FlightSensors {
 public:
  static constexpr uint8_t INA_ADDRESS = 0x40;
  bool inaPresent = false, inaValid = false, bmpPresent = false, altitudeValid = false;
  float voltage = 0, current = 0, power = 0, altitude = 0, maxAltitude = 0;
  float pressurePa = 0, referencePa = 0;
  uint32_t powerAt = 0;
  uint32_t powerSequence = 0;
  float shuntOhms = 0.01f; // Replace R100 physically with R010 (10 milliohms) before use.
  float currentScale = 1.0f;
  float currentOffsetA = 0.0f;

  void begin() {
    Wire.begin(PIN_SDA, PIN_SCL, 100000);
    Wire.setTimeOut(10);
    uint16_t manufacturer = 0, device = 0;
    inaPresent = read16(0xFE, manufacturer) && manufacturer == 0x5449 &&
                 read16(0xFF, device) && (device & 0xFFF0) == 0x2260;
    // AVG=4, VBUSCT=588us, VSHCT=588us, continuous shunt+bus: 4.704ms.
    if (inaPresent) inaPresent = write16(0x00, 0x42DF);
    if (bmp.begin(0x76)) baroAddress = 0x76;
    else if (bmp.begin(0x77)) baroAddress = 0x77;
    bmpPresent = baroAddress != 0;
    if (bmpPresent) bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X1, Adafruit_BMP280::SAMPLING_X4,
      Adafruit_BMP280::FILTER_X4, Adafruit_BMP280::STANDBY_MS_1);
  }

  bool fresh(uint32_t now) const { return inaValid && uint32_t(now-powerAt) <= 30; }
  void poll(bool allowBarometer = true) {
    const uint32_t now = millis();
    if (inaPresent && uint32_t(now-lastIna) >= 5) {
      lastIna = now;
      uint16_t flags, rawBus, rawShunt;
      if (!read16(0x06, flags)) { inaValid = false; }
      else if (flags & 0x0008) { // Conversion ready, cleared by this read.
        if (!read16(0x02, rawBus) || !read16(0x01, rawShunt) || (flags & 0x0004)) {
          inaValid = false;
        } else {
          voltage = rawBus * 0.00125f; // VBUS must sense IN- (load side).
          const int16_t signedShunt = int16_t(rawShunt);
          current = (signedShunt * 0.0000025f / shuntOhms - currentOffsetA) * currentScale;
          voltage += signedShunt * 0.0000025f; // Reconstruct battery-side voltage.
          power = voltage * fmaxf(current, 0.0f);
          inaValid = isfinite(power) && isfinite(current) && voltage >= 1 && voltage <= 16 &&
                     current >= -0.005f && current <= 7.5f && abs(int(signedShunt)) <= 30000;
          if (inaValid) { powerAt = millis(); ++powerSequence; }
        }
      }
      if (uint32_t(millis()-powerAt) > 30) inaValid = false;
    }
    if (allowBarometer && bmpPresent && uint32_t(now-lastBmp) >= 100) {
      lastBmp = now;
      // Verify communication as the library's pressure API alone has no I2C error result.
      if (!barometerResponding()) { altitudeValid = false; return; }
      pressurePa = bmp.readPressure();
      altitudeValid = isfinite(pressurePa) && pressurePa >= 30000 && pressurePa <= 110000;
      if (altitudeValid) {
        if (!referencePa) referencePa = pressurePa;
        altitude = 44330.0f * (1.0f-powf(pressurePa/referencePa, 0.19029495f));
        maxAltitude = fmaxf(maxAltitude, altitude);
      }
    }
  }
  bool zeroAltitude() {
    if (!altitudeValid) return false;
    referencePa = pressurePa; altitude = maxAltitude = 0; return true;
  }
 private:
  Adafruit_BMP280 bmp;
  uint8_t baroAddress = 0;
  uint32_t lastIna = 0, lastBmp = 0;
  bool barometerResponding() {
    Wire.beginTransmission(baroAddress); Wire.write(0xD0);
    return Wire.endTransmission(false) == 0 && Wire.requestFrom(baroAddress, uint8_t(1)) == 1 &&
           Wire.read() == 0x58;
  }
  bool read16(uint8_t reg, uint16_t &out) {
    Wire.beginTransmission(INA_ADDRESS); Wire.write(reg);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(INA_ADDRESS, uint8_t(2)) != 2) return false;
    out = uint16_t(Wire.read()) << 8; out |= Wire.read(); return true;
  }
  bool write16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(INA_ADDRESS); Wire.write(reg);
    Wire.write(uint8_t(value >> 8)); Wire.write(uint8_t(value));
    return Wire.endTransmission() == 0;
  }
};
