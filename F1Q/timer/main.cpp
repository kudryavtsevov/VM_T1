// SPDX-License-Identifier: Apache-2.0
// 2026: New F1Q firmware derived from the VM-T1 hardware layout.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <TinyGPSPlus.h>
#include "../common/Radio.h"
#include "Barometer.h"
#include "OnboardBaroLog.h"
using namespace f1q;
// Board: ESP32-S3 Zero. Check physical breakout before assembly.
constexpr int ESC_PIN=10, BUTTON_PIN=7, SDA_PIN=11, SCL_PIN=12;
constexpr int GPS_RX=13, GPS_TX=14;
constexpr int SERVO_PINS[3]={4,5,6};
constexpr uint8_t INA=0x40;
Servo esc, servo[3];
Preferences prefs;
TinyGPSPlus gps;
Barometer barometer;
OnboardBaroLog onboardLog;
float groundPressurePa=0, flightPressurePa=0, gpsStartM=0;
bool gpsStartValid=false;
uint32_t flightId=0;
Config cfg;
Telemetry status;
State state=IDLE;
Reason reason=NONE;
bool configured=false, sensorOk=false, radioOk=false;
bool flightUsed=false, markerPending=false;
bool rawButton=false, stableButton=false, armEligible=false;
uint32_t rawSince=0, pressedAt=0, bootAt=0, flightAt=0;
uint32_t sampledAt=0, lastPoll=0, lastTelemetry=0;
float voltage=0, current=0, previousW=0;
double joules=0;

bool writeReg(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA); Wire.write(reg); Wire.write(value>>8); Wire.write(value&255);
  return Wire.endTransmission()==0;
}
bool readReg(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(INA); Wire.write(reg);
  if (Wire.endTransmission(false)!=0 || Wire.requestFrom(INA,uint8_t(2))!=2) return false;
  value=(Wire.read()<<8)|Wire.read(); return true;
}
void angles(const uint16_t *us) {
  for(int i=0;i<3;i++) if (servo[i].attached()) servo[i].writeMicroseconds(us[i]);
}
void attachOutputs() {
  for(int i=0;i<3;i++) {
    if (cfg.servoEnabled & (1<<i)) {
      if (!servo[i].attached()) { servo[i].setPeriodHertz(50); servo[i].attach(SERVO_PINS[i],500,2400); }
    } else if (servo[i].attached()) servo[i].detach();
  }
  angles(cfg.idleUs);
}
void stopMotor(Reason why, bool dt=false, bool fault=false) {
  esc.writeMicroseconds(cfg.escOffUs);
  reason=why;
  state=fault ? FAULT : (dt ? DEThermalized : GLIDE);
  angles((dt || fault) ? cfg.dtUs : cfg.glideUs);
}
void sample(uint32_t now) {
  if (now-lastPoll<5) return;
  lastPoll=now;
  uint16_t mask, shunt, bus;
  if (!readReg(6,mask)) { sensorOk=false; return; }
  if (!(mask & 8)) return; // Only integrate fresh conversions.
  if (!readReg(1,shunt) || !readReg(2,bus)) { sensorOk=false; return; }
  const float drop=int16_t(shunt)*0.0000025f;
  voltage=bus*0.00125f+drop; // source-side voltage for high-side shunt
  current=drop/cfg.shuntOhm;
  sensorOk=abs(int16_t(shunt))<32000 && voltage>0 && voltage<36 && current>=-0.5f;
  if (sensorOk && state==MOTOR) {
    joules=integrate(joules,previousW,voltage*current,now-sampledAt);
    previousW=voltage*current;
  }
  sampledAt=now;
}
bool save(const Config &candidate) {
  if(!prefs.begin("vmf1q",false)) return false;
  bool ok=prefs.putBytes("cfg",&candidate,sizeof(candidate))==sizeof(candidate);
  prefs.end(); return ok;
}
void packets() {
  Packet p;
  while(radio::read(p)) {
    if (p.kind==DT) {
      stopMotor(REMOTE_DT,true); // idempotent; always inhibits an armed launch
      armEligible=false;
    } else if (p.kind==CONFIG && state!=MOTOR && state!=GLIDE && state!=ARMED && valid(p.config)) {
      // Same revision must be identical; revision is an application-level ACK.
      if (configured && !newer(p.config.revision,cfg.revision)) continue;
      if(!save(p.config)) continue;
      cfg=p.config; configured=true; attachOutputs();
      if(state==DEThermalized || state==FAULT) angles(cfg.dtUs);
      esc.writeMicroseconds(cfg.escOffUs);
      sensorOk=false; // require a new reading with the new calibration
    }
  }
}
void telemetry(uint32_t now) {
  if (now-lastTelemetry<250) return;
  lastTelemetry=now;
  Packet p; p.kind=TELEMETRY;
  auto &t=p.telemetry;
  t.appliedRevision=configured?cfg.revision:0; t.config=cfg;
  t.flightId=flightId;
  t.onboardRecords=onboardLog.records();t.onboardLogStatus=onboardLog.status();
  t.flightUsed=flightUsed;
  t.altitudeAgeMs=gps.altitude.isValid()?gps.altitude.age():UINT32_MAX;
  t.gpsAltitudeValid=gps.altitude.isValid() && t.altitudeAgeMs<2000;
  if(gps.altitude.isValid()) t.gpsMslM=gps.altitude.meters();
  t.gpsRelativeValid=t.gpsAltitudeValid && gpsStartValid;
  if(t.gpsRelativeValid) t.gpsRelativeM=t.gpsMslM-gpsStartM;
  t.baroAgeMs=barometer.valid?now-barometer.sampledAt:UINT32_MAX;
  t.baroValid=barometer.valid && t.baroAgeMs<500 && flightPressurePa>0;
  t.pressurePa=barometer.pressurePa;
  if(t.baroValid) t.baroRelativeM=44330.0f*(1-powf(t.pressurePa/flightPressurePa,0.19029495f));
  t.state=state; t.reason=reason; t.sensorOk=sensorOk && now-sampledAt<=50;
  t.voltageV=voltage; t.currentA=current; t.energyJ=joules;
  t.flightMs=(state==MOTOR || state==GLIDE || state==DEThermalized || state==FAULT)?now-flightAt:0;
  t.gpsValid=gps.location.isValid();
  if(t.gpsValid) { t.latitude=gps.location.lat(); t.longitude=gps.location.lng(); t.fixAgeMs=gps.location.age(); }
  t.satellites=gps.satellites.isValid()?gps.satellites.value():0;
  radio::send(p);
}
void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  // Start ESC at minimum before any initialization that can block.
  esc.setPeriodHertz(50); esc.attach(ESC_PIN,900,2000); esc.writeMicroseconds(1000);
  onboardLog.begin(barometer);
  prefs.begin("vmf1q",true);
  Config stored;
  if (prefs.getBytesLength("cfg")==sizeof(stored) && prefs.getBytes("cfg",&stored,sizeof(stored))==sizeof(stored) && valid(stored)) {
    cfg=stored; configured=true;
  }
  prefs.end(); esc.writeMicroseconds(cfg.escOffUs); attachOutputs();
  Wire.begin(SDA_PIN,SCL_PIN); Wire.setClock(400000); Wire.setTimeOut(5);
  writeReg(0,0x4127); // continuous bus+shunt, 1.1 ms each, no averaging
  barometer.begin();
  Serial1.begin(9600,SERIAL_8N1,GPS_RX,GPS_TX);
  WiFi.mode(WIFI_STA); WiFi.setSleep(false);
  radioOk=radio::begin(GROUND_MAC);
  Serial.printf("F1Q timer STA MAC: %s; ESP-NOW: %s\n",WiFi.macAddress().c_str(),radioOk?"ready":"configure Pairing.local.h");
  bootAt=millis(); rawButton=stableButton=digitalRead(BUTTON_PIN)==LOW;
  rawSince=pressedAt=bootAt;
  // A button held during boot must be released, then pressed again.
}
void loop() {
  uint32_t now=millis();
  while(Serial1.available()) gps.encode(Serial1.read());
  sample(now);
  packets();
  bool pressed=digitalRead(BUTTON_PIN)==LOW;
  if(pressed!=rawButton) { rawButton=pressed; rawSince=now; }
  if(now-rawSince>=40 && stableButton!=rawButton) {
    stableButton=rawButton;
    if(stableButton) {
      pressedAt=now;
      armEligible=!flightUsed && state!=MOTOR && state!=GLIDE && now-bootAt>=3000;
    } else {
      if(state==ARMED && armEligible && sensorOk && now-sampledAt<=50 && voltage>=cfg.minVoltageV && current<cfg.maxCurrentA) {
        state=MOTOR; flightUsed=true; reason=NONE; flightAt=now; joules=0;
        flightId=esp_random(); if(!flightId)flightId=1;
        flightPressurePa=barometer.valid && now-barometer.sampledAt<500?groundPressurePa:0;
        gpsStartValid=gps.altitude.isValid() && gps.altitude.age()<2000;
        if(gpsStartValid) gpsStartM=gps.altitude.meters();
        sampledAt=now; previousW=fmaxf(0,voltage*current);
        angles(cfg.motorUs); esc.writeMicroseconds(cfg.escRunUs);
        markerPending=true;
      } else if(state==ARMED) { state=IDLE; angles(cfg.idleUs); }
      armEligible=false;
    }
  }
  if(stableButton && armEligible && now-pressedAt>=1000 && state!=ARMED && configured &&
      cfg.shuntConfirmed && sensorOk && now-sampledAt<=50 && voltage>=cfg.minVoltageV &&
      current<cfg.maxCurrentA) {
    state=ARMED; reason=NONE; angles(cfg.idleUs);
  }
  if(state==MOTOR) {
    if(!sensorOk || now-sampledAt>50) stopMotor(SENSOR,false,true);
    else if(current>=cfg.maxCurrentA) stopMotor(OVERCURRENT,false,true);
    else if(voltage<cfg.minVoltageV) stopMotor(UNDERVOLTAGE,false,true);
    else if(cfg.energyJ>0 && joules>=cfg.energyJ) stopMotor(ENERGY);
    else if(now-flightAt>=cfg.motorMs) stopMotor(TIME);
  }
  if((state==GLIDE || state==MOTOR) && now-flightAt>=cfg.flightMs) stopMotor(FLIGHT_TIME,true);
  barometer.poll(now);
  if((state==IDLE || state==ARMED || state==DEThermalized || state==FAULT) && barometer.valid && now-barometer.sampledAt<500)
    groundPressurePa=groundPressurePa>0?0.95f*groundPressurePa+0.05f*barometer.pressurePa:barometer.pressurePa;
  BaroContext logContext;
  logContext.pauseWrites=rawButton || stableButton || state==ARMED;
  logContext.referencePa=flightPressurePa;logContext.state=state;logContext.reason=reason;
  logContext.gpsValid=gps.altitude.isValid() && gps.altitude.age()<2000;
  if(logContext.gpsValid){logContext.gpsMslM=gps.altitude.meters();logContext.gpsAt=millis()-gps.altitude.age();}
  logContext.gpsRelativeValid=logContext.gpsValid && gpsStartValid;
  if(logContext.gpsRelativeValid)logContext.gpsRelativeM=logContext.gpsMslM-gpsStartM;
  onboardLog.update(logContext);
  if(markerPending){onboardLog.markFlightStarted(flightAt);markerPending=false;}
  onboardLog.serial(state!=MOTOR && state!=GLIDE && state!=ARMED);
  telemetry(millis());
  delay(1);
}
