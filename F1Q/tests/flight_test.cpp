// SPDX-License-Identifier: Apache-2.0
// Runs production timer setup/loop against simulated GPIO, I2C, PWM and radio.
#define F1Q_HOST_TEST
#include "../timer/main.cpp"
#include <cassert>
#include <cstdio>
void advance(uint32_t ms) {for(uint32_t i=0;i<ms;i+=5){fakeNow+=5;loop();}}
void press(){fakeButton=true;advance(1100);}
void release(){fakeButton=false;advance(60);}
void remoteDT(){Packet p;p.kind=DT;radio::queue.push_back(p);advance(5);}
// Each launch helper starts a new simulated power session; production never resets this latch.
void launch(){flightUsed=false;press();assert(state==ARMED);release();assert(state==MOTOR);assert(esc.pulse==cfg.escRunUs);}
int main(){
  fakeButton=true;setup();configured=true;cfg.shuntConfirmed=1;
  advance(4000);release();assert(state==IDLE);assert(esc.pulse==cfg.escOffUs); // boot-held button
  cfg.shuntConfirmed=0;press();release();assert(state==IDLE); // uncalibrated sensor
  cfg.shuntConfirmed=1;launch();
  advance(25100);assert(state==GLIDE);assert(reason==TIME);assert(esc.pulse==cfg.escOffUs);
  assert(joules>740 && joules<760); // 12 V * 2.5 A * 25 s
  press();release();assert(state==GLIDE); // no restart in flight
  remoteDT();assert(state==DEThermalized);
  press();release();assert(state==DEThermalized); // one flight per power session
  launch();fakeSensor=false;advance(5);assert(state==FAULT);assert(reason==SENSOR);assert(esc.pulse==cfg.escOffUs);
  fakeSensor=true;advance(10);cfg.energyJ=3;launch();advance(150);assert(state==GLIDE);assert(reason==ENERGY);
  remoteDT();cfg.energyJ=0;launch();fakeDt=true;advance(5);assert(state==DEThermalized);assert(reason==LOCAL_DT);fakeDt=false;
  launch();fakeShunt=16000;advance(5);assert(state==FAULT);assert(reason==OVERCURRENT);fakeShunt=1000;advance(10);
  launch();fakeBus=6000;advance(5);assert(state==FAULT);assert(reason==UNDERVOLTAGE);fakeBus=9600;advance(10);
  launch();fakeReady=false;advance(60);assert(state==FAULT);assert(reason==SENSOR);fakeReady=true;advance(10);
  flightUsed=false;press();assert(state==ARMED);remoteDT();release();assert(state==DEThermalized); // DT cancels armed release
  cfg.motorMs=100;cfg.flightMs=200;launch();advance(250);assert(state==DEThermalized);assert(reason==FLIGHT_TIME);
  // Apply config outside flight, acknowledge only a newer valid revision.
  Packet p;p.kind=CONFIG;p.config=cfg;p.config.revision=10;p.config.motorMs=150;
  radio::queue.push_back(p);advance(5);assert(cfg.revision==10);
  p.config.revision=9;radio::queue.push_back(p);advance(5);assert(cfg.revision==10);
  launch();p.config.revision=11;radio::queue.push_back(p);advance(5);assert(cfg.revision==10);
  remoteDT();cfg.motorMs=25000;cfg.flightMs=180000;
  barometer.valid=true;barometer.pressurePa=101325;
  gps.altitude.valid=true;gps.altitude.value=120;
  advance(300);launch();assert(flightPressurePa>101324 && gpsStartValid);
  gps.altitude.value=150;barometer.pressurePa=100129;advance(300);
  assert(radio::last.telemetry.gpsAltitudeValid && radio::last.telemetry.gpsRelativeValid);
  assert(fabs(radio::last.telemetry.gpsRelativeM-30)<0.01);
  assert(radio::last.telemetry.baroValid && radio::last.telemetry.baroRelativeM>90 && radio::last.telemetry.baroRelativeM<110);
  gps.altitude.ageMs=3000;advance(300);assert(!radio::last.telemetry.gpsAltitudeValid);
  puts("Flight checks passed: physical release, boot inhibition, energy/time/voltage/current/sensor cutoffs, DT, configuration lock");
}
