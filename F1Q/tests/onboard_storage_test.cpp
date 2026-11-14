// SPDX-License-Identifier: Apache-2.0
#define F1Q_HOST_TEST
#define F1Q_TEST_ONBOARD_LOG
#include "fakes/Hardware.h"
#include "../timer/Barometer.h"
#include "../timer/OnboardBaroLog.h"
#include <cstdio>
barolog::Header headerAt(int slot){barolog::Header h;memcpy(&h,flash.data()+slot*barolog::SLOT_BYTES,sizeof(h));return h;}
void runUntil(uint32_t limit){fakeTaskHook=[limit]{if(fakeNow>=limit)throw StopTask{};};try{fakeTask(fakeArg);}catch(StopTask&){} }
int main(){
  Barometer baro;baro.valid=true;baro.pressurePa=101325;
  OnboardBaroLog first;first.begin(baro);assert(erases==1);assert(!barolog::committed(headerAt(0)));
  BaroContext ctx;ctx.referencePa=101325;ctx.state=0;first.update(ctx);
  unsigned heldPrograms=0;
  fakeTaskHook=[&]{
    if(fakeNow>=250 && fakeNow<1250){
      if(!launchButtonHeld){heldPrograms=programs;launchButtonHeld=true;ctx.pauseWrites=1;first.update(ctx);}
      assert(programs==heldPrograms); // samples buffer in RAM; no flash programming
    }
    if(fakeNow>=1250 && launchButtonHeld){launchButtonHeld=false;pwmStarted=true;ctx.pauseWrites=0;ctx.state=2;first.update(ctx);first.markFlightStarted(fakeNow);}
    if(fakeNow>=2250)throw StopTask{};
  };
  try{fakeTask(fakeArg);}catch(StopTask&){}
  assert(barolog::committed(headerAt(0)));assert(erases==1);
  unsigned ground=0,flight=0;
  for(unsigned i=0;i<first.records();i++){
    barolog::Record r;memcpy(&r,flash.data()+sizeof(barolog::Header)+i*sizeof(r),sizeof(r));assert(barolog::valid(r));
    if(r.state==0)++ground;else ++flight;
  }
  assert(ground>=4 && flight>=3); // one continuous file, including buffered preflight
  auto previous=headerAt(0);
  OnboardBaroLog second;second.begin(baro);assert(erases==2);assert(barolog::committed(headerAt(0)));assert(!barolog::committed(headerAt(1)));
  runUntil(fakeNow+750);assert(headerAt(0).crc==previous.crc); // boot without flight preserves last
  OnboardBaroLog third;third.begin(baro);assert(erases==3);assert(barolog::committed(headerAt(0)));
  third.markFlightStarted(fakeNow);runUntil(fakeNow+750);
  assert(barolog::committed(headerAt(1)));assert(headerAt(1).sequence>headerAt(0).sequence);
  OnboardBaroLog fourth;fourth.begin(baro);assert(erases==4);assert(barolog::committed(headerAt(1)));assert(!barolog::committed(headerAt(0)));
  puts("Onboard storage checks passed: last flight retained across boots, preflight draft promotion after PWM, no erase/program during launch hold, continuous CRC-valid records");
}
