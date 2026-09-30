// SPDX-License-Identifier: Apache-2.0
#include "../VM_T1/EnergyLimit.h"
#include <cassert>
#include <cstring>
#include <cstdio>
int main() {
  EnergyLimit e;
  e.start(0,100);
  for (uint32_t t=5;t<=1000;t+=5) { e.sample(t,100); e.check(t,100,30000,true); }
  assert(fabs(e.joules-100)<1e-6 && e.locked && !e.running);
  assert(!strcmp(e.reason,"energy"));
  e.start(2000,10); assert(!e.running); // no restart after cutoff
  EnergyLimit ramp; ramp.start(0,0); ramp.sample(20,100);
  assert(fabs(ramp.joules-1)<1e-6);
  EnergyLimit stale; stale.start(100,10); stale.check(131,100,30000,true);
  assert(!strcmp(stale.reason,"sensor/stale"));
  EnergyLimit fault; fault.start(0,10); fault.check(1,100,30000,false);
  assert(fault.locked);
  EnergyLimit time; time.start(0,10); time.sample(20,10); time.check(20,100,20,true);
  assert(!strcmp(time.reason,"time"));
  EnergyLimit wrap; wrap.start(0xfffffff0u,100); wrap.sample(4,100);
  assert(fabs(wrap.joules-2)<1e-6); wrap.check(5,100,100,true); assert(wrap.running);
  EnergyLimit extrap; extrap.start(0,100); extrap.check(10,1,100,true);
  assert(!strcmp(extrap.reason,"energy"));
  EnergyLimit negative; negative.start(0,-10); negative.sample(20,-20);
  assert(negative.joules==0);
  puts("Energy limit tests passed");
}
