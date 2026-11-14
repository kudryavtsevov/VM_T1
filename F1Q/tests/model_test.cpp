// SPDX-License-Identifier: Apache-2.0
#include "../common/Model.h"
#include <cassert>
#include <limits>
#include <cstdio>
using namespace f1q;
int main() {
  Config c; assert(valid(c));
  c.motorMs=30000; c.minVoltageV=12; assert(valid(c));
  c.shuntOhm=0; assert(!valid(c));
  c=Config(); c.shuntOhm=std::numeric_limits<float>::quiet_NaN(); assert(!valid(c));
  c=Config(); c.shuntOhm=0.01; assert(!valid(c)); // 30 A would saturate shunt ADC
  c=Config(); c.flightMs=c.motorMs-1; assert(!valid(c));
  c=Config(); c.escRunUs=c.escOffUs; assert(!valid(c));
  c=Config(); c.dtUs[2]=2500; assert(!valid(c));
  // 12 V * 10 A for 25 seconds, 5 ms conversion intervals: 3000 J.
  double energy=0;
  for(int i=0;i<5000;i++) energy=integrate(energy,120,120,5);
  assert(fabs(energy-3000)<0.001);
  assert(fabs(integrate(0,0,120,10)-0.6)<0.0001); // trapezoidal rising load
  assert(integrate(0,-10,-20,5)==0); // reverse current cannot reduce consumed energy
  assert(newer(1,UINT32_MAX)); assert(!newer(UINT32_MAX,1));
  assert(!newer(4,4));
  printf("Model checks passed; ESP-NOW packet: %zu bytes\n",sizeof(Packet));
}
