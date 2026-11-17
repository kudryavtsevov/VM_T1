// SPDX-License-Identifier: Apache-2.0
#include "../common/DtButton.h"
#include <cassert>
#include <cstdio>
int main(){
  f1q::DtButton b;b.begin(0,false);
  assert(!b.poll(100,true));assert(!b.poll(105,false));assert(!b.poll(110,true));assert(!b.poll(129,true));
  assert(b.poll(130,true));assert(!b.poll(225,true));assert(b.poll(230,true)); // 20 ms stable press, then 100 ms retries
  unsigned sent=2;
  for(uint32_t t=235;t<2130;t+=5)if(b.poll(t,true))++sent;
  assert(sent==20);assert(!b.poll(2130,true));assert(!b.active());
  assert(!b.poll(2300,false));assert(!b.poll(2320,false));assert(!b.poll(2400,true));assert(b.poll(2420,true));
  b.begin(0,true);assert(!b.poll(1000,true)); // held at power-on must not issue DT
  assert(!b.poll(1100,false));assert(!b.poll(1120,false));assert(!b.poll(1200,true));assert(b.poll(1220,true));
  b.begin(UINT32_MAX-50,false);assert(b.poll(UINT32_MAX-40,false,true));
  assert(b.poll(59,false)); // retry works through millis wrap
  assert(!b.poll(1960,false));assert(!b.active());
  puts("DT checks passed: bounce, immediate press trigger, retries, expiry, release/repress, boot-held inhibition, web request, millis wrap");
}
