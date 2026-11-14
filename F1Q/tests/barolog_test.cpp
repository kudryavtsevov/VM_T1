// SPDX-License-Identifier: Apache-2.0
#include "../common/BaroLogFormat.h"
#include <cassert>
#include <cstring>
#include <cstdio>
int main(){
  using namespace barolog;
  Header h;h.sequence=1;h.crc=crc32(&h,offsetof(Header,crc));assert(valid(h));
  assert(!committed(h));
  h.reserved[0]=3000;h.reserved[1]=crc32(&h.reserved[0],4);
  assert(valid(h) && committed(h)); // append-only launch marker promotes the draft
  h.reserved[1]^=1;assert(valid(h) && !committed(h)); // torn marker preserves previous flight
  h.bootId^=1;assert(!valid(h));
  Record blank;memset(static_cast<void*>(&blank),255,sizeof(blank));assert(!valid(blank));
  Record r;r.uptimeMs=250;r.pressurePa=101325;r.flags=1;
  r.crc=crc32(&r,offsetof(Record,crc));assert(valid(r));
  r.pressurePa+=1;assert(!valid(r)); // corruption / incomplete flash write is not exported
  assert(CAPACITY>=3600);assert(CAPACITY==8191);
  assert(sizeof(Header)+CAPACITY*sizeof(Record)<=SLOT_BYTES);
  assert(sizeof(Header)+(CAPACITY+1)*sizeof(Record)>SLOT_BYTES);
  assert(crc32("123456789",9)==0xCBF43926);
  printf("Barometer log format checks passed: %u samples, %.2f minutes at 4 Hz, CRC recovery\n",CAPACITY,CAPACITY*PERIOD_MS/60000.0);
}
