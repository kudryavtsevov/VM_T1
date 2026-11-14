// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdint.h>
#include <stddef.h>
namespace barolog {
constexpr uint32_t MAGIC=0x314C4256, SLOT_BYTES=256*1024, PERIOD_MS=250;
#pragma pack(push,1)
struct Header {
  uint32_t magic=MAGIC, sequence=0, bootId=0, startedMs=0;
  uint16_t version=2, recordBytes=32;
  uint32_t reserved[2]={UINT32_MAX,UINT32_MAX};
  uint32_t crc=0;
};
struct Record {
  uint32_t uptimeMs=0;
  float pressurePa=0, baroRelativeM=0, gpsMslM=0, gpsRelativeM=0;
  uint32_t sensorAgeMs=UINT32_MAX;
  uint8_t flags=0,state=0,reason=0,reserved=0;
  uint32_t crc=0;
};
#pragma pack(pop)
static_assert(sizeof(Header)==32 && sizeof(Record)==32,"Onboard log format");
constexpr uint32_t CAPACITY=(SLOT_BYTES-sizeof(Header))/sizeof(Record);
static_assert(CAPACITY*PERIOD_MS>=15*60*1000,"At least 15 minutes at 4 Hz");
inline uint32_t crc32(const void *ptr,size_t bytes) {
  auto p=static_cast<const uint8_t*>(ptr);uint32_t c=0xFFFFFFFF;
  while(bytes--){c^=*p++;for(int i=0;i<8;i++)c=(c>>1)^(0xEDB88320u & (0u-(c&1)));}
  return ~c;
}
inline bool valid(const Header &h){
  Header copy=h;copy.reserved[0]=copy.reserved[1]=UINT32_MAX;
  return h.magic==MAGIC && h.version==2 && h.recordBytes==sizeof(Record) && h.crc==crc32(&copy,offsetof(Header,crc));
}
inline bool committed(const Header &h){return valid(h) && h.reserved[0]!=UINT32_MAX && h.reserved[1]==crc32(&h.reserved[0],4);}
inline bool valid(const Record &r){return r.crc==crc32(&r,offsetof(Record,crc));}
}
