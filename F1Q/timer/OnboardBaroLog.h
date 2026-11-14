// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "../common/BaroLogFormat.h"
struct BaroContext {
  float referencePa=0,gpsMslM=0,gpsRelativeM=0;
  uint32_t gpsAt=0;
  uint8_t state=0,reason=0,gpsValid=0,gpsRelativeValid=0,pauseWrites=0;
};
#if defined(F1Q_HOST_TEST) && !defined(F1Q_TEST_ONBOARD_LOG)
struct OnboardBaroLog {
  uint32_t records()const{return 0;} uint8_t status()const{return 1;}
  void markFlightStarted(uint32_t){}
  void begin(Barometer&){} void update(const BaroContext&){} void serial(bool){}
};
#else
#include <esp_partition.h>
#include <atomic>
class OnboardBaroLog {
  const esp_partition_t *partition=nullptr;
  Barometer *barometer=nullptr;
  QueueHandle_t contextQueue=nullptr;
  TaskHandle_t worker=nullptr;
  uint32_t slot=0;
  std::atomic<uint32_t> count{0},requestedStart{0};
  std::atomic<bool> failed{false},promoted{false};
  barolog::Record buffered[256]; // 64 seconds at 4 Hz while launch button is held
  String command;
  barolog::Header header;
  bool readHeader(uint32_t i,barolog::Header &h) {
    return partition && esp_partition_read(partition,i*barolog::SLOT_BYTES,&h,sizeof(h))==ESP_OK && barolog::valid(h);
  }
  int latestFlight() {
    int found=-1;uint32_t seq=0;
    for(uint32_t i=0;i<2;i++){barolog::Header h;if(readHeader(i,h) && barolog::committed(h) && (found<0 || int32_t(h.sequence-seq)>0)){found=i;seq=h.sequence;}}
    return found;
  }
  static void task(void *arg) {
    auto self=static_cast<OnboardBaroLog*>(arg);
    uint32_t nextAt=millis(),head=0,queued=0;
    bool overflow=false;
    while(self->count<barolog::CAPACITY && !self->failed) {
      BaroContext c;xQueuePeek(self->contextQueue,&c,0);
      uint32_t start=self->requestedStart.load();
      if(start && !self->promoted && !c.pauseWrites) {
        uint32_t commit[2]={start,barolog::crc32(&start,4)};
        // One append-only marker, in this background task. Launch path never writes flash.
        if(esp_partition_write(self->partition,self->slot*barolog::SLOT_BYTES+offsetof(barolog::Header,reserved),commit,sizeof(commit))!=ESP_OK){self->failed=true;break;}
        self->promoted=true;
      }
      uint32_t now=millis();
      if(int32_t(now-nextAt)>=0) {
        barolog::Record r;
        r.uptimeMs=now;r.state=c.state;r.reason=c.reason;
        uint32_t at=0;bool valid=self->barometer->snapshot(r.pressurePa,at);
        r.sensorAgeMs=valid?now-at:UINT32_MAX;
        if(valid && r.sensorAgeMs<500){r.flags|=1;if(c.referencePa>0){r.baroRelativeM=44330*(1-powf(r.pressurePa/c.referencePa,0.19029495f));r.flags|=2;}}
        if(c.gpsValid && now-c.gpsAt<2000){r.gpsMslM=c.gpsMslM;r.flags|=4;if(c.gpsRelativeValid){r.gpsRelativeM=c.gpsRelativeM;r.flags|=8;}}
        r.crc=barolog::crc32(&r,offsetof(barolog::Record,crc));
        if(queued<256 && self->count+queued<barolog::CAPACITY) {
          if(overflow){r.reserved=1;r.crc=barolog::crc32(&r,offsetof(barolog::Record,crc));overflow=false;}
          self->buffered[(head+queued)%256]=r;++queued;
        } else overflow=true;
        nextAt+=barolog::PERIOD_MS;
        if(int32_t(millis()-nextAt)>=0)nextAt=millis()+barolog::PERIOD_MS;
      }
      // Do not touch flash while the launch button is pressed / the timer is armed.
      // Store samples in RAM, then drain after PWM has already been started.
      if(!c.pauseWrites && queued) {
        auto &r=self->buffered[head];
        size_t offset=self->slot*barolog::SLOT_BYTES+sizeof(barolog::Header)+self->count*sizeof(r);
        if(esp_partition_write(self->partition,offset,&r,sizeof(r))!=ESP_OK){self->failed=true;break;}
        ++self->count;head=(head+1)%256;--queued;
      }
      int32_t wait=int32_t(nextAt-millis());
      ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(!c.pauseWrites && queued?2:wait>0?wait:1));
    }
    // Keep the task handle valid for late notifications after capacity/error.
    for(;;)ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
  }
public:
  uint32_t records()const{return count.load();}
  uint8_t status()const{return failed?3:count>=barolog::CAPACITY?2:partition?1:0;}
  void begin(Barometer &baro) {
    uint32_t started=millis();barometer=&baro;
    partition=esp_partition_find_first(ESP_PARTITION_TYPE_DATA,esp_partition_subtype_t(0x40),"barolog");
    if(!partition || partition->size<2*barolog::SLOT_BYTES){failed=true;return;}
    int previous=latestFlight();slot=previous==0?1:0;
    uint32_t seq=0;
    for(uint32_t i=0;i<2;i++){barolog::Header h;if(readHeader(i,h) && (!seq || int32_t(h.sequence-seq)>0))seq=h.sequence;}
    // Prepare the other slot before arming; leave the last flight untouched.
    if(esp_partition_erase_range(partition,slot*barolog::SLOT_BYTES,barolog::SLOT_BYTES)!=ESP_OK){failed=true;return;}
    header.sequence=seq+1;header.bootId=esp_random();header.startedMs=started;
    header.crc=barolog::crc32(&header,offsetof(barolog::Header,crc));
    if(esp_partition_write(partition,slot*barolog::SLOT_BYTES,&header,sizeof(header))!=ESP_OK){failed=true;return;}
    contextQueue=xQueueCreate(1,sizeof(BaroContext));BaroContext c;
    if(!contextQueue){failed=true;return;}xQueueOverwrite(contextQueue,&c);
    if(xTaskCreate(task,"barolog",4096,this,1,&worker)!=pdPASS)failed=true;
  }
  void markFlightStarted(uint32_t at){requestedStart=at;if(worker)xTaskNotifyGive(worker);}
  void update(const BaroContext &c){if(contextQueue)xQueueOverwrite(contextQueue,&c);}
  void serial(bool safe) {
    while(Serial.available()) {
      char ch=Serial.read();if(ch=='\r')continue;
      if(ch!='\n'){if(command.length()<48)command+=ch;continue;}
      String cmd=command;command="";cmd.trim();
      if(!cmd.startsWith("BARO"))continue;
      if(!safe){Serial.println("BARO export refused: armed or flying");continue;}
      int selected=latestFlight();
      if(cmd=="BARO LIST") {
        Serial.printf("# last_flight=%s draft_records=%lu capacity=%lu status=%s\n",selected>=0?"available":"none",(unsigned long)count.load(),(unsigned long)barolog::CAPACITY,failed?"error":count==barolog::CAPACITY?"full":promoted?"recording flight":"recording preflight");
      } else if(cmd=="BARO CSV") {
        if(selected<0){Serial.println("# no recorded flight");continue;}
        barolog::Header h;if(!readHeader(selected,h))continue;
        Serial.printf("# barolog_v2 boot=%08lx started_ms=%lu motor_start_ms=%lu\n",(unsigned long)h.bootId,(unsigned long)h.startedMs,(unsigned long)h.reserved[0]);
        Serial.println("uptime_ms,pressure_pa,baro_relative_m,gps_msl_m,gps_relative_m,sensor_age_ms,flags,state,reason");
        for(uint32_t n=0;n<barolog::CAPACITY;n++) {
          barolog::Record r;
          if(esp_partition_read(partition,selected*barolog::SLOT_BYTES+sizeof(h)+n*sizeof(r),&r,sizeof(r))!=ESP_OK || !barolog::valid(r))break;
          Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%lu,%u,%u,%u\n",(unsigned long)r.uptimeMs,r.pressurePa,r.baroRelativeM,r.gpsMslM,r.gpsRelativeM,(unsigned long)r.sensorAgeMs,r.flags,r.state,r.reason);
        }
        Serial.println("# end");
      } else Serial.println("# BARO LIST | BARO CSV");
    }
  }
};
#endif
