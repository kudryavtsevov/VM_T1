// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <LittleFS.h>
#include "../common/Model.h"
// All filesystem operations run in the main loop, never in a Wi-Fi callback.
class FlightLog {
  File file;
  uint32_t currentId=0, lastId=0, lastFlush=0;
public:
  bool mounted=false;
  String error;
  void begin() {
    mounted=LittleFS.begin(false);
    if(!mounted){error="Память журнала не подготовлена или не монтируется";return;}
    File previous=LittleFS.open("/flights/latest.csv",FILE_READ);
    if(previous){String line=previous.readStringUntil('\n');int p=line.indexOf("flight_id=");if(p>=0)lastId=strtoul(line.c_str()+p+10,nullptr,16);previous.close();}
  }
  void close() {if(file){file.flush();file.close();}currentId=0;}
  bool prepare() {
    if(mounted)return true; // Never erase an already mounted log volume.
    if(!LittleFS.format() || !LittleFS.begin(false))return false;
    mounted=true;error="";return true;
  }
  void flush(uint32_t now) {if(file && now-lastFlush>=1000){file.flush();lastFlush=now;}}
  void record(const f1q::Packet &p,uint32_t now) {
    const auto &t=p.telemetry;
    const bool flying=t.state==f1q::MOTOR || t.state==f1q::GLIDE;
    if(!mounted || !t.flightId)return;
    if(!file && !flying)return;
    if(currentId!=t.flightId) {
      close();if(!flying)return;
      if(!LittleFS.exists("/flights"))LittleFS.mkdir("/flights");
      file=LittleFS.open("/flights/latest.csv",t.flightId==lastId?FILE_APPEND:FILE_WRITE);
      lastId=t.flightId;
      if(!file){error="Не удалось открыть журнал";return;}
      currentId=t.flightId;
      if(!file.size()) {
        file.printf("# flight_id=%08lx,schema=2,revision=%lu,shunt_ohm=%.7f,motor_ms=%lu,energy_limit_j=%.3f\n",
          (unsigned long)t.flightId,(unsigned long)t.appliedRevision,t.config.shuntOhm,(unsigned long)t.config.motorMs,t.config.energyJ);
        file.println("rx_uptime_ms,packet_sequence,flight_id,flight_ms,voltage_v,current_a,energy_j,latitude,longitude,gps_fix_age_ms,gps_msl_m,gps_relative_m,gps_altitude_age_ms,baro_relative_m,pressure_pa,baro_age_ms,gps_fix_valid,gps_altitude_valid,gps_relative_valid,baro_valid,sensor_ok,satellites,state,reason");
      }
    }
    if(LittleFS.totalBytes()-LittleFS.usedBytes()<16384){error="Журнал заполнен: скачайте и удалите latest.csv";close();return;}
    char row[512];
    int n=snprintf(row,sizeof(row),"%lu,%lu,%08lx,%lu,%.3f,%.3f,%.3f,%.7f,%.7f,%lu,%.2f,%.2f,%lu,%.2f,%.2f,%lu,%u,%u,%u,%u,%u,%u,%u,%u\n",
      (unsigned long)now,(unsigned long)p.sequence,(unsigned long)t.flightId,(unsigned long)t.flightMs,
      t.voltageV,t.currentA,t.energyJ,t.latitude,t.longitude,(unsigned long)t.fixAgeMs,
      t.gpsMslM,t.gpsRelativeM,(unsigned long)t.altitudeAgeMs,t.baroRelativeM,t.pressurePa,(unsigned long)t.baroAgeMs,
      t.gpsValid,t.gpsAltitudeValid,t.gpsRelativeValid,t.baroValid,t.sensorOk,t.satellites,t.state,t.reason);
    if(n<=0 || n>=int(sizeof(row)) || file.write((const uint8_t*)row,n)!=size_t(n)) {error="Ошибка записи журнала";close();return;}
    flush(now);
    if(!flying)close();
  }
};
