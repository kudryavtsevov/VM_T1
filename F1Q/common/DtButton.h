// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdint.h>
namespace f1q {
// Driven by the dedicated ground task. No dependency on HTTP or filesystem.
class DtButton {
  bool raw=false,stable=false,running=false;
  uint32_t changedAt=0,until=0,lastSend=0;
public:
  void begin(uint32_t now,bool pressed){raw=stable=pressed;changedAt=now;running=false;}
  void request(uint32_t now){running=true;until=now+2000;lastSend=now-100;}
  bool poll(uint32_t now,bool pressed,bool webRequest=false){
    if(pressed!=raw){raw=pressed;changedAt=now;}
    if(raw!=stable && now-changedAt>=20){stable=raw;if(stable)request(now);}
    if(webRequest)request(now);
    if(running && int32_t(until-now)<=0)running=false;
    if(running && now-lastSend>=100){lastSend=now;return true;}
    return false;
  }
  bool active()const{return running;}
};
}
