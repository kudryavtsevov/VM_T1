// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <atomic>
#include <freertos/task.h>
#include "../common/DtButton.h"
// A higher-priority task keeps the physical button independent of blocking HTTP
// clients, CSV downloads and log writes. The radio link itself must be available.
class PhysicalDt {
  TaskHandle_t worker=nullptr;
  std::atomic<bool> webRequest{false},transmitting{false};
  static void task(void *arg){
    auto self=static_cast<PhysicalDt*>(arg);
    f1q::DtButton button;
    button.begin(millis(),digitalRead(PIN)==LOW);
    TickType_t wake=xTaskGetTickCount();
    for(;;){
      if(button.poll(millis(),digitalRead(PIN)==LOW,self->webRequest.exchange(false))){
        f1q::Packet p;p.kind=f1q::DT;radio::send(p);
      }
      self->transmitting=button.active();
      vTaskDelayUntil(&wake,pdMS_TO_TICKS(5));
    }
  }
public:
  static constexpr int PIN=7;
  bool begin(){
    pinMode(PIN,INPUT_PULLUP);
    return xTaskCreate(task,"ground_dt",3072,this,2,&worker)==pdPASS;
  }
  void request(){webRequest=true;}
  bool active()const{return transmitting.load();}
};
