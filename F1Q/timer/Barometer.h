// SPDX-License-Identifier: Apache-2.0
#pragma once
#ifdef F1Q_HOST_TEST
struct Barometer {
  bool valid=false; uint32_t sampledAt=0; float pressurePa=0;
  void begin(){} void poll(uint32_t now){if(valid)sampledAt=now;}
  bool snapshot(float &p,uint32_t &at){p=pressurePa;at=sampledAt;return valid;}
};
#else
#include <Adafruit_BMP3XX.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
// Conversion waits run in a background task. Wire in pinned Arduino Core 2.0.11
// serializes individual I2C transactions; no bus lock is held during conversion.
class Barometer {
  Adafruit_BMP3XX sensor;
  struct Reading {float pressure;uint32_t at;bool ok;};
  std::atomic<QueueHandle_t> readings{nullptr};
  static void task(void *arg) {
    auto self=static_cast<Barometer*>(arg);
    for(;;) {
      Reading r;
      r.ok=self->sensor.performReading();
      r.pressure=self->sensor.pressure;r.at=millis();
      r.ok=r.ok && isfinite(r.pressure) && r.pressure>30000 && r.pressure<125000;
      xQueueOverwrite(self->readings,&r);
      vTaskDelay(pdMS_TO_TICKS(75));
    }
  }
public:
  bool valid=false;
  uint32_t sampledAt=0;
  float pressurePa=0;
  void begin() {
    if(!(sensor.begin_I2C(0x76,&Wire) || sensor.begin_I2C(0x77,&Wire)))return;
    sensor.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    sensor.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    sensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    sensor.setOutputDataRate(BMP3_ODR_25_HZ);
    readings=xQueueCreate(1,sizeof(Reading));
    if(readings && xTaskCreate(task,"barometer",4096,this,1,nullptr)!=pdPASS) {
      vQueueDelete(readings);readings=nullptr;
    }
  }
  bool snapshot(float &p,uint32_t &at) {
    Reading r;
    if(!readings || xQueuePeek(readings,&r,0)!=pdTRUE || !r.ok)return false;
    p=r.pressure;at=r.at;return true;
  }
  void poll(uint32_t) {
    Reading r;
    if(readings && xQueuePeek(readings,&r,0)==pdTRUE) {
      valid=r.ok;if(valid){pressurePa=r.pressure;sampledAt=r.at;}
    }
  }
};
#endif
