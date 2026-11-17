// SPDX-License-Identifier: Apache-2.0
#pragma once
#ifdef F1Q_HOST_TEST
#include "../tests/fakes/Radio.h"
#else
#include <WiFi.h>
#include <atomic>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "Model.h"
#include "Pairing.h"
namespace radio {
static QueueHandle_t inbox;
static uint8_t peerMac[6];
static bool ready = false;
static std::atomic<uint32_t> sequence{0};
static void receive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(f1q::Packet) || memcmp(mac,peerMac,6)) return;
  f1q::Packet p;
  memcpy(&p,data,sizeof(p));
  if (p.magic != f1q::MAGIC || p.version != f1q::VERSION || p.bytes != sizeof(p)) return;
  // No hardware, flash writes, or JSON in Wi-Fi callback.
  xQueueSend(inbox,&p,0);
}
inline bool nonzero(const uint8_t *v, size_t n) {
  uint8_t r=0; for(size_t i=0;i<n;i++) r|=v[i]; return r;
}
inline bool begin(const uint8_t *peer) {
  if (!nonzero(peer,6) || !nonzero(RADIO_PMK,16) || !nonzero(RADIO_LMK,16)) return false;
  memcpy(peerMac,peer,6);
  inbox=xQueueCreate(6,sizeof(f1q::Packet));
  if (!inbox || esp_wifi_set_channel(f1q::CHANNEL,WIFI_SECOND_CHAN_NONE)!=ESP_OK ||
      esp_now_init()!=ESP_OK || esp_now_set_pmk(RADIO_PMK)!=ESP_OK) return false;
  esp_now_peer_info_t info={};
  memcpy(info.peer_addr,peer,6); memcpy(info.lmk,RADIO_LMK,16);
  info.channel=f1q::CHANNEL; info.ifidx=WIFI_IF_STA; info.encrypt=true;
  if (esp_now_add_peer(&info)!=ESP_OK) return false;
  ready=esp_now_register_recv_cb(receive)==ESP_OK;
  return ready;
}
inline bool read(f1q::Packet &p) { return ready && xQueueReceive(inbox,&p,0)==pdTRUE; }
inline bool send(f1q::Packet &p) {
  p.sequence=++sequence;
  return ready && esp_now_send(peerMac,reinterpret_cast<uint8_t*>(&p),sizeof(p))==ESP_OK;
}
}

#endif
