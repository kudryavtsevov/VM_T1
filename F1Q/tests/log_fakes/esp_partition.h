#pragma once
#include <vector>
#include <functional>
#include <stdexcept>
#include <cstring>
#include <cassert>
using QueueHandle_t=struct FakeQueue*;
using TaskHandle_t=void*;
constexpr int pdTRUE=1,pdPASS=1,ESP_OK=0,ESP_PARTITION_TYPE_DATA=1;
constexpr uint32_t portMAX_DELAY=UINT32_MAX;
#define pdMS_TO_TICKS(x) (x)
struct FakeQueue {std::vector<uint8_t> bytes;};
inline QueueHandle_t xQueueCreate(int,size_t bytes){return new FakeQueue{std::vector<uint8_t>(bytes)};}
inline int xQueuePeek(QueueHandle_t q,void*p,int){memcpy(p,q->bytes.data(),q->bytes.size());return 1;}
inline int xQueueOverwrite(QueueHandle_t q,const void*p){memcpy(q->bytes.data(),p,q->bytes.size());return 1;}
inline void (*fakeTask)(void*)=nullptr;
inline void *fakeArg=nullptr;
inline int xTaskCreate(void(*fn)(void*),const char*,int,void*arg,int,TaskHandle_t*handle){fakeTask=fn;fakeArg=arg;*handle=reinterpret_cast<void*>(1);return pdPASS;}
inline void xTaskNotifyGive(TaskHandle_t){}
inline std::function<void()> fakeTaskHook;
struct StopTask{};
inline uint32_t ulTaskNotifyTake(int,uint32_t wait){fakeNow+=wait;if(fakeTaskHook)fakeTaskHook();return 1;}
using esp_partition_subtype_t=int;
struct esp_partition_t{size_t size;};
inline esp_partition_t fakePartition{0x160000};
inline std::vector<uint8_t> flash(fakePartition.size,255);
inline unsigned erases=0,programs=0;
inline bool launchButtonHeld=false,pwmStarted=false;
inline const esp_partition_t* esp_partition_find_first(int,int,const char*){return &fakePartition;}
inline int esp_partition_read(const esp_partition_t*,size_t at,void*p,size_t bytes){assert(at+bytes<=flash.size());memcpy(p,flash.data()+at,bytes);return ESP_OK;}
inline int esp_partition_write(const esp_partition_t*,size_t at,const void*p,size_t bytes){
  assert(!launchButtonHeld);assert(at+bytes<=flash.size());
  if(at%barolog::SLOT_BYTES==offsetof(barolog::Header,reserved))assert(pwmStarted);
  auto b=static_cast<const uint8_t*>(p);
  for(size_t i=0;i<bytes;i++){assert((flash[at+i]&b[i])==b[i]);flash[at+i]&=b[i];}
  ++programs;return ESP_OK;
}
inline int esp_partition_erase_range(const esp_partition_t*,size_t at,size_t bytes){
  assert(!launchButtonHeld);assert(at%4096==0 && bytes%4096==0);memset(flash.data()+at,255,bytes);++erases;return ESP_OK;
}
class String {
  std::string s;
public:
  String()=default;String(const char*v):s(v){}
  size_t length()const{return s.size();}
  String& operator+=(char c){s+=c;return *this;}
  void trim(){while(!s.empty() && isspace(s.back()))s.pop_back();while(!s.empty()&&isspace(s.front()))s.erase(s.begin());}
  bool startsWith(const char*p)const{return s.rfind(p,0)==0;}
  bool operator==(const char*p)const{return s==p;}
};
