#pragma once
#include <stdint.h>
#include <string.h>
#include <string>
#include <deque>
#include <cstdarg>
#include <cmath>
inline uint32_t fakeNow=0;
inline bool fakeButton=false,fakeDt=false,fakeSensor=true,fakeReady=true;
inline uint16_t fakeBus=9600, fakeShunt=1000;
constexpr int INPUT_PULLUP=0,LOW=0,SERIAL_8N1=0,WIFI_STA=0;
inline uint32_t millis(){return fakeNow;}
inline uint32_t esp_random(){static uint32_t id=0;return ++id;}
inline void delay(int){}
inline void pinMode(int,int){}
inline int digitalRead(int pin){return (pin==7?fakeButton:fakeDt)?0:1;}
struct SerialMock {
  void begin(int,int=0,int=0,int=0){}
  void printf(const char*,...){}
  void println(const char*){}
  int available(){return 0;}
  int read(){return 0;}
};
inline SerialMock Serial,Serial1;
struct StringMock { const char *c_str(){return "00:00:00:00:00:00";} };
struct WifiMock { void mode(int){} void setSleep(bool){} StringMock macAddress(){return {};} };
inline WifiMock WiFi;
class Servo {
 public:
  int pulse=0; bool active=false;
  void setPeriodHertz(int){}
  void attach(int,int,int){active=true;}
  bool attached(){return active;}
  void detach(){active=false;}
  void writeMicroseconds(int p){pulse=p;}
};
struct Preferences {
  bool begin(const char*,bool){return true;} void end(){}
  size_t getBytesLength(const char*){return 0;}
  size_t getBytes(const char*,void*,size_t){return 0;}
  size_t putBytes(const char*,const void*,size_t n){return n;}
};
struct WireMock {
  uint8_t reg=0; bool first=true; uint16_t value=0; int idx=0;
  void begin(int,int){} void setClock(int){} void setTimeOut(int){}
  void beginTransmission(int){first=true;}
  void write(uint8_t b){if(first){reg=b;first=false;}}
  int endTransmission(bool=true){return fakeSensor?0:1;}
  int requestFrom(uint8_t,uint8_t){value=reg==6?(fakeReady?8:0):reg==1?fakeShunt:fakeBus;idx=0;return 2;}
  int read(){return idx++==0?value>>8:value&255;}
};
inline WireMock Wire;
struct TinyGPSPlus {
  struct Altitude {bool valid=false;float value=0;uint32_t ageMs=0;bool isValid(){return valid;}float meters(){return value;}uint32_t age(){return ageMs;}} altitude;
  struct Location {bool isValid(){return false;}double lat(){return 0;}double lng(){return 0;}uint32_t age(){return 0;}} location;
  struct Satellites {bool isValid(){return false;}uint32_t value(){return 0;}} satellites;
  void encode(int){}
};
