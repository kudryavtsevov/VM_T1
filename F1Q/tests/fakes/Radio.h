#pragma once
#include "Hardware.h"
#include "../../common/Model.h"
inline uint8_t GROUND_MAC[6]={};
namespace radio {
inline std::deque<f1q::Packet> queue;
inline bool begin(const uint8_t*){return true;}
inline bool read(f1q::Packet &p){if(queue.empty())return false;p=queue.front();queue.pop_front();return true;}
inline bool send(f1q::Packet&){return true;}
}
