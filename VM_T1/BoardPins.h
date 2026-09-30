// SPDX-License-Identifier: Apache-2.0
#pragma once
#if !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "This F1Q branch requires ESP32C3 Dev Module, not ESP32-S3."
#endif
constexpr int PIN_SERVO_S=4, PIN_SERVO_R=5, PIN_SERVO_W=6;
constexpr int PIN_ESC=3, PIN_ARM=0, PIN_START=1;
constexpr int PIN_RC_DT=7, PIN_OFF_BUTTON=20, PIN_BUZZER=21;
constexpr int PIN_LED=8, PIN_SDA=2, PIN_SCL=10;
// GPIO8 onboard blue LED is active LOW. GPIO9/BOOT stays unused.
inline void statusLed(bool on) { digitalWrite(PIN_LED,on ? LOW : HIGH); }
