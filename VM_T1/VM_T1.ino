/*
 * Copyright 2026 Мстислав
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * 2026-09-24: Added author attribution and license notice;
 * original import subsequently extended; see docs/F1Q.md.
 * 2026-09-25: Added optional INA226/BMP280 sensing and experimental F1Q control.
 * Existing waits now service the limiter; corrected the final DT delay.
 */

// VM-T1 F1Q: ESP32-C3 Super Mini, Arduino ESP32 Core 2.0.11.
// Pin assignments are shared with the carrier documentation in BoardPins.h.

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <ArduinoJson.h>

void servicedDelay(uint32_t ms);

#include "BoardPins.h"

// ================== КОНСТАНТЫ ==================
const uint32_t DEBOUNCE_MS       = 50;
const uint32_t WIFI_ON_MS        = 3UL * 60UL * 1000UL;
const uint8_t  NUM_JUMPS         = 5;
const uint32_t WIFI_PULSE_WINDOW = 2000;
const bool     DEBUG             = true;

// ================== ОБЪЕКТЫ ==================
Servo servo[3];
WebServer server(80);
Preferences prefs;

// ================== СТРУКТУРЫ ==================
struct Jump {
  uint16_t timeMs;
  float    s[3];
  uint8_t  enabled;
};

struct Config {
  char ssid[33]     = "VM-T1";
  char password[33] = "12345678";

  uint8_t hookBackInvert  = 0;
  uint8_t hookFrontInvert = 0;

  uint16_t servoMinUs = 500;
  uint16_t servoMaxUs = 2400;

  uint8_t servoEnabled[3] = {1, 0, 0};
  uint8_t reverseS = 0, reverseR = 0, reverseW = 0;
  float   zeroS    = 0.0f, zeroR = 0.0f, zeroW = 0.0f;
  uint8_t unlockZero = 0;

  float idleS    = 0.0f,   idleR    = 0.0f, idleW = 0.0f;
  float circleS  = 30.0f,  circleR  = 0.0f, circleW = 0.0f;
  float tensionS = 60.0f,  tensionR = 0.0f, tensionW = 0.0f;
  float readyS   = 90.0f,  readyR   = 0.0f, readyW = 0.0f;
  float dtS      = 180.0f, dtR      = 0.0f, dtW = 0.0f;

  // RC DT
  uint8_t rcDtActive  = 1;       // 1 = кнопка на GND (INPUT_PULLUP, нажатие = LOW)
  float   rcDtS       = 0.0f;    // угол Servo S при RC DT
  float   rcDtR       = 0.0f;
  float   rcDtW       = 0.0f;

  Jump jump[NUM_JUMPS] = {
    { .timeMs = 2000, .s = {0,   0, 0}, .enabled = 1 },
    { .timeMs = 2000, .s = {45,  0, 0}, .enabled = 1 },
    { .timeMs = 2000, .s = {90,  0, 0}, .enabled = 1 },
    { .timeMs = 2000, .s = {135, 0, 0}, .enabled = 1 },
    { .timeMs = 2000, .s = {180, 0, 0}, .enabled = 1 }
  };

  uint16_t dtExtraS     = 10;
  uint16_t emergencyDtS = 30;

  uint8_t  rebootPulses   = 3;
  uint16_t rebootWindowMs = 3000;
  uint8_t  melodyIndex    = 7;
  uint8_t  melodyOnBoot   = 6;

  uint8_t  soundCircling   = 1;
  uint8_t  soundTension    = 0;
  uint8_t  soundReady      = 2;
  uint8_t  soundStart      = 1;
  uint8_t  findBeepEnabled = 1;

  uint8_t  skipBootMelody  = 0;
} cfg;

// ================== СОСТОЯНИЯ ==================
enum State { ST_IDLE, ST_CIRCLING, ST_TENSION, ST_READY, ST_WORK, ST_SLEEP_OFF };
State state = ST_IDLE;

uint32_t stateEnteredMs    = 0;
uint32_t lastBeepMs        = 0;
uint32_t lastSecBeepMs     = 0;
uint32_t offButtonPressMs  = 0;
uint8_t  offButtonCount    = 0;

int8_t   activeList[NUM_JUMPS + 1];
uint8_t  activeCount = 0;
uint8_t  currentSlot = 0;
uint32_t sectionStartMs = 0;
bool     dtReached      = false;

uint8_t  frontHookCount     = 0;
bool     lastHookFrontState = false;
uint32_t lastFrontMs        = 0;
uint32_t lastFrontRawMs     = 0;
bool     rebootFirstRun     = true;

uint8_t  backPulseCount     = 0;
bool     lastBackState      = false;
uint32_t lastBackPulseMs    = 0;
uint32_t lastBackRawMs      = 0;
bool     wifiActive         = false;
uint32_t wifiOnMs           = 0;

bool     rcDtWasPressed     = false;   // для отслеживания отпускания

// ================== ИМЯ СОСТОЯНИЯ ==================
String stateName(State s) {
  switch (s) {
    case ST_IDLE:      return "Ожидание";
    case ST_CIRCLING:  return "Кружение";
    case ST_TENSION:   return "Затяжка";
    case ST_READY:     return "Готовность";
    case ST_WORK:      return "Старт/полёт";
    case ST_SLEEP_OFF: return "SLEEP_OFF";
  }
  return "?";
}

// ================== ЗУММЕР ==================
void buzzerOn()  { digitalWrite(PIN_BUZZER, HIGH); }
void buzzerOff() { digitalWrite(PIN_BUZZER, LOW);  }

void beep(uint16_t ms) {
  buzzerOn();
  servicedDelay(ms);
  buzzerOff();
}

void duckQuack() {
  beep(80);  servicedDelay(60);
  beep(80);  servicedDelay(60);
  beep(800); servicedDelay(200);
  beep(80);  servicedDelay(60);
  beep(80);
}

// ================== МЕЛОДИИ ==================
const uint16_t D1[] = {120, 80, 120, 80, 120, 80, 400, 200};
const uint8_t  P1[] = {1,   0,  1,   0,  1,   0,  1,   0};
const uint16_t D2[] = {100, 50, 100, 50, 100, 50, 100, 50, 250, 200};
const uint8_t  P2[] = {1,   0,  1,   0,  1,   0,  1,   0,  1,   0};
const uint16_t D3[] = {220, 80, 220, 80, 220, 80, 150, 60, 100, 60, 250, 80};
const uint8_t  P3[] = {1,   0,  1,   0,  1,   0,  1,   0,  1,   0,  1,   0};
const uint16_t D4[] = {140, 60, 140, 60, 140, 60, 400, 200};
const uint8_t  P4[] = {1,   0,  1,   0,  1,   0,  1,   0};
const uint16_t D5[] = {130, 70, 450, 200};
const uint8_t  P5[] = {1,   0,  1,   0};
const uint16_t D6[] = {50, 30, 50, 30, 50, 30, 50, 30, 50, 30, 50, 30, 50, 30};
const uint8_t  P6[] = {1,  0,  1,  0,  1,  0,  1,  0,  1,  0,  1,  0,  1,  0};
const uint16_t D7[] = {400, 200, 400, 200, 800, 400};
const uint8_t  P7[] = {1,   0,   1,   0,   1,   0};

void playMelody(uint8_t idx) {
  const uint16_t *dur = nullptr;
  const uint8_t  *pat = nullptr;
  size_t len = 0;
  switch (idx) {
    case 1: dur = D1; pat = P1; len = sizeof(D1)/sizeof(uint16_t); break;
    case 2: dur = D2; pat = P2; len = sizeof(D2)/sizeof(uint16_t); break;
    case 3: dur = D3; pat = P3; len = sizeof(D3)/sizeof(uint16_t); break;
    case 4: dur = D4; pat = P4; len = sizeof(D4)/sizeof(uint16_t); break;
    case 5: dur = D5; pat = P5; len = sizeof(D5)/sizeof(uint16_t); break;
    case 6: dur = D6; pat = P6; len = sizeof(D6)/sizeof(uint16_t); break;
    case 7: dur = D7; pat = P7; len = sizeof(D7)/sizeof(uint16_t); break;
    default: return;
  }
  for (size_t i = 0; i < len; i++) {
    if (pat[i]) { buzzerOn(); servicedDelay(dur[i]); buzzerOff(); }
    else        { servicedDelay(dur[i]); }
  }
}

// ================== УТИЛИТЫ ==================
bool hookBackClosed() { return false; } // no tow hooks on F1Q
bool hookFrontClosed() { return false; }
bool rcDtPressed() {
  bool v = digitalRead(PIN_RC_DT) == LOW;
  return cfg.rcDtActive ? v : !v;
}

#include "F1QExtension.h"
#include "F1QPage.h"

void setServo(uint8_t idx, float cmdDeg) {
  if (idx > 2) return;
  if (!cfg.servoEnabled[idx]) return;

  float zero  = (idx == 0) ? cfg.zeroS  : (idx == 1 ? cfg.zeroR  : cfg.zeroW);
  uint8_t rev = (idx == 0) ? cfg.reverseS : (idx == 1 ? cfg.reverseR : cfg.reverseW);

  float actual = rev ? (zero - cmdDeg) : (zero + cmdDeg);
  if (actual < 0)   actual = 0;
  if (actual > 180) actual = 180;

  int us = cfg.servoMinUs + (int)((cfg.servoMaxUs - cfg.servoMinUs) * actual / 180.0f);
  servo[idx].writeMicroseconds(us);
}

void applyAngles(float s, float r, float w) {
  setServo(0, s);
  setServo(1, r);
  setServo(2, w);
}

void attachServos() {
  for (uint8_t i = 0; i < 3; i++) {
    if (!cfg.servoEnabled[i]) continue;
    if (!servo[i].attached()) {
      servo[i].setPeriodHertz(50);
      servo[i].attach(i == 0 ? PIN_SERVO_S : (i == 1 ? PIN_SERVO_R : PIN_SERVO_W),
                      cfg.servoMinUs, cfg.servoMaxUs);
      if (DEBUG) Serial.printf("[SERVO] Servo %u attached on pin %u\n", i,
                               i == 0 ? PIN_SERVO_S : (i == 1 ? PIN_SERVO_R : PIN_SERVO_W));
    }
  }
}

float readVbat() {
  return flightSensors.fresh(millis()) ? flightSensors.voltage : NAN;
}

// ================== ПАМЯТЬ ==================
void loadConfig() {
  prefs.begin("vmt1-c3", true);
  size_t sz = prefs.getBytesLength("cfg");
  if (sz == sizeof(cfg)) {
    prefs.getBytes("cfg", &cfg, sizeof(cfg));
    if (DEBUG) Serial.println("[CFG] Loaded from Preferences");
  } else {
    if (DEBUG) Serial.println("[CFG] Using defaults");
  }
  prefs.end();
}
void saveConfig() {
  prefs.begin("vmt1-c3", false);
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
}

// ================== Wi-Fi ==================
void wifiEnable() {
  if (f1q.enabled && (digitalRead(PIN_ARM)==LOW || flightBusy())) return;
  if (wifiActive) { wifiOnMs = millis(); return; }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(cfg.ssid, cfg.password);
  server.begin();
  wifiActive = true;
  wifiOnMs = millis();
  duckQuack();
  if (DEBUG) Serial.println("[WIFI] Enabled for 3 min");
}
void wifiDisable() {
  if (!wifiActive) return;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiActive = false;
  if (DEBUG) Serial.println("[WIFI] Disabled");
}

// ================== ПРОГРАММА ==================
uint32_t totalProgramMs() {
  uint32_t t = 0;
  for (uint8_t i = 0; i < NUM_JUMPS; i++)
    if (cfg.jump[i].enabled) t += cfg.jump[i].timeMs;
  return t;
}

void runProgram() {
  if (currentSlot < activeCount) {
    int8_t sectionId = activeList[currentSlot];

    if (sectionId < 10) {
      const Jump &j = cfg.jump[sectionId];
      uint32_t elapsed = millis() - sectionStartMs;

      if (elapsed >= j.timeMs) {
        applyAngles(j.s[0], j.s[1], j.s[2]);
        if (DEBUG) {
          float zero = cfg.zeroS;
          uint8_t rev = cfg.reverseS;
          float actual = rev ? (zero - j.s[0]) : (zero + j.s[0]);
          if (actual < 0) actual = 0;
          if (actual > 180) actual = 180;
          int us = cfg.servoMinUs + (int)((cfg.servoMaxUs - cfg.servoMinUs) * actual / 180.0f);
          Serial.printf("[JUMP] Jump%d done. cmdS=%.0f actualS=%.0f us=%d -> slot %u\n",
                        sectionId + 1, j.s[0], actual, us, currentSlot + 1);
        }
        currentSlot++;
        sectionStartMs = millis();
      }
    } else {
      uint32_t totalDtMs = (uint32_t)cfg.dtExtraS * 1000; // Jump delays have already elapsed.
      if (!dtReached && (millis() - sectionStartMs) >= totalDtMs) {
        dtReached = true;
        if (DEBUG) Serial.println("[DT] DT reached");
        applyAngles(cfg.dtS, cfg.dtR, cfg.dtW);
      }
    }
  }
}

// ================== СОСТОЯНИЯ ==================
void enterState(State s) {
  state = s;
  stateEnteredMs = millis();
  buzzerOff();
  attachServos();

  if (DEBUG) Serial.printf("[STATE] -> %s\n", stateName(s).c_str());

  switch (s) {
    case ST_IDLE:
      statusLed(false);
      applyAngles(cfg.idleS, cfg.idleR, cfg.idleW);
      break;
    case ST_CIRCLING:
      statusLed(false);
      applyAngles(cfg.circleS, cfg.circleR, cfg.circleW);
      break;
    case ST_TENSION:
      statusLed(false);
      applyAngles(cfg.tensionS, cfg.tensionR, cfg.tensionW);
      break;
    case ST_READY:
      statusLed(true);
      applyAngles(cfg.readyS, cfg.readyR, cfg.readyW);
      break;
    case ST_WORK:
      statusLed(false);
      activeCount = 0;
      for (uint8_t i = 0; i < NUM_JUMPS; i++)
        if (cfg.jump[i].enabled) activeList[activeCount++] = i;
      activeList[activeCount++] = 10;
      currentSlot = 0;
      sectionStartMs = millis();
      dtReached = false;
      if (DEBUG) {
        Serial.printf("[WORK] Started. %u active sections:\n", activeCount);
        for (uint8_t i = 0; i < activeCount; i++) {
          if (activeList[i] < 10)
            Serial.printf("   slot %u = Jump%u (time=%u ms, S=%.0f)\n",
                          i, activeList[i] + 1, cfg.jump[activeList[i]].timeMs,
                          cfg.jump[activeList[i]].s[0]);
          else
            Serial.printf("   slot %u = DT\n", i);
        }
      }
      break;
    case ST_SLEEP_OFF:
      statusLed(false);
      break;
  }
}

// ================== ЗВУКИ ==================
void handleStateSounds(uint32_t now) {
  switch (state) {
    case ST_IDLE:      buzzerOff(); break;
    case ST_CIRCLING:
      if (cfg.soundCircling == 1 && now - lastSecBeepMs >= 1000) {
        lastSecBeepMs = now;
        beep(60);
      }
      break;
    case ST_TENSION:   buzzerOff(); break;
    case ST_READY:     buzzerOn();  break;
    case ST_WORK:
      if (dtReached && cfg.findBeepEnabled) {
        if (now - lastSecBeepMs >= 1000) {
          lastSecBeepMs = now;
          beep(150);
        }
      } else {
        buzzerOff();
      }
      break;
    default: buzzerOff(); break;
  }
}

// ================== ПЕРЕЗАГРУЗКА ==================
// OFF also enables setup Wi-Fi when held 3s with ARM removed.
void checkWiFiCondition() {
  static uint32_t heldAt=0;
  static bool handled=false;
  if (wifiActive && uint32_t(millis()-wifiOnMs)>=WIFI_ON_MS) wifiDisable();
  if (digitalRead(PIN_ARM)!=LOW && digitalRead(PIN_OFF_BUTTON)==LOW && !flightBusy()) {
    if (!heldAt) heldAt=millis();
    if (!handled && uint32_t(millis()-heldAt)>=3000) { handled=true; wifiEnable(); }
  } else { heldAt=0; handled=false; }
}

// ================== RC DT ==================
void handleRcDt() {
  bool pressed = rcDtPressed();

  if (pressed && !rcDtWasPressed) {
    if (DEBUG) Serial.println("[RC-DT] Pressed -> apply RC DT angles");
    applyAngles(cfg.rcDtS, cfg.rcDtR, cfg.rcDtW);
  } else if (pressed && rcDtWasPressed) {
    // держим кнопку — постоянно держим углы (на случай если логика состояний перебьёт)
    applyAngles(cfg.rcDtS, cfg.rcDtR, cfg.rcDtW);
  } else if (!pressed && rcDtWasPressed) {
    if (DEBUG) Serial.println("[RC-DT] Released -> return to current state");
    // возвращаем углы текущего состояния
    if (state != ST_WORK) enterState(state); // Never restart the flight program after RC DT.
  }

  rcDtWasPressed = pressed;
}

// ================== ВЕБ ==================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>VM-T1</title>
<style>
body{font-family:Arial;background:#111;color:#eee;margin:0;padding:16px;font-size:14px}
h1{font-size:20px;color:#4fc3f7}
h2{font-size:16px;color:#81c784;margin-top:20px;border-bottom:1px solid #333;padding-bottom:4px}
label{display:block;margin:4px 0}
input,select{width:100%;padding:6px;background:#222;color:#eee;border:1px solid #444;border-radius:3px;box-sizing:border-box}
input:disabled{background:#151515;color:#555;border-color:#252525}
button{width:100%;padding:12px;margin-top:14px;background:#4fc3f7;color:#000;border:0;border-radius:4px;font-weight:bold;font-size:16px}
.status{background:#1e1e1e;padding:10px;border-radius:4px;margin-bottom:12px;font-family:monospace;white-space:pre-line;font-size:12px}
.row{display:flex;gap:6px}
.row>div{flex:1}
.section{background:#1a1a1a;padding:10px;border-radius:6px;margin:8px 0}
.section.off{opacity:0.45}
.section h3{margin:0 0 6px;font-size:14px;color:#ffb74d;display:flex;justify-content:space-between;align-items:center}
.srv{display:flex;gap:6px;margin:4px 0}
.srv label{width:60px;font-size:12px;line-height:28px;margin:0}
.srv input{flex:1}
.sum{background:#1a1a1a;padding:10px;border-radius:6px;margin:8px 0;font-family:monospace}
.sum b{color:#4fc3f7}
.lock{background:#1a1a1a;padding:8px;border-radius:4px;margin:6px 0}
</style></head><body>
<h1>VM-T1 F1Q / C3 — настройки</h1>
<div class="status" id="st">Загрузка...</div>
<p><a href="/f1q" style="color:#4fc3f7">INA226 / BMP280 / F1Q</a></p>
<form id="f">

<h2>Сервоприводы</h2>
<label><input type="checkbox" name="servoEnabled0" class="servoEnableCheck" style="width:auto"> Servo S (GPIO 4)</label>
<label><input type="checkbox" name="servoEnabled1" class="servoEnableCheck" style="width:auto"> Servo R (GPIO 5)</label>
<label><input type="checkbox" name="servoEnabled2" class="servoEnableCheck" style="width:auto"> Servo W (GPIO 6)</label>
<div class="row">
  <div><label>Servo Min µs</label><input name="servoMinUs" type="number"></div>
  <div><label>Servo Max µs</label><input name="servoMaxUs" type="number"></div>
</div>

<h2>RC DT (кнопка GPIO 7)</h2>
<label><input type="checkbox" name="rcDtActive" style="width:auto"> Активна (замыкает на GND)</label>
<div class="srv srv-row" data-srv="0"><label>S</label><input name="rcDtS" type="number" step="0.1"></div>
<div class="srv srv-row" data-srv="1"><label>R</label><input name="rcDtR" type="number" step="0.1"></div>
<div class="srv srv-row" data-srv="2"><label>W</label><input name="rcDtW" type="number" step="0.1"></div>

<h2>Нули и реверс</h2>
<div class="lock">
<label><input type="checkbox" name="unlockZero" id="unlockZero" style="width:auto"> Разрешить изменение нуля</label>
</div>
<div class="row srv-row" data-srv="0">
  <div><label>Zero S, °</label><input name="zeroS" type="number" step="0.1" class="zeroField"></div>
  <div><label>Reverse S</label><input type="checkbox" name="reverseS" style="width:auto"></div>
</div>
<div class="row srv-row" data-srv="1">
  <div><label>Zero R, °</label><input name="zeroR" type="number" step="0.1" class="zeroField"></div>
  <div><label>Reverse R</label><input type="checkbox" name="reverseR" style="width:auto"></div>
</div>
<div class="row srv-row" data-srv="2">
  <div><label>Zero W, °</label><input name="zeroW" type="number" step="0.1" class="zeroField"></div>
  <div><label>Reverse W</label><input type="checkbox" name="reverseW" style="width:auto"></div>
</div>

<h2>Состояния и звуки</h2>
<label>Старт (0=тихо, 1=мелодия)</label><input name="soundStart" type="number" min="0" max="1">

<h2>Углы серв в состояниях</h2>
<div class="section"><h3>Ожидание</h3>
  <div class="srv srv-row" data-srv="0"><label>S</label><input name="idleS" type="number" step="0.1"></div>
  <div class="srv srv-row" data-srv="1"><label>R</label><input name="idleR" type="number" step="0.1"></div>
  <div class="srv srv-row" data-srv="2"><label>W</label><input name="idleW" type="number" step="0.1"></div></div>
<div class="section"><h3>DT</h3>
  <div class="srv srv-row" data-srv="0"><label>S</label><input name="dtS" type="number" step="0.1"></div>
  <div class="srv srv-row" data-srv="1"><label>R</label><input name="dtR" type="number" step="0.1"></div>
  <div class="srv srv-row" data-srv="2"><label>W</label><input name="dtW" type="number" step="0.1"></div></div>

<h2>Jump 1–5</h2>
<div id="jumps"></div>

<h2>DT</h2>
<div class="sum">Сумма активных Jump: <b id="sumJg">0</b> мс</div>
<label>Доп. время, с</label><input name="dtExtraS" type="number" min="0" max="3600">
<div class="sum">DT_total: <b id="sumTotal">0</b> с</div>
<h2>Прочее</h2>
<label>Мелодия при включении</label>
<select name="melodyOnBoot">
  <option value="0">Нет</option><option value="1">Арпеджио</option>
  <option value="2">Mario</option><option value="3">Star Wars</option>
  <option value="4">Windows XP</option><option value="5">Та-да</option>
  <option value="6">Трещотка</option><option value="7">Победная</option>
</select>
<label><input type="checkbox" name="findBeepEnabled" style="width:auto"> Писк после DT</label>
<h2>Wi-Fi</h2>
<label>SSID</label><input name="ssid">
<label>Пароль</label><input name="password">
<p style="font-size:12px;color:#aaa">Wi-Fi включается на 3 минуты при включении с ARM снятым. Повторно: снять ARM и держать OFF 3 секунды.</p>

<button type="submit">Сохранить и перезагрузить</button>
</form>

<script>
function refreshServoVisibility(){
  const f = document.getElementById('f');
  for (let i = 0; i < 3; i++){
    const enabled = f.elements['servoEnabled'+i].checked;
    document.querySelectorAll('[data-srv="'+i+'"]').forEach(el=>{
      if (el.classList.contains('srv-row')) el.style.display = enabled ? '' : 'none';
    });
  }
}
function buildSections(id, prefix){
  const c = document.getElementById(id);
  for (let n = 1; n <= 5; n++){
    const d = document.createElement('div');
    d.className = 'section';
    d.innerHTML = '<h3><span>' + prefix + n + '</span>' +
      '<span><input type="checkbox" name="'+prefix.toLowerCase()+n+'_en" style="width:auto"> вкл</span></h3>' +
      '<div class="srv"><label>Время</label>' +
      '<input name="'+prefix.toLowerCase()+n+'_t" type="number" step="1" placeholder="мс"></div>' +
      ['S','R','W'].map((name,i)=>
        '<div class="srv srv-row" data-srv="'+i+'"><label>'+name+'</label>'+
        '<input name="'+prefix.toLowerCase()+n+'_p'+i+'" type="number" step="0.1" placeholder="°">'+
        '</div>').join('');
    c.appendChild(d);
  }
}
buildSections('jumps','Jump');

function refreshZeroLock(){
  const unlocked = document.getElementById('unlockZero').checked;
  document.querySelectorAll('.zeroField').forEach(el => el.disabled = !unlocked);
}
document.getElementById('unlockZero').addEventListener('change', refreshZeroLock);

function refreshSectionsAppearance(){
  const f = document.getElementById('f');
  for (let n = 1; n <= 5; n++){
    const en = f.elements['jump'+n+'_en'];
    if (!en) continue;
    const container = en.closest('.section');
    if (en.checked) container.classList.remove('off');
    else container.classList.add('off');
  }
}
function updateSum(){
  const f = document.getElementById('f');
  let sumMs = 0;
  for (let n = 1; n <= 5; n++){
    const en = f.elements['jump'+n+'_en'];
    if (!en || !en.checked) continue;
    const t = parseFloat(f.elements['jump'+n+'_t'].value) || 0;
    sumMs += t;
  }
  document.getElementById('sumJg').textContent = sumMs;
  const extra = parseFloat(f.elements['dtExtraS'].value) || 0;
  document.getElementById('sumTotal').textContent = (sumMs / 1000 + extra).toFixed(1);
}
document.getElementById('f').addEventListener('input', () => {
  refreshServoVisibility();
  refreshSectionsAppearance();
  updateSum();
});

async function load(){
  const r = await fetch('/api/config');
  const c = await r.json();
  const f = document.getElementById('f');
  const basicFields = [
    'soundCircling','soundTension','soundReady','soundStart',
    'dtExtraS','emergencyDtS','rebootPulses','rebootWindowMs',
    'melodyIndex','melodyOnBoot','servoMinUs','servoMaxUs',
    'hookBackInvert','hookFrontInvert','ssid','password',
    'zeroS','zeroR','zeroW',
    'idleS','idleR','idleW','circleS','circleR','circleW',
    'tensionS','tensionR','tensionW','readyS','readyR','readyW',
    'dtS','dtR','dtW',
    'rcDtS','rcDtR','rcDtW'
  ];
  for (const k of basicFields){
    if (f.elements[k] && c[k] !== undefined) f.elements[k].value = c[k];
  }
  for (const k of ['servoEnabled0','servoEnabled1','servoEnabled2','reverseS','reverseR','reverseW','unlockZero','findBeepEnabled','rcDtActive']){
    if (f.elements[k]) f.elements[k].checked = !!c[k];
  }
  for (let n = 1; n <= 5; n++){
    const enName = 'jump'+n+'_en';
    if (f.elements[enName]) f.elements[enName].checked = !!c.jump[n-1].enabled;
    const tName = 'jump'+n+'_t';
    if (f.elements[tName]) f.elements[tName].value = c.jump[n-1].timeMs;
    for (let i = 0; i < 3; i++){
      const pName = 'jump'+n+'_p'+i;
      if (f.elements[pName]) f.elements[pName].value = c.jump[n-1].s[i];
    }
  }
  refreshServoVisibility();
  refreshZeroLock();
  refreshSectionsAppearance();
  updateSum();
}

async function save(e){
  e.preventDefault();
  const f = document.getElementById('f');
  const data = {};
  for (const el of f.elements){
    if (!el.name) continue;
    if (el.type === 'checkbox') data[el.name] = el.checked;
    else if (el.type === 'number') data[el.name] = parseFloat(el.value);
    else data[el.name] = el.value;
  }
  data.jump = [];
  for (let n = 1; n <= 5; n++){
    const j = {timeMs: parseFloat(data['jump'+n+'_t']) || 0, s: [], enabled: f.elements['jump'+n+'_en'].checked ? 1 : 0};
    for (let i = 0; i < 3; i++){
      j.s.push(parseFloat(data['jump'+n+'_p'+i]) || 0);
    }
    data.jump.push(j);
  }
  await fetch('/api/config', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
  alert('Сохранено. Перезагрузка...');
}
document.getElementById('f').addEventListener('submit', save);

async function status(){
  try{
    const r = await fetch('/api/status');
    const s = await r.json();
    document.getElementById('st').textContent =
      'Состояние: ' + s.state +
      '\nRC DT: ' + (s.rcDt?'НАЖАТА':'отпущена') +
      '\nWi-Fi: ' + (s.wifiActive?'ВКЛ':'выкл') +
      '\nVBAT: ' + s.vbat + ' В' +
      '\nINA226: ' + (s.ina226 ? s.voltageV.toFixed(2)+' В / '+s.currentA.toFixed(3)+' А / '+s.powerW.toFixed(1)+' Вт' : 'нет данных') +
      '\nЭнергия: ' + s.energyJ.toFixed(1)+' / '+s.energyLimitJ.toFixed(1)+' Дж' +
      '\nВысота: ' + (s.bmp280 ? s.heightM.toFixed(1)+' м; максимум '+s.maxHeightM.toFixed(1)+' м' : 'нет данных') +
      '\nF1Q: ' + (s.f1qEnabled ? s.motorState+(s.armed?' / готов к отпусканию START':'') : 'выключен');
  }catch(e){}
}
load(); status(); setInterval(status, 1000);
</script>
</body></html>
)HTML";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleGetConfig() {
  JsonDocument doc;
  doc["soundCircling"]   = cfg.soundCircling;
  doc["soundTension"]    = cfg.soundTension;
  doc["soundReady"]      = cfg.soundReady;
  doc["soundStart"]      = cfg.soundStart;
  doc["dtExtraS"]        = cfg.dtExtraS;
  doc["emergencyDtS"]    = cfg.emergencyDtS;
  doc["rebootPulses"]    = cfg.rebootPulses;
  doc["rebootWindowMs"]  = cfg.rebootWindowMs;
  doc["melodyIndex"]     = cfg.melodyIndex;
  doc["melodyOnBoot"]    = cfg.melodyOnBoot;
  doc["findBeepEnabled"] = cfg.findBeepEnabled;
  doc["servoMinUs"]      = cfg.servoMinUs;
  doc["servoMaxUs"]      = cfg.servoMaxUs;
  doc["hookBackInvert"]  = cfg.hookBackInvert;
  doc["hookFrontInvert"] = cfg.hookFrontInvert;
  doc["ssid"]            = cfg.ssid;
  doc["password"]        = cfg.password;

  doc["servoEnabled0"] = cfg.servoEnabled[0];
  doc["servoEnabled1"] = cfg.servoEnabled[1];
  doc["servoEnabled2"] = cfg.servoEnabled[2];
  doc["reverseS"] = cfg.reverseS;
  doc["reverseR"] = cfg.reverseR;
  doc["reverseW"] = cfg.reverseW;
  doc["zeroS"]    = cfg.zeroS;
  doc["zeroR"]    = cfg.zeroR;
  doc["zeroW"]    = cfg.zeroW;
  doc["unlockZero"] = cfg.unlockZero;

  doc["rcDtActive"] = cfg.rcDtActive;
  doc["rcDtS"]      = cfg.rcDtS;
  doc["rcDtR"]      = cfg.rcDtR;
  doc["rcDtW"]      = cfg.rcDtW;

  doc["idleS"]    = cfg.idleS;    doc["idleR"]    = cfg.idleR;    doc["idleW"]    = cfg.idleW;
  doc["circleS"]  = cfg.circleS;  doc["circleR"]  = cfg.circleR;  doc["circleW"]  = cfg.circleW;
  doc["tensionS"] = cfg.tensionS; doc["tensionR"] = cfg.tensionR; doc["tensionW"] = cfg.tensionW;
  doc["readyS"]   = cfg.readyS;   doc["readyR"]   = cfg.readyR;   doc["readyW"]   = cfg.readyW;
  doc["dtS"]      = cfg.dtS;      doc["dtR"]      = cfg.dtR;      doc["dtW"]      = cfg.dtW;

  JsonArray jArr = doc["jump"].to<JsonArray>();
  for (uint8_t n = 0; n < NUM_JUMPS; n++) {
    JsonObject jo = jArr.add<JsonObject>();
    jo["timeMs"]  = cfg.jump[n].timeMs;
    jo["enabled"] = cfg.jump[n].enabled;
    JsonArray js = jo["s"].to<JsonArray>();
    for (uint8_t i = 0; i < 3; i++) js.add(cfg.jump[n].s[i]);
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handlePostConfig() {
  if (f1q.enabled && (digitalRead(PIN_ARM)==LOW || flightBusy())) { server.send(409,"text/plain","Remove ARM jumper"); return; }
  if (!server.hasArg("plain")) { server.send(400); return; }
  JsonDocument doc;
  if (deserializeJson(doc, server.arg("plain"))) { server.send(400, "text/plain", "Bad JSON"); return; }

  if (doc["soundCircling"].is<int>())    cfg.soundCircling   = doc["soundCircling"];
  if (doc["soundTension"].is<int>())     cfg.soundTension    = doc["soundTension"];
  if (doc["soundReady"].is<int>())       cfg.soundReady      = doc["soundReady"];
  if (doc["soundStart"].is<int>())       cfg.soundStart      = doc["soundStart"];
  if (doc["dtExtraS"].is<int>())         cfg.dtExtraS        = doc["dtExtraS"];
  if (doc["emergencyDtS"].is<int>())     cfg.emergencyDtS    = doc["emergencyDtS"];
  if (doc["rebootPulses"].is<int>())     cfg.rebootPulses    = doc["rebootPulses"];
  if (doc["rebootWindowMs"].is<int>())   cfg.rebootWindowMs  = doc["rebootWindowMs"];
  if (doc["melodyIndex"].is<int>())      cfg.melodyIndex     = doc["melodyIndex"];
  if (doc["melodyOnBoot"].is<int>())     cfg.melodyOnBoot    = doc["melodyOnBoot"];
  if (doc["findBeepEnabled"].is<bool>()) cfg.findBeepEnabled = doc["findBeepEnabled"];
  if (doc["servoMinUs"].is<int>())       cfg.servoMinUs      = doc["servoMinUs"];
  if (doc["servoMaxUs"].is<int>())       cfg.servoMaxUs      = doc["servoMaxUs"];
  if (doc["hookBackInvert"].is<int>())   cfg.hookBackInvert  = doc["hookBackInvert"];
  if (doc["hookFrontInvert"].is<int>())  cfg.hookFrontInvert = doc["hookFrontInvert"];

  if (doc["servoEnabled0"].is<bool>()) cfg.servoEnabled[0] = doc["servoEnabled0"];
  if (doc["servoEnabled1"].is<bool>()) cfg.servoEnabled[1] = doc["servoEnabled1"];
  if (doc["servoEnabled2"].is<bool>()) cfg.servoEnabled[2] = doc["servoEnabled2"];

  if (doc["reverseS"].is<bool>()) cfg.reverseS = doc["reverseS"];
  if (doc["reverseR"].is<bool>()) cfg.reverseR = doc["reverseR"];
  if (doc["reverseW"].is<bool>()) cfg.reverseW = doc["reverseW"];
  if (doc["zeroS"].is<float>())   cfg.zeroS    = doc["zeroS"];
  if (doc["zeroR"].is<float>())   cfg.zeroR    = doc["zeroR"];
  if (doc["zeroW"].is<float>())   cfg.zeroW    = doc["zeroW"];
  if (doc["unlockZero"].is<bool>()) cfg.unlockZero = doc["unlockZero"];

  if (doc["rcDtActive"].is<bool>()) cfg.rcDtActive = doc["rcDtActive"];
  if (doc["rcDtS"].is<float>())     cfg.rcDtS      = doc["rcDtS"];
  if (doc["rcDtR"].is<float>())     cfg.rcDtR      = doc["rcDtR"];
  if (doc["rcDtW"].is<float>())     cfg.rcDtW      = doc["rcDtW"];

  if (doc["idleS"].is<float>())    cfg.idleS    = doc["idleS"];
  if (doc["idleR"].is<float>())    cfg.idleR    = doc["idleR"];
  if (doc["idleW"].is<float>())    cfg.idleW    = doc["idleW"];
  if (doc["circleS"].is<float>())  cfg.circleS  = doc["circleS"];
  if (doc["circleR"].is<float>())  cfg.circleR  = doc["circleR"];
  if (doc["circleW"].is<float>())  cfg.circleW  = doc["circleW"];
  if (doc["tensionS"].is<float>()) cfg.tensionS = doc["tensionS"];
  if (doc["tensionR"].is<float>()) cfg.tensionR = doc["tensionR"];
  if (doc["tensionW"].is<float>()) cfg.tensionW = doc["tensionW"];
  if (doc["readyS"].is<float>())   cfg.readyS   = doc["readyS"];
  if (doc["readyR"].is<float>())   cfg.readyR   = doc["readyR"];
  if (doc["readyW"].is<float>())   cfg.readyW   = doc["readyW"];
  if (doc["dtS"].is<float>())      cfg.dtS      = doc["dtS"];
  if (doc["dtR"].is<float>())      cfg.dtR      = doc["dtR"];
  if (doc["dtW"].is<float>())      cfg.dtW      = doc["dtW"];

  if (doc["ssid"].is<const char*>()) {
    strncpy(cfg.ssid, doc["ssid"], sizeof(cfg.ssid)-1);
    cfg.ssid[sizeof(cfg.ssid)-1] = 0;
  }
  if (doc["password"].is<const char*>()) {
    strncpy(cfg.password, doc["password"], sizeof(cfg.password)-1);
    cfg.password[sizeof(cfg.password)-1] = 0;
  }

  if (doc["jump"].is<JsonArray>()) {
    JsonArray arr = doc["jump"].as<JsonArray>();
    for (uint8_t n = 0; n < NUM_JUMPS && n < arr.size(); n++) {
      cfg.jump[n].timeMs  = arr[n]["timeMs"] | 0;
      cfg.jump[n].enabled = arr[n]["enabled"] | 0;
      JsonArray s = arr[n]["s"].as<JsonArray>();
      for (uint8_t i = 0; i < 3 && i < s.size(); i++) {
        cfg.jump[n].s[i] = s[i] | 0.0f;
      }
    }
  }

  if (f1q.enabled) motorStop("configuration restart");
  saveConfig();
  server.send(200, "application/json", "{\"ok\":true}");
  servicedDelay(500);
  ESP.restart();
}

void handleStatus() {
  JsonDocument doc;
  addSensorStatus(doc);
  doc["state"]      = stateName(state);
  doc["backClosed"] = hookBackClosed();
  doc["frontClosed"]= hookFrontClosed();
  doc["vbat"]       = readVbat();
  doc["wifiActive"] = wifiActive;
  doc["rcDt"]       = rcDtPressed();
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  if (DEBUG) Serial.println("\n=== VM-T1 booting (ESP32-C3) ===");

  digitalWrite(PIN_LED,HIGH);
  pinMode(PIN_LED,        OUTPUT);
  pinMode(PIN_BUZZER,     OUTPUT);
  pinMode(PIN_OFF_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RC_DT,      INPUT_PULLUP);

  digitalWrite(PIN_BUZZER, LOW);

  loadConfig();
  beginExtension();
  WiFi.mode(WIFI_OFF);

  attachServos();
  applyAngles(cfg.idleS, cfg.idleR, cfg.idleW);
  servicedDelay(300);

  server.on("/f1q", HTTP_GET, [](){server.send_P(200,"text/html",F1Q_HTML);});
  server.on("/api/f1q", HTTP_GET, handleF1QGet);
  server.on("/api/f1q", HTTP_POST, handleF1QConfig);
  server.on("/api/zero-height", HTTP_POST, []() {
    if (flightBusy() || digitalRead(PIN_ARM)==LOW) { server.send(409); return; }
    server.send(flightSensors.zeroAltitude()?200:503);
  });
  server.on("/",            HTTP_GET,  handleRoot);
  server.on("/api/config",  HTTP_GET,  handleGetConfig);
  server.on("/api/config",  HTTP_POST, handlePostConfig);
  server.on("/api/status",  HTTP_GET,  handleStatus);

  if (cfg.skipBootMelody) {
    cfg.skipBootMelody = 0;
    saveConfig();
  } else if (cfg.melodyOnBoot > 0) {
    playMelody(cfg.melodyOnBoot);
  }

  enterState(ST_IDLE);
  rcDtWasPressed = rcDtPressed();
  if (digitalRead(PIN_ARM)!=LOW) wifiEnable();
  if (DEBUG) Serial.println("=== Ready ===\n");
}

// ================== LOOP ==================
void loop() {
  serviceExtension();
  if (f1q.enabled && digitalRead(PIN_ARM)==LOW && wifiActive) wifiDisable();
  if (flightStartEvent) { flightStartEvent=false; enterState(ST_WORK); }
  if (wifiActive && !flightBusy()) server.handleClient();

  checkWiFiCondition();
  handleRcDt();   // обработка RC DT — приоритетная

  uint32_t now = millis();

  handleStateSounds(now);

  // Если RC DT нажата — не обрабатываем обычные состояния
  if (rcDtPressed()) {
    applyAngles(cfg.rcDtS, cfg.rcDtR, cfg.rcDtW);
    servicedDelay(5);
    return;
  }

  // Disabled F1Q is setup-only; never fall back to the F1A hook state machine.
  if (f1q.enabled && state==ST_WORK && !dtReached) runProgram();
  servicedDelay(1);
}
