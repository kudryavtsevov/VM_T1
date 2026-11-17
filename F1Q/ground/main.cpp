// SPDX-License-Identifier: Apache-2.0
#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "../common/Radio.h"
#include "FlightLog.h"
#include "PhysicalDt.h"
using namespace f1q;
WebServer server(80);
Preferences prefs;
FlightLog flightLog;
PhysicalDt physicalDt;
bool physicalDtReady=false;
Config desired;
Telemetry latest;
bool seen=false, pending=false;
uint32_t receivedAt=0,lastSend=0;
bool radioOk=false;

void configJson(JsonObject o,const Config &c) {
  o["revision"]=c.revision; o["shuntOhm"]=c.shuntOhm; o["shuntConfirmed"]=c.shuntConfirmed;
  o["energyJ"]=c.energyJ; o["maxCurrentA"]=c.maxCurrentA; o["minVoltageV"]=c.minVoltageV;
  o["motorMs"]=c.motorMs; o["flightMs"]=c.flightMs;
  o["escOffUs"]=c.escOffUs; o["escRunUs"]=c.escRunUs; o["servoEnabled"]=c.servoEnabled;
  const uint16_t *v[]={c.idleUs,c.motorUs,c.glideUs,c.dtUs};
  const char *names[]={"idleUs","motorUs","glideUs","dtUs"};
  for(int j=0;j<4;j++) { auto a=o[names[j]].to<JsonArray>(); for(int i=0;i<3;i++) a.add(v[j][i]); }
}
void respond(JsonDocument &d) { String s; serializeJson(d,s); server.send(200,"application/json",s); }
bool parseConfig(JsonVariantConst o, Config &c) {
  // Require full typed payload. Reject fractional/out-of-range integer values before narrowing.
#define NUMBER(name) if(!o[#name].is<float>()) return false; c.name=o[#name].as<float>();
#define UINT(name, type) if(!o[#name].is<type>()) return false; c.name=o[#name].as<type>();
  NUMBER(shuntOhm); NUMBER(energyJ); NUMBER(maxCurrentA); NUMBER(minVoltageV);
  UINT(motorMs,uint32_t); UINT(flightMs,uint32_t); UINT(escOffUs,uint16_t); UINT(escRunUs,uint16_t);
  UINT(shuntConfirmed,uint8_t); UINT(servoEnabled,uint8_t);
#undef NUMBER
#undef UINT
  uint16_t *v[]={c.idleUs,c.motorUs,c.glideUs,c.dtUs};
  const char *names[]={"idleUs","motorUs","glideUs","dtUs"};
  for(int j=0;j<4;j++) {
    auto a=o[names[j]].as<JsonArrayConst>(); if(a.size()!=3) return false;
    for(int i=0;i<3;i++) { if(!a[i].is<uint16_t>()) return false; v[j][i]=a[i].as<uint16_t>(); }
  }
  return valid(c);
}
const char PAGE[] PROGMEM=R"HTML(<!doctype html><html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>VM F1Q — наземный модуль</title><style>body{font:17px system-ui;max-width:850px;margin:auto;padding:20px;background:#101923;color:#edf4fa}section{background:#1c2a38;padding:18px;margin:16px 0;border-radius:12px}label{display:block;margin:12px 0}input{float:right;width:140px;font:inherit}button{padding:12px;margin:8px 6px 8px 0;font:inherit;cursor:pointer}a{color:#8edbff}pre{white-space:pre-wrap}#dt{background:#b82638;color:white}small{color:#b5c7d6}</style>
<h1>VM F1Q</h1><p id="connection">Ожидание телеметрии…</p><section><p id="battery"></p><p>Быстрый DT: физическая кнопка на наземном модуле. Кнопка ниже — дополнительная.</p><pre id="status"></pre><a id="map" hidden target="_blank" rel="noopener">Открыть последние координаты на карте</a><br><button id="dt">Остановить мотор и включить DT</button><p id="dtResult"></p></section>
<section><h2>Настройки борта</h2><p>Изменения передаются после нажатия «Сохранить». Готовность подтверждает борт. Во время полёта изменения запрещены.</p><form id="form"><div id="fields"></div>
<label><input type="checkbox" id="confirmed"> Сопротивление шунта измерено, показания откалиброваны</label><p><small>Начальное значение 0,001 Ом — пример. Уменьшение шунта «примерно в 10 раз» недостаточно для точного ограничения энергии.</small></p>
<h3>Сервоприводы S / R / W</h3><label>Маска включения: S=1, R=2, W=4<input id="servoEnabled" type="number" min="0" max="7" required></label><div id="servos"></div>
<button type="submit">Сохранить и передать на борт</button><button type="button" id="load">Прочитать настройки</button></form><p id="result"></p></section>
<section><h2>Последний полёт</h2><p>Запись на наземной плате, независимо от телефона. Единственный latest.csv содержит принятые пакеты полёта и обе высоты; при новом полёте заменяется. Данные за последнюю секунду могут потеряться при внезапном отключении питания.</p><button id="logsRefresh">Обновить список</button><button id="logsPrepare" hidden>Подготовить память журнала</button><p id="logsStatus"></p><div id="logsList"></div></section>
<section><h2>Запуск</h2><p>Подождите минимум 3 секунды после включения. Нажмите кнопку на борту, удерживайте 1 секунду и отпустите — мотор запустится. Один запуск на включение; для следующего полёта перезапустите питание. Проверяйте первый запуск без пропеллера.</p><p>Отсечка мотора: время, энергия (если задана), ток, напряжение или ошибка датчика. Пропадание радиосвязи не отменяет бортовые ограничения. Координаты после потери связи — последняя принятая точка.</p></section>
<script>
const defs=[['shuntOhm','Шунт, Ом',0.00005,0.1,0.00001],['motorMs','Время мотора, мс',100,120000,1],['escOffUs','ESC: стоп, мкс',900,1200,1],['escRunUs','ESC: работа, мкс',901,2000,1],['energyJ','Лимит энергии, Дж (0 — выключен)',0,100000,0.1],['minVoltageV','Минимальное напряжение, В',2,35,0.1],['maxCurrentA','Максимальный ток, А',0.1,200,0.1],['flightMs','DT от запуска, мс',100,3600000,1]];
for(const [id,title,min,max,step] of defs) fields.insertAdjacentHTML('beforeend',`<label>${title}<input required id="${id}" type="number" min="${min}" max="${max}" step="${step}"></label>`);
for(const [id,title] of [['idleUs','Ожидание'],['motorUs','Мотор'],['glideUs','Планирование'],['dtUs','DT']]) for(let i=0;i<3;i++)servos.insertAdjacentHTML('beforeend',`<label>${title}, ${'SRW'[i]}, мкс<input required id="${id}${i}" type="number" min="500" max="2400" step="1"></label>`);
let editing=false;form.oninput=()=>editing=true;
async function request(url,options){const r=await fetch(url,options);if(!r.ok)throw Error(await r.text());return r.json()}
async function loadConfig(){try{const c=await request('/api/config');for(const [id] of defs)document.getElementById(id).value=c[id];servoEnabled.value=c.servoEnabled;confirmed.checked=!!c.shuntConfirmed;for(const id of ['idleUs','motorUs','glideUs','dtUs'])c[id].forEach((v,i)=>document.getElementById(id+i).value=v);editing=false}catch(e){result.textContent=e.message}}
load.onclick=loadConfig;
form.onsubmit=async e=>{e.preventDefault();const c={};for(const [id]of defs)c[id]=Number(document.getElementById(id).value);c.servoEnabled=Number(servoEnabled.value);c.shuntConfirmed=confirmed.checked?1:0;for(const id of ['idleUs','motorUs','glideUs','dtUs'])c[id]=[0,1,2].map(i=>Number(document.getElementById(id+i).value));try{const r=await request('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(c)});result.textContent='Передача ревизии '+r.revision+'…';editing=false}catch(e){result.textContent=e.message}};
dt.onclick=async()=>{try{await request('/api/dt',{method:'POST'});dtResult.textContent='Команда отправляется. Ожидается подтверждение состояния DT от борта.'}catch(e){dtResult.textContent=e.message}};
const states=['Ожидание','Готов к запуску по отпусканию','Мотор','Планирование','DT','Ошибка'];
const reasons=['—','Лимит времени','Лимит энергии','DT с земли','Резерв','Ошибка INA226','Превышение тока','Низкое напряжение','Время полёта'];
async function poll(){try{const s=await request('/api/status');connection.textContent=!s.radioReady?'ESP-NOW не настроен: задайте MAC и ключи перед прошивкой':!s.seen?'Борт ещё не обнаружен':s.link?'Борт на связи':'Связь потеряна; возраст телеметрии '+(s.ageMs/1000).toFixed(1)+' с';if(s.physicalDtReady===false)connection.textContent+=' · Кнопка DT не готова';battery.textContent=!s.batteryFresh?'Бортовой аккумулятор: нет свежих данных':`Бортовой аккумулятор: ${s.voltageV.toFixed(2)} В${s.batteryLow?' — НИЗКОЕ НАПРЯЖЕНИЕ':''}; порог ${s.batteryMinVoltageV.toFixed(1)} В`;battery.style.color=s.batteryLow?'#ff8b8b':'';document.getElementById('status').textContent=s.seen?`${states[s.state]} · ${reasons[s.reason]}\n${s.voltageV.toFixed(2)} В · ${s.currentA.toFixed(2)} А · ${s.energyJ.toFixed(2)} Дж\nПолёт ${(s.flightMs/1000).toFixed(1)} с · INA226 ${s.sensorOk?'OK':'нет свежих данных'}\nGPS: ${s.gpsValid?s.latitude.toFixed(7)+', '+s.longitude.toFixed(7)+'; возраст '+((s.fixAgeMs+s.ageMs)/1000).toFixed(1)+' с':'нет координат'}\nВысота GPS над уровнем моря: ${s.gpsAltitudeValid && (s.altitudeAgeMs||0)+s.ageMs<2000?s.gpsMslM.toFixed(1)+' м':'нет свежих данных'}\nGPS от старта: ${s.gpsRelativeValid && (s.altitudeAgeMs||0)+s.ageMs<2000?s.gpsRelativeM.toFixed(1)+' м':'нет отсчёта'}\nБарометр от старта: ${s.baroValid && (s.baroAgeMs||0)+s.ageMs<500?s.baroRelativeM.toFixed(1)+' м':'нет свежих данных'}\nСпутников: ${s.satellites}\nБортовой журнал: ${['не готов','запись','заполнен','ошибка'][s.onboardLogStatus]||'нет статуса'}, ${s.onboardRecords||0} записей\n${s.flightUsed?'Запуск использован: для нового вылета перезапустите питание':'Запуск ещё не использован'}`:'Нет данных';if(s.gpsValid){map.hidden=false;map.href=`https://www.openstreetmap.org/?mlat=${s.latitude}&mlon=${s.longitude}#map=17/${s.latitude}/${s.longitude}`}if(s.logError)logsStatus.textContent=s.logError;if(s.pending)result.textContent='Ожидается подтверждение ревизии '+s.desiredRevision;else if(s.seen&&!editing)result.textContent='На борту подтверждена ревизия '+s.appliedRevision;if(s.link&&s.reason===3&&s.state===4)dtResult.textContent='Борт подтвердил DT';}catch(e){connection.textContent='Нет связи с наземным веб-сервером'}setTimeout(poll,500)}
async function refreshLogs(){try{const d=await request('/api/logs');logsPrepare.hidden=d.ready;logsStatus.textContent=d.error||(d.ready?'Свободно '+Math.round(d.freeBytes/1024)+' КиБ':'Память не подготовлена');logsList.replaceChildren();for(const f of d.files){const p=document.createElement('p'),a=document.createElement('a');a.href='/api/log?name='+encodeURIComponent(f.name);a.textContent=f.name+' ('+Math.round(f.bytes/1024)+' КиБ)';p.append(a);const b=document.createElement('button');b.textContent='Удалить';b.onclick=async()=>{if(!confirm('Удалить '+f.name+'?'))return;try{await request('/api/log?name='+encodeURIComponent(f.name),{method:'DELETE'});refreshLogs()}catch(e){logsStatus.textContent=e.message}};p.append(b);logsList.append(p)}}catch(e){logsStatus.textContent=e.message}}
logsRefresh.onclick=refreshLogs;logsPrepare.onclick=async()=>{if(!confirm('Форматировать немонтируемую память? Если в ней были журналы, они будут удалены.'))return;try{await request('/api/logs/prepare',{method:'POST'});refreshLogs()}catch(e){logsStatus.textContent=e.message}};
loadConfig();poll();refreshLogs();
</script></html>)HTML";
void setup() {
  Serial.begin(115200);
  flightLog.begin();
  prefs.begin("f1qground",true);
  if(prefs.getBytesLength("cfg")==sizeof(desired)) { Config c; prefs.getBytes("cfg",&c,sizeof(c)); if(valid(c)) desired=c; }
  prefs.end();
  WiFi.mode(WIFI_AP_STA); WiFi.setSleep(false);
  WiFi.softAP("VM-F1Q-Ground",F1Q_AP_PASSWORD,CHANNEL);
  radioOk=radio::begin(TIMER_MAC);
  physicalDtReady=physicalDt.begin();
  Serial.printf("Ground STA MAC: %s; AP: %s; ESP-NOW: %s\n",WiFi.macAddress().c_str(),WiFi.softAPIP().toString().c_str(),radioOk?"ready":"configure Pairing.local.h");
  server.on("/",HTTP_GET,[]{server.send_P(200,"text/html; charset=utf-8",PAGE);});
  server.on("/api/config",HTTP_GET,[]{JsonDocument d; configJson(d.to<JsonObject>(),desired); respond(d);});
  server.on("/api/config",HTTP_POST,[]{
    if(!seen || millis()-receivedAt>2000 || latest.state==MOTOR || latest.state==GLIDE || latest.state==ARMED) {
      server.send(409,"text/plain; charset=utf-8","Нужна связь с бортом в состоянии ожидания, DT или ошибки"); return;
    }
    if(server.arg("plain").length()>2048) {server.send(413,"text/plain","Payload too large");return;}
    JsonDocument d; Config c;
    if(deserializeJson(d,server.arg("plain")) || !parseConfig(d.as<JsonVariantConst>(),c)) {
      server.send(400,"text/plain; charset=utf-8","Некорректные параметры: проверьте диапазоны и падение на шунте (Imax × R ≤ 0,075 В)");return;
    }
    c.revision=max(desired.revision,latest.appliedRevision)+1; if(!c.revision)c.revision=1;
    desired=c; pending=true;
    prefs.begin("f1qground",false); prefs.putBytes("cfg",&desired,sizeof(desired));prefs.end();
    JsonDocument out;out["revision"]=desired.revision;respond(out);
  });
  server.on("/api/dt",HTTP_POST,[]{if(!physicalDtReady){server.send(503,"text/plain","DT task unavailable");return;}physicalDt.request();JsonDocument d;d["sending"]=true;respond(d);});
  server.on("/api/status",HTTP_GET,[]{
    JsonDocument d; uint32_t age=seen?millis()-receivedAt:UINT32_MAX;
    d["physicalDtReady"]=physicalDtReady;d["dtSending"]=physicalDt.active();
    d["batteryFresh"]=seen && age<2000 && latest.sensorOk;
    d["batteryLow"]=seen && age<2000 && latest.sensorOk && latest.voltageV<latest.config.minVoltageV;
    d["batteryMinVoltageV"]=latest.config.minVoltageV;
    d["radioReady"]=radioOk;d["seen"]=seen;d["link"]=seen&&age<2000;d["ageMs"]=age;
    d["pending"]=pending;d["desiredRevision"]=desired.revision;d["appliedRevision"]=latest.appliedRevision;
    d["state"]=latest.state;d["reason"]=latest.reason;d["sensorOk"]=latest.sensorOk;
    d["voltageV"]=latest.voltageV;d["currentA"]=latest.currentA;d["energyJ"]=latest.energyJ;
    d["flightMs"]=latest.flightMs;d["gpsValid"]=latest.gpsValid;d["fixAgeMs"]=latest.fixAgeMs;
    d["onboardRecords"]=latest.onboardRecords;d["onboardLogStatus"]=latest.onboardLogStatus;d["flightUsed"]=latest.flightUsed;
    d["gpsMslM"]=latest.gpsMslM;d["gpsRelativeM"]=latest.gpsRelativeM;
    d["gpsAltitudeValid"]=latest.gpsAltitudeValid;d["gpsRelativeValid"]=latest.gpsRelativeValid;d["altitudeAgeMs"]=latest.altitudeAgeMs;
    d["baroRelativeM"]=latest.baroRelativeM;d["baroValid"]=latest.baroValid;d["baroAgeMs"]=latest.baroAgeMs;d["pressurePa"]=latest.pressurePa;
    d["logReady"]=flightLog.mounted;d["logError"]=flightLog.error;
    d["latitude"]=latest.latitude;d["longitude"]=latest.longitude;d["satellites"]=latest.satellites;respond(d);
  });
  server.on("/api/logs",HTTP_GET,[]{
    JsonDocument d;d["ready"]=flightLog.mounted;d["error"]=flightLog.error;
    auto a=d["files"].to<JsonArray>();
    if(flightLog.mounted){
      d["freeBytes"]=LittleFS.totalBytes()-LittleFS.usedBytes();
      File dir=LittleFS.open("/flights");
      if(dir && dir.isDirectory()){
        File f=dir.openNextFile();
        while(f){auto o=a.add<JsonObject>();String name=f.name();name=name.substring(name.lastIndexOf('/')+1);o["name"]=name;o["bytes"]=f.size();f=dir.openNextFile();}
      }
    }
    respond(d);
  });
  server.on("/api/logs/prepare",HTTP_POST,[]{
    if(seen && (latest.state==MOTOR || latest.state==GLIDE || latest.state==ARMED)) {server.send(409,"text/plain","Board is armed or flying");return;}
    JsonDocument d;d["ready"]=flightLog.prepare();respond(d);
  });
  auto validLogName=[](const String &name){
    return name=="latest.csv";
  };
  server.on("/api/log",HTTP_GET,[validLogName]{
    if(seen && millis()-receivedAt<15000 && (latest.state==MOTOR || latest.state==GLIDE || latest.state==ARMED)) {server.send(409,"text/plain; charset=utf-8","Выгрузка доступна после полёта или 15 секунд без связи");return;}
    String name=server.arg("name");
    if(!flightLog.mounted || !validLogName(name)){server.send(400,"text/plain","Invalid log name");return;}
    flightLog.flush(millis()+1000);
    File f=LittleFS.open("/flights/"+name,FILE_READ);
    if(!f){server.send(404,"text/plain","Log not found");return;}
    server.sendHeader("Content-Disposition",String("attachment; filename=\"")+name+"\"");
    server.streamFile(f,"text/csv; charset=utf-8");f.close();
  });
  server.on("/api/log",HTTP_DELETE,[validLogName]{
    if(seen && (latest.state==MOTOR || latest.state==GLIDE || latest.state==ARMED)){server.send(409,"text/plain","Board is armed or flying");return;}
    String name=server.arg("name");
    if(!flightLog.mounted || !validLogName(name)){server.send(400,"text/plain","Invalid log name");return;}
    JsonDocument d;d["deleted"]=LittleFS.remove("/flights/"+name);respond(d);
  });
  server.begin();
}
void loop() {
  Packet p;
  while(radio::read(p)) if(p.kind==TELEMETRY) {
    latest=p.telemetry; receivedAt=millis();
    flightLog.record(p,receivedAt);
    if(!seen && !pending && latest.appliedRevision && valid(latest.config)) desired=latest.config;
    seen=true;
    if(pending && latest.appliedRevision==desired.revision && !memcmp(&latest.config,&desired,sizeof(desired))) pending=false;
  }
  uint32_t now=millis();
  if(pending && seen && now-receivedAt<2000 && now-lastSend>=500) {
    Packet out;out.kind=CONFIG;out.config=desired;radio::send(out);lastSend=now;
  }
  flightLog.flush(now);
  server.handleClient();delay(1);
}
