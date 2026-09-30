// SPDX-License-Identifier: Apache-2.0
#pragma once
const char F1Q_HTML[] PROGMEM = R"PAGE(
<!doctype html><html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>VM-T1 — датчики и F1Q</title><style>
body{max-width:680px;margin:24px auto;padding:0 16px;font:16px sans-serif;background:#132029;color:#eef}
label{display:block;margin:14px 0}input{display:block;padding:8px;width:90%}button{padding:12px;margin:8px 0}a{color:#80d6ff}
</style><h1>Датчики и F1Q</h1><a href="/">Назад к таймеру</a>
<p>Экспериментальная версия без одобрения EDIC. Перед настройкой снимите ARM и воздушный винт.
Модуль INA226: перед использованием замените R100 на R010 (0,01 Ом), номинальной мощностью не менее 2 Вт. Рабочий предел 7,5 А; проверьте пусковой ток. С оставленным R100 эта прошивка измеряет неверно.
BMP280 измеряет высоту относительно наземного нуля; высота не управляет двигателем.</p>
<form id="q">
<label><input type="checkbox" name="enabled">Включить режим F1Q</label>
<label>Масса модели, г<input name="massG" type="number" min="50" max="5000" step="0.1" required></label>
<label>Энергия, Дж/г (2 или 3)<input name="joulesPerGram" type="number" min="2" max="3" required></label>
<label>Запас до лимита, Дж<input name="reserveJ" type="number" min="5" step="0.1" required></label>
<label>Время двигателя, мс (до 29500; при 2 Дж/г — до 19500)<input name="motorMs" type="number" min="100" max="29500" required></label>
<label>Команда ESC при работе, мкс<input name="runUs" type="number" min="1000" max="2000" required></label>
<label>Сопротивление R010, Ом (номинал 0,01)<input name="shuntOhms" type="number" min="0.009" max="0.011" step="0.000001" required></label>
<label>Коэффициент калибровки тока<input name="currentScale" type="number" min="0.5" max="1.5" step="0.0001" required></label>
<label>Нулевое смещение тока, А<input name="currentOffsetA" type="number" min="-0.1" max="0.1" step="0.0001" required></label>
<button>Сохранить для следующего включения</button></form>
<button id="zero">Принять текущую высоту за ноль</button><p id="message"></p>
<p>Последовательность: ARM снят при включении → установить ARM → держать START ≥1 с → отпустить START.
После остановки повторный запуск заблокирован до перезапуска питания. Wi-Fi отключается при ARM.
Команда остановки ESC — 1000 мкс; её нужно проверить на вашем регуляторе.</p>
<script>
const form=document.getElementById('q'),msg=document.getElementById('message');
fetch('/api/f1q').then(r=>r.json()).then(d=>{for(const e of form.elements){if(!e.name)continue;if(e.type==='checkbox')e.checked=d[e.name];else e.value=d[e.name];}}).catch(()=>msg.textContent='Ошибка загрузки');
form.onsubmit=async e=>{e.preventDefault();let d={};for(const x of form.elements)if(x.name)d[x.name]=x.type==='checkbox'?x.checked:Number(x.value);
try{const r=await fetch('/api/f1q',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});
msg.textContent=r.ok?'Сохранено. Снимите ARM и перезапустите питание.':await r.text();}catch(e){msg.textContent='Ошибка связи';}};
document.getElementById('zero').onclick=async()=>{try{const r=await fetch('/api/zero-height',{method:'POST'});msg.textContent=r.ok?'Ноль высоты установлен':'Нет данных BMP280 или ARM установлен';}catch(e){msg.textContent='Ошибка связи';}};
</script></html>
)PAGE";
void handleF1QGet() {
  JsonDocument d;
  d["enabled"]=f1q.enabled;d["massG"]=f1q.massG;d["joulesPerGram"]=f1q.joulesPerGram;
  d["reserveJ"]=f1q.reserveJ;d["motorMs"]=f1q.motorMs;d["runUs"]=f1q.runUs;
  d["shuntOhms"]=flightSensors.shuntOhms;d["currentScale"]=flightSensors.currentScale;
  d["currentOffsetA"]=flightSensors.currentOffsetA;
  String out;serializeJson(d,out);server.send(200,"application/json",out);
}
