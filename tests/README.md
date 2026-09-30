# Проверка расчёта энергии

`energy_limit_test.cpp` — автономный тест C++17, без Arduino. В MSVC Developer Command Prompt:

```bat
cl /EHsc /std:c++17 tests\energy_limit_test.cpp /Fe:energy_limit_test.exe
energy_limit_test.exe
```

Либо GCC/Clang: `c++ -std=c++17 tests/energy_limit_test.cpp -o energy_limit_test`.

Проверены интегрирование постоянной и изменяющейся мощности, прогноз до следующего отсчёта, отсечки по энергии/времени/потере данных, блокировка повторного запуска, отрицательная мощность и переход uint32 через ноль. Эти тесты не проверяют физическую точность INA226, реакцию ESC, I²C и целиком автомат состояний таймера; для них нужен стенд по `docs/F1Q.md`.
