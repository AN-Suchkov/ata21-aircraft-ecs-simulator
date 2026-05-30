# skv_sim_21 — SSJ-100 Air Conditioning System Simulator (ATA-21)

Симулятор системы кондиционирования воздуха (СКВ) самолёта SSJ-100 по стандарту ATA-21.
Реализован на ISO C99 без динамической памяти и без стандартной математической библиотеки.

A C99 simulator of the SSJ-100 aircraft air conditioning system (ATA-21).
No dynamic memory allocation, no `<math.h>` — suitable for embedded/MISRA-style targets.

---

## Структура репозитория / Repository Structure

```
skv_sim_21/
├── 21_defs.h          — все структуры, перечисления, константы, прототипы
├── 21_manager.c       — главный цикл симуляции (точка входа)
├── 21_cu.c            — двухканальный контроллер (ПИД, конечный автомат)
├── 21_phys.c          — термодинамическая физическая модель
├── 21_lib.c           — математические утилиты, кодирование ARINC-429
├── 21_inout.c         — чтение сценариев CSV, запись лога CSV
├── Makefile           — сборка (Linux / Windows MinGW)
├── run_all.sh         — запуск всех сценариев (Linux)
├── run_all.bat        — запуск всех сценариев (Windows)
├── scenarios/         — входные CSV-файлы сценариев (13 штук)
├── logs/              — выходные CSV-логи (генерируются при запуске)
└── docs/              — техническая документация (.docx, на русском)
```

---

## Сборка / Build

### Linux (GCC)

```bash
make          # сборка: ./skv_sim_21
make run      # сборка + запуск сценария по умолчанию
make clean    # удалить объектные файлы, бинарник, логи
```

### Windows (MinGW / MSYS2)

Установите [MSYS2](https://www.msys2.org/), затем в терминале MSYS2:

```bash
pacman -S mingw-w64-x86_64-gcc make   # один раз
make          # сборка: skv_sim_21.exe
make run      # сборка + запуск сценария по умолчанию
make clean
```

> **Требования:** GCC с поддержкой C99, GNU Make. Внешних библиотек нет.

---

## Запуск / Run

### Один сценарий / Single scenario

```bash
# Linux
./skv_sim_21 scenarios/scenario_default.csv logs/log_default.csv

# Windows
skv_sim_21.exe scenarios\scenario_default.csv logs\log_default.csv
```

Если аргументы не указаны, используются пути по умолчанию:
- сценарий: `scenarios/scenario_default.csv`
- лог: `logs/log_21.csv`

### Все сценарии / All scenarios

```bash
# Linux
chmod +x run_all.sh
./run_all.sh

# Windows
run_all.bat
```

---

## Сценарии / Scenarios

| Файл | Описание | Description |
|------|----------|-------------|
| `scenario_default.csv` | Полный рабочий цикл | Normal cruise cycle |
| `scenario_hot_day.csv` | Жаркий день (+45 °C) | Hot day stress test |
| `scenario_extreme_cold_start.csv` | Запуск при −50 °C | Extreme cold start |
| `scenario_pid_step_response.csv` | Ступенчатое изменение уставки | PID step response |
| `scenario_anti_ice_test.csv` | Работа противообледенительного клапана | Anti-ice valve test |
| `scenario_line_break_and_recovery.csv` | Обрыв воздухозаборника и восстановление | Line break & recovery |
| `scenario_ground_test.csv` | Наземные проверки | Ground operations test |
| `scenario_dual_fault.csv` | Два одновременных отказа | Dual simultaneous faults |
| `scenario_channel_fault.csv` | Отказ канала A, переход на канал B | Channel A fault, switchover |
| `scenario_sensor_power_loss.csv` | Потеря питания 28 В | 28 V sensor power loss |
| `scenario_ch_disagr.csv` | Разногласие каналов | Channel disagreement |
| `scenario_temp_high.csv` | Перегрев конденсатора | Condenser overtemperature |
| `scenario_both_ch_fault.csv` | Отказ обоих каналов | Both channels fail |

**Формат сценария** — CSV без заголовка, поля: `время_с,параметр,значение`.
Строки, начинающиеся с `#`, являются комментариями.

---

## Формат лога / Log Format

Каждая строка соответствует шагу симуляции DT = 0.02 с.

| Столбец | Единица | Описание |
|---------|---------|----------|
| `time_s` | с | Время симуляции |
| `pack_valve_cmd` | 0–1 | Команда пакетного клапана |
| `outflow_valve_cmd` | 0–1 | Команда клапана вытяжки |
| `trim_valve_cmd` | 0–1 | Команда триммирующего клапана |
| `recirc_fan_cmd` | 0–1 | Команда вентилятора рециркуляции |
| `anti_ice_cmd` | 0/1 | Команда противообледенения |
| `pack_valve_pos` | 0–1 | Положение пакетного клапана |
| `outflow_valve_pos` | 0–1 | Положение клапана вытяжки |
| `T_after_PHX` | °C | Температура после первичного теплообменника |
| `T_after_COMP` | °C | Температура после компрессора |
| `T_after_MHX` | °C | Температура после главного теплообменника |
| `T_after_REGEN_H` | °C | Температура горячей стороны регенератора |
| `T_after_COND` | °C | Температура после конденсатора |
| `T_after_TURB` | °C | Температура после турбины |
| `T_mix` | °C | Температура в камере смешения |
| `cabin_temp` | °C | Температура в кабине |
| `cabin_press` | бар | Давление в кабине |
| `cabin_hum` | — | Влажность в кабине |
| `sens_T_cabin_V` | В | Аналоговый датчик температуры (0–10 В) |
| `sens_P_cabin_V` | В | Аналоговый датчик давления (0–10 В) |
| `sens_T_pack_V` | В | Аналоговый датчик температуры пакета (0–10 В) |
| `valve_fbk` | 0/1 | Обратная связь клапана |
| `vent_active` | 0/1 | Вентиляция активна |
| `fault_flags` | битовая маска | Коды отказов (см. ниже) |
| `cu_state_A` | строка | Состояние канала A |
| `cu_state_B` | строка | Состояние канала B |
| `active_ch` | 0/1 | Активный канал (0 = A, 1 = B) |
| `arinc_temp` | ARINC BNR | Слово ARINC-429: температура |
| `arinc_press` | ARINC BNR | Слово ARINC-429: давление |
| `arinc_status` | ARINC DW | Слово ARINC-429: статус |

---

## Инжекция отказов / Fault Injection

Параметры сценария для моделирования отказов:

| Параметр | `fault_flags` бит | Описание |
|----------|-------------------|----------|
| `inject_valve_jam` | `0x0001` | Заклинивание клапана |
| `inject_line_break` | `0x0002` | Обрыв воздуховода |
| `inject_press_loss` | `0x0004` | Разгерметизация кабины |
| `inject_temp_high` | `0x0008` | Перегрев конденсатора (> 60 °C) |
| `inject_ch_fault` | `0x0010` | Принудительный отказ канала |
| *(потеря питания)* | `0x0020` | `pwr_28v = 0` → FAULT_SENSOR_PWR |

---

## Архитектура / Architecture

```
main() [21_manager.c]
  │
  ├─ inout_apply_events()  [21_inout.c]   — применить события сценария
  ├─ cu_step()             [21_cu.c]      — контроллер: вычислить команды → Bus_t
  ├─ phys_step()           [21_phys.c]   — физика: выполнить команды, обновить Output_t
  └─ inout_write_row()     [21_inout.c]   — записать строку лога
```

Конечный автомат контроллера (7 состояний): `INIT → IDLE → STARTING → NORMAL ⇄ FAULT_PHYS / FAULT_CU → RESET → IDLE`

Двухканальная резервированная схема: каналы A и B вычисляют команды независимо; активный канал передаёт их на физическую модель. При разногласии > 0.15 оба канала переходят в `FAULT_CU`.

---

## CI / Continuous Integration

GitHub Actions автоматически собирает проект на Ubuntu и Windows при каждом push:

```
.github/workflows/build.yml
```

---

## Документация / Documentation

Техническая документация на русском языке находится в папке `docs/`:
- `ATA21_Documentation.docx` — основная техническая спецификация
- `Документация_skv_sim_21.docx` — руководство пользователя
- `Шпаргалка_skv_sim_21.docx` — краткий справочник по параметрам
