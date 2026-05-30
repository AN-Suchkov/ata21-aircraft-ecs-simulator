/*
 * 21_cu.c — Контроллер управления СКВ (ATA-21).
 *
 * Логика реализована как КОНЕЧНЫЙ АВТОМАТ (State Machine) с двумя
 * независимыми каналами A и B (резервирование).
 *
 * Состояния КА (CU_State_t):
 *   INIT       → сброс всех переменных при первом вызове
 *   IDLE       → система отключена (pack_switch = OFF)
 *   STARTING   → плавное открытие клапанов (startup_timer)
 *   NORMAL     → ПИД-регулирование температуры и давления
 *   FAULT_CU   → отказ собственного канала КУ
 *   FAULT_PHYS → зафиксирован физический отказ агрегатов
 *   RESET      → сброс после устранения причины
 *
 * Резервирование:
 *   Активный канал управляет Bus_t.
 *   Пассивный — вычисляет команды «вхолостую» (мониторинг).
 *   При отказе активного → переключение на резервный.
 *   При рассогласовании команд обоих каналов > CHAN_DIFF_THRESH → FAULT_CU.
 *
 * Стандарт: ISO C99. Глобальных переменных нет.
 */

#include "21_defs.h"
#include <string.h>   /* memset */

/* =========================================================================
 * ПИД-РЕГУЛЯТОР (встроенная функция)
 *
 * Возвращает управляющий сигнал [0..1].
 * Параметры:
 *   setpoint   — уставка
 *   measured   — измеренное значение
 *   kp, ki, kd — коэффициенты
 *   dt         — шаг (с)
 *   integral   — указатель на состояние интегратора (хранится в CU_Channel_t)
 *   prev_err   — указатель на предыдущую ошибку (для D-составляющей)
 *   int_limit  — предел накопления интегратора (anti-windup)
 * ========================================================================= */
static float pid(float setpoint, float measured,
                 float kp, float ki, float kd,
                 float dt, float *integral, float *prev_err,
                 float int_limit)
{
    float err = setpoint - measured;

    /* Дифференциальная составляющая */
    float deriv = (err - *prev_err) / dt;
    *prev_err   = err;

    /* Предварительный расчёт выхода */
    float u = kp * err + ki * (*integral) + kd * deriv;

    /* Conditional anti-windup:
     * Не накапливаем интегратор, если выход насыщён И ошибка "усиливает"
     * насыщение. Это предотвращает залипание регулятора при резкой смене
     * условий (например, переход с земли на крейсерскую высоту).
     *   u > 1 и err > 0  → выход уже на максимуме, ошибка тянет вверх
     *   u < 0 и err < 0  → выход уже на минимуме, ошибка тянет вниз
     */
    int sat_high = (u > 1.0f && err > 0.0f);
    int sat_low  = (u < 0.0f && err < 0.0f);
    if (!sat_high && !sat_low) {
        *integral += err * dt;
        *integral  = lib_limit(*integral, -int_limit, int_limit);
    }

    return lib_limit(u, 0.0f, 1.0f);
}

/* =========================================================================
 * ИНИЦИАЛИЗАЦИЯ ОДНОГО КАНАЛА КУ
 * ========================================================================= */
void cu_init(CU_Channel_t *ch, int ch_id)
{
    memset(ch, 0, sizeof(CU_Channel_t));
    ch->ch_id     = ch_id;
    ch->state     = CU_STATE_INIT;
    ch->active    = (ch_id == 0) ? 1 : 0; /* Канал A активен по умолчанию */
    ch->self_fault = 0;
}

/* =========================================================================
 * ИМЕНА СОСТОЯНИЙ (для логирования)
 * ========================================================================= */
const char *cu_state_name(CU_State_t s)
{
    switch (s) {
        case CU_STATE_INIT:       return "INIT";
        case CU_STATE_IDLE:       return "IDLE";
        case CU_STATE_STARTING:   return "STARTING";
        case CU_STATE_NORMAL:     return "NORMAL";
        case CU_STATE_FAULT_CU:   return "FAULT_CU";
        case CU_STATE_FAULT_PHYS: return "FAULT_PHYS";
        case CU_STATE_RESET:      return "RESET";
        default:                  return "UNKNOWN";
    }
}

/* =========================================================================
 * ВЫЧИСЛЕНИЕ КОМАНД ОДНОГО КАНАЛА (без записи в Bus)
 *
 * Вызывается для обоих каналов. Активный канал запишет команды в Bus.
 * ========================================================================= */
static void cu_compute(CU_Channel_t *ch, const Input_t *inp, const Output_t *sens)
{
    /* --- Преобразование аналоговых датчиков в физические значения ---
     * КУ работает ТОЛЬКО с сигналами датчиков, не с физическими значениями.
     * (Разделение логики: «напряжения, коды» — как указано в задании.)
     *
     * cabin_temp:  U = (T + 50) / 13  → T = U * 13 - 50
     * cabin_press: U = P / 0.2        → P = U * 0.2
     */
    float meas_cabin_temp  = sens->sens_cabin_temp  * 13.0f - 50.0f;
    float meas_cabin_press = sens->sens_cabin_press * 0.2f;

    /* ---- КОНЕЧНЫЙ АВТОМАТ ---- */
    switch (ch->state) {

    /* ---------------------------------------------------------------
     * INIT: однократная инициализация переменных
     * --------------------------------------------------------------- */
    case CU_STATE_INIT:
        ch->startup_timer       = 0.0f;
        ch->fault_timer         = 0.0f;
        ch->temp_pid_integral   = 0.0f;
        ch->temp_pid_prev_err   = 0.0f;
        ch->press_pid_integral  = 0.0f;
        ch->press_pid_prev_err  = 0.0f;
        ch->pack_valve_cmd      = 0.0f;
        ch->outflow_valve_cmd   = 0.5f;
        ch->trim_valve_cmd      = 0.0f;
        ch->recirc_fan_cmd      = 0.0f;
        ch->anti_ice_cmd        = 0;
        ch->state = CU_STATE_IDLE;
        break;

    /* ---------------------------------------------------------------
     * IDLE: ждём команды включения
     * Переходы:
     *   pack_switch ≠ OFF и есть питание → STARTING
     * --------------------------------------------------------------- */
    case CU_STATE_IDLE:
        /* Клапаны закрыты, вентилятор выключен */
        ch->pack_valve_cmd    = 0.0f;
        ch->outflow_valve_cmd = 0.8f; /* вытяжной приоткрыт на земле */
        ch->trim_valve_cmd    = 0.0f;
        ch->recirc_fan_cmd    = 0.0f;
        ch->anti_ice_cmd      = 0;

        if (inp->pack_switch != PACK_OFF && inp->pwr_28v && inp->pwr_115v) {
            /* Сброс таймеров и интеграторов перед запуском */
            ch->startup_timer      = 0.0f;
            ch->temp_pid_integral  = 0.0f;
            ch->press_pid_integral = 0.0f;
            ch->state = CU_STATE_STARTING;
        }
        break;

    /* ---------------------------------------------------------------
     * STARTING: плавное открытие пак-клапана за STARTUP_TIMER_MAX сек
     * Переходы:
     *   startup_timer ≥ STARTUP_TIMER_MAX → NORMAL
     *   pack_switch = OFF → IDLE
     *   отказ питания → FAULT_CU
     * --------------------------------------------------------------- */
    case CU_STATE_STARTING:
        ch->startup_timer += DT;
        {
            float ramp = lib_limit(ch->startup_timer / STARTUP_TIMER_MAX, 0.0f, 1.0f);
            ch->pack_valve_cmd    = ramp * 0.7f;   /* открываем до 70% */
            /* Вытяжной клапан: плавно открывается до 0.5 к концу запуска.
             * Ограничено 0.5 (вместо 0.8) — иначе при малом расходе клапан
             * сбрасывает давление быстрее, чем идёт нагнетание. */
            ch->outflow_valve_cmd = 0.3f + ramp * 0.2f;
            float cold_heat = lib_limit(
                (inp->cabin_temp_setpoint - meas_cabin_temp) * 0.015f,
                0.0f, 0.4f);
            ch->trim_valve_cmd = cold_heat * ramp;
            ch->recirc_fan_cmd    = ramp;
            ch->anti_ice_cmd      = 0;
        }

        if (inp->pack_switch == PACK_OFF) {
            ch->state = CU_STATE_IDLE;
        } else if (!inp->pwr_28v || !inp->pwr_115v) {
            ch->self_fault = 1;
            ch->state = CU_STATE_FAULT_CU;
        } else if (ch->startup_timer >= STARTUP_TIMER_MAX) {
            ch->state = CU_STATE_NORMAL;
        }
        break;

    /* ---------------------------------------------------------------
     * NORMAL: ПИД-регулирование температуры и давления
     *
     * Температура: выход ПИД → pack_valve_cmd (расход = холод)
     *              + trim_valve_cmd (тонкая подстройка нагревом)
     * Давление:    выход ПИД → outflow_valve_cmd (сброс давления)
     *
     * Переходы:
     *   pack_switch = OFF → IDLE
     *   физический отказ  → FAULT_PHYS
     *   отказ питания / датчиков → FAULT_CU
     * --------------------------------------------------------------- */
    case CU_STATE_NORMAL:
    {
        /* heat_demand: 1.0 = кабина холодная → нужен обогрев
                        0.0 = кабина горячая → нужно охлаждение  */
        float heat_demand = pid(inp->cabin_temp_setpoint, meas_cabin_temp,
                                0.04f, 0.008f, 0.002f, DT,
                                &ch->temp_pid_integral, &ch->temp_pid_prev_err, 100.0f);

        /* Двухзонное управление:
         * Зона охлаждения (heat_demand 0..0.5): пак открывается, трим закрыт.
         * Зона обогрева  (heat_demand 0.5..1): пак на минимуме, трим открывается. */
        if (heat_demand <= 0.5f) {
            ch->pack_valve_cmd = lib_limit(1.0f - 2.0f * heat_demand, 0.30f, 1.0f);
            ch->trim_valve_cmd = 0.0f;
        } else {
            ch->pack_valve_cmd = 0.30f;
            ch->trim_valve_cmd = lib_limit((heat_demand - 0.5f) * 2.0f * 0.70f, 0.0f, 0.70f);
        }
        /* --- Контур давления (kp=1.5, ki=0.1, kd=0.05) --- */
        float u_press = pid(inp->cabin_press_setpoint, meas_cabin_press,
                            1.5f, 0.1f, 0.05f, DT,
                            &ch->press_pid_integral, &ch->press_pid_prev_err, 8.0f);
        /* Вытяжной клапан: чем больше давление превышает уставку → тем он открытее */
        ch->outflow_valve_cmd = lib_limit(1.0f - u_press, 0.1f, 0.95f);

        /* --- Рециркуляционный вентилятор: всегда максимальный расход --- */
        ch->recirc_fan_cmd = 1.0f;

        /* --- ПОС конденсатора:
         *     Включаем, если температура за конденсатором близка к 0°С.
         *     Датчик: sens_pack_out_temp в Вольтах → Т = U * 16 - 60 */
        float t_pack_out = sens->sens_pack_out_temp * 16.0f - 60.0f;
        ch->anti_ice_cmd = (t_pack_out < 2.0f) ? 1 : 0;

        /* --- Обнаружение физических отказов --- */
        int phys_fault = (int)(sens->fault_flags & (FAULT_VALVE_JAM  |
                                                     FAULT_LINE_BREAK |
                                                     FAULT_PRESS_LOSS |
                                                     FAULT_TEMP_HIGH));
        if (phys_fault) {
            ch->fault_timer += DT;
            if (ch->fault_timer >= FAULT_CONFIRM_TIME) {
                ch->fault_timer = 0.0f;
                ch->state = CU_STATE_FAULT_PHYS;
            }
        } else {
            ch->fault_timer = 0.0f;
        }

        /* --- Переключение режима --- */
        if (inp->pack_switch == PACK_OFF) {
            ch->state = CU_STATE_IDLE;
        } else if (!inp->pwr_28v || !inp->pwr_115v) {
            ch->self_fault = 1;
            ch->state = CU_STATE_FAULT_CU;
        }
        break;
    }

    /* ---------------------------------------------------------------
     * FAULT_CU: собственный отказ канала
     * Все команды = безопасное состояние (fail-safe).
     * Переход: только через RESET (после reset_cmd)
     * --------------------------------------------------------------- */
    case CU_STATE_FAULT_CU:
        ch->pack_valve_cmd    = 0.0f;   /* клапан закрыт */
        ch->outflow_valve_cmd = 0.5f;
        ch->trim_valve_cmd    = 0.0f;
        ch->recirc_fan_cmd    = 0.0f;
        ch->anti_ice_cmd      = 0;

        /* Фронт сигнала Reset (0→1) */
        if (!ch->prev_reset_cmd && inp->reset_cmd) {
            ch->self_fault = 0;
            ch->state = CU_STATE_RESET;
        }
        break;

    /* ---------------------------------------------------------------
     * FAULT_PHYS: физический отказ агрегата
     * Клапана в безопасное положение; ждём Reset.
     * --------------------------------------------------------------- */
    case CU_STATE_FAULT_PHYS:
        ch->pack_valve_cmd    = 0.0f;
        ch->outflow_valve_cmd = 0.8f;   /* вытяжной открыт для вентиляции */
        ch->trim_valve_cmd    = 0.0f;
        ch->recirc_fan_cmd    = 0.3f;   /* минимальная рециркуляция */
        ch->anti_ice_cmd      = 0;

        if (!ch->prev_reset_cmd && inp->reset_cmd) {
            ch->state = CU_STATE_RESET;
        }
        break;

    /* ---------------------------------------------------------------
     * RESET: один цикл сброса, затем → IDLE
     * --------------------------------------------------------------- */
    case CU_STATE_RESET:
        ch->startup_timer      = 0.0f;
        ch->fault_timer        = 0.0f;
        ch->temp_pid_integral  = 0.0f;
        ch->temp_pid_prev_err  = 0.0f;
        ch->press_pid_integral = 0.0f;
        ch->press_pid_prev_err = 0.0f;
        ch->pack_valve_cmd     = 0.0f;
        ch->outflow_valve_cmd  = 0.5f;
        ch->trim_valve_cmd     = 0.0f;
        ch->recirc_fan_cmd     = 0.0f;
        ch->anti_ice_cmd       = 0;
        ch->self_fault         = 0;
        ch->state = CU_STATE_IDLE;
        break;

    default:
        ch->state = CU_STATE_INIT;
        break;
    }

    /* --- Инжекция принудительного отказа: только для канала A ---
     * Имитирует отказ электронного блока КУ: при inject_ch_fault=1
     * канал A немедленно переходит в состояние FAULT_CU (если был в NORMAL).
     * Канал B остаётся исправным и принимает управление.
     * Используется в сценариях тестирования резервирования.
     */
    if (inp->inject_ch_fault && ch->ch_id == 0 && ch->state == CU_STATE_NORMAL) {
        ch->self_fault = 1;
        ch->state = CU_STATE_FAULT_CU;
    }

    /* Запоминаем предыдущее состояние reset для обнаружения фронта */
    ch->prev_reset_cmd = inp->reset_cmd;
}

/* =========================================================================
 * ГЛАВНАЯ ФУНКЦИЯ ШАГА КУ
 *
 * Оба канала вычисляют команды. Активный канал пишет в Bus.
 * Мониторинг рассогласования → переключение каналов.
 * ========================================================================= */
void cu_step(CU_Channel_t *ch_a, CU_Channel_t *ch_b,
             const Input_t *inp, const Output_t *sens, Bus_t *bus)
{
    /* Сбрасываем флаг рассогласования перед каждым шагом */
    bus->disagr_detected = 0;

    /* Вычисляем команды для обоих каналов */
    cu_compute(ch_a, inp, sens);
    cu_compute(ch_b, inp, sens);

    /* ---- Резервирование: выбор активного канала ----
     *
     * Правила переключения:
     *  1. Если активный канал ушёл в FAULT_CU → переключиться на резервный,
     *     если резервный исправен.
     *  2. Если оба канала дают разные команды (рассогласование) → FAULT_CU
     *     у канала, чьи команды отклоняются сильнее (консервативная логика:
     *     генерируем отказ у обоих, но оставляем тот, кто ближе к нулю).
     *  3. Если оба в отказе → fail-safe: записываем нули.
     */

    /* Проверка рассогласования */
    float diff_pack = lib_abs(ch_a->pack_valve_cmd - ch_b->pack_valve_cmd);
    float diff_out  = lib_abs(ch_a->outflow_valve_cmd - ch_b->outflow_valve_cmd);

    if ((diff_pack > CHAN_DIFF_THRESH || diff_out > CHAN_DIFF_THRESH) &&
        ch_a->state == CU_STATE_NORMAL && ch_b->state == CU_STATE_NORMAL &&
        !ch_a->self_fault && !ch_b->self_fault)
    {
        /* Рассогласование в рабочем режиме: ни одному каналу нельзя доверять.
         * Оба переходят в FAULT_CU → fail-safe.
         * Восстановление — только через reset_cmd (после диагностики).
         * Флаг disagr_detected защёлкивается в PhysState_t и остаётся
         * до следующего reset_cmd.
         */
        bus->disagr_detected = 1;
        ch_a->self_fault = 1;
        ch_b->self_fault = 1;
        ch_a->state = CU_STATE_FAULT_CU;
        ch_b->state = CU_STATE_FAULT_CU;
    }

    /* Переключение активного канала при отказе */
    int a_ok = (ch_a->state != CU_STATE_FAULT_CU) && !ch_a->self_fault;
    int b_ok = (ch_b->state != CU_STATE_FAULT_CU) && !ch_b->self_fault;

    if (ch_a->active && !a_ok && b_ok) {
        /* Канал A отказал → переключаем на B */
        ch_a->active = 0;
        ch_b->active = 1;
    } else if (ch_b->active && !b_ok && a_ok) {
        /* Канал B отказал → переключаем на A */
        ch_b->active = 0;
        ch_a->active = 1;
    } else if (!a_ok && !b_ok) {
        /* Оба в отказе — fail-safe */
        ch_a->active = 0;
        ch_b->active = 0;
    }

    /* Восстановление: если ни один канал не активен, но есть исправный —
     * назначаем A первичным (при равных правах), иначе B.
     * Это необходимо после одновременного отказа и сброса обоих каналов
     * (например, после FAULT_CH_DISAGR → reset → оба вернулись в IDLE). */
    if (!ch_a->active && !ch_b->active) {
        if      (a_ok) ch_a->active = 1;
        else if (b_ok) ch_b->active = 1;
    }

    /* ---- Запись команд активного канала в Bus ---- */
    CU_Channel_t *active = NULL;
    if      (ch_a->active) active = ch_a;
    else if (ch_b->active) active = ch_b;

    if (active != NULL) {
        bus->pack_valve_cmd     = active->pack_valve_cmd;
        bus->outflow_valve_cmd  = active->outflow_valve_cmd;
        bus->trim_valve_cmd     = active->trim_valve_cmd;
        bus->recirc_fan_cmd     = active->recirc_fan_cmd;
        bus->anti_ice_valve_cmd = active->anti_ice_cmd;
        bus->active_channel     = active->ch_id;
    } else {
        /* Оба канала в отказе → fail-safe состояние */
        bus->pack_valve_cmd     = 0.0f;
        bus->outflow_valve_cmd  = 0.5f;
        bus->trim_valve_cmd     = 0.0f;
        bus->recirc_fan_cmd     = 0.0f;
        bus->anti_ice_valve_cmd = 0;
        bus->active_channel     = -1;
    }

    /* Отражаем состояния КА в Output (для лога) */
    /* (Запись cu_state_a/b производится в 21_manager.c после cu_step) */
}
