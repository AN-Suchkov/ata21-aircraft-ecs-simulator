/*
 * 21_phys.c — Физическая модель СКВ (ATA-21).
 *
 * Моделирует: источник воздуха, узел предварительного охлаждения (УПО),
 *             ТХУ (компрессор + вентилятор + турбина), конденсаторный блок,
 *             камеру смешения, гермокабину.
 *
 * Разделение логики (по заданию):
 *   КУ (21_cu.c) работает с сигналами (напряжения, коды).
 *   Физика (21_phys.c) — «железо»: расходы, давления, температуры.
 *
 * Стандарт: ISO C99. Глобальных переменных нет. math.h запрещён.
 */

#include "21_defs.h"
#include <string.h>   /* memset */

/* =========================================================================
 * ВСПОМОГАТЕЛЬНЫЕ (локальные функции)
 * ========================================================================= */

/*
 * Адиабатная температура: T2 = T1 * (P2/P1)^((γ-1)/γ)
 * Применяется для компрессора и турбины ТХУ.
 * T1_C — температура на входе (°С), P1, P2 — давление (бар).
 */
static float adiab_temp(float T1_C, float P1, float P2)
{
    float T1_K = T1_C + T_KELVIN_OFFSET;
    float ratio = P2 / P1;
    if (ratio < 0.001f) ratio = 0.001f;
    float T2_K = T1_K * lib_power(ratio, ISENTROPIC_EXP);
    return T2_K - T_KELVIN_OFFSET;
}

/*
 * Модель теплообменника (counter-flow approximation).
 * Возвращает температуру горячего воздуха на выходе.
 * T_hot_in  — температура горячего потока на входе (°С)
 * T_cold_in — температура холодного (продувочного) потока на входе (°С)
 * eff       — эффективность ТО [0..1]
 */
static float hx_hot_out(float T_hot_in, float T_cold_in, float eff)
{
    return T_hot_in - eff * (T_hot_in - T_cold_in);
}

/*
 * Первый закон термодинамики: нагрев кабины.
 * dT/dt = (Q_in - Q_out - Q_loss) / (m * Cp)
 * Возвращает dT/dt (°С/с).
 */
static float cabin_dT_dt(float T_cabin, float T_supply,
                          float T_outside, float mdot)
{
    float Q_in   = mdot * CP_AIR * (T_supply - T_cabin);  /* Вт */
    float Q_loss = CABIN_UA_FUSELAGE * (T_cabin - T_outside);
    float Q_gen  = CABIN_HEAT_GEN;
    return (Q_in + Q_gen - Q_loss) / CABIN_THERM_MASS;
}

/* =========================================================================
 * ИНИЦИАЛИЗАЦИЯ ФИЗИЧЕСКОГО СОСТОЯНИЯ
 * ========================================================================= */
void phys_init(PhysState_t *ps, const Input_t *inp)
{
    memset(ps, 0, sizeof(PhysState_t));

    /* Начальное давление кабины:
     *   - Если уставка > забортного (полёт): предполагаем, что кабина уже
     *     загерметизирована до уставки (самолёт "уже в воздухе").
     *   - Если забортное > уставки (земля): давление = атмосферному.
     * Это исключает нефизичный старт симуляции с нулевым избыточным давлением
     * в крейсерских сценариях и убирает ложный насыщение интегратора ПИД.  */
    ps->cabin_temp = inp->cabin_temp_initial;
    ps->cabin_pressure = (inp->cabin_press_setpoint > inp->ram_air_pressure)
                         ? inp->cabin_press_setpoint
                         : inp->ram_air_pressure;
    ps->cabin_humidity = 0.50f;

    /* Все клапаны закрыты, вентилятор стоит */
    ps->pack_valve_pos    = 0.0f;
    ps->outflow_valve_pos = 0.0f;
    ps->trim_valve_pos    = 0.0f;
    ps->recirc_fan_speed  = 0.0f;

    /* Температурный тракт: всё при комнатной температуре */
    ps->t_after_phx        = 20.0f;
    ps->t_after_comp       = 20.0f;
    ps->t_after_mhx        = 20.0f;
    ps->t_after_regen_hot  = 20.0f;
    ps->t_after_cond       = 20.0f;
    ps->t_after_regen_cold = 20.0f;
    ps->t_after_turbine    = 20.0f;
    ps->t_mix              = 20.0f;

    ps->valve_jammed      = 0;
    ps->line_broken       = 0;
    ps->pressurization_loss = 0;
    ps->anti_ice_active   = 0;
}

/* =========================================================================
 * ШАГОВАЯ ФУНКЦИЯ ФИЗИЧЕСКОЙ МОДЕЛИ
 *
 * Вызывается из 21_manager.c каждые DT секунд.
 * Входы: inp (внешние условия) + bus (команды от КУ).
 * Выходы: обновляет ps (состояние) и заполняет out (датчики, параметры).
 * ========================================================================= */
void phys_step(const Input_t *inp, const Bus_t *bus,
               PhysState_t *ps, Output_t *out)
{
    /* ------------------------------------------------------------------
     * 1. ИНЖЕКЦИЯ ОТКАЗОВ (зависит от входных сигналов теста)
     * ------------------------------------------------------------------ */
    if (inp->inject_valve_jam && !ps->valve_jammed) {
        ps->valve_jammed  = 1;
        ps->valve_jam_pos = ps->pack_valve_pos;
    }
    if (!inp->inject_valve_jam) {
        ps->valve_jammed = 0;
    }
    ps->line_broken         = inp->inject_line_break;
    ps->pressurization_loss = inp->inject_press_loss;
    ps->anti_ice_active     = bus->anti_ice_valve_cmd;

    /* ------------------------------------------------------------------
     * 2. ИСПОЛНИТЕЛЬНЫЕ МЕХАНИЗМЫ (с динамикой и ограничением скорости)
     * ------------------------------------------------------------------ */

    /* Пак-клапан: если заклинен, остаётся на месте */
    float pack_cmd = ps->valve_jammed ? ps->valve_jam_pos : bus->pack_valve_cmd;
    ps->pack_valve_pos = lib_rate_limit(
        lib_limit(pack_cmd, 0.0f, 1.0f),
        VALVE_RATE_MAX, DT, &ps->pack_valve_pos);

    ps->outflow_valve_pos = lib_rate_limit(
        lib_limit(bus->outflow_valve_cmd, 0.0f, 1.0f),
        VALVE_RATE_MAX, DT, &ps->outflow_valve_pos);

    ps->trim_valve_pos = lib_rate_limit(
        lib_limit(bus->trim_valve_cmd, 0.0f, 1.0f),
        VALVE_RATE_MAX, DT, &ps->trim_valve_pos);

    ps->recirc_fan_speed = lib_rate_limit(
        lib_limit(bus->recirc_fan_cmd, 0.0f, 1.0f),
        FAN_RATE_MAX, DT, &ps->recirc_fan_speed);

    /* ------------------------------------------------------------------
     * 3. ИСТОЧНИК ВОЗДУХА
     * Двигатель → сжатый воздух 3 бар / +200°С
     * При обрыве магистрали расход = 0
     * ------------------------------------------------------------------ */
    float bleed_press = inp->eng_bleed_pressure;
    float bleed_temp  = inp->eng_bleed_temp;

    /* Если нет источника или обрыв — нет потока */
    int source_ok = (inp->engine_running || inp->apu_avail) && inp->pwr_28v;
    if (!source_ok || ps->line_broken) {
        bleed_press = 0.0f;
        bleed_temp  = inp->ram_air_temp;
    }

    /* Расход через пак-клапан (линейная зависимость от положения) */
    float bleed_flow = PACK_FLOW_NOM * ps->pack_valve_pos;
    if (!source_ok || ps->line_broken) bleed_flow = 0.0f;

    /* ------------------------------------------------------------------
     * 4. УЗЕЛ ПРЕДВАРИТЕЛЬНОГО ОХЛАЖДЕНИЯ (УПО)
     *
     * Схема воздушного тракта:
     *  Bleed → [PHX первичный ТО] → [Компрессор ТХУ] → [MHX основной ТО]
     *
     * Охлаждающий забортный воздух прокачивается вентилятором ТХУ.
     * Температура продувочного воздуха ≈ ram_air_temp.
     * ------------------------------------------------------------------ */

    /* 4.1 Первичный ТО: охлаждает bleed до ~+100°С */
    float t_phx_out = hx_hot_out(bleed_temp, inp->ram_air_temp, HX_PRIM_EFF);

    /* Фильтр первого порядка для инерционности теплообменника (τ ≈ 5 с) */
    float tau_hx = 5.0f;
    ps->t_after_phx += (t_phx_out - ps->t_after_phx) * (DT / tau_hx);

    /* 4.2 Компрессор ТХУ: повышает давление 3→4 бар, нагревает воздух.
     *     Адиабатный нагрев + учёт КПД компрессора (η≈0.80). */
    float t_comp_ideal;
    if (bleed_press > 0.0f) {
        t_comp_ideal = adiab_temp(ps->t_after_phx, bleed_press, TCU_TURB_PRESS_IN);
    } else {
        t_comp_ideal = ps->t_after_phx;  // нет потока – нет нагрева
    }
    float t_comp_out = ps->t_after_phx
                   + (t_comp_ideal - ps->t_after_phx) / 0.80f;
    
    /* Если нет потока — без нагрева */
    if (bleed_flow < 0.01f) t_comp_out = ps->t_after_phx;
    ps->t_after_comp += (t_comp_out - ps->t_after_comp) * (DT / 3.0f);

    /* 4.3 Основной ТО: охлаждает воздух за компрессором до ~+80...+100°С */
    float t_mhx_out = hx_hot_out(ps->t_after_comp, inp->ram_air_temp, HX_MAIN_EFF);
    ps->t_after_mhx += (t_mhx_out - ps->t_after_mhx) * (DT / tau_hx);

    /* ------------------------------------------------------------------
     * 5. КОНДЕНСАТОРНЫЙ БЛОК
     *
     * Схема:
     *  [Рег. горячий тракт] → [Конденсатор горячий] → [Влагоотделитель]
     *      → [Рег. холодный тракт] → [Турбина ТХУ]
     *      → [Конденсатор холодный тракт] → выход пака
     * ------------------------------------------------------------------ */

    /* 5.1 Регенератор (горячий тракт): охлаждает до ~+40..+50°С.
     *     Теплоноситель — холодный воздух из холодного тракта рег. (предыдущий шаг) */
    float t_reg_hot_out = hx_hot_out(ps->t_after_mhx, ps->t_after_regen_cold, REGEN_EFF);
    ps->t_after_regen_hot += (t_reg_hot_out - ps->t_after_regen_hot) * (DT / 4.0f);

    /* 5.2 Конденсатор (горячий тракт):
     *     Целевая температура = COND_TARGET_TEMP (+5..+10°С).
     *     Теплоноситель — холодный воздух за турбиной (предыдущий шаг). */
    float t_cond_out = hx_hot_out(ps->t_after_regen_hot, ps->t_after_turbine, 0.85f);
    /* Не даём упасть ниже точки росы при наличии ПОС */
    float t_cond_min = ps->anti_ice_active ? 2.0f : COND_TARGET_TEMP;
    if (t_cond_out < t_cond_min) t_cond_out = t_cond_min;
    ps->t_after_cond += (t_cond_out - ps->t_after_cond) * (DT / 4.0f);

    /* 5.3 Регенератор (холодный тракт): подогрев воздуха для испарения остатков влаги.
     *     Выход ≈ +50..+60°С. */
    float t_reg_cold_out = ps->t_after_cond
                           + REGEN_EFF * (ps->t_after_regen_hot - ps->t_after_cond);
    ps->t_after_regen_cold += (t_reg_cold_out - ps->t_after_regen_cold) * (DT / 3.0f);

    /* 5.4 Турбина ТХУ: расширение 4→1 бар, адиабатное охлаждение.
     *     η_турбины учтён: фактическое падение Т меньше идеального. */
    float t_turb_ideal = adiab_temp(ps->t_after_regen_cold,
                                    TCU_TURB_PRESS_IN, TCU_TURB_PRESS_OUT);
    float dT_turb_ideal = ps->t_after_regen_cold - t_turb_ideal;
    float t_turb_out    = ps->t_after_regen_cold - TCU_TURB_EFF * dT_turb_ideal;
    if (bleed_flow < 0.01f) t_turb_out = ps->t_after_regen_cold;
    ps->t_after_turbine += (t_turb_out - ps->t_after_turbine) * (DT / 2.0f);

    /* 5.5 Конденсатор (холодный тракт): воздух за турбиной нагревается
     *     продувая конденсатор. Выход ≈ −8..−10°С. */
    float t_pack_out = ps->t_after_turbine
                       + 0.85f * (ps->t_after_cond - ps->t_after_turbine);
    /* Обрыв = нет потока → выход = забортная температура */
    if (bleed_flow < 0.01f) t_pack_out = inp->ram_air_temp;

    /* ------------------------------------------------------------------
     * 6. КАМЕРА СМЕШЕНИЯ
     * 50 % свежего воздуха (T_pack_out) + 50 % рециркуляционного (T_cabin).
     * Расход рециркуляции пропорционален скорости вентилятора.
     * ------------------------------------------------------------------ */
    float recirc_fraction = ps->recirc_fan_speed * (1.0f - MIX_FRESH_RATIO);
    /* Т смешения (уравнение теплового баланса): */
    float fresh_frac = MIX_FRESH_RATIO;
    ps->t_mix = fresh_frac * t_pack_out + recirc_fraction * ps->cabin_temp
                + (1.0f - fresh_frac - recirc_fraction) * t_pack_out;

    /* Трим-клапан: байпасирует долю горячего отбора (bleed_temp) прямо в смесь.
     * При trim=0.7: trim_frac=0.245 → ~25% горячего воздуха в потоке.
     * Формула = взвешенное смешение двух потоков (физически корректно).      */
    float trim_frac    = ps->trim_valve_pos * 0.35f;
    float t_trim_blend = (1.0f - trim_frac) * ps->t_mix + trim_frac * bleed_temp;
    ps->t_mix = t_trim_blend;

    /* ------------------------------------------------------------------
     * 7. ГЕРМОКАБИНА (тепловой баланс)
     * ------------------------------------------------------------------ */
    float total_flow = bleed_flow * (fresh_frac + recirc_fraction);
    float dT = cabin_dT_dt(ps->cabin_temp, ps->t_mix,
                           inp->ram_air_temp, total_flow);
    ps->cabin_temp += dT * DT;

    /* Давление кабины:
     *   dp/dt = PRESS_INFLOW_COEFF * bleed_flow
     *         - PRESS_OUTFLOW_COEFF * outflow_valve_pos * (P_cabin - P_ram)
     *
     * Нагнетание (bleed) поднимает давление; вытяжной клапан компенсирует его,
     * сбрасывая избыточный перепад. КУ управляет клапаном через ПИД-регулятор.
     * При разгерметизации — быстрое падение к забортному давлению (τ = 5 с).
     * При выключенном паке — медленная утечка через щели (τ = 300 с).
     */
    if (ps->pressurization_loss) {
        ps->cabin_pressure += (inp->ram_air_pressure - ps->cabin_pressure) * (DT / 5.0f);
    } else if (bleed_flow > 0.01f) {
        float delta_p = ps->cabin_pressure - inp->ram_air_pressure;
        float dp_in   = PRESS_INFLOW_COEFF * bleed_flow;
        float dp_out  = PRESS_OUTFLOW_COEFF * ps->outflow_valve_pos
                        * (delta_p > 0.0f ? delta_p : 0.0f);
        ps->cabin_pressure += (dp_in - dp_out) * DT;
    } else {
        ps->cabin_pressure += (inp->ram_air_pressure - ps->cabin_pressure) * (DT / 300.0f);
    }

    ps->cabin_pressure = lib_limit(ps->cabin_pressure, inp->ram_air_pressure, 1.1f);
    /* Влажность: упрощённая модель — конденсатор снижает влажность отбора */
    float humidity_bleed = 0.1f;  /* отбор от двигателя — очень сухой */
    float humidity_recirc = ps->cabin_humidity;
    ps->cabin_humidity = fresh_frac * humidity_bleed + (1.0f - fresh_frac) * humidity_recirc;
    ps->cabin_humidity += (0.45f - ps->cabin_humidity) * (DT / 120.0f); /* пассажиры */
    ps->cabin_humidity = lib_limit(ps->cabin_humidity, 0.0f, 1.0f);

    /* ------------------------------------------------------------------
     * 8. ФОРМИРОВАНИЕ ВЫХОДОВ (Output_t)
     * ------------------------------------------------------------------ */

    /* Физические параметры */
    out->pack_valve_pos    = ps->pack_valve_pos;
    out->outflow_valve_pos = ps->outflow_valve_pos;
    out->bleed_flow        = bleed_flow;

    /* Температурный тракт */
    out->t_after_phx       = ps->t_after_phx;
    out->t_after_comp      = ps->t_after_comp;
    out->t_after_mhx       = ps->t_after_mhx;
    out->t_after_regen_hot = ps->t_after_regen_hot;
    out->t_after_cond      = ps->t_after_cond;
    out->t_after_turbine   = ps->t_after_turbine;
    out->t_mix             = ps->t_mix;

    /* Гермокабина */
    out->cabin_temp     = ps->cabin_temp;
    out->cabin_pressure = ps->cabin_pressure;
    out->cabin_humidity = ps->cabin_humidity;

    /* ------------------------------------------------------------------
     * 9. ДАТЧИКИ (преобразование физических значений в сигналы)
     *
     * Зависимость датчика от питания (требование задания):
     *   При pwr_28v == 0 → датчик выдаёт 0 В ("0В" согласно заданию).
     * ------------------------------------------------------------------ */

    /* Аналоговые датчики (0–10 В):
     *   cabin_temp:  0 В = −50°С, 10 В = +80°С   → (T + 50) / 13
     *   cabin_press: 0 В = 0 бар, 10 В = 2 бар   → P / 0.2
     *   pack_out_T:  0 В = −60°С, 10 В = +100°С  → (T + 60) / 16
     */
    if (inp->pwr_28v) {
        out->sens_cabin_temp  = lib_limit((ps->cabin_temp  + 50.0f) / 13.0f, 0.0f, 10.0f);
        out->sens_cabin_press = lib_limit( ps->cabin_pressure / 0.2f,         0.0f, 10.0f);
        out->sens_pack_out_temp = lib_limit((t_pack_out + 60.0f) / 16.0f,   0.0f, 10.0f);
    } else {
        /* Отказ питания — датчики выдают 0В ("Обрыв" согласно заданию) */
        out->sens_cabin_temp    = 0.0f;
        out->sens_cabin_press   = 0.0f;
        out->sens_pack_out_temp = 0.0f;
    }

    /* Дискретные датчики (0/1):
     *   Концевик пак-клапана — открыт, если pos > 0.05 */
    out->sens_pack_valve_fbk = (inp->pwr_28v && ps->pack_valve_pos > 0.05f) ? 1 : 0;
    out->sens_vent_active    = (inp->pwr_28v && ps->recirc_fan_speed > 0.1f) ? 1 : 0;

    /* ------------------------------------------------------------------
     * 10. ФЛАГИ ОТКАЗОВ (физические)
     * ------------------------------------------------------------------ */

    /* Инжекция перегрева конденсатора: имитирует сбой системы охлаждения
     * (например, отказ вентилятора ТХУ или отказ датчика температуры).
     * Форсируем температуру за конденсатором выше порога TEMP_FAULT_THRESH,
     * чтобы КУ зафиксировал FAULT_TEMP_HIGH.                              */
    if (inp->inject_temp_high) {
        ps->t_after_cond = TEMP_FAULT_THRESH + 10.0f;
    }

    /* FAULT_CH_DISAGR: рассогласование фиксируется КУ в течение одного шага,
     * но в fault_flags должно оставаться до сброса (иначе флаг теряется между
     * выборками консоли/лога). Используем защёлку в PhysState_t:
     *   - устанавливается при bus->disagr_detected = 1
     *   - сбрасывается при фронте reset_cmd (0→1) */
    if (bus->disagr_detected)  ps->disagr_latched = 1;
    if (inp->reset_cmd)        ps->disagr_latched = 0;

    uint32_t faults = FAULT_NONE;
    if (ps->valve_jammed)          faults |= FAULT_VALVE_JAM;
    if (ps->line_broken)           faults |= FAULT_LINE_BREAK;
    if (ps->pressurization_loss)   faults |= FAULT_PRESS_LOSS;
    if (ps->t_after_cond > TEMP_FAULT_THRESH)  faults |= FAULT_TEMP_HIGH;
    if (ps->disagr_latched)        faults |= FAULT_CH_DISAGR;
    if (!inp->pwr_28v)             faults |= FAULT_SENSOR_PWR;
    out->fault_flags = faults;

    /* ------------------------------------------------------------------
     * 11. ARINC-429 ВЫХОДНЫЕ СЛОВА
     * Метки (восьмеричные):
     *   0201(8) = 0x81 — температура кабины (BCD, °С × 10 для индикации)
     *   0203(8) = 0x83 — давление кабины (BNR, бар, LSB = 0.001 бар)
     *   0270(8) = 0xB8 — слово статуса (DW, флаги отказов)
     * ------------------------------------------------------------------ */
    int arinc_ok = inp->pwr_115v && inp->pwr_28v;

    /* BCD: температура × 10 (1 десятичное знак после запятой → целые дециградусы) */
    int temp_bcd = (int)(ps->cabin_temp * 10.0f);
    if (temp_bcd < 0) temp_bcd = 0;
    out->arinc_cabin_temp  = lib_arinc_bcd(temp_bcd, 0x81, arinc_ok);

    /* BNR: давление, LSB = 0.001 бар */
    out->arinc_cabin_press = lib_arinc_bnr(ps->cabin_pressure, 0.001f, 0x83, arinc_ok);

    /* DW: биты 0..5 = коды отказов из fault_flags */
    out->arinc_status = lib_arinc_dw(faults & 0x3Fu, 0xB8, arinc_ok);
}
