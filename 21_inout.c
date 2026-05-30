/*
 * 21_inout.c — Интерфейс ввода-вывода СКВ (ATA-21).
 *
 * Функции:
 *   - Чтение сценариев работы из командной строки (CSV-файл событий).
 *   - Применение событий сценария к Input_t в нужный момент времени.
 *   - Запись лога симуляции в CSV-файл.
 *   - Вывод оперативных данных в консоль.
 *
 * Формат файла сценария (CSV, без заголовка):
 *   время_с,имя_параметра,значение
 *   Пример:
 *     0.0,pack_switch,1
 *     5.0,engine_running,1
 *     60.0,cabin_temp_setpoint,24.0
 *     90.0,inject_valve_jam,1
 *    120.0,inject_valve_jam,0
 *    150.0,reset_cmd,1
 *    151.0,reset_cmd,0
 *
 * Стандарт: ISO C99.
 */

#include "21_defs.h"
#include <stdio.h>
#include <stdlib.h>   /* atof, atoi */
#include <string.h>   /* strncmp, memset */

/* =========================================================================
 * ИНИЦИАЛИЗАЦИЯ: чтение файла сценария
 * ========================================================================= */
void inout_init(InoutState_t *io, const char *scenario_file)
{
    memset(io, 0, sizeof(InoutState_t));
    io->event_count  = 0;
    io->next_event   = 0;
    io->sim_time_max = SIM_TIME_MAX;  /* по умолчанию — глобальная константа */

    if (scenario_file == NULL) return;

    FILE *f = fopen(scenario_file, "r");
    if (f == NULL) {
        fprintf(stderr, "[INOUT] Файл сценария '%s' не найден. "
                        "Работаем без сценария.\n", scenario_file);
        return;
    }

    char line[128];
    while (fgets(line, sizeof(line), f) != NULL &&
           io->event_count < SCENARIO_MAX_EVENTS)
    {
        /* Пропускаем комментарии и пустые строки */
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;

        ScenarioEvent_t ev;
        char param[32];
        float t = 0.0f, val = 0.0f;

        /* Разбор: время,параметр,значение */
        int n = sscanf(line, "%f,%31[^,],%f", &t, param, &val);
        if (n != 3) continue;

        ev.time  = t;
        ev.value = val;
        strncpy(ev.param, param, sizeof(ev.param) - 1);
        ev.param[sizeof(ev.param) - 1] = '\0';

        io->events[io->event_count++] = ev;
    }
    fclose(f);

    fprintf(stderr, "[INOUT] Загружено событий: %d\n", io->event_count);
}

/* =========================================================================
 * ПРИМЕНЕНИЕ СОБЫТИЙ СЦЕНАРИЯ К Input_t
 *
 * Вызывается каждый шаг. Применяет все события с time <= t.
 * ========================================================================= */
void inout_apply_events(InoutState_t *io, float t, Input_t *inp)
{
    while (io->next_event < io->event_count &&
           io->events[io->next_event].time <= t)
    {
        ScenarioEvent_t *ev = &io->events[io->next_event];
        float v = ev->value;

        /* Применяем параметр по имени */
        if      (!strncmp(ev->param, "pwr_28v",               32)) inp->pwr_28v               = (int)v;
        else if (!strncmp(ev->param, "pwr_115v",              32)) inp->pwr_115v              = (int)v;
        else if (!strncmp(ev->param, "engine_running",        32)) inp->engine_running        = (int)v;
        else if (!strncmp(ev->param, "apu_avail",             32)) inp->apu_avail             = (int)v;
        else if (!strncmp(ev->param, "eng_bleed_pressure",    32)) inp->eng_bleed_pressure    = v;
        else if (!strncmp(ev->param, "eng_bleed_temp",        32)) inp->eng_bleed_temp        = v;
        else if (!strncmp(ev->param, "ram_air_temp",          32)) inp->ram_air_temp          = v;
        else if (!strncmp(ev->param, "ram_air_pressure",      32)) inp->ram_air_pressure      = v;
        else if (!strncmp(ev->param, "pack_switch",           32)) inp->pack_switch           = (PackSwitch_t)(int)v;
        else if (!strncmp(ev->param, "cabin_temp_setpoint",   32)) inp->cabin_temp_setpoint   = v;
        else if (!strncmp(ev->param, "cabin_press_setpoint",  32)) inp->cabin_press_setpoint  = v;
        else if (!strncmp(ev->param, "cabin_temp_initial",    32)) inp->cabin_temp_initial    = v;
        else if (!strncmp(ev->param, "reset_cmd",             32)) inp->reset_cmd             = (int)v;
        else if (!strncmp(ev->param, "inject_valve_jam",      32)) inp->inject_valve_jam      = (int)v;
        else if (!strncmp(ev->param, "inject_line_break",     32)) inp->inject_line_break     = (int)v;
        else if (!strncmp(ev->param, "inject_press_loss",     32)) inp->inject_press_loss     = (int)v;
        else if (!strncmp(ev->param, "inject_ch_fault",       32)) inp->inject_ch_fault       = (int)v;
        else if (!strncmp(ev->param, "inject_temp_high",      32)) inp->inject_temp_high      = (int)v;
        else if (!strncmp(ev->param, "sim_duration",          32)) io->sim_time_max           = v;
        else {
            fprintf(stderr, "[INOUT] t=%.2f: неизвестный параметр '%s'\n",
                    t, ev->param);
        }

        io->next_event++;
    }
}

/* =========================================================================
 * ЗАГОЛОВОК CSV-ЛОГА
 * ========================================================================= */
void inout_write_header(FILE *f)
{
    fprintf(f,
        "time_s,"
        /* Команды КУ */
        "pack_valve_cmd,outflow_valve_cmd,trim_valve_cmd,recirc_fan_cmd,anti_ice_cmd,"
        /* Физика: положения */
        "pack_valve_pos,outflow_valve_pos,"
        /* Температурный тракт */
        "T_after_PHX,T_after_COMP,T_after_MHX,"
        "T_after_REGEN_H,T_after_COND,T_after_TURB,T_mix,"
        /* Кабина */
        "cabin_temp,cabin_press,cabin_hum,"
        /* Датчики */
        "sens_T_cabin_V,sens_P_cabin_V,sens_T_pack_V,"
        "valve_fbk,vent_active,"
        /* Отказы и состояния КА */
        "fault_flags,cu_state_A,cu_state_B,active_ch,"
        /* ARINC-429 */
        "arinc_temp,arinc_press,arinc_status\n");
}

/* =========================================================================
 * ЗАПИСЬ СТРОКИ ДАННЫХ В CSV
 * ========================================================================= */
void inout_write_row(FILE *f, float t, const Input_t *inp,
                     const Bus_t *bus, const Output_t *out)
{
    (void)inp; /* inp зарезервирован для возможного расширения */
    fprintf(f,
        "%.3f,"
        "%.4f,%.4f,%.4f,%.4f,%d,"
        "%.4f,%.4f,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.4f,%.3f,"
        "%.4f,%.4f,%.4f,"
        "%d,%d,"
        "%u,%s,%s,%d,"
        "%u,%u,%u\n",
        t,
        /* Команды */
        bus->pack_valve_cmd, bus->outflow_valve_cmd,
        bus->trim_valve_cmd, bus->recirc_fan_cmd, bus->anti_ice_valve_cmd,
        /* Положения */
        out->pack_valve_pos, out->outflow_valve_pos,
        /* Тракт */
        out->t_after_phx, out->t_after_comp, out->t_after_mhx,
        out->t_after_regen_hot, out->t_after_cond,
        out->t_after_turbine, out->t_mix,
        /* Кабина */
        out->cabin_temp, out->cabin_pressure, out->cabin_humidity,
        /* Датчики */
        out->sens_cabin_temp, out->sens_cabin_press, out->sens_pack_out_temp,
        out->sens_pack_valve_fbk, out->sens_vent_active,
        /* Отказы / КА */
        (unsigned int)out->fault_flags,
        cu_state_name((CU_State_t)out->cu_state_a),
        cu_state_name((CU_State_t)out->cu_state_b),
        bus->active_channel,
        /* ARINC */
        (unsigned int)out->arinc_cabin_temp,
        (unsigned int)out->arinc_cabin_press,
        (unsigned int)out->arinc_status
    );
}

/* =========================================================================
 * КОНСОЛЬНЫЙ ВЫВОД (каждые ~1 с симуляционного времени)
 * ========================================================================= */
void inout_print_row(float t, const Input_t *inp,
                     const Bus_t *bus, const Output_t *out)
{
    (void)inp; (void)bus;
    printf("t=%6.1f s | "
           "Tcab=%5.1f°C  Pcab=%.3f bar | "
           "Tpak=%5.1f°C | "
           "Tturb=%5.1f°C | "
           "Valve=%.2f | "
           "CUa=%-12s CUb=%-12s | "
           "Faults=0x%04X | "
           "ARINC429: BCD=0x%08X BNR=0x%08X DW=0x%08X\n",
           t,
           out->cabin_temp, out->cabin_pressure,
           out->t_mix, out->t_after_turbine,
           out->pack_valve_pos,
           cu_state_name((CU_State_t)out->cu_state_a),
           cu_state_name((CU_State_t)out->cu_state_b),
           (unsigned int)out->fault_flags,
           (unsigned int)out->arinc_cabin_temp,
           (unsigned int)out->arinc_cabin_press,
           (unsigned int)out->arinc_status);
}
