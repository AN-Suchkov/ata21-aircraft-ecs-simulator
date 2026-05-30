/*
 * 21_manager.c — Диспетчер симуляции СКВ (ATA-21).
 *
 * Содержит главный цикл симуляции с шагом DT = 0.02 с.
 * Порядок вызовов за каждый шаг:
 *   1. inout_apply_events  — применить события сценария к Input_t
 *   2. cu_step             — контроллер: вычислить команды → Bus_t
 *   3. phys_step           — физика:     выполнить команды, обновить Output_t
 *   4. inout_write_row     — записать строку в лог
 *   5. inout_print_row     — вывод в консоль (каждую секунду)
 *
 * Аргументы командной строки:
 *   argv[1] — файл сценария (необязателен, по умолчанию scenario_default.csv)
 *   argv[2] — файл лога     (необязателен, по умолчанию log_21.csv)
 *
 * Стандарт: ISO C99. Глобальных переменных нет.
 */

#include "21_defs.h"
#include <stdio.h>
#include <string.h>   /* memset */
#include <locale.h>   /* setlocale для UTF-8 */

/* =========================================================================
 * НАЧАЛЬНЫЕ УСЛОВИЯ ДЛЯ Input_t
 * Устанавливают «стоянку с заведёнными двигателями» если нет сценария.
 * ========================================================================= */
static void input_defaults(Input_t *inp)
{
    memset(inp, 0, sizeof(Input_t));

    inp->pwr_28v              = 1;
    inp->pwr_115v             = 1;
    inp->engine_running       = 0;
    inp->apu_avail            = 1;
    inp->eng_bleed_pressure   = BLEED_PRESS_NOM;
    inp->eng_bleed_temp       = BLEED_TEMP_NOM;
    inp->ram_air_temp         = -30.0f;   /* крейсерская высота */
    inp->ram_air_pressure     = 0.265f;   /* ≈ 10 000 м */
    inp->pack_switch          = PACK_OFF;
    inp->cabin_temp_setpoint  = CABIN_TEMP_DEFAULT;
    inp->cabin_press_setpoint = CABIN_PRESS_DEFAULT;
    inp->cabin_temp_initial   = 20.0f;
    inp->reset_cmd            = 0;

    inp->inject_valve_jam     = 0;
    inp->inject_line_break    = 0;
    inp->inject_press_loss    = 0;
    inp->inject_ch_fault      = 0;
}

/* =========================================================================
 * ТОЧКА ВХОДА
 * ========================================================================= */
int main(int argc, char *argv[])
{
    /* Установка UTF-8 кодировки для вывода кириллицы в консоли.
     * LC_NUMERIC принудительно возвращается к "C", чтобы числа
     * в CSV-логе записывались с точкой, а не запятой. */
    setlocale(LC_ALL, ".UTF-8");
    setlocale(LC_NUMERIC, "C");

    /* --- Параметры запуска --- */
    const char *scenario_file = (argc > 1) ? argv[1] : "scenarios/scenario_default.csv";
    const char *log_file      = (argc > 2) ? argv[2] : "logs/log_21.csv";

    /* --- Структуры данных (все в стеке — malloc запрещён) --- */
    Input_t      inp;
    Bus_t        bus;
    Output_t     out;
    PhysState_t  phys;
    CU_Channel_t cu_a, cu_b;
    InoutState_t io;

    /* --- Инициализация --- */
    input_defaults(&inp);
    memset(&bus, 0, sizeof(Bus_t));
    memset(&out, 0, sizeof(Output_t));

    inout_init(&io, scenario_file);

    /* Применяем события сценария в момент t=0 (например, cabin_temp_initial) */
    inout_apply_events(&io, 0.0f, &inp);

    phys_init(&phys, &inp);
    cu_init(&cu_a, 0);   /* Канал A */
    cu_init(&cu_b, 1);   /* Канал B */

    /* --- Открытие лог-файла --- */
    FILE *log_f = fopen(log_file, "w");
    if (log_f == NULL) {
        fprintf(stderr, "[MANAGER] Не удалось открыть файл лога: %s\n", log_file);
        return 1;
    }
    inout_write_header(log_f);

    printf("============================================================\n");
    printf("  Симуляция СКВ SSJ-100 (ATA-21)  |  DT=%.3f с  |  T_max=%.0f с\n",
           DT, io.sim_time_max);
    printf("  Сценарий: %-30s  Лог: %s\n", scenario_file, log_file);
    printf("============================================================\n");

    /* --- Главный цикл симуляции --- */
    float t = 0.0f;
    float print_timer = 0.0f;   /* таймер консольного вывода */

    while (t <= io.sim_time_max)
    {
        /* Шаг 1: применить события сценария */
        inout_apply_events(&io, t, &inp);

        /* Шаг 2: контроллер */
        cu_step(&cu_a, &cu_b, &inp, &out, &bus);

        /* Шаг 3: физическая модель */
        phys_step(&inp, &bus, &phys, &out);

        /* Обновляем состояния КА в Output (для лога) */
        out.cu_state_a  = (int)cu_a.state;
        out.cu_state_b  = (int)cu_b.state;
        out.ch_active   = bus.active_channel;

        /* Шаг 4: запись в лог */
        inout_write_row(log_f, t, &inp, &bus, &out);

        /* Шаг 5: вывод в консоль раз в секунду */
        print_timer += DT;
        if (print_timer >= 1.0f) {
            print_timer = 0.0f;
            inout_print_row(t, &inp, &bus, &out);
        }

        t += DT;
    }

    fclose(log_f);

    printf("============================================================\n");
    printf("  Симуляция завершена (T=%.0f с). Лог: %s\n", io.sim_time_max, log_file);
    printf("============================================================\n");

    return 0;
}
