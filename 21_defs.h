/*
 * 21_defs.h — Объявление структур данных, констант и прототипов функций.
 *              Модель системы кондиционирования воздуха (СКВ) SSJ-100.
 *
 * Стандарт: ISO C99.
 * Запрещено: глобальные переменные, extern, malloc, #include <math.h>.
 * Взаимодействие между модулями — только через структуры данных по указателю.
 *
 * КБВО-06-23: Павлов С., Сучков А.
 */

#ifndef ATA21_DEFS_H
#define ATA21_DEFS_H

#include <stdint.h>   /* uint32_t, uint8_t */
#include <stdio.h>    /* FILE */

/* =========================================================================
 * КОНСТАНТЫ СИМУЛЯЦИИ
 * ========================================================================= */

#define DT                  0.02f    /* Шаг симуляции, с (50 Гц) */
#define SIM_TIME_MAX        1000.0f   /* Максимальное время, с */

/* =========================================================================
 * ФИЗИЧЕСКИЕ КОНСТАНТЫ ВОЗДУХА
 * ========================================================================= */

#define GAMMA_AIR           1.4f     /* Показатель адиабаты */
#define CP_AIR              1005.0f  /* Теплоёмкость при P=const, Дж/(кг·К) */
#define ISENTROPIC_EXP      0.2857f  /* (γ−1)/γ = 0.4/1.4 */
#define T_KELVIN_OFFSET     273.15f  /* Перевод °С → К */

/* =========================================================================
 * ПАРАМЕТРЫ СКВ
 * ========================================================================= */

/* Источник воздуха (двигатель) */
#define BLEED_PRESS_NOM     3.0f     /* Давление отбираемого воздуха, бар */
#define BLEED_TEMP_NOM      200.0f   /* Температура отбираемого воздуха, °С */

/* ТХУ (турбохолодильная установка) */
#define TCU_COMP_RATIO      1.333f   /* Степень повышения давления в компрессоре */
#define TCU_TURB_PRESS_IN   4.0f     /* Давление на входе турбины, бар */
#define TCU_TURB_PRESS_OUT  1.0f     /* Давление на выходе турбины, бар */
#define TCU_COMP_TEMP_RISE  50.0f    /* Нагрев воздуха в компрессоре ТХУ, °С */
#define TCU_TURB_EFF        0.85f    /* КПД турбины (учёт потерь) */

/* Теплообменники */
#define HX_PRIM_EFF         0.80f    /* КПД первичного ТО */
#define HX_MAIN_EFF         0.85f    /* КПД основного ТО */
#define REGEN_EFF           0.75f    /* КПД регенератора */
#define COND_TARGET_TEMP    7.0f     /* Целевая темп. за конденсатором, °С */

/* Камера смешения */
#define MIX_FRESH_RATIO     0.50f    /* Доля свежего воздуха (50 %) */

/* Расход */
#define PACK_FLOW_NOM       0.5f     /* Номинальный расход через пак, кг/с */

/* Коэффициенты модели давления кабины */
#define PRESS_INFLOW_COEFF  0.004f   /* бар/(кг/с)/с — рост давления от нагнетания отбором */
#define PRESS_OUTFLOW_COEFF 0.005f   /* бар/(доля·бар)/с — сброс давления через вытяжной клапан */

/* Гермокабина */
#define CABIN_AIR_MASS      160.0f   /* Масса воздуха в кабине, кг */
#define CABIN_THERM_MASS    150000.0f  /* Тепловая масса, Дж/°С */
#define CABIN_UA_FUSELAGE   80.0f   /* Коэф. теплопередачи через обшивку, Вт/°С */
#define CABIN_HEAT_GEN      5000.0f  /* Тепловыделение пассажиров/оборудования, Вт */

/* Динамика исполнительных механизмов */
#define VALVE_RATE_MAX      0.5f     /* Макс. скорость клапана, ед/с */
#define FAN_RATE_MAX        0.3f     /* Макс. скорость изменения об/мин вентилятора */

/* =========================================================================
 * ПОРОГИ И УСТАВКИ
 * ========================================================================= */

#define CABIN_TEMP_DEFAULT   22.0f   /* Уставка температуры кабины, °С */
#define CABIN_PRESS_DEFAULT  0.753f  /* Уставка давления кабины (≈2400 м), бар */
#define STARTUP_TIMER_MAX    30.0f   /* Время запуска системы, с */
#define FAULT_CONFIRM_TIME   2.0f    /* Время подтверждения отказа, с */
#define TEMP_FAULT_THRESH    60.0f   /* Порог температуры за конденсатором, °С */
#define PRESS_FAULT_THRESH   0.60f   /* Мин. давление кабины для отказа, бар */
#define CHAN_DIFF_THRESH      0.15f   /* Порог рассогласования каналов КУ */

/* =========================================================================
 * ПЕРЕЧИСЛЕНИЯ
 * ========================================================================= */

/*
 * Состояния конечного автомата контроллера (CU State Machine).
 * Переходы описаны в skv_cu.c.
 */
typedef enum {
    CU_STATE_INIT       = 0,  /* Инициализация (первый шаг) */
    CU_STATE_IDLE       = 1,  /* Ожидание (система отключена) */
    CU_STATE_STARTING   = 2,  /* Запуск: плавное открытие клапанов */
    CU_STATE_NORMAL     = 3,  /* Рабочий режим: ПИД-регулирование */
    CU_STATE_FAULT_CU   = 4,  /* Отказ этого канала КУ */
    CU_STATE_FAULT_PHYS = 5,  /* Физический отказ зафиксирован */
    CU_STATE_RESET      = 6   /* Сброс после устранения причины отказа */
} CU_State_t;

/*
 * Коды отказов (битовая маска).
 */
typedef enum {
    FAULT_NONE       = 0x0000,
    FAULT_VALVE_JAM  = 0x0001,  /* Заклинивание пак-клапана */
    FAULT_LINE_BREAK = 0x0002,  /* Обрыв магистрали отбора воздуха */
    FAULT_PRESS_LOSS = 0x0004,  /* Потеря герметичности кабины */
    FAULT_TEMP_HIGH  = 0x0008,  /* Перегрев (Т за конденсатором > порога) */
    FAULT_CH_DISAGR  = 0x0010,  /* Рассогласование каналов КУ */
    FAULT_SENSOR_PWR = 0x0020   /* Отказ питания датчика */
} FaultCode_t;

/*
 * Положение переключателя пака на пульте управления.
 */
typedef enum {
    PACK_OFF    = 0,
    PACK_AUTO   = 1,
    PACK_MANUAL = 2
} PackSwitch_t;

/* =========================================================================
 * СТРУКТУРЫ ДАННЫХ
 * ========================================================================= */

/*
 * Input_t — Внешние входы в систему.
 *
 * Источники сигналов:
 *   - Система электроснабжения → pwr_28v, pwr_115v
 *   - Двигатель/ВСУ → eng_bleed_*, apu_avail, engine_running
 *   - Атмосфера → ram_air_*
 *   - Пульт управления (CPCS) → pack_switch, setpoints, reset_cmd
 *   - Инжектор отказов → inject_* (только для тестовых сценариев)
 */
typedef struct {
    /* --- Система электроснабжения --- */
    int          pwr_28v;             /* 28 В постоянного тока (дискрет: 0/1) */
    int          pwr_115v;            /* 115 В переменного тока (дискрет: 0/1) */

    /* --- Двигатель и ВСУ --- */
    int          engine_running;      /* Двигатель работает (дискрет) */
    int          apu_avail;           /* ВСУ доступна (дискрет) */
    float        eng_bleed_pressure;  /* Давление отбираемого воздуха, бар (аналог) */
    float        eng_bleed_temp;      /* Температура отбираемого воздуха, °С (аналог) */

    /* --- Атмосфера --- */
    float        ram_air_temp;        /* Температура забортного воздуха, °С (аналог) */
    float        ram_air_pressure;    /* Давление забортного воздуха, бар (аналог) */

    /* --- Пульт управления (CPCS Panel) --- */
    PackSwitch_t pack_switch;         /* Положение переключателя пака (дискрет 3-позиц.) */
    float        cabin_temp_setpoint; /* Уставка температуры кабины, °С (аналог) */
    float        cabin_press_setpoint;/* Уставка давления кабины, бар (аналог) */
    float        cabin_temp_initial;  /* Начальная температура кабины при инициализации, °С */
    int          reset_cmd;           /* Команда сброса: 0→1 = фронт (дискрет) */

    /* --- Инжектор отказов (только для сценариев тестирования) --- */
    int          inject_valve_jam;    /* Заклинить пак-клапан */
    int          inject_line_break;   /* Обрыв магистрали */
    int          inject_press_loss;   /* Разгерметизация */
    int          inject_ch_fault;     /* Принудительный отказ канала A */
    int          inject_temp_high;    /* Инжекция перегрева за конденсатором */
} Input_t;

/*
 * Bus_t — Внутрисистемный интерфейс: команды от КУ к физической части.
 *
 * Тип сигналов: аналоговые 0–10 В (положение клапанов, скорость вентилятора),
 * дискретные 0/28 В (ПОС клапан).
 * Обоснование выбора: нормированный аналог (0–10 В) исключает неоднозначность
 * при длинных линиях. Дискретные сигналы используются для двухпозиционных команд.
 */
typedef struct {
    float pack_valve_cmd;       /* Команда пак-клапана [0.0–1.0] → 0–10 В */
    float outflow_valve_cmd;    /* Команда вытяжного клапана [0.0–1.0] → 0–10 В */
    float trim_valve_cmd;       /* Команда трим-клапана [0.0–1.0] → 0–10 В */
    float recirc_fan_cmd;       /* Команда вентилятора рециркуляции [0.0–1.0] */
    int   anti_ice_valve_cmd;   /* ПОС конденсатора: 0=выкл / 1=вкл (28 В дискрет) */
    int   active_channel;       /* Активный канал: 0=A, 1=B (информация) */
    int   disagr_detected;      /* 1 = рассогласование каналов зафиксировано */
} Bus_t;

/*
 * Output_t — Физические выходы + показания датчиков + ARINC-429 слова.
 *
 * Тип сигналов датчиков:
 *   - Аналоговые 0–10 В: температуры, давление
 *   - Дискретные 0/28 В: концевики клапанов, признаки работы агрегатов
 *   - Цифровые ARINC-429: температура (BCD), давление (BNR), статус (DW)
 */
typedef struct {
    /* --- Физические параметры (истинные значения) --- */
    float pack_valve_pos;       /* Положение пак-клапана [0.0–1.0] */
    float outflow_valve_pos;    /* Положение вытяжного клапана [0.0–1.0] */
    float bleed_flow;           /* Расход отбираемого воздуха, кг/с */

    /* Температурный тракт (для анализа и логирования) */
    float t_after_phx;          /* Температура за первичным ТО, °С */
    float t_after_comp;         /* Температура за компрессором ТХУ, °С */
    float t_after_mhx;          /* Температура за основным ТО, °С */
    float t_after_regen_hot;    /* Температура за регенератором (горяч. тракт), °С */
    float t_after_cond;         /* Температура за конденсатором, °С */
    float t_after_turbine;      /* Температура за турбиной ТХУ, °С */
    float t_mix;                /* Температура за камерой смешения, °С */

    /* Параметры гермокабины */
    float cabin_temp;           /* Температура в кабине, °С */
    float cabin_pressure;       /* Давление в кабине, бар */
    float cabin_humidity;       /* Относительная влажность [0.0–1.0] */

    /* --- Показания датчиков (с учётом питания и отказов) --- */
    /* Аналоговые: 0–10 В */
    float sens_cabin_temp;      /* Датчик Т кабины: 0 В = −50°С, 10 В = +80°С */
    float sens_cabin_press;     /* Датчик P кабины: 0 В = 0 бар, 10 В = 2 бар */
    float sens_pack_out_temp;   /* Датчик Т выхода пака: 0 В = −60°С, 10 В = +100°С */
    /* Дискретные: 0/1 (0 = нет сигнала / 1 = 28 В) */
    int   sens_pack_valve_fbk;  /* Концевик пак-клапана (1 = клапан открыт) */
    int   sens_vent_active;     /* Признак работы вентилятора рециркуляции */

    /* --- Флаги отказов (битовая маска FaultCode_t) --- */
    uint32_t fault_flags;

    /* Активный канал КУ */
    int  ch_active;

    /* --- ARINC-429 выходные слова --- */
    /* Обоснование: стандарт ARINC-429 — штатный интерфейс авиационного борта.
     * BCD для индикации на дисплее экипажа, BNR для FMS, DW для CAS-сообщений. */
    uint32_t arinc_cabin_temp;  /* BCD-слово, метка 0201(8), температура кабины */
    uint32_t arinc_cabin_press; /* BNR-слово, метка 0203(8), давление кабины */
    uint32_t arinc_status;      /* DW-слово,  метка 0270(8), флаги статуса/отказов */

    /* Состояния КА (для лога) */
    int cu_state_a;
    int cu_state_b;
} Output_t;

/*
 * PhysState_t — Внутренние переменные физической части между шагами симуляции.
 * Передаётся по указателю; содержит всё «железо».
 */
typedef struct {
    /* Положения исполнительных механизмов (текущие, с учётом динамики) */
    float pack_valve_pos;
    float outflow_valve_pos;
    float trim_valve_pos;
    float recirc_fan_speed;

    /* Температурный тракт (динамические состояния) */
    float t_after_phx;
    float t_after_comp;
    float t_after_mhx;
    float t_after_regen_hot;
    float t_after_cond;
    float t_after_regen_cold;
    float t_after_turbine;
    float t_mix;

    /* Гермокабина */
    float cabin_temp;
    float cabin_pressure;
    float cabin_humidity;

    /* Флаги инжектированных отказов */
    int   valve_jammed;         /* Клапан заклинен */
    float valve_jam_pos;        /* Положение в момент заклинивания */
    int   line_broken;          /* Обрыв магистрали */
    int   pressurization_loss;  /* Разгерметизация */

    int   anti_ice_active;      /* ПОС конденсатора включена */
    int   disagr_latched;       /* Защёлка: рассогласование каналов зафиксировано */
} PhysState_t;

/*
 * CU_Channel_t — Состояние одного канала контроллера (КУ).
 * Система имеет два независимых канала: A (ch_id=0) и B (ch_id=1).
 */
typedef struct {
    int        ch_id;           /* Идентификатор: 0=A, 1=B */
    CU_State_t state;           /* Текущее состояние КА */
    int        active;          /* 1 = этот канал активный (управляет) */
    int        self_fault;      /* 1 = обнаружен собственный отказ */

    /* Таймеры КА */
    float startup_timer;
    float fault_timer;

    /* Состояния ПИД-регуляторов */
    float temp_pid_integral;
    float temp_pid_prev_err;
    float press_pid_integral;
    float press_pid_prev_err;

    /* Вычисленные команды этого канала */
    float pack_valve_cmd;
    float outflow_valve_cmd;
    float trim_valve_cmd;
    float recirc_fan_cmd;
    int   anti_ice_cmd;

    /* Предыдущие значения для обнаружения фронтов */
    int prev_reset_cmd;
} CU_Channel_t;

/*
 * ScenarioEvent_t — Событие в файле сценария.
 */
typedef struct {
    float time;             /* Момент применения события, с */
    char  param[32];        /* Имя параметра (поле Input_t) */
    float value;            /* Новое значение */
} ScenarioEvent_t;

#define SCENARIO_MAX_EVENTS 256

/*
 * InoutState_t — Состояние модуля ввода-вывода.
 */
typedef struct {
    ScenarioEvent_t events[SCENARIO_MAX_EVENTS]; /* Массив событий сценария */
    int             event_count;                  /* Всего событий */
    int             next_event;                   /* Индекс следующего не применённого */
    float           sim_time_max;                 /* Длительность симуляции (из сценария) */
} InoutState_t;

/* =========================================================================
 * ПРОТОТИПЫ ФУНКЦИЙ (объявления)
 * ========================================================================= */

/* --- skv_lib.c: математическая библиотека --- */
float lib_integral(float x, float dt, float *state);
float lib_delay(float x, float delay_s, float dt, float *buf, int buf_size, int *head);
int   lib_latch(int set, int reset, int *state);
float lib_abs(float x);
float lib_sin(float x);
float lib_cos(float x);
float lib_power(float base, float exponent);
float lib_interpolate(float x, const float *xs, const float *ys, int n);
float lib_limit(float x, float lo, float hi);
float lib_rate_limit(float x, float rate, float dt, float *prev);
uint32_t lib_arinc_bnr(float value, float lsb, uint8_t label, int ssm_ok);
uint32_t lib_arinc_bcd(int value, uint8_t label, int ssm_ok);
uint32_t lib_arinc_dw(uint32_t flags, uint8_t label, int ssm_ok);

/* --- skv_phys.c: физическая модель --- */
void phys_init(PhysState_t *ps, const Input_t *inp);
void phys_step(const Input_t *inp, const Bus_t *bus, PhysState_t *ps, Output_t *out);

/* --- skv_cu.c: контроллер (КУ) --- */
void cu_init(CU_Channel_t *ch, int ch_id);
void cu_step(CU_Channel_t *ch_a, CU_Channel_t *ch_b,
             const Input_t *inp, const Output_t *sens, Bus_t *bus);
const char *cu_state_name(CU_State_t s);

/* --- skv_inout.c: ввод-вывод --- */
void inout_init(InoutState_t *io, const char *scenario_file);
void inout_apply_events(InoutState_t *io, float t, Input_t *inp);
void inout_write_header(FILE *f);
void inout_write_row(FILE *f, float t, const Input_t *inp,
                     const Bus_t *bus, const Output_t *out);
void inout_print_row(float t, const Input_t *inp,
                     const Bus_t *bus, const Output_t *out);

#endif /* ATA21_DEFS_H */
