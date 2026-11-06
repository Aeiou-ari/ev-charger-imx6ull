/*
 * app_sensors.c - 新能源汽车智能充电桩数据模型 + 模拟 + 故障判定 + 状态机
 *
 * 传感器真实接口现状(用户已替换):
 *   LM75     /dev/lm75_misc     (I2C)   -> 柜内温度, 真实读取
 *   ADXL345  /dev/adxl345_misc  (SPI)   -> 震动/倾倒, 真实读取(三轴 g 值)
 *
 * 预留接口(当前模拟, 用户后续替换为真实驱动即可, UI 与逻辑不变):
 *   DHT11    1-Wire   -> 真实 /dev/dht11_misc           (枪头温度, 量程0~50℃)
 *   RS485 电表 UART    -> 预留 meter_read_once()   (电压/电流/功率/电量/费用)
 *   MQ 烟雾    ADC     -> 预留 smoke_read_once()   (烟雾浓度)
 *   CAN                -> 预留 can_read_once()     (车辆连接/SOC)
 *
 * 模拟剧本演示: SIM_FAULT_DEMO=1 时周期性触发各故障, 便于界面联调;
 * 真实传感器接入后, 真实读数会覆盖模拟值, 演示故障自然不再出现.
 */
#define _DEFAULT_SOURCE    /* 让 usleep 在 -std=c99 下正常声明 */
#include "app_ui.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include "../mqtt.h"        /* OneNET MQTT 上报接口 (main.c 实现 mqtt_init/mqtt_send) */

/* ================= 演示/配置 ================= */
#define SIM_FAULT_DEMO   1       /* 1=周期触发故障演示; 0=纯正常模拟 */
#define MQTT_REPORT_ENABLE 1     /* 1=每 10 秒上报 OneNET; 0=关闭上报 */
#define MQTT_REPORT_PERIOD 10    /* 上报周期(秒) */
#define PRICE_PER_KWH    1.0f    /* 模拟电价 元/kWh, 后续可改为配置 */
#define SOC_FULL         100.0f  /* SOC 充满阈值 */

/* ================= 全局数据 ================= */
sensor_t g_sensors[SID_MAX] = {
    /* name        unit value  lo   hi   dmin dmax   en    alarm */
    [SID_VOLT]     = { "电压",      "V",   220.0f,   0.0f, 0.0f, 0.0f, 300.0f,  true, false, {0}, 0 },
    [SID_CURR]     = { "电流",      "A",     0.0f,   0.0f, 0.0f, 0.0f,  40.0f,  true, false, {0}, 0 },
    [SID_POWER]    = { "功率",      "kW",    0.0f,   0.0f, 0.0f, 0.0f,  20.0f,  true, false, {0}, 0 },
    [SID_ENERGY]   = { "累计电量",  "kWh",   0.0f,   0.0f, 0.0f, 0.0f, 100.0f,  true, false, {0}, 0 },
    [SID_FEE]      = { "本次费用",  "元",     0.0f,   0.0f, 0.0f, 0.0f, 100.0f,  true, false, {0}, 0 },
    [SID_GUN_TEMP] = { "枪头温度",  "C",     26.0f,  0.0f, 85.0f, 0.0f, 100.0f,  true, false, {0}, 0 },
    [SID_CAB_TEMP] = { "柜内温度",  "C",     28.0f,  0.0f, 60.0f, 0.0f, 100.0f,  true, false, {0}, 0 },
    [SID_SMOKE]    = { "烟雾浓度",  "ppm",  128.0f,  0.0f,300.0f, 0.0f, 500.0f,  true, false, {0}, 0 },
    [SID_SOC]      = { "电池电量",  "%",      0.0f,  0.0f,100.0f, 0.0f, 100.0f,  true, false, {0}, 0 },
};

const sensor_group_t g_groups[GROUP_MAX] = {
    { 2, { SID_VOLT,   SID_CURR    }, "电能计量" },
    { 2, { SID_GUN_TEMP, SID_CAB_TEMP }, "温度监测" },
    { 1, { SID_SMOKE,  SID_SMOKE   }, "安全监测" },
    { 1, { SID_SOC,    SID_SOC     }, "充电进度" },
};

charger_t g_charger = {
    .state = CHG_IDLE,
    .fault = FAULT_NONE,
    .can_connected = true,
    .can_lost_cnt = 0,
    .relay_on = false,
    .buzzer_on = false,
    .move_flags = 0,
    .soc = 0.0f,
    .energy_session = 0.0f,
};

/* ================= 历史充电记录(内存) ================= */
static charge_record_t g_records[CHARGE_RECORD_MAX];
static int g_rec_cnt = 0;
static int g_rec_head = 0;      /* 环形头 */

int charge_record_add(const charge_record_t *rec)
{
    if (!rec) return 0;
    if (g_rec_cnt < CHARGE_RECORD_MAX) {
        g_records[g_rec_cnt++] = *rec;
        return 1;
    }
    /* 满则覆盖最旧 */
    g_records[g_rec_head] = *rec;
    g_rec_head = (g_rec_head + 1) % CHARGE_RECORD_MAX;
    return 1;
}

int charge_record_count(void) { return g_rec_cnt; }

const charge_record_t *charge_record_get(int idx)
{
    if (idx < 0 || idx >= g_rec_cnt) return NULL;
    return &g_records[idx];
}

const char *charge_state_name(charge_state_t s)
{
    switch (s) {
    case CHG_IDLE:     return "空闲";
    case CHG_PENDING:  return "待启动";
    case CHG_CHARGING: return "充电中";
    case CHG_FINISH:   return "充满结算";
    case CHG_FAULT:    return "故障";
    default:           return "空闲";
    }
}

const char *fault_name(fault_type_t f)
{
    switch (f) {
    case FAULT_GUN_OVER_TEMP: return "枪头过温";
    case FAULT_CAB_OVER_TEMP: return "柜内过温";
    case FAULT_SMOKE:         return "检测到烟雾";
    case FAULT_MOVE:          return "设备异常移动";
    case FAULT_CAN_LOST:      return "车辆通信中断";
    default:                  return "";
    }
}

/* ================= 工具 ================= */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void rand_init(void)
{
    static bool seeded = false;
    if (!seeded) {
        srand((unsigned)time(NULL));
        seeded = true;
    }
}

/* ±range 的随机小步进 */
static float step(float range)
{
    return ((rand() % 2001) - 1000) / 1000.0f * range;
}

static void push_hist(sensor_t *s, float v)
{
    if (s->hist_cnt < APP_HIST_LEN) {
        s->hist[s->hist_cnt++] = v;
    } else {
        memmove(s->hist, s->hist + 1, (APP_HIST_LEN - 1) * sizeof(float));
        s->hist[APP_HIST_LEN - 1] = v;
    }
}

/* ================= 真实传感器接口 ================= */

/* LM75(I2C): /dev/lm75_misc, 12位温度 0.125°C/LSB; 失败保持旧值 */
static void lm75_read_once(float *temp)
{
    static int fd = -1;
    unsigned char b[2];
    short raw;

    if (fd < 0) {
        fd = open("/dev/lm75_misc", O_RDWR);
        if (fd < 0) return;
    }
    if (read(fd, b, 2) != 2) return;

    raw = (short)((b[0] << 8) | b[1]);
    *temp = (float)(raw >> 5) * 0.125f;
}

/* ADXL345(SPI): /dev/adxl345_misc, 3个short(x/y/z), ±16g 3.9mg/LSB
 * 返回三轴 g 值; 失败返回 -1 */
static int adxl345_read_once(float *gx, float *gy, float *gz)
{
    static int fd = -1;
    short acc[3];

    if (fd < 0) {
        fd = open("/dev/adxl345_misc", O_RDWR);
        if (fd < 0) return -1;
    }
    if (read(fd, acc, sizeof(acc)) != (ssize_t)sizeof(acc)) return -1;

    printf("[adxl-raw] acc=%d %d %d\n", acc[0], acc[1], acc[2]);

    *gx = acc[0] * 0.0039f;
    *gy = acc[1] * 0.0039f;
    *gz = acc[2] * 0.0039f;
    return 0;
}

/* ================= 预留接口(当前模拟) ================= */

/* DHT11(1-Wire) 枪头温度 - 真实接口(/dev/dht11_misc, 原项目驱动).
 * read 返回 4 字节: [0]=湿度整数 [1]=湿度小数 [2]=温度整数 [3]=温度小数.
 * 驱动 read 为 GPIO 忙等(约 20ms), 可在 UI tick 内同步调用, 不阻塞界面.
 * 注意: DHT11 量程 0~50℃, 85℃ 枪头过温报警无法由真实 DHT11 触发,
 *       SIM_FAULT_DEMO=1 的模拟剧本可演示该故障. 读取失败自动回落模拟值(不崩). */
static void dht11_read_once(float *temp)
{
    static int fd = -1;
    unsigned char data[4];

    if (fd < 0) {
        fd = open("/dev/dht11_misc", O_RDWR);
        if (fd < 0) return;              /* 未接驱动: 保持模拟 */
    }
    if (read(fd, data, sizeof(data)) != (ssize_t)sizeof(data))
        return;                          /* 读失败: 保持旧值 */

    *temp = (float)data[2] + (float)data[3] * 0.1f;  /* 温度 整数.小数 */
}

/* 预留: RS485 电表(Modbus RTU over UART) 电压/电流/功率/累计电量/本次费用.
 * 接入真实驱动后替换本函数内部实现即可 */
static void meter_read_once(float *volt, float *curr, float *power,
                            float *energy, float *fee)
{
    (void)volt; (void)curr; (void)power; (void)energy; (void)fee;
}

/* 预留: MQ 烟雾浓度(ADC).
 * 接入真实驱动后替换为 open("/dev/adc...")+read */
static void smoke_read_once(float *ppm)
{
    (void)ppm;
}

/* 预留: CAN 报文(车辆连接状态 + SOC). 模拟: 连接状态由 tick 内剧本控制 */
static void can_read_once(bool *connected, float *soc)
{
    (void)connected; (void)soc;
}

/* ================= 模拟剧本(故障演示) ================= */
static long sim_t = 0;
static int g_fault_cooldown = 0;   /* 故障确认后冷却秒数 */

/* 烟雾: 约每 60 秒冲高一次, 触发"检测到烟雾" */
static void sim_smoke_tick(void)
{
    sensor_t *s = &g_sensors[SID_SMOKE];
    s->value += step(8.0f);
#if SIM_FAULT_DEMO
    if (sim_t % 60 == 0)
        s->value += 200.0f + (rand() % 80);   /* 冲高触发 >300 */
    else if (s->value > 60.0f)
        s->value -= 35.0f;                    /* 脉冲后快速回落 */
#endif
    s->value = clampf(s->value, 20.0f, 490.0f);
}

/* 枪头温度: DHT11 真实值优先; 未接驱动时按模拟游走,
 * 演示时约每 180 秒冲高到 88 触发过温(真实 DHT11 量程到不了 85℃) */
static void sim_gun_temp_tick(void)
{
    sensor_t *s = &g_sensors[SID_GUN_TEMP];
    if (g_charger.state == CHG_CHARGING)
        s->value += step(0.6f);
    else
        s->value += step(0.15f);      /* 空闲缓慢漂移 */
#if SIM_FAULT_DEMO
    if (sim_t % 180 == 0)
        s->value = 88.0f;             /* 触发 85℃ 过温演示 */
    else if (s->value > 85.0f)
        s->value -= 2.0f;             /* 过温后快速回落 */
#endif
    s->value = clampf(s->value, 20.0f, 92.0f);

    /* 真实 DHT11 在场时以真实读数覆盖(失败则保持模拟值) */
    dht11_read_once(&s->value);
}

/* 柜内温度: LM75 真实值优先; 演示时约每 360 秒冲高到 65 触发柜内过温 */
static void sim_cab_temp_tick(void)
{
    sensor_t *s = &g_sensors[SID_CAB_TEMP];
    float t = s->value + step(0.2f);
#if SIM_FAULT_DEMO
    if (sim_t % 360 == 0)
        t = 65.0f;                    /* 触发 60℃ 柜内过温演示 */
    else if (s->value > 60.0f)
        t = s->value - 2.0f;          /* 过温后快速回落 */
#endif
    s->value = clampf(t, 20.0f, 70.0f);

    /* 真实 LM75 在场时以真实读数覆盖(失败则保持模拟值) */
    lm75_read_once(&s->value);
}

/* 震动/倾倒: ADXL345 真实值优先; 设备不在时按剧本演示 */
static void sim_move_tick(void)
{
    float gx = 0.0f, gy = 0.0f, gz = 1.0f;
    float mag, pitch, roll;

    if (adxl345_read_once(&gx, &gy, &gz) == 0) {
        /* 真实数据: 加速度模长 >2.5g 视为震动; 倾角 >60° 视为倾倒 */
        mag = sqrtf(gx * gx + gy * gy + gz * gz);
        pitch = atan2f(-gx, sqrtf(gy * gy + gz * gz)) * 57.29578f;
        roll  = atan2f(gy,  sqrtf(gx * gx + gz * gz)) * 57.29578f;
        printf("[adxl] ok gx=%.2f gy=%.2f gz=%.2f mag=%.2f pitch=%.1f roll=%.1f flags=%d\n",
               gx, gy, gz, mag, pitch, roll, g_charger.move_flags);
        g_charger.move_flags = 0;
        if (mag > 2.5f)  g_charger.move_flags = 1;
        if (fabsf(pitch) > 60.0f || fabsf(roll) > 60.0f)
            g_charger.move_flags = 2;
        return;
    }

    printf("[adxl] read fail, use sim\n");
    /* 剧本演示(未接真实 ADXL345 时) */
#if SIM_FAULT_DEMO
    if (sim_t % 120 == 0) g_charger.move_flags = 1;   /* 震动 3 秒 */
    if (sim_t % 300 == 0) g_charger.move_flags = 2;   /* 倾倒 5 秒 */
    if (g_charger.move_flags == 1 && sim_t % 120 != 0 && (sim_t % 120) > 3)
        g_charger.move_flags = 0;
    if (g_charger.move_flags == 2 && (sim_t % 300) > 5)
        g_charger.move_flags = 0;
#else
    g_charger.move_flags = 0;
#endif
}

/* CAN 连接: 默认连接; 演示时约每 300 秒断开 8 秒(>5s 触发通信中断) */
static void sim_can_tick(void)
{
#if SIM_FAULT_DEMO
    if (sim_t % 300 == 0)
        g_charger.can_connected = false;
    if (!g_charger.can_connected && (sim_t % 300) > 8)
        g_charger.can_connected = true;
#else
    g_charger.can_connected = true;
#endif

    if (g_charger.can_connected)
        g_charger.can_lost_cnt = 0;
    else if (g_charger.can_lost_cnt < 9999)
        g_charger.can_lost_cnt++;
}

/* ================= 充电状态机 ================= */

/* 记录一条历史充电记录 */
static void record_add(int state)
{
    charge_record_t rec;
    time_t t = time(NULL);
    struct tm *ptm = localtime(&t);

    snprintf(rec.time, sizeof(rec.time), "%02d-%02d %02d:%02d",
             ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min);
    rec.energy = g_sensors[SID_ENERGY].value;
    rec.fee    = g_sensors[SID_FEE].value;
    rec.state  = state;
    charge_record_add(&rec);
}

/* 进入充满结算: 电压/电流/功率清零, 费用定格 */
static void enter_finish(int rec_state)
{
    g_charger.state = CHG_FINISH;
    g_charger.relay_on = false;
    g_charger.soc = SOC_FULL;
    g_sensors[SID_VOLT].value = 0.0f;
    g_sensors[SID_CURR].value = 0.0f;
    g_sensors[SID_POWER].value = 0.0f;
    record_add(rec_state);
}

/* 充电中: 电表游走 + SOC 增长 */
static void charging_tick(void)
{
    sensor_t *v = &g_sensors[SID_VOLT];
    sensor_t *c = &g_sensors[SID_CURR];
    sensor_t *p = &g_sensors[SID_POWER];
    sensor_t *e = &g_sensors[SID_ENERGY];
    sensor_t *f = &g_sensors[SID_FEE];
    float volt, curr, power;

    v->value = 220.0f + step(6.0f);
    c->value = clampf(c->value + step(0.8f), 5.0f, 32.0f);
    volt = v->value;
    curr = c->value;
    power = volt * curr / 1000.0f;
    p->value = power;
    e->value += power / 3600.0f;                 /* 1s 采样, kW*h */
    f->value = e->value * PRICE_PER_KWH;

    g_charger.energy_session = e->value;
    g_charger.soc = clampf(g_charger.soc + 0.5f + step(0.5f), 0.0f, SOC_FULL);
    g_sensors[SID_SOC].value = g_charger.soc;
    g_charger.relay_on = true;

    if (g_charger.soc >= SOC_FULL)
        enter_finish(0);   /* 充满 */
}

/* 空闲/待启动/充满结算: 数据区清零(累计电量保留) */
static void idle_data_clear(void)
{
    g_sensors[SID_VOLT].value = 0.0f;
    g_sensors[SID_CURR].value = 0.0f;
    g_sensors[SID_POWER].value = 0.0f;
    g_sensors[SID_FEE].value = 0.0f;
    g_sensors[SID_SOC].value = 0.0f;
    g_charger.soc = 0.0f;
    g_charger.relay_on = false;
}

/* 故障检测: 返回当前应报告的故障(FAULT_NONE=无) */
static fault_type_t detect_fault(void)
{
    if (g_fault_cooldown > 0) return FAULT_NONE;
    if (!g_charger.can_connected && g_charger.can_lost_cnt >= 5)
        return FAULT_CAN_LOST;
    if (g_sensors[SID_GUN_TEMP].value > g_sensors[SID_GUN_TEMP].hi)
        return FAULT_GUN_OVER_TEMP;
    if (g_sensors[SID_CAB_TEMP].value > g_sensors[SID_CAB_TEMP].hi)
        return FAULT_CAB_OVER_TEMP;
    if (g_sensors[SID_SMOKE].value > g_sensors[SID_SMOKE].hi)
        return FAULT_SMOKE;
    if (g_charger.move_flags != 0)
        return FAULT_MOVE;
    return FAULT_NONE;
}

/* 进入故障: 继电器断开, 蜂鸣器报警, 记录故障中止 */
static void enter_fault(fault_type_t f)
{
    if (g_charger.state == CHG_CHARGING)
        record_add(2);
    g_charger.state = CHG_FAULT;
    g_charger.fault = f;
    g_charger.relay_on = false;
    g_charger.buzzer_on = true;
}

/* ================= 对外接口 ================= */

/* ---- OneNET MQTT 周期上报 (原项目 main.c 已实现 mqtt_init/mqtt_send) ---- */
#if MQTT_REPORT_ENABLE
static int mqtt_report_cnt = 0;

static void mqtt_report(void)
{
    mqtt_send("volt",     g_sensors[SID_VOLT].value);
    mqtt_send("curr",     g_sensors[SID_CURR].value);
    mqtt_send("power",    g_sensors[SID_POWER].value);
    mqtt_send("energy",   g_sensors[SID_ENERGY].value);
    mqtt_send("fee",      g_sensors[SID_FEE].value);
    mqtt_send("gun_temp", g_sensors[SID_GUN_TEMP].value);
    mqtt_send("cab_temp", g_sensors[SID_CAB_TEMP].value);
    mqtt_send("smoke",    g_sensors[SID_SMOKE].value);
    mqtt_send("soc",      g_sensors[SID_SOC].value);
    mqtt_send("state",    (float)g_charger.state);   /* 0空闲 1待启动 2充电中 3充满结算 4故障 */
    mqtt_send("fault",    (float)g_charger.fault);   /* 0无 1枪头过温 2柜内过温 3烟雾 4移动 5通信中断 */
}
#endif /* MQTT_REPORT_ENABLE */

/* 每秒推进一次: 传感器采样 + 状态机 + 故障判定 */
bool app_sensor_tick(fault_type_t *new_fault)
{
    int i;
    fault_type_t f;

    rand_init();
    sim_t++;
    if (new_fault) *new_fault = FAULT_NONE;

    /* ---- 传感器采样 ---- */
    sim_gun_temp_tick();        /* DS18B20(模拟+预留) */
    sim_cab_temp_tick();        /* LM75(真实优先) */
    sim_smoke_tick();           /* MQ(模拟+预留) */
    sim_move_tick();            /* ADXL345(真实优先) */
    sim_can_tick();             /* CAN(模拟+预留) */

    /* ---- 充电状态机 ---- */
    switch (g_charger.state) {
    case CHG_CHARGING:
        charging_tick();
        break;
    case CHG_IDLE:
    case CHG_PENDING:
    case CHG_FINISH:
        idle_data_clear();
        break;
    case CHG_FAULT:
        g_charger.relay_on = false;
        break;
    default:
        break;
    }

    /* ---- 历史缓冲 ---- */
    for (i = 0; i < SID_MAX; i++)
        push_hist(&g_sensors[i], g_sensors[i].value);

    /* ---- 故障冷却: 确认后 10 秒内不重复报警 ---- */
    if (g_fault_cooldown > 0)
        g_fault_cooldown--;

    /* ---- 故障判定(任何状态下都生效) ---- */
    f = detect_fault();
    if (f != FAULT_NONE && g_charger.state != CHG_FAULT) {
        enter_fault(f);
        if (new_fault) *new_fault = f;
        return true;
    }

    /* ---- OneNET MQTT 周期上报 ---- */
#if MQTT_REPORT_ENABLE
    if (++mqtt_report_cnt >= MQTT_REPORT_PERIOD) {
        mqtt_report_cnt = 0;
        mqtt_report();
    }
#endif
    return false;
}

void app_sensor_reset(void)
{
    int i;
    for (i = 0; i < SID_MAX; i++) {
        g_sensors[i].hist_cnt = 0;
        g_sensors[i].in_alarm = false;
    }
    sim_t = 0;
    g_charger.state = CHG_IDLE;
    g_charger.fault = FAULT_NONE;
    g_charger.relay_on = false;
    g_charger.buzzer_on = false;
    g_charger.move_flags = 0;
    g_charger.soc = 0.0f;
    g_charger.energy_session = 0.0f;
}

/* "启动充电": 空闲->待启动; 待启动->充电中 */
void charger_start(void)
{
    switch (g_charger.state) {
    case CHG_IDLE:
        g_charger.state = CHG_PENDING;
        break;
    case CHG_PENDING:
        if (!g_charger.can_connected) break;    /* CAN 未连不允许启动 */
        g_charger.state = CHG_CHARGING;
        g_charger.energy_session = 0.0f;
        g_sensors[SID_ENERGY].value = 0.0f;
        break;
    case CHG_FINISH:
        /* 结算完成, 拔枪回空闲 */
        g_charger.state = CHG_IDLE;
        break;
    default:
        break;
    }
}

/* "结束充电": 充电中->充满结算(记录); 充满结算->空闲 */
void charger_stop(void)
{
    switch (g_charger.state) {
    case CHG_CHARGING:
        enter_finish(1);   /* 手动结束 */
        break;
    case CHG_FINISH:
        g_charger.state = CHG_IDLE;
        g_charger.soc = 0.0f;
        break;
    default:
        break;
    }
}

/* 故障弹窗"确认": 关闭蜂鸣器, 回空闲(下个 tick 若条件仍在会再次报警) */
void charger_fault_ack(void)
{
    g_charger.buzzer_on = false;
    g_charger.fault = FAULT_NONE;
    g_charger.can_lost_cnt = 0;
    g_charger.move_flags = 0;
    g_fault_cooldown = 10;              /* 冷却 10 秒, 避免确认后立即重报 */
    if (g_charger.state == CHG_FAULT)
        g_charger.state = CHG_IDLE;
}
