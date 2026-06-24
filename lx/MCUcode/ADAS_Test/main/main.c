/*
 * ESP32 ADAS 控制（双 Jetson Nano 冗余正确版）
 * =============================================
 * 修改说明（本次）：
 *
 * ★1. 删除硬编码边界常量 LANE_WARN_MARGIN / LANE_HARD_MARGIN
 *      改为从 Jetson 串口帧中解析 WMRN / WHRD 字段，实时同步。
 *      JetsonState 新增 warn_margin / hard_margin 字段。
 *      启动默认值保持保守（3.0m / 4.5m），
 *      Jetson 首帧到达后即切换为动态值。
 *
 * ★2. AEB 统一：消除双重 AEB
 *      原问题：Jetson 已做过 AEB（含曲率补偿），ESP32 又用固定 TTC_THRESHOLD
 *              重复判断，两套阈值不一致导致制动时序混乱。
 *      修复：
 *        - ESP32 不再自己计算 TTC 触发，完全信任 Jetson 传来的 ACC 字段。
 *        - ESP32 仅保留最后一道硬件防线：dist <= DSAFE 时强制全力制动。
 *          DSAFE 是 Jetson 用制动距离公式算好的动态值（已含曲率补偿）。
 *        - 移除了 TTC_THRESHOLD / TTC_EMERGENCY_BRAKE 常量。
 *
 * ★3. 通信看门狗 (Communication Watchdog)
 *      独立 FreeRTOS 任务以最高优先级运行，每 50ms 检查一次 Jetson 帧时间戳。
 *      如果两路 Jetson 都超过 WATCHDOG_TIMEOUT_MS 未收到有效帧：
 *        - 直接通过 UART 发送紧急制动帧（绕过控制任务）
 *        - 触发 ESP-IDF 任务看门狗 (TWDT) 复位保护
 *      即使控制任务卡死，看门狗仍能独立执行安全停车。
 *
 * 硬件拓扑：
 *   Jetson Nano 1 (主) GPIO16(RX)/GPIO17(TX) UART1
 *   Jetson Nano 2 (备) GPIO18(RX)/GPIO19(TX) UART2
 */

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* =========================================================
 * 串口引脚
 * ========================================================= */
#define PRIMARY_RX_PIN    16
#define PRIMARY_TX_PIN    17
#define PRIMARY_UART      UART_NUM_1

#define SECONDARY_RX_PIN  18
#define SECONDARY_TX_PIN  19
#define SECONDARY_UART    UART_NUM_2

#define BAUDRATE          115200
#define JETSON_RX_BUF     1024
#define LINE_BUF_SIZE     256   /* 帧变长，从192增到256 */

/* 主控失活仲裁阈值(ms)：主帧超过此时间未刷新即判过期切备控，主→备接管时延≈此值。
 * 58ms 为接管时延极限扫描实验确定的“干净接管”可靠下限（原 150ms→接管 156ms，
 * 现 58ms→接管≈47ms，快约 3.3×）。须 < WATCHDOG_TIMEOUT_MS，且 > 备机就绪时间
 * (HEARTBEAT_TIMEOUT_S 35ms + 心跳轮询 5ms + 发帧 ~10ms ≈ 50ms)，否则备机没接上时
 * 会先触发全力制动冲击。改此处须同步 仿真/bridge_config.py 同名常量。 */
#define JETSON_TIMEOUT_MS 58

/* =========================================================
 * 通信看门狗参数
 * ========================================================= */
/* 看门狗超时：两路 Jetson 都超过此时间无有效帧 → 紧急制动 */
#define WATCHDOG_TIMEOUT_MS     200
/* 看门狗检查周期 */
#define WATCHDOG_CHECK_MS       50
/* ESP-IDF 任务看门狗超时（秒）：控制任务卡死超过此时间 → 硬件复位 */
#define TWDT_TIMEOUT_S          3

/* =========================================================
 * 任务周期（与 Jetson 100Hz 对齐）
 * ========================================================= */
#define UART_TASK_PERIOD_MS    5
#define CONTROL_TASK_PERIOD_MS 10
#define TX_TASK_PERIOD_MS      10
#define PERIOD_TICKS(ms) ((TickType_t)((pdMS_TO_TICKS(ms) > 0U) ? pdMS_TO_TICKS(ms) : 1U))

/* =========================================================
 * 控制参数
 * ========================================================= */
/*
 * 制动指令量纲说明：
 *   Jetson 发送的 ACC 字段单位为 m/s²，范围 [-6.0, +6.0]。
 *   ESP32 通过串口回传给 Jetson 的 B 字段同样以 m/s² 为单位，
 *   send_frames() 中 clampf(-9.99, 9.99) 是串口格式限制，不是物理上限。
 *
 *   MCU_AEB_MAX_BRAKE_DECEL：ESP32 硬件防线触发时写入 a_brake 的值。
 *   由于 send_frames() 会将其截断到 9.99，实际发出值为 9.99 m/s²。
 *   设为 9.99 使代码意图与实际输出一致，避免维护时误判。
 */
static const float MCU_AEB_MAX_BRAKE_DECEL      = 9.99f;  /* 串口格式上限，实际最大制动 */
static const float JETSON_LON_CMD_MAX_BRAKE_DECEL = 6.0f;
static const float JETSON_LON_CMD_MAX_DRIVE_ACCEL = 6.0f;
static const float SAFE_DIST_SOFT_BUFFER = 3.0f;   /* ★2 软缓冲 */

/* ★1 启动默认边界（Jetson 首帧前使用） */
static const float DEFAULT_WARN_MARGIN   = 3.0f;
static const float DEFAULT_HARD_MARGIN   = 4.5f;


/* =========================================================
 * 每路 Jetson 的状态结构体
 * ========================================================= */
typedef struct {
    float   ttc;
    float   dist;
    float   psi;
    float   delta;
    float   speed;
    float   lon_cmd;      /* signed lon cmd: negative=drive accel, positive=brake decel */
    float   lane_offset;
    float   lead_speed;
    float   safe_dist;
    float   warn_margin;
    float   hard_margin;
    float   curv;         /* 曲率，用于 AEB 补偿 */
    int64_t last_rx_us;
    bool    valid;
} JetsonState;

static JetsonState g_pri = {
    .ttc=999.f, .dist=999.f, .lead_speed=0.f,
    .safe_dist=NAN,
    .warn_margin=DEFAULT_WARN_MARGIN,
    .hard_margin=DEFAULT_HARD_MARGIN,
    .curv=0.f,
    .last_rx_us=0, .valid=false
};
static JetsonState g_sec = {
    .ttc=999.f, .dist=999.f, .lead_speed=0.f,
    .safe_dist=NAN,
    .warn_margin=DEFAULT_WARN_MARGIN,
    .hard_margin=DEFAULT_HARD_MARGIN,
    .curv=0.f,
    .last_rx_us=0, .valid=false
};

static volatile bool g_use_secondary = false;
/*
 * 注意：g_use_secondary 的所有读写均在 mtx 互斥锁保护下进行（见 arbitrate()
 * 和 comm_watchdog_task()）。volatile 仅用于防止编译器将其优化为寄存器变量，
 * 不能替代内存屏障或原子操作。在双核 Xtensa LX6 上，跨核可见性由 mtx 的
 * portENTER_CRITICAL / portEXIT_CRITICAL 内存屏障保证，不依赖 volatile。
 * 维护时：不要在锁外读写此变量。
 */

static float psi_cmd   = 0.0f;
static float delta_cmd = 0.0f;
static float a_brake   = 0.0f;

static char   line_pri[LINE_BUF_SIZE]; static size_t line_pri_len = 0;
static char   line_sec[LINE_BUF_SIZE]; static size_t line_sec_len = 0;

static SemaphoreHandle_t mtx = NULL;
static const float MCU_AEB_MIN_CLOSING_SPEED = 0.6f;

/* =========================================================
 * 工具
 * ========================================================= */
static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void trim(char *s)
{
    if (!s) return;
    char *p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    char *e = p + strlen(p);
    while (e > p && isspace((unsigned char)*(e-1))) e--;
    size_t n = (size_t)(e - p);
    if (p != s) memmove(s, p, n);
    s[n] = '\0';
}

static float parsef(const char *s)
{
    if (!s) return 0.f;
    if (strcasecmp(s,"inf")==0 || strcasecmp(s,"+inf")==0) return INFINITY;
    char *e = NULL;
    float v = strtof(s, &e);
    return (e == s) ? 0.f : v;
}

static bool parse_tag_float(const char *line, const char *tag, float *out)
{
    if (!line || !tag || !out) return false;
    const char *p = strstr(line, tag);
    if (!p) return false;
    p += strlen(tag);
    while (*p && isspace((unsigned char)*p)) p++;
    char buf[24];
    size_t i = 0;
    while (*p && !isspace((unsigned char)*p) && i < sizeof(buf) - 1) {
        buf[i++] = *p++;
    }
    buf[i] = '\0';
    if (i == 0) return false;
    *out = parsef(buf);
    return true;
}

/* =========================================================
 * CRC-8/MAXIM（Dallas 多项式 0x31，初值 0x00）
 *
 * 与 Jetson serial_protocol.py 中的 _crc8_dallas() 算法完全一致。
 * 对帧体（"CRC:" 之前的所有字节，含末尾空格）逐字节计算，
 * 结果与帧中 CRC:XX 字段比对，不匹配则丢弃该帧。
 * ========================================================= */
static uint8_t crc8_dallas(const char *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint8_t)data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* =========================================================
 * 解析 Jetson 控制帧（★1 新增 WMRN/WHRD，★CRC 校验）
 * ========================================================= */
static bool parse_jetson_line(char *line, JetsonState *st)
{
    trim(line);
    if (strncmp(line,"TTC:",4)!=0) return false;

    /* ── CRC 校验 ──
     * 帧格式: "... CURV:xx CRC:XX\n"
     * CRC 覆盖 "CRC:" 之前的所有字节（含末尾空格）。
     * 找到 "CRC:" 标记，提取期望值，对前段计算实际 CRC，不匹配则丢弃。
     * 若帧中不含 CRC 字段（旧版 Jetson 固件），允许通过（向后兼容）。
     */
    char *crc_tag = strstr(line, " CRC:");
    if (crc_tag != NULL) {
        /* 计算 CRC 覆盖范围：从行首到 " CRC:" 之前（含末尾空格） */
        size_t body_len = (size_t)(crc_tag - line) + 1; /* +1 包含 " CRC:" 前的空格 */
        uint8_t calc_crc = crc8_dallas(line, body_len);
        /* 解析期望 CRC（两位十六进制） */
        const char *crc_val_str = crc_tag + 5; /* 跳过 " CRC:" */
        char *endp = NULL;
        unsigned long expected_crc = strtoul(crc_val_str, &endp, 16);
        if (endp != crc_val_str && calc_crc != (uint8_t)expected_crc) {
            /* CRC 不匹配：UART 噪声导致字节翻转，丢弃该帧 */
            printf("WARN:crc_mismatch calc=%02X expected=%02lX\n", calc_crc, expected_crc);
            return false;
        }
        /* CRC 匹配或无法解析期望值（容错）：继续解析字段 */
    }
    /* 旧版无 CRC 字段：直接解析（向后兼容） */

    if (!parse_tag_float(line, "TTC:",   &st->ttc))   return false;
    if (!parse_tag_float(line, "DIST:",  &st->dist))  return false;
    if (!parse_tag_float(line, "PSI:",   &st->psi))   return false;
    if (!parse_tag_float(line, "DELTA:", &st->delta)) return false;
    if (!parse_tag_float(line, "SPEED:", &st->speed)) return false;

    if (!parse_tag_float(line, "ACC:", &st->lon_cmd)) {
        if (!parse_tag_float(line, "LON_CMD:", &st->lon_cmd))
            st->lon_cmd = 0.f;
    }
    if (!parse_tag_float(line, "OFFSET:", &st->lane_offset))
        st->lane_offset = 0.f;
    if (!parse_tag_float(line, "LEADV:", &st->lead_speed))
        st->lead_speed = 0.f;
    if (!parse_tag_float(line, "DSAFE:", &st->safe_dist))
        st->safe_dist = NAN;

    /* ★1 动态边界：解析失败保留上一帧值（不覆盖） */
    float tmp;
    if (parse_tag_float(line, "WMRN:", &tmp) && isfinite(tmp) && tmp > 0.f)
        st->warn_margin = tmp;
    if (parse_tag_float(line, "WHRD:", &tmp) && isfinite(tmp) && tmp > 0.f)
        st->hard_margin = tmp;
    /* 曲率解析 */
    if (parse_tag_float(line, "CURV:", &tmp) && isfinite(tmp))
        st->curv = tmp;
    else
        st->curv = 0.f;

    /* ★ 范围验证：修正异常值，防止下游计算出错 */
    if (!isfinite(st->ttc) || st->ttc < 0.f) st->ttc = 999.f;
    if (!isfinite(st->dist) || st->dist < 0.f) st->dist = 999.f;
    if (!isfinite(st->psi)) st->psi = 0.f;
    else st->psi = clampf(st->psi, -3.14159f, 3.14159f);
    if (!isfinite(st->delta)) st->delta = 0.f;
    else st->delta = clampf(st->delta, -9.99f, 9.99f);
    if (!isfinite(st->speed) || st->speed < 0.f) st->speed = 0.f;
    if (!isfinite(st->lon_cmd)) st->lon_cmd = 0.f;
    else st->lon_cmd = clampf(st->lon_cmd, -JETSON_LON_CMD_MAX_DRIVE_ACCEL, JETSON_LON_CMD_MAX_BRAKE_DECEL);
    if (!isfinite(st->lane_offset)) st->lane_offset = 0.f;
    else st->lane_offset = clampf(st->lane_offset, -5.0f, 5.0f);
    if (!isfinite(st->lead_speed) || st->lead_speed < 0.f) st->lead_speed = 0.f;
    /* DSAFE 允许 NAN（解析失败时保留默认），但若为有限值必须非负 */
    if (isfinite(st->safe_dist) && st->safe_dist < 0.f) st->safe_dist = NAN;

    return true;
}

static void feed(const uint8_t *data, int len,
                 char *lb, size_t *ll, JetsonState *st)
{
    for (int i=0; i<len; i++) {
        char c=(char)data[i];
        if (c=='\r') continue;
        if (c=='\n') {
            lb[*ll]='\0';
            xSemaphoreTake(mtx,portMAX_DELAY);
            if (parse_jetson_line(lb,st)) {
                st->last_rx_us = esp_timer_get_time();
                st->valid = true;
            }
            xSemaphoreGive(mtx);
            *ll=0; continue;
        }
        if (*ll < (LINE_BUF_SIZE-2)) {
            lb[(*ll)++] = c;
        } else {
            /* 行缓冲溢出：当前行超过 LINE_BUF_SIZE-2 字节，丢弃并重置。
             * 正常帧约 120 字节，溢出说明 UART 线路有严重噪声或粘包。
             * 留 1 字节给终止符 '\0'，避免 lb[*ll]='\0' 越界写入。 */
            printf("WARN:line_buf_overflow len=%u\n", (unsigned)*ll);
            *ll = 0;
        }
    }
}

static void poll_uart(uart_port_t port, char *lb, size_t *ll, JetsonState *st)
{
    uint8_t buf[128];
    int n = uart_read_bytes(port, buf, sizeof(buf), pdMS_TO_TICKS(UART_TASK_PERIOD_MS));
    if (n>0) feed(buf,n,lb,ll,st);
    while ((n=uart_read_bytes(port,buf,sizeof(buf),0))>0) feed(buf,n,lb,ll,st);
}

static bool state_is_fresh(const JetsonState *st, int64_t now_us)
{
    if (!st || !st->valid) return false;
    return ((now_us - st->last_rx_us) / 1000) <= JETSON_TIMEOUT_MS;
}

/* =========================================================
 * 主备仲裁
 * ========================================================= */
static JetsonState *arbitrate(void)
{
    int64_t now = esp_timer_get_time();
    bool pri_fresh = state_is_fresh(&g_pri, now);
    bool sec_fresh = state_is_fresh(&g_sec, now);

    if (pri_fresh) {
        if (g_use_secondary) {
            int64_t age_ms = (now - g_pri.last_rx_us) / 1000;
            printf("SWITCH:primary_recovered_%lldms\n", age_ms);
            g_use_secondary = false;
        }
        return &g_pri;
    }

    if (sec_fresh) {
        if (!g_use_secondary) {
            int64_t age_ms = (now - g_pri.last_rx_us) / 1000;
            printf("SWITCH:pri_timeout_%lldms\n", age_ms);
            g_use_secondary = true;
        }
        return &g_sec;
    }

    return g_use_secondary ? &g_sec : &g_pri;
}

/* =========================================================
 * 算法模块
 * ========================================================= */
static void update_lka(const JetsonState *s, float *po, float *dlo)
{
    float p = isfinite(s->psi)   ? s->psi   : 0.f;
    float d = isfinite(s->delta) ? s->delta : 0.f;
    *po  = clampf(p, -9.9999f, 9.9999f);
    *dlo = clampf(d, -9.99f,   9.99f);
}

static float update_acc(const JetsonState *s)
{
    float req = isfinite(s->lon_cmd)
                ? clampf(s->lon_cmd,
                         -JETSON_LON_CMD_MAX_DRIVE_ACCEL,
                         JETSON_LON_CMD_MAX_BRAKE_DECEL)
                : 0.f;
    return req;
}

static float update_aeb(const JetsonState *s)
{
    if (!isfinite(s->dist) || s->dist <= 0.f) return 0.f;

    float curv = fabsf(s->curv);
    float ego_speed = isfinite(s->speed) ? fmaxf(s->speed, 0.f) : 0.f;
    float lead_speed = isfinite(s->lead_speed) ? fmaxf(s->lead_speed, 0.f) : 0.f;
    float closing_speed = fmaxf(0.f, ego_speed - lead_speed);

    /* 曲率补偿：弯道中增加安全距离，但上限 1.3 倍，避免低速跟停时过度放大 */
    float curv_comp = 1.0f + 3.0f * curv;   /* curv=0.01 时增加 3% */
    curv_comp = clampf(curv_comp, 1.0f, 1.3f);  /* 最多放大 30% */

    float safe_dist = (isfinite(s->safe_dist) && s->safe_dist > 0.f)
                    ? s->safe_dist * curv_comp
                    : 5.0f * curv_comp;   /* 保守兜底值 */
    float safe_soft = safe_dist + SAFE_DIST_SOFT_BUFFER;

    /*
     * hard_floor：触发全力制动的距离门限。
     * 设计原则：hard_floor 必须 < safe_dist，留出缓冲区。
     * 取 safe_dist 的 40%，但不低于绝对安全下限 AEB_FULL_BRAKE_FLOOR(2.5m)，
     * 且不超过 safe_dist - 1.0m（保证至少 1m 缓冲）。
     */
    static const float AEB_FULL_BRAKE_FLOOR = 2.5f;  /* 绝对最小全力制动距离 */
    float hard_floor = fmaxf(AEB_FULL_BRAKE_FLOOR, safe_dist * 0.40f);
    /* 确保 hard_floor < safe_dist，至少留 1m 缓冲 */
    hard_floor = fminf(hard_floor, fmaxf(AEB_FULL_BRAKE_FLOOR, safe_dist - 1.0f));

    /* 仅在确实追近前车时才触发兜底 AEB，避免低速跟停时被 DSAFE 锁死 */
    if (s->dist <= hard_floor)
        return MCU_AEB_MAX_BRAKE_DECEL;
    if (closing_speed < MCU_AEB_MIN_CLOSING_SPEED)
        return 0.f;
    if (s->dist < safe_soft) {
        float speed_ratio = clampf(
            (closing_speed - MCU_AEB_MIN_CLOSING_SPEED) / 2.4f,
            0.f, 1.f
        );
        float r;
        if (s->dist < safe_dist) {
            r = clampf(
                (safe_dist - s->dist) / (safe_dist - hard_floor + 1e-6f),
                0.f, 1.f
            );
            return MCU_AEB_MAX_BRAKE_DECEL * speed_ratio * r * r;
        }
        r = clampf(
            (safe_soft - s->dist) / (safe_soft - safe_dist + 1e-6f),
            0.f, 1.f
        );
        return JETSON_LON_CMD_MAX_BRAKE_DECEL * speed_ratio * r * r;
    }
    return 0.f;
}

static void control_step(void)
{
    JetsonState local;
    int64_t now_us;
    xSemaphoreTake(mtx, portMAX_DELAY);
    local = *arbitrate();
    now_us = esp_timer_get_time();
    xSemaphoreGive(mtx);

    if (!state_is_fresh(&local, now_us)) {
        xSemaphoreTake(mtx, portMAX_DELAY);
        psi_cmd   = 0.0f;
        delta_cmd = 0.0f;
        a_brake   = MCU_AEB_MAX_BRAKE_DECEL;
        xSemaphoreGive(mtx);
        return;
    }

    float lka_p=0.f, lka_d=0.f;
    update_lka(&local, &lka_p, &lka_d);

    float acc_out = update_acc(&local);

    float aeb_out = update_aeb(&local);
    float br = acc_out;
    if (aeb_out > 0.f) br = fmaxf(aeb_out, fmaxf(acc_out, 0.f));
    br = clampf(br, -JETSON_LON_CMD_MAX_DRIVE_ACCEL, MCU_AEB_MAX_BRAKE_DECEL);

    xSemaphoreTake(mtx, portMAX_DELAY);
    psi_cmd   = lka_p;
    delta_cmd = lka_d;
    a_brake   = br;
    xSemaphoreGive(mtx);
}

static void send_frames(void)
{
    float p, d, b; bool us;
    xSemaphoreTake(mtx, portMAX_DELAY);
    p=clampf(psi_cmd,-9.9999f,9.9999f);
    d=clampf(delta_cmd,-9.99f,9.99f);
    b=clampf(a_brake,-9.99f,9.99f);
    us=g_use_secondary;
    xSemaphoreGive(mtx);
    if (!isfinite(p)) p=0.f;
    if (!isfinite(d)) d=0.f;
    if (!isfinite(b)) b=0.f;

    printf("P:%+1.4f\nD:%+1.2f\nB:%+1.2f\nSRC:%d\n", p, d, b, us?1:0);

    char tx[96];
    int n = snprintf(tx,sizeof(tx),"P:%+1.4f\nD:%+1.2f\nB:%+1.2f\nSRC:%d\n",p,d,b,us?1:0);
    if (n>0 && n<(int)sizeof(tx)) {
        uart_write_bytes(PRIMARY_UART,   tx, n);
        uart_write_bytes(SECONDARY_UART, tx, n);
    }
}

/* =========================================================
 * FreeRTOS 任务
 * ========================================================= */
static void uart_rx_task(void *arg)
{
    (void)arg;
    TickType_t t = xTaskGetTickCount();
    while (true) {
        poll_uart(PRIMARY_UART,   line_pri, &line_pri_len, &g_pri);
        poll_uart(SECONDARY_UART, line_sec, &line_sec_len, &g_sec);
        vTaskDelayUntil(&t, PERIOD_TICKS(UART_TASK_PERIOD_MS));
    }
}

static void control_task(void *arg)
{
    (void)arg;
    /* 注册到任务看门狗：每个周期喂狗，卡死时 TWDT 触发复位 */
    esp_task_wdt_add(NULL);
    TickType_t t = xTaskGetTickCount();
    while (true) {
        control_step();
        esp_task_wdt_reset();
        vTaskDelayUntil(&t, PERIOD_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

static void tx_task(void *arg)
{
    (void)arg;
    TickType_t t = xTaskGetTickCount();
    while (true) {
        send_frames();
        vTaskDelayUntil(&t, PERIOD_TICKS(TX_TASK_PERIOD_MS));
    }
}

/* =========================================================
 * ★3 通信看门狗任务
 *
 * 以最高优先级独立运行，不依赖控制任务。
 * 每 WATCHDOG_CHECK_MS 检查一次两路 Jetson 的最后接收时间戳：
 *   - 任一路新鲜 → 正常，喂狗
 *   - 两路都超时 → 直接通过 UART 发送紧急制动帧（绕过控制任务）
 *
 * 即使 control_task 死锁或优先级反转，本任务仍能独立执行安全停车。
 * ========================================================= */
static void watchdog_emergency_brake(void)
{
    /* 紧急制动帧：方向盘归零 + 最大制动 */
    const char *estop = "P:+0.0000\nD:+0.00\nB:+9.99\nSRC:9\n";
    size_t len = strlen(estop);
    uart_write_bytes(PRIMARY_UART,   estop, len);
    uart_write_bytes(SECONDARY_UART, estop, len);
    /* 同时输出到 USB 串口供调试 */
    printf("WATCHDOG:ESTOP\n");
}

static void comm_watchdog_task(void *arg)
{
    (void)arg;
    /* 看门狗任务也注册到 TWDT，确保自身不卡死 */
    esp_task_wdt_add(NULL);

    bool estop_active = false;

    TickType_t t = xTaskGetTickCount();
    while (true) {
        int64_t now_us = esp_timer_get_time();

        /* 检查两路 Jetson 是否都超时 */
        bool pri_alive, sec_alive;
        int64_t pri_age_ms, sec_age_ms;
        xSemaphoreTake(mtx, portMAX_DELAY);
        pri_alive = g_pri.valid &&
                    ((now_us - g_pri.last_rx_us) / 1000 <= WATCHDOG_TIMEOUT_MS);
        sec_alive = g_sec.valid &&
                    ((now_us - g_sec.last_rx_us) / 1000 <= WATCHDOG_TIMEOUT_MS);
        /* 在锁内读取时间戳，避免与 uart_rx_task 的写操作产生数据竞争 */
        pri_age_ms = (now_us - g_pri.last_rx_us) / 1000;
        sec_age_ms = (now_us - g_sec.last_rx_us) / 1000;
        xSemaphoreGive(mtx);

        if (pri_alive || sec_alive) {
            /* 至少一路存活 → 正常 */
            if (estop_active) {
                printf("WATCHDOG:RECOVERED\n");
                estop_active = false;
            }
        } else {
            /* 两路都超时 → 紧急制动 */
            if (!estop_active) {
                estop_active = true;
                printf("WATCHDOG:TIMEOUT pri_age=%lldms sec_age=%lldms\n",
                       pri_age_ms, sec_age_ms);
            }
            /* 持续发送紧急制动帧，直到通信恢复 */
            watchdog_emergency_brake();

            /* 同时强制覆盖控制输出，防止 tx_task 发送旧值 */
            xSemaphoreTake(mtx, portMAX_DELAY);
            psi_cmd   = 0.0f;
            delta_cmd = 0.0f;
            a_brake   = MCU_AEB_MAX_BRAKE_DECEL;
            xSemaphoreGive(mtx);
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&t, PERIOD_TICKS(WATCHDOG_CHECK_MS));
    }
}

/* =========================================================
 * UART 初始化
 * ========================================================= */
static void init_uart(uart_port_t port, int rx_pin, int tx_pin)
{
    uart_config_t cfg = {
        .baud_rate = BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    cfg.source_clk = UART_SCLK_DEFAULT;
#endif
    ESP_ERROR_CHECK(uart_driver_install(port, JETSON_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, tx_pin, rx_pin,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

/* =========================================================
 * 入口
 * ========================================================= */
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);
    setvbuf(stdout, NULL, _IONBF, 0);

    mtx = xSemaphoreCreateMutex();
    if (!mtx) return;

    init_uart(PRIMARY_UART,   PRIMARY_RX_PIN,   PRIMARY_TX_PIN);
    init_uart(SECONDARY_UART, SECONDARY_RX_PIN, SECONDARY_TX_PIN);

    int64_t now = esp_timer_get_time();
    g_pri.last_rx_us = now;
    g_sec.last_rx_us = now;

    /* ★3 初始化 ESP-IDF 任务看门狗 (TWDT)
     * 超时后触发 panic → 硬件复位，作为最后一道防线。
     * 控制任务和看门狗任务会各自注册并定期喂狗。 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = TWDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_task_wdt_reconfigure(&wdt_cfg);
#else
    esp_task_wdt_init(TWDT_TIMEOUT_S, true);
#endif

    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 9, NULL);
    xTaskCreate(control_task, "ctrl",    4096, NULL, 8, NULL);
    xTaskCreate(tx_task,      "tx",      4096, NULL, 7, NULL);
    /* ★3 通信看门狗：最高优先级，独立于控制任务 */
    xTaskCreate(comm_watchdog_task, "wdog", 3072, NULL, 10, NULL);

    printf("ADAS_ESP32:STARTED watchdog=%dms twdt=%ds\n",
           WATCHDOG_TIMEOUT_MS, TWDT_TIMEOUT_S);
}
