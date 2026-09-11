/*
 * config_params：车载运行参数的纯 C 模型，宿主机可测（无 IDF 依赖）。
 * 配置存储由 main 的 config_store 负责（NVS 薄封装），这里只做：
 * 默认值、逐字段校验、NVS blob 打包/解包。
 *
 * blob 版本迁移：v1（32B）是一期的布局；v2（72B）在尾部追加舵机与
 * 锁定策略参数。解包按首字节版本号分派——v1 blob 仍可解，新字段取默认值，
 * 上层解出后再回写 v2，升级不丢用户已设的配置。
 */
#ifndef CONFIG_PARAMS_H
#define CONFIG_PARAMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_WHEEL_COUNT 4
#define CFG_SERVO_COUNT 2 /* 0=前差速 1=后差速 */

#define CFG_MAGNETS_MIN 1
#define CFG_MAGNETS_MAX 16
#define CFG_DIAM_MM_MIN 10
#define CFG_DIAM_MM_MAX 300
#define CFG_ALPHA_MIN 0.05f
#define CFG_ALPHA_MAX 1.0f

/* 舵机脉宽：标准角度舵机的常见量程，两端各留余量 */
#define CFG_SERVO_US_MIN 500
#define CFG_SERVO_US_MAX 2500

#define CFG_SERVO_MODE_MANUAL 0
#define CFG_SERVO_MODE_AUTO 1

#define CFG_SLIP_ENGAGE_MIN 0.05f
#define CFG_SLIP_MIN_RPM_MAX 2000.0f

#define CFG_LOCK_HOLD_MIN_MS 200
#define CFG_LOCK_HOLD_MAX_MS 60000
#define CFG_PROBE_WINDOW_MIN_MS 200
#define CFG_PROBE_WINDOW_MAX_MS 10000
#define CFG_SIM_SPEED_MIN 100
#define CFG_SIM_SPEED_MAX 20000

/* blob 定长布局（尾部补零）：
 *   v1 段 0..31：版本(1B) + magnets(1B) + diam(4B) + alpha(4B float)
 *               + wheel_enabled(1B 位图) + sim_on(1B) + sim_rpm[4](16B) + debounce_ms(4B)
 *   v2 段 32..71：servo_sim(1B) + servo_en(1B 位图) + servo_mode(1B)
 *               + min/center/max(u16×3) + manual_us(u16×2)
 *               + engage/min_rpm(float×2) + lock_hold/max/probe(u32×3)
 *               + sim_speed(u32) + 保留(3B) */
#define CFG_PARAMS_BLOB_VERSION 2
#define CFG_PARAMS_BLOB_SIZE 72
#define CFG_PARAMS_BLOB_VERSION_1 1
#define CFG_PARAMS_BLOB_SIZE_V1 32

typedef struct {
    uint32_t magnets;    /* 每转磁铁数 1-16 */
    uint32_t wheel_diam_mm; /* 轮径（毫米）10-300 */
    float alpha;         /* EMA 系数 0.05-1.0 */
    bool wheel_enabled[CFG_WHEEL_COUNT];
    bool sim_on;         /* 模拟模式总开关 */
    float sim_rpm[CFG_WHEEL_COUNT];
    uint32_t debounce_ms; /* 最小脉冲间隔去抖；0=关闭 */

    /* ---- v2：差速舵机 ---- */
    bool servo_sim; /* true=模拟舵机模型；false=LEDC 驱动真实舵机 */
    bool servo_en[CFG_SERVO_COUNT];
    uint32_t servo_min_us;    /* 脉宽下限（满偏一端） */
    uint32_t servo_center_us; /* 中位脉宽 = 解锁位置 */
    uint32_t servo_max_us;    /* 脉宽上限（满偏另一端） */
    bool servo_invert[CFG_SERVO_COUNT];
    uint32_t servo_mode; /* CFG_SERVO_MODE_* */
    uint32_t servo_manual_us[CFG_SERVO_COUNT]; /* 手动模式目标脉宽 */

    /* ---- v2：打滑判定与锁定策略 ---- */
    float slip_engage_ratio;   /* 打滑进入门槛 */
    float slip_min_rpm;        /* 低于此转速不判定 */
    uint32_t lock_hold_ms;     /* 初始锁定保持时长（"地形时间"） */
    uint32_t lock_hold_max_ms; /* 保持时长翻倍上限 */
    uint32_t probe_window_ms;  /* 试探窗口：车连续动满此时长才算脱困 */
    uint32_t servo_sim_speed_us_s; /* 模拟舵机行程速率（µs/秒） */
} cfg_params_t;

void cfg_params_default(cfg_params_t *cfg);

/* 校验单字段；字段名与 JSON 命令一致（magnets、diam、alpha、en、sim、
 * sim_rpm、debounce，以及 v2 的舵机与锁定策略各字段，见上表）。
 * 未知字段返回 ESP_OK（命令层忽略）。
 * 跨字段约束（min<center<max、hold_max>=hold）随任一相关字段被写时一并检查，
 * 前端整组提交即保持一致。 */
esp_err_t cfg_params_validate(const cfg_params_t *cfg, const char *field);

/* v1 与 v2 两种长度的 blob 都可解（按首字节版本号分派）。 */
esp_err_t cfg_params_unpack(const uint8_t *buf, size_t len, cfg_params_t *out);

esp_err_t cfg_params_pack(const cfg_params_t *cfg, uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_PARAMS_H */
