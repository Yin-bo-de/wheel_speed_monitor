/*
 * strategy：差速策略引擎接口桩，v1 全实现为空/NOT_SUPPORTED。
 * 一期只定义二期所需的数据流挂载点（feeds）与硬件接口（PWM 输出/RC 输入），
 * 供上层（main/web 命令/页面）稳定调用；v1 不变更语义，仅作协议骨架。
 *
 * v2 规划：
 *  - 模式切换：manual（RC 遥控器通道直控舵机）/ auto（策略脚本依据四轮 + 姿态判定差速）
 *  - feed：telemetry 帧中的轮速 + 姿态采集器数据，策略引擎消费
 *  - 输出：LEDC PWM 驱动前后差速舵机（GPIO41/42 候选，需关闭 JTAG 重配）
 *  - 输入：RC 接收机 PWM 捕获（RMT），脉宽映射到模式/舵机
 *  - 策略脚本：夹角的差速比例控制（如前后轴转差 → 舵机角度）
 */
#ifndef STRATEGY_H
#define STRATEGY_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STRATEGY_MODE_MANUAL = 0, /* 手动：RC 通道直控（v2） */
    STRATEGY_MODE_AUTO,       /* 自动：策略脚本（v2） */
} strategy_mode_t;

typedef struct {
    bool supported;          /* v1 恒 false：二层未实现 */
    strategy_mode_t mode;    /* 当前生效模式（仅记录，无动作） */
} strategy_status_t;

typedef struct {
    int servo_gpio;          /* 舵机输出 GPIO；-1 表示未分配（v2 设置） */
    uint32_t servo_min_us;   /* 舵机脉宽下限（µs） */
    uint32_t servo_max_us;   /* 舵机脉宽上限（µs） */
    uint32_t servo_hz;       /* 舵机 PWM 频率 */
} strategy_pwm_config_t;

esp_err_t strategy_init(void);

esp_err_t strategy_get_status(strategy_status_t *out);
esp_err_t strategy_set_mode(strategy_mode_t mode);

/* 策略输入挂载点：喂四轮 RPM（每窗口调换，v2 由策略引擎消费）。 */
esp_err_t strategy_feed_wheel_rpm(const float rpm[4]);

esp_err_t strategy_get_pwm_config(strategy_pwm_config_t *out);
esp_err_t strategy_set_servo(uint8_t channel, uint32_t us);

/* RC 通道读捕获脉宽（µs）；v1 未接 RC 输入。 */
esp_err_t strategy_rc_read(uint8_t channel, uint16_t *us);

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_H */
