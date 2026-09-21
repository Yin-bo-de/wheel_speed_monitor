/*
 * servo_drv：标准角度舵机的 LEDC 硬件输出（依赖 IDF，不参与宿主机测试）。
 *
 * 与 servo_act 组成"纯 C 接口 + 硬件孪生"的一对：上层持有 servo_act_ops_t，
 * 控制链不知道底下是模型还是引脚（对照轮速链的 sim_source ↔ wheel_sensor）。
 *
 * 时序：50Hz、14 位占空比。LEDC 取 APB 80MHz 源时钟时
 *     div = 80e6 / (50 × 2^14) = 97.65625 = 97 + 168/256
 * 分频器有 8 位小数，恰好整除，输出频率零误差；一个计数 = 20000µs / 16384
 * ≈ 1.22µs，折算到常见 1000µs/90° 的舵机约 0.11°，分辨率有余量。
 */
#ifndef SERVO_DRV_H
#define SERVO_DRV_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "servo_act.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_LEDC_FREQ_HZ 50
#define SERVO_LEDC_PERIOD_US 20000 /* 1e6 / SERVO_LEDC_FREQ_HZ */
#define SERVO_LEDC_DUTY_BITS 14
#define SERVO_LEDC_DUTY_RES LEDC_TIMER_14_BIT
#define SERVO_LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_TIMER LEDC_TIMER_0

/* 调用方须传零初始化的实例（用 static 即可）：inited 是"是否已 bring-up"的
 * 唯一依据，靠它把重复初始化挡在外面，不做 memset 复位。 */
typedef struct {
    int gpio[SERVO_ACT_CHANNELS];
    uint32_t target_us[SERVO_ACT_CHANNELS]; /* 最近一次指令脉宽 */
    bool enabled[SERVO_ACT_CHANNELS];
    bool inited;
} servo_ledc_t;

/* 初始化两个通道并停输出（脱力）。unlock_us 逐通道给定，作为通道被使能时
 * 的首个输出值——控制链随即按策略给出真实目标，这里填的只是"补位"值，
 * 取解锁档最稳：那一刻差速本来就是松开的。
 *
 * 幂等：已初始化则直接返回 ESP_OK，执行器来回切换不会重复占用 LEDC 资源。
 * 频率按 FREQ_HZ 设定，实际频率不符（超差 1Hz）时打警告但继续——舵机能容忍
 * 轻微频偏，这里不该因为一个警告把整条链路停掉。 */
esp_err_t servo_ledc_init(servo_ledc_t *s, int gpio_front, int gpio_rear,
                          const uint32_t unlock_us[SERVO_ACT_CHANNELS]);

extern const servo_act_ops_t servo_ledc_ops;

#ifdef __cplusplus
}
#endif

#endif /* SERVO_DRV_H */
