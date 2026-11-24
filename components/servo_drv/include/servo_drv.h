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
/* 相机 XCLK 用 LEDC_TIMER_0 / CHANNEL_0。舵机必须避开这一组，否则
 * ledc_timer_config(50Hz) 会把相机 XCLK 从 20MHz 改成 50Hz，画面直接停。 */
#define SERVO_LEDC_TIMER LEDC_TIMER_1
#define SERVO_LEDC_CHANNEL_BASE LEDC_CHANNEL_2
/* 自控舵机第二组用 CHANNEL_4/5，与差速组共用 TIMER_1（50Hz/14bit 同参数，
 * 重复配置是幂等的），仍然避开相机 XCLK 的 TIMER_0/CHANNEL_0。 */
#define SERVO_LEDC_AUTO_CHANNELS 2
#define SERVO_LEDC_AUTO_CHANNEL_BASE LEDC_CHANNEL_4

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

/* ---- 自控舵机组（独立于策略链，将来由 MPU6050 姿态控制驱动）----
 *
 * 不复用 servo_act_ops_t：这一组不接策略状态机，只需要"设脉宽/停波"两个
 * 直给接口，套一层 ops 反而多出用不上的 step/get_state。初始化后立即输出
 * center_us（中位 1500µs），上电就能用示波器或真舵机验证两路口。
 *
 * 并发约定：只允许在任务启动前初始化一次；set_us/stop 将来由唯一控制方
 * 调用，与差速组一样不做跨任务加锁。 */
typedef struct {
    int gpio[SERVO_LEDC_AUTO_CHANNELS];
    uint32_t target_us[SERVO_LEDC_AUTO_CHANNELS];
    bool inited;
} servo_ledc_auto_t;

/* 幂等：已初始化直接返回 ESP_OK。center_us 越出 SERVO_ACT_MIN/MAX_US 报参数错。 */
esp_err_t servo_ledc_auto_init(servo_ledc_auto_t *s, int gpio_a, int gpio_b,
                               uint32_t center_us);
esp_err_t servo_ledc_auto_set_us(servo_ledc_auto_t *s, uint8_t ch, uint32_t us);
/* 停波 = 舵机脱力，输出钉低（与差速组停波同一语义）。 */
esp_err_t servo_ledc_auto_stop(servo_ledc_auto_t *s, uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_DRV_H */
