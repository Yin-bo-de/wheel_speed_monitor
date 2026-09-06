/*
 * wheel_sensor：PCNT 硬件层薄封装（4 单元初始化/读计数/启停）。
 * A3144 开集电极输出，南极触发拉低（下降沿计数）。
 * 本层只做 PCNT 生命周期与计数读取，回绕/计算在采集器（wheel_speed collector）
 * 与框架（telemetry）处理。板级 GPIO 映射在 main/wheel_config.h。
 */
#ifndef WHEEL_SENSOR_H
#define WHEEL_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WHEEL_SENSOR_COUNT 4

/* 初始化 4 个 PCNT 单元。gpios 是各轮霍尔输入 GPIO 号（长度 WHEEL_SENSOR_COUNT）。 */
esp_err_t wheel_sensor_init(const uint8_t gpios[WHEEL_SENSOR_COUNT]);

/* 读某轮最近两次"有效"脉冲间隔（微秒）；0 表示从未检测到有效脉冲。
 * 由 GPIO 下降沿中断维护（PCNT 计数与 GPIO 中断共存同一引脚）。
 * 有效 = 间隔 >= 5ms（去抖，剔除磁铁贴近时的抖动/双探针）。 */
esp_err_t wheel_sensor_get_period_us(uint8_t wheel, uint32_t *interval_us);

/* 读某轮有效脉冲总数（去抖后，替代被抖污染 的 PCNT 原始计数做累计显示）。 */
esp_err_t wheel_sensor_get_valid_count(uint8_t wheel, uint16_t *count);

/* 读某轮当前 PCNT 计数值（16 位回绕由采集器判定）。 */
esp_err_t wheel_sensor_read_count(uint8_t wheel, uint16_t *count);

/* 启用/停用某轮：启动或停止对应 PCNT 单元。 */
esp_err_t wheel_sensor_set_enabled(uint8_t wheel, bool enabled);

/* 读 4 路 GPIO 原始电平（页面 IO 监视用，读取不受 PCNT 影响）。 */
esp_err_t wheel_sensor_read_gpio_levels(uint8_t levels[WHEEL_SENSOR_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SENSOR_H */
