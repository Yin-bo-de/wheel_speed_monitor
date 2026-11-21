/* 车速计算（纯 C，不依赖 IDF，宿主机可测）。
 *
 * 慢轮近似车身速度：空转轮转得比车身快、不推车，所以取慢的。
 * 取启用且有效的轮子里最小的 RPM，乘轮胎直径换算 km/h：
 *   v(km/h) = π × D(mm) × n(rpm) × 6e-5
 * 坏轮/禁用轮由调用方按轮速帧的 en 字段剔除，不让坏轮把车速拉到 0。 */
#ifndef SPEED_CALC_H
#define SPEED_CALC_H

#include <stdbool.h>
#include <stdint.h>

#define SPEED_CALC_WHEELS 4

typedef struct {
    bool valid[SPEED_CALC_WHEELS]; /* false = 该轮禁用/故障，剔除 */
    float rpm[SPEED_CALC_WHEELS];
    float diam_mm;                 /* 轮胎直径（毫米），<= 0 视为配置无效 */
} speed_calc_in_t;

/* 算车速（km/h）。无有效轮或轮径无效返回 false，*out 不写。 */
bool speed_calc_kmh(const speed_calc_in_t *in, float *out);

#endif /* SPEED_CALC_H */
