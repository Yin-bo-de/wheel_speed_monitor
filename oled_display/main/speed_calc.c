/* speed_calc 实现：见头文件注释。 */
#include "speed_calc.h"

#include <stddef.h>

/* 不用 M_PI：严格 C 模式下不保证有，自己定义省心 */
#define SPEED_CALC_PI 3.14159265f

bool speed_calc_kmh(const speed_calc_in_t *in, float *out)
{
    if (in == NULL || out == NULL) {
        return false;
    }
    if (!(in->diam_mm > 0.0f)) {
        return false;
    }

    bool any = false;
    float min_rpm = 0.0f;
    for (int i = 0; i < SPEED_CALC_WHEELS; i++) {
        if (!in->valid[i]) {
            continue;
        }
        float rpm = in->rpm[i];
        if (rpm < 0.0f) {
            continue; /* 负值是坏数据，不当有效轮 */
        }
        if (!any || rpm < min_rpm) {
            min_rpm = rpm;
            any = true;
        }
    }
    if (!any) {
        return false;
    }

    *out = SPEED_CALC_PI * in->diam_mm * min_rpm * 6.0e-5f;
    return true;
}
