/* sim_source 实现：见头文件注释。 */
#include <string.h>

#include "sim_source.h"

void sim_source_init(sim_source_t *src)
{
    memset(src, 0, sizeof(*src));
}

void sim_source_set_target_rpm(sim_source_t *src, uint8_t ch, float rpm)
{
    if (ch < SIM_SOURCE_CHANNELS) {
        src->target_rpm[ch] = rpm;
    }
}

void sim_source_step(sim_source_t *src, uint32_t window_ms)
{
    for (uint8_t ch = 0; ch < SIM_SOURCE_CHANNELS; ch++) {
        /* 每窗口期望增量 = RPM × (窗口秒 / 60)；
         * 用累积误差取整（Bresenham 式），长时间平均频率精确等于目标。 */
        const double delta_f = (double)src->target_rpm[ch] / 60.0 *
                               (double)window_ms / 1000.0;
        src->acc[ch] += delta_f;
        /* 钳位防止异常配置导致累积器爆炸（正常目标下 |acc| < 1 恒成立） */
        if (src->acc[ch] > 65536.0) {
            src->acc[ch] = 65536.0;
        }
        const uint32_t emit = (uint32_t)src->acc[ch];
        src->acc[ch] -= (double)emit;
        src->counter[ch] = (uint16_t)(src->counter[ch] + emit); /* 自然回绕 */
    }
}

uint16_t sim_source_counter(const sim_source_t *src, uint8_t ch)
{
    if (ch >= SIM_SOURCE_CHANNELS) {
        return 0;
    }
    return src->counter[ch];
}
