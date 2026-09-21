/* strategy_render 实现：见头文件注释。 */
#include <stdio.h>

#include "strategy_render.h"

const char *strategy_phase_name(strategy_phase_t phase)
{
    switch (phase) {
    case STRATEGY_PHASE_LOCKED:
        return "locked";
    case STRATEGY_PHASE_PROBE:
        return "probe";
    case STRATEGY_PHASE_IDLE:
    default:
        return "idle";
    }
}

esp_err_t servo_render_block(const servo_render_state_t *rs,
                             char *buf, size_t len, size_t *used)
{
    if (rs == NULL || buf == NULL || used == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    const strategy_config_t *cfg = &rs->cfg;
    const char *mode = (cfg->mode == STRATEGY_MODE_MANUAL) ? "manual" : "auto";
    size_t off = 0;

#define RENDER_SNPRINTF(...) do { \
        int n_ = snprintf((buf) + off, (len) - off, __VA_ARGS__); \
        if (n_ < 0 || (size_t)n_ >= (len) - off) { return ESP_ERR_INVALID_SIZE; } \
        off += (size_t)n_; \
    } while (0)

    RENDER_SNPRINTF("{\"type\":\"servo\",\"src\":\"%s\",\"mode\":\"%s\",\"sim\":%s,\"servos\":[",
                    (rs->src != NULL) ? rs->src : "sim", mode,
                    rs->sim ? "true" : "false");

    for (int i = 0; i < STRATEGY_SERVO_COUNT; i++) {
        const servo_render_chan_t *c = &rs->ch[i];
        /* 只报脉宽不报角度：自动模式的落点由配置直接给定（两档），
         * "多少度"在这个映射下没有确定含义，编一个出来只会误导调试。 */
        RENDER_SNPRINTF("%s{\"i\":%d,\"en\":%s,\"phase\":\"%s\",\"hold_ms\":%lu,"
                        "\"ratio\":%.2f,\"slip_left\":%s,"
                        "\"target_us\":%u,\"cur_us\":%.1f,\"reached\":%s}",
                        i ? "," : "",
                        i, c->enabled ? "true" : "false", strategy_phase_name(c->phase),
                        (unsigned long)c->hold_ms,
                        (double)c->ratio, c->slip_fast_left ? "true" : "false",
                        (unsigned)c->target_us, (double)c->cur_us,
                        c->reached ? "true" : "false");
    }

    RENDER_SNPRINTF("],\"cfg\":{\"sim\":%s,\"unlock_us\":[%lu,%lu],\"lock_us\":[%lu,%lu],"
                    "\"manual_us\":[%lu,%lu],\"mode\":\"%s\","
                    "\"engage\":%.2f,\"min_rpm\":%.1f,\"hold_ms\":%lu,\"hold_max_ms\":%lu,"
                    "\"probe_ms\":%lu,\"sim_speed\":%lu}}",
                    rs->sim ? "true" : "false",
                    (unsigned long)cfg->unlock_us[0], (unsigned long)cfg->unlock_us[1],
                    (unsigned long)cfg->lock_us[0], (unsigned long)cfg->lock_us[1],
                    (unsigned long)cfg->manual_us[0], (unsigned long)cfg->manual_us[1], mode,
                    (double)cfg->slip_engage_ratio, (double)cfg->slip_min_rpm,
                    (unsigned long)cfg->lock_hold_ms, (unsigned long)cfg->lock_hold_max_ms,
                    (unsigned long)cfg->probe_window_ms, (unsigned long)rs->sim_speed_us_s);

#undef RENDER_SNPRINTF

    buf[off] = '\0';
    *used = off;
    return ESP_OK;
}
