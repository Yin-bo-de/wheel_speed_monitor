/* rc_render 实现：见头文件注释。 */
#include <stdio.h>

#include "rc_render.h"

esp_err_t rc_render_block(const rc_render_state_t *rs, char *buf, size_t len, size_t *used)
{
    if (rs == NULL || buf == NULL || used == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t off = 0;

#define RENDER_SNPRINTF(...) do { \
        int n_ = snprintf((buf) + off, (len) - off, __VA_ARGS__); \
        if (n_ < 0 || (size_t)n_ >= (len) - off) { return ESP_ERR_INVALID_SIZE; } \
        off += (size_t)n_; \
    } while (0)

    /* 空指针兜底成默认值而不是报错：这是个纯展示块，缺字段时让页面显示
     * 一个明确的默认态，比整帧少一块更容易排查 */
    RENDER_SNPRINTF("{\"type\":\"rc\",\"src\":\"%s\",\"mode\":\"%s\",\"mode_src\":\"%s\",\"ch\":[",
                    (rs->src != NULL) ? rs->src : "sim",
                    (rs->mode != NULL) ? rs->mode : "auto",
                    (rs->mode_src != NULL) ? rs->mode_src : "cfg");

    for (int i = 0; i < RC_CH_COUNT; i++) {
        const rc_render_chan_t *c = &rs->ch[i];
        RENDER_SNPRINTF("%s{\"us\":%u,\"valid\":%s,\"seen\":%s,\"age_ms\":%lu}",
                        i ? "," : "", (unsigned)c->us,
                        c->valid ? "true" : "false", c->seen ? "true" : "false",
                        (unsigned long)c->age_ms);
    }

    RENDER_SNPRINTF("],\"target\":[");
    for (int i = 0; i < RC_DIFF_COUNT; i++) {
        const rc_render_target_t *t = &rs->target[i];
        RENDER_SNPRINTF("%s{\"us\":%u,\"from_rc\":%s}",
                        i ? "," : "", (unsigned)t->us, t->from_rc ? "true" : "false");
    }

    RENDER_SNPRINTF("],\"cfg\":{\"sim\":%s,\"sim_us\":[%lu,%lu,%lu]}}",
                    rs->sim ? "true" : "false",
                    (unsigned long)rs->sim_us[0], (unsigned long)rs->sim_us[1],
                    (unsigned long)rs->sim_us[2]);

#undef RENDER_SNPRINTF

    buf[off] = '\0';
    *used = off;
    return ESP_OK;
}
