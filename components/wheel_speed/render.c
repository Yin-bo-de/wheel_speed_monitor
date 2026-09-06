/* render 实现：见头文件注释。 */
#include <stdio.h>

#include "wheel_speed_render.h"

esp_err_t wheel_render_block(const wheel_render_state_t *rs,
                             char *buf, size_t len, size_t *used)
{
    if (rs == NULL || buf == NULL || used == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const char *src = (rs->source == WHEEL_RENDER_SRC_SIM) ? "sim" : "pcnt";
    size_t off = 0;

#define RENDER_SNPRINTF(...) do { \
        int n_ = snprintf((buf) + off, (len) - off, __VA_ARGS__); \
        if (n_ < 0 || (size_t)n_ >= (len) - off) { return ESP_ERR_INVALID_SIZE; } \
        off += (size_t)n_; \
    } while (0)

    RENDER_SNPRINTF("{\"type\":\"wheel_speed\",\"src\":\"%s\",\"wheels\":[", src);

    for (int i = 0; i < WHEEL_RENDER_COUNT; i++) {
        const wheel_render_wheel_t *w = &rs->wheels[i];
        RENDER_SNPRINTF("%s{\"i\":%d,\"en\":%s,\"rpm\":%.1f,\"speed_cms\":%.1f,"
                        "\"freq\":%.3f,\"pulses\":%lu,\"trigger\":%s}",
                        i ? "," : "",
                        i, w->enabled ? "true" : "false",
                        w->rpm, w->speed_cm_s, w->freq_hz,
                        (unsigned long)w->pulses,
                        w->trigger ? "true" : "false");
    }

    RENDER_SNPRINTF("],\"cfg\":{\"magnets\":%lu,\"diam_mm\":%lu,\"alpha\":%.2f,"
                    "\"sim\":%s,\"sim_rpm\":[%.1f,%.1f,%.1f,%.1f]}}",
                    (unsigned long)rs->magnets, (unsigned long)rs->wheel_diam_mm,
                    rs->alpha,
                    rs->sim_on ? "true" : "false",
                    rs->sim_rpm[0], rs->sim_rpm[1], rs->sim_rpm[2], rs->sim_rpm[3]);

#undef RENDER_SNPRINTF

    buf[off] = '\0';
    *used = off;
    return ESP_OK;
}
