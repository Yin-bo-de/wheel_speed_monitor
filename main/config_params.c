/* config_params 实现：见头文件注释。 */
#include <string.h>

#include "config_params.h"

void cfg_params_default(cfg_params_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->magnets = 1;
    cfg->wheel_diam_mm = 105;
    cfg->alpha = 0.3f;
    for (int i = 0; i < CFG_WHEEL_COUNT; i++) {
        cfg->wheel_enabled[i] = true;
    }
    cfg->sim_on = false;
    cfg->debounce_ms = 0;
}

esp_err_t cfg_params_validate(const cfg_params_t *cfg, const char *field)
{
    if (strcmp(field, "magnets") == 0) {
        if (cfg->magnets < CFG_MAGNETS_MIN || cfg->magnets > CFG_MAGNETS_MAX) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (strcmp(field, "diam") == 0 || strcmp(field, "diam_mm") == 0) {
        if (cfg->wheel_diam_mm < CFG_DIAM_MM_MIN || cfg->wheel_diam_mm > CFG_DIAM_MM_MAX) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (strcmp(field, "alpha") == 0) {
        if (cfg->alpha < CFG_ALPHA_MIN || cfg->alpha > CFG_ALPHA_MAX) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (strcmp(field, "debounce") == 0 || strcmp(field, "debounce_ms") == 0) {
        if (cfg->debounce_ms > 1000) {
            return ESP_ERR_INVALID_ARG; /* 上限 1s（窗口周期 50ms，去抖带宽合理） */
        }
    }
    /* en/sim/sim_rpm 为 bool/float 数组，JSON 解析层负责类型约束，这里默认放行 */
    return ESP_OK;
}

esp_err_t cfg_params_pack(const cfg_params_t *cfg, uint8_t *buf, size_t len)
{
    if (buf == NULL || len < CFG_PARAMS_BLOB_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(buf, 0, CFG_PARAMS_BLOB_SIZE);
    buf[0] = CFG_PARAMS_BLOB_VERSION;
    buf[1] = (uint8_t)cfg->magnets;
    memcpy(buf + 2, &cfg->wheel_diam_mm, 4);
    memcpy(buf + 6, &cfg->alpha, 4);
    uint8_t en = 0;
    for (int i = 0; i < CFG_WHEEL_COUNT; i++) {
        if (cfg->wheel_enabled[i]) {
            en |= (uint8_t)(1 << i);
        }
    }
    buf[10] = en;
    buf[11] = cfg->sim_on ? 1 : 0;
    memcpy(buf + 12, cfg->sim_rpm, 16);
    memcpy(buf + 28, &cfg->debounce_ms, 4);
    return ESP_OK;
}

esp_err_t cfg_params_unpack(const uint8_t *buf, size_t len, cfg_params_t *out)
{
    if (buf == NULL || out == NULL || len < CFG_PARAMS_BLOB_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (buf[0] != CFG_PARAMS_BLOB_VERSION) {
        return ESP_ERR_INVALID_VERSION;
    }
    memset(out, 0, sizeof(*out));
    out->magnets = buf[1];
    memcpy(&out->wheel_diam_mm, buf + 2, 4);
    memcpy(&out->alpha, buf + 6, 4);
    uint8_t en = buf[10];
    for (int i = 0; i < CFG_WHEEL_COUNT; i++) {
        out->wheel_enabled[i] = (en >> i) & 1;
    }
    out->sim_on = buf[11] != 0;
    memcpy(out->sim_rpm, buf + 12, 16);
    memcpy(&out->debounce_ms, buf + 28, 4);
    return ESP_OK;
}
