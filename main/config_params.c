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

    cfg->servo_sim = true;
    for (int i = 0; i < CFG_SERVO_COUNT; i++) {
        cfg->servo_en[i] = true;
        cfg->servo_invert[i] = false;
        cfg->servo_manual_us[i] = 1500;
    }
    cfg->servo_min_us = 1000;
    cfg->servo_center_us = 1500;
    cfg->servo_max_us = 2000;
    cfg->servo_mode = CFG_SERVO_MODE_AUTO;

    cfg->slip_engage_ratio = 0.30f;
    cfg->slip_min_rpm = 10.0f;
    cfg->lock_hold_ms = 3000;
    cfg->lock_hold_max_ms = 30000;
    cfg->probe_window_ms = 2000;
    cfg->servo_sim_speed_us_s = 3000;
}

/* 脉宽三元组：三个字段任一被写都要满足 min < center < max，
 * 否则一侧量程为零，映射会退化成常数。 */
static bool servo_triplet_ok(const cfg_params_t *cfg)
{
    return cfg->servo_min_us < cfg->servo_center_us &&
           cfg->servo_center_us < cfg->servo_max_us;
}

esp_err_t cfg_params_validate(const cfg_params_t *cfg, const char *field)
{
    if (cfg == NULL || field == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

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
    } else if (strcmp(field, "servo_min_us") == 0) {
        if (cfg->servo_min_us < CFG_SERVO_US_MIN || !servo_triplet_ok(cfg)) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (strcmp(field, "servo_center_us") == 0) {
        if (cfg->servo_center_us < CFG_SERVO_US_MIN || !servo_triplet_ok(cfg)) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (strcmp(field, "servo_max_us") == 0) {
        if (cfg->servo_max_us > CFG_SERVO_US_MAX || !servo_triplet_ok(cfg)) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (strcmp(field, "servo_manual_us") == 0) {
        for (int i = 0; i < CFG_SERVO_COUNT; i++) {
            if (cfg->servo_manual_us[i] < CFG_SERVO_US_MIN ||
                cfg->servo_manual_us[i] > CFG_SERVO_US_MAX) {
                return ESP_ERR_INVALID_ARG;
            }
        }
    } else if (strcmp(field, "servo_mode") == 0) {
        if (cfg->servo_mode > CFG_SERVO_MODE_AUTO) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (strcmp(field, "slip_engage_ratio") == 0) {
        if (cfg->slip_engage_ratio < CFG_SLIP_ENGAGE_MIN || cfg->slip_engage_ratio > 1.0f) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (strcmp(field, "slip_min_rpm") == 0) {
        if (cfg->slip_min_rpm < 0.0f || cfg->slip_min_rpm > CFG_SLIP_MIN_RPM_MAX) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (strcmp(field, "lock_hold_ms") == 0) {
        if (cfg->lock_hold_ms < CFG_LOCK_HOLD_MIN_MS ||
            cfg->lock_hold_ms > CFG_LOCK_HOLD_MAX_MS) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (strcmp(field, "lock_hold_max_ms") == 0) {
        if (cfg->lock_hold_max_ms < CFG_LOCK_HOLD_MIN_MS ||
            cfg->lock_hold_max_ms > CFG_LOCK_HOLD_MAX_MS ||
            cfg->lock_hold_max_ms < cfg->lock_hold_ms) {
            return ESP_ERR_INVALID_ARG; /* 上限低于初始 → 翻倍退避失去意义 */
        }
    } else if (strcmp(field, "probe_window_ms") == 0) {
        if (cfg->probe_window_ms < CFG_PROBE_WINDOW_MIN_MS ||
            cfg->probe_window_ms > CFG_PROBE_WINDOW_MAX_MS) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (strcmp(field, "servo_sim_speed_us_s") == 0) {
        if (cfg->servo_sim_speed_us_s < CFG_SIM_SPEED_MIN ||
            cfg->servo_sim_speed_us_s > CFG_SIM_SPEED_MAX) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    /* en/sim/sim_rpm/servo_en/servo_invert 为 bool 数组，JSON 解析层负责类型约束 */
    return ESP_OK;
}

/* ---- v1 段（0..31）：两版共用 ---- */

static void pack_v1_segment(const cfg_params_t *cfg, uint8_t *buf)
{
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
}

static void unpack_v1_segment(const uint8_t *buf, cfg_params_t *out)
{
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
}

/* ---- v2 段（32..71）：舵机与锁定策略 ---- */

static void pack_v2_segment(const cfg_params_t *cfg, uint8_t *buf)
{
    uint16_t min_us = (uint16_t)cfg->servo_min_us;
    uint16_t center_us = (uint16_t)cfg->servo_center_us;
    uint16_t max_us = (uint16_t)cfg->servo_max_us;
    uint16_t manual[CFG_SERVO_COUNT];
    for (int i = 0; i < CFG_SERVO_COUNT; i++) {
        manual[i] = (uint16_t)cfg->servo_manual_us[i];
    }

    /* buf[34] 低 4 位装模式、高 4 位装 invert 位图：两者都是小枚举，
     * 合用一个字节免得再往后挤，v2 段正好落在 72 字节内。 */
    uint8_t inv = 0;
    uint8_t en = 0;
    for (int i = 0; i < CFG_SERVO_COUNT; i++) {
        if (cfg->servo_en[i]) {
            en |= (uint8_t)(1 << i);
        }
        if (cfg->servo_invert[i]) {
            inv |= (uint8_t)(1 << i);
        }
    }

    buf[32] = cfg->servo_sim ? 1 : 0;
    buf[33] = en;
    buf[34] = (uint8_t)((cfg->servo_mode & 0x0F) | (inv << 4));
    memcpy(buf + 35, &min_us, 2);
    memcpy(buf + 37, &center_us, 2);
    memcpy(buf + 39, &max_us, 2);
    memcpy(buf + 41, manual, 4);
    memcpy(buf + 45, &cfg->slip_engage_ratio, 4);
    memcpy(buf + 49, &cfg->slip_min_rpm, 4);
    memcpy(buf + 53, &cfg->lock_hold_ms, 4);
    memcpy(buf + 57, &cfg->lock_hold_max_ms, 4);
    memcpy(buf + 61, &cfg->probe_window_ms, 4);
    memcpy(buf + 65, &cfg->servo_sim_speed_us_s, 4);
    /* 69..71 保留 */
}

static void unpack_v2_segment(const uint8_t *buf, cfg_params_t *out)
{
    uint16_t min_us = 0, center_us = 0, max_us = 0;
    uint16_t manual[CFG_SERVO_COUNT] = {0};

    out->servo_sim = buf[32] != 0;
    uint8_t en = buf[33];
    for (int i = 0; i < CFG_SERVO_COUNT; i++) {
        out->servo_en[i] = (en >> i) & 1;
        out->servo_invert[i] = ((buf[34] >> (4 + i)) & 1) != 0;
    }
    out->servo_mode = buf[34] & 0x0F;
    memcpy(&min_us, buf + 35, 2);
    memcpy(&center_us, buf + 37, 2);
    memcpy(&max_us, buf + 39, 2);
    memcpy(manual, buf + 41, 4);
    out->servo_min_us = min_us;
    out->servo_center_us = center_us;
    out->servo_max_us = max_us;
    for (int i = 0; i < CFG_SERVO_COUNT; i++) {
        out->servo_manual_us[i] = manual[i];
    }
    memcpy(&out->slip_engage_ratio, buf + 45, 4);
    memcpy(&out->slip_min_rpm, buf + 49, 4);
    memcpy(&out->lock_hold_ms, buf + 53, 4);
    memcpy(&out->lock_hold_max_ms, buf + 57, 4);
    memcpy(&out->probe_window_ms, buf + 61, 4);
    memcpy(&out->servo_sim_speed_us_s, buf + 65, 4);
}

esp_err_t cfg_params_pack(const cfg_params_t *cfg, uint8_t *buf, size_t len)
{
    if (cfg == NULL || buf == NULL || len < CFG_PARAMS_BLOB_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(buf, 0, CFG_PARAMS_BLOB_SIZE);
    pack_v1_segment(cfg, buf);
    pack_v2_segment(cfg, buf);
    return ESP_OK;
}

esp_err_t cfg_params_unpack(const uint8_t *buf, size_t len, cfg_params_t *out)
{
    if (buf == NULL || out == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 先铺默认值：v1 blob 没有 v2 字段，新参数必须落到默认值而不是 0，
     * 否则升级后会得到一组非法配置（如脉宽三元组全 0）。 */
    cfg_params_default(out);

    if (buf[0] == CFG_PARAMS_BLOB_VERSION_1) {
        if (len < CFG_PARAMS_BLOB_SIZE_V1) {
            return ESP_ERR_INVALID_ARG;
        }
        unpack_v1_segment(buf, out);
        return ESP_OK; /* 新版字段保持默认；上层随后回写 v2 */
    }
    if (buf[0] == CFG_PARAMS_BLOB_VERSION) {
        if (len < CFG_PARAMS_BLOB_SIZE) {
            return ESP_ERR_INVALID_ARG;
        }
        unpack_v1_segment(buf, out);
        unpack_v2_segment(buf, out);
        return ESP_OK;
    }
    return ESP_ERR_INVALID_VERSION;
}
