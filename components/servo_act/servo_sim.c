/* servo_sim 实现：见头文件注释。 */
#include <string.h>

#include "servo_act.h"

/* 到达容差：小于 1µs 直接吸附到目标。浮点逐周期逼近永远差最后一点点，
 * 不吸附就会一直 reported 未到达。 */
#define SERVO_SIM_EPS_US 1.0f

void servo_sim_init(servo_sim_t *s, float max_rate_us_s, uint16_t start_us)
{
    if (s == NULL) {
        return;
    }
    memset(s, 0, sizeof(*s));
    s->max_rate_us_s = (max_rate_us_s > 0.0f) ? max_rate_us_s : 1.0f;
    for (int i = 0; i < SERVO_ACT_CHANNELS; i++) {
        s->cur_us[i] = (float)start_us;
        s->target_us[i] = (float)start_us;
        s->enabled[i] = true;
    }
    s->primed = false;
}

static esp_err_t sim_set_us(void *ctx, uint8_t ch, uint32_t us)
{
    if (ctx == NULL || ch >= SERVO_ACT_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (us < SERVO_ACT_MIN_US || us > SERVO_ACT_MAX_US) {
        return ESP_ERR_INVALID_ARG;
    }
    ((servo_sim_t *)ctx)->target_us[ch] = (float)us;
    return ESP_OK;
}

static esp_err_t sim_set_enabled(void *ctx, uint8_t ch, bool en)
{
    if (ctx == NULL || ch >= SERVO_ACT_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    ((servo_sim_t *)ctx)->enabled[ch] = en;
    return ESP_OK;
}

static esp_err_t sim_step(void *ctx, uint32_t now_ms)
{
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    servo_sim_t *s = (servo_sim_t *)ctx;

    /* 首次只建立时间基准：last_ms 初值 0 会算出巨大的 dt，让"到达过程"
     * 在第一个周期一步到位——验证台正要观察这个过程，不能被吃掉。 */
    if (!s->primed) {
        s->primed = true;
        s->last_ms = now_ms;
        return ESP_OK;
    }

    float dt_s = (float)(now_ms - s->last_ms) / 1000.0f;
    s->last_ms = now_ms;
    if (dt_s <= 0.0f) {
        return ESP_OK;
    }
    float max_move = s->max_rate_us_s * dt_s;

    for (int i = 0; i < SERVO_ACT_CHANNELS; i++) {
        if (!s->enabled[i]) {
            continue;
        }
        float diff = s->target_us[i] - s->cur_us[i];
        float mag = (diff < 0.0f) ? -diff : diff;
        if (mag <= SERVO_SIM_EPS_US) {
            s->cur_us[i] = s->target_us[i]; /* 吸附到位 */
        } else if (mag <= max_move) {
            s->cur_us[i] = s->target_us[i]; /* 本周期内可走完 */
        } else {
            s->cur_us[i] += (diff > 0.0f) ? max_move : -max_move;
        }
    }
    return ESP_OK;
}

static esp_err_t sim_get_state(void *ctx, servo_act_chan_state_t out[SERVO_ACT_CHANNELS])
{
    if (ctx == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const servo_sim_t *s = (const servo_sim_t *)ctx;
    for (int i = 0; i < SERVO_ACT_CHANNELS; i++) {
        float diff = s->target_us[i] - s->cur_us[i];
        out[i].enabled = s->enabled[i];
        out[i].cur_us = s->cur_us[i];
        out[i].target_us = s->target_us[i];
        out[i].reached = (diff < SERVO_SIM_EPS_US) && (diff > -SERVO_SIM_EPS_US);
    }
    return ESP_OK;
}

const servo_act_ops_t servo_sim_ops = {
    .set_us = sim_set_us,
    .set_enabled = sim_set_enabled,
    .step = sim_step,
    .get_state = sim_get_state,
};
