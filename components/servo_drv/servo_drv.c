/* servo_drv 实现：见头文件注释。 */
#include "esp_log.h"
#include "servo_drv.h"

static const char *TAG = "servo_drv";

/* 脉宽 → 占空比计数。先乘后除再四舍五入：整除截断会把中位 1500µs 少算 0.8 个
 * 计数（约 1µs），数值虽小，但没有白送误差的理由。 */
static uint32_t us_to_duty(uint32_t us)
{
    uint64_t ticks = ((uint64_t)us << SERVO_LEDC_DUTY_BITS) + SERVO_LEDC_PERIOD_US / 2;
    return (uint32_t)(ticks / SERVO_LEDC_PERIOD_US);
}

/* set_duty 只写影子寄存器，update_duty 才让它生效；update_duty 内部会把
 * sig_out_en 置回 true，所以它同时是 ledc_stop 之后重新出波的那一步。 */
static esp_err_t ledc_output(servo_ledc_t *s, uint8_t ch)
{
    esp_err_t err = ledc_set_duty(SERVO_LEDC_SPEED_MODE, (ledc_channel_t)ch,
                                  us_to_duty(s->target_us[ch]));
    if (err != ESP_OK) {
        return err;
    }
    return ledc_update_duty(SERVO_LEDC_SPEED_MODE, (ledc_channel_t)ch);
}

esp_err_t servo_ledc_init(servo_ledc_t *s, int gpio_front, int gpio_rear, uint32_t center_us)
{
    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s->inited) {
        return ESP_OK;
    }
    if (center_us < SERVO_ACT_MIN_US || center_us > SERVO_ACT_MAX_US) {
        return ESP_ERR_INVALID_ARG;
    }

    ledc_timer_config_t timer = {
        .speed_mode = SERVO_LEDC_SPEED_MODE,
        .duty_resolution = SERVO_LEDC_DUTY_RES,
        .timer_num = SERVO_LEDC_TIMER,
        .freq_hz = SERVO_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config 失败: %d", err);
        return err;
    }

    const int gpios[SERVO_ACT_CHANNELS] = {gpio_front, gpio_rear};
    for (int i = 0; i < SERVO_ACT_CHANNELS; i++) {
        ledc_channel_config_t chan = {
            .gpio_num = gpios[i],
            .speed_mode = SERVO_LEDC_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = SERVO_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
        };
        err = ledc_channel_config(&chan);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config 通道%d(GPIO%d) 失败: %d", i, gpios[i], err);
            return err;
        }
        s->gpio[i] = gpios[i];
        s->target_us[i] = center_us;
        s->enabled[i] = false;
        /* 建好先停输出：这一刻策略还没跑过第一拍，先让舵机脱力，
         * 免得挂着一个 duty=0 的波形把舵机拽到行程一端。 */
        ledc_stop(SERVO_LEDC_SPEED_MODE, (ledc_channel_t)i, 0);
    }
    s->inited = true;

    /* 读了实际频率再报"就绪"：分频器是算出来的，不是承诺出来的 */
    uint32_t actual_hz = ledc_get_freq(SERVO_LEDC_SPEED_MODE, SERVO_LEDC_TIMER);
    if (actual_hz < SERVO_LEDC_FREQ_HZ - 1 || actual_hz > SERVO_LEDC_FREQ_HZ + 1) {
        ESP_LOGW(TAG, "LEDC 实际频率 %luHz 偏离 %dHz 目标，检查时钟源配置",
                 (unsigned long)actual_hz, SERVO_LEDC_FREQ_HZ);
    }
    ESP_LOGI(TAG, "LEDC 舵机就绪 前=GPIO%d 后=GPIO%d 目标%luHz/%dbit 实际%luHz",
             gpio_front, gpio_rear, (unsigned long)SERVO_LEDC_FREQ_HZ, SERVO_LEDC_DUTY_BITS,
             (unsigned long)actual_hz);
    return ESP_OK;
}

static esp_err_t ledc_set_us(void *ctx, uint8_t ch, uint32_t us)
{
    if (ctx == NULL || ch >= SERVO_ACT_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (us < SERVO_ACT_MIN_US || us > SERVO_ACT_MAX_US) {
        return ESP_ERR_INVALID_ARG;
    }
    servo_ledc_t *s = (servo_ledc_t *)ctx;
    if (s->target_us[ch] == us) {
        return ESP_OK; /* 目标没变就不写寄存器：50ms 一拍，绝大多数拍都是重复值 */
    }
    s->target_us[ch] = us;
    if (!s->enabled[ch]) {
        return ESP_OK; /* 通道关着，记下目标即可，使能时按它出波 */
    }
    return ledc_output(s, ch);
}

static esp_err_t ledc_set_enabled(void *ctx, uint8_t ch, bool en)
{
    if (ctx == NULL || ch >= SERVO_ACT_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    servo_ledc_t *s = (servo_ledc_t *)ctx;
    if (!s->inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s->enabled[ch] == en) {
        return ESP_OK; /* 幂等：控制链每拍都调，不能每次都动寄存器 */
    }
    s->enabled[ch] = en;
    if (en) {
        return ledc_output(s, ch);
    }
    /* 停波 = 舵机脱力。停在低电平而不是让它浮空：悬空脚容易被旁边走线带出抖动，
     * 而标准舵机收不到脉冲本来就不给力矩，正是"松开"要的效果。 */
    return ledc_stop(SERVO_LEDC_SPEED_MODE, (ledc_channel_t)ch, 0);
}

static esp_err_t ledc_step(void *ctx, uint32_t now_ms)
{
    (void)now_ms;
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK; /* 波形由硬件自己走，没有需要推进的软件状态 */
}

static esp_err_t ledc_get_state(void *ctx, servo_act_chan_state_t out[SERVO_ACT_CHANNELS])
{
    if (ctx == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const servo_ledc_t *s = (const servo_ledc_t *)ctx;
    for (int i = 0; i < SERVO_ACT_CHANNELS; i++) {
        out[i].enabled = s->enabled[i];
        out[i].target_us = s->target_us[i];
        /* 舵机不回传位置，能知道的只有"让它去哪"。如实报指令值，
         * 不编一条假的到达曲线；没有"在途"这回事，reached 恒真。 */
        out[i].cur_us = (float)s->target_us[i];
        out[i].reached = true;
    }
    return ESP_OK;
}

const servo_act_ops_t servo_ledc_ops = {
    .set_us = ledc_set_us,
    .set_enabled = ledc_set_enabled,
    .step = ledc_step,
    .get_state = ledc_get_state,
};
