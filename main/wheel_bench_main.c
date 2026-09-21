/*
 * wheel_bench_main：入口编排。
 * NVS → 传感器 → 采集器注册 → 任务 → WiFi AP → httpd。
 *
 * 数据流：sampler_task（Core0）每 50ms 读 4 路 PCNT 计数与 GPIO 电平
 * → wheel 采集器运算 → telemetry 聚合渲染 → ws_push_task（Core0）每 100ms
 * 经 WebSocket 推给验证台网页。命令在 httpd 上下文处理（config 变更 + NVS）。
 *
 * 舵机控制链挂在采样任务里，同一节拍推进：
 *   四轮 rpm_ema → strategy（状态机算目标脉宽）→ 舵机执行器（模拟模型 / LEDC）
 *   → 填 servo 渲染快照。不另起任务：只有几次浮点比较和几个寄存器写。
 *
 * 任务表（优先/栈/核心见 wheel_config.h；固化进项目 CLAUDE.md）：
 *   sampler_task  Core0 22 3072  50ms 采样 + 舵机控制链
 *   ws_push_task  Core0 15 8192  100ms 推遥测帧
 *   httpd server  Core1  5 8192  事件驱动
 */
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bench_http_server.h"
#include "config_store.h"
#include "servo_act.h"
#include "servo_drv.h"
#include "strategy.h"
#include "strategy_render.h"
#include "telemetry.h"
#include "wheel_bench.h"
#include "wheel_config.h"
#include "wheel_sensor.h"

static const char *TAG = "wheelbench";

static cfg_params_t s_cfg;
static SemaphoreHandle_t s_cfg_mutex;
static wheel_unit_t s_units[WHEEL_COUNT];
static wheel_chan_cfg_t s_chan_cfg;

/* ---- 舵机控制链状态 ----
 * 全部只由 sampler_task 写（启动时的初始化除外，那时任务还没起来）。
 * 配置从 s_cfg 每拍现读现用，不跨任务共享可变结构。 */
static servo_sim_t s_servo_sim;
static servo_ledc_t s_servo_ledc;
static strategy_config_t s_strat_cfg; /* 采样任务专用，不跨任务共享 */
static servo_render_state_t s_servo_disp; /* 采样任务写、ws_push 读（同 disp 约定） */

/* 执行器绑定：ops 和它的 ctx 必须同源。拆成两个独立变量就会出现"新 ops + 旧 ctx"
 * 的错配窗口——两边的 ctx 类型不同，等于拿 A 的结构体当 B 用。成对放进结构体，
 * 切换只改这一个指针。 */
typedef struct {
    const servo_act_ops_t *ops;
    void *ctx;
} act_binding_t;

static const act_binding_t s_act_sim = {&servo_sim_ops, &s_servo_sim};
static const act_binding_t s_act_ledc = {&servo_ledc_ops, &s_servo_ledc};
static const act_binding_t *s_act = &s_act_sim;
static bool s_ledc_failed; /* 初始化失败过：不每 50ms 重试刷日志，等用户重新选一次 */

/* ---- wheel_speed 采集器 ops（telemetry 框架注册） ---- */

esp_err_t wheel_bench_apply_config(const cfg_params_t *cfg)
{
    wheel_chan_cfg_t *cc = &s_chan_cfg;
    cc->window_ms = WHEEL_SAMPLE_MS;
    cc->magnets = cfg->magnets;
    cc->wheel_diam_mm = cfg->wheel_diam_mm;
    cc->alpha = cfg->alpha;
    cc->debounce_ms = cfg->debounce_ms;

    /* 换源/启停变更会改变基准，重置避免毛刺 */
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (cfg->sim_on) {
            sim_source_init(&s_units[i].sim);
        }
        if (cfg->wheel_enabled[i]) {
            uint16_t c = 0;
            if (cfg->sim_on) {
                c = sim_source_counter(&s_units[i].sim, (uint8_t)i);
            } else {
                wheel_sensor_get_valid_count((uint8_t)i, &c);
            }
            wheel_chan_reset_baseline(&s_units[i].chan, c);
        } else {
            wheel_chan_reset_baseline(&s_units[i].chan, 0);
        }
    }
    return ESP_OK;
}

static esp_err_t wheel_collector_sample(void *ctx, uint32_t now_ms)
{
    (void)ctx;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        if (!s_cfg.wheel_enabled[i]) {
            continue;
        }
        if (s_cfg.sim_on) {
            /* 模拟源：sim_source 产出跨窗口正确间隔，与真实源走同一间隔法链 */
            sim_source_set_target_rpm(&s_units[i].sim, (uint8_t)i, s_cfg.sim_rpm[i]);
            sim_source_step(&s_units[i].sim, WHEEL_SAMPLE_MS);
            uint32_t interval_us = sim_source_period_us(&s_units[i].sim, (uint8_t)i);
            if (interval_us > 0) {
                wheel_chan_sample_period(&s_units[i].chan, &s_chan_cfg, interval_us, now_ms);
            }
            uint16_t c = sim_source_counter(&s_units[i].sim, (uint8_t)i);
            /* sample 只累计脉冲 + 静止归零兜底，不计算频率 */
            wheel_chan_sample(&s_units[i].chan, &s_chan_cfg, c, now_ms);
        } else {
            /* 真实源：先间隔法（更新频率/RPM/EMA），再 sample（累计+静止归零）。
             * 计数用 ISR 维护的有效计数（去抖后），避免 PCNT 原始计数
             * 被磁铁贴近抖动污染成百地增加。 */
            uint32_t interval_us = 0;
            wheel_sensor_get_period_us((uint8_t)i, &interval_us);
            if (interval_us > 0) {
                wheel_chan_sample_period(&s_units[i].chan, &s_chan_cfg, interval_us, now_ms);
            }
            uint16_t c = 0;
            wheel_sensor_get_valid_count((uint8_t)i, &c);
            wheel_chan_sample(&s_units[i].chan, &s_chan_cfg, c, now_ms);
        }
    }
    return ESP_OK;
}

static esp_err_t wheel_collector_render(void *ctx, char *buf, size_t len, size_t *used)
{
    (void)ctx;
    return wheel_render_block(&s_units[0].disp, buf, len, used);
}

static esp_err_t wheel_collector_reset(void *ctx)
{
    (void)ctx;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        uint16_t c = 0;
        if (s_cfg.sim_on) {
            sim_source_init(&s_units[i].sim);
        } else {
            wheel_sensor_get_valid_count((uint8_t)i, &c);
        }
        wheel_chan_reset_baseline(&s_units[i].chan, c);
    }
    return ESP_OK;
}

static const telem_collector_ops_t s_wheel_ops = {
    .type = TELEM_TYPE_WHEEL_SPEED,
    .init = NULL,
    .sample = wheel_collector_sample,
    .render = wheel_collector_render,
    .reset_baseline = wheel_collector_reset,
};

/* ---- 舵机控制链（采样任务内推进） ---- */

/* cfg_params → strategy 配置。每拍现读现构，避免跨任务共享可变结构。 */
static void strat_cfg_build(const cfg_params_t *cfg, strategy_config_t *out)
{
    strategy_config_default(out);
    for (int i = 0; i < STRATEGY_SERVO_COUNT; i++) {
        out->unlock_us[i] = cfg->servo_unlock_us[i];
        out->lock_us[i] = cfg->servo_lock_us[i];
        out->ch_enabled[i] = cfg->servo_en[i];
        out->manual_us[i] = cfg->servo_manual_us[i];
    }
    out->mode = (cfg->servo_mode == CFG_SERVO_MODE_MANUAL) ? STRATEGY_MODE_MANUAL
                                                           : STRATEGY_MODE_AUTO;
    out->slip_engage_ratio = cfg->slip_engage_ratio;
    out->slip_min_rpm = cfg->slip_min_rpm;
    out->lock_hold_ms = cfg->lock_hold_ms;
    out->lock_hold_max_ms = cfg->lock_hold_max_ms;
    out->probe_window_ms = cfg->probe_window_ms;
}

/* 按配置在模拟模型与 LEDC 之间选执行器。
 *
 * 刻意放在采样任务里做，而不是挂在 httpd 的配置变更回调上：这样 LEDC 的初始化和
 * 寄存器写全部落在同一个任务上下文，httpd 只改配置数据、永不碰硬件句柄，
 * 省掉跨核的初始化窗口和一把锁。 */
static void servo_act_select(bool sim)
{
    if (sim) {
        if (s_act == &s_act_ledc) {
            /* 切回模拟前先松开真舵机，否则它一直吃着最后那条脉宽 */
            servo_ledc_ops.set_enabled(&s_servo_ledc, 0, false);
            servo_ledc_ops.set_enabled(&s_servo_ledc, 1, false);
            servo_sim_init(&s_servo_sim, (float)s_cfg.servo_sim_speed_us_s,
                           s_cfg.servo_unlock_us);
        }
        s_act = &s_act_sim;
        s_ledc_failed = false; /* 切回模拟 = 给下一次选择留一次重试机会 */
        return;
    }

    if (s_ledc_failed) {
        return;
    }
    if (!s_servo_ledc.inited) {
        esp_err_t e = servo_ledc_init(&s_servo_ledc, WHEEL_SERVO_GPIO_FRONT,
                                      WHEEL_SERVO_GPIO_REAR, s_cfg.servo_unlock_us);
        if (e != ESP_OK) {
            /* 不静默退回：日志留证，遥测的 src 也会照实报 "sim"，
             * 页面上能直接看出"配置要 LEDC、实际跑模拟"这个不一致。 */
            ESP_LOGE(TAG, "LEDC 舵机初始化失败(%d)，继续用模拟舵机", e);
            s_ledc_failed = true;
            return;
        }
    }
    s_act = &s_act_ledc;
}

static void servo_control_step(uint32_t now_ms)
{
    servo_act_select(s_cfg.servo_sim);
    strat_cfg_build(&s_cfg, &s_strat_cfg);
    s_servo_sim.max_rate_us_s = (float)s_cfg.servo_sim_speed_us_s;

    /* 被禁用的轮子不再更新 math，值是停用前的残留；喂 0 免得拿陈旧转速判打滑 */
    float rpm[WHEEL_COUNT];
    for (int i = 0; i < WHEEL_COUNT; i++) {
        rpm[i] = s_cfg.wheel_enabled[i] ? s_units[i].chan.math.rpm_ema : 0.0f;
    }

    strategy_state_t st;
    if (strategy_feed_wheel_rpm(rpm, &s_strat_cfg, now_ms) != ESP_OK ||
        strategy_get_state(&st) != ESP_OK) {
        return;
    }

    for (int i = 0; i < STRATEGY_SERVO_COUNT; i++) {
        /* 先给目标、再使能：使能那一步会照已记下的目标出波，
         * 省掉"先输出旧值、再改成新值"的多余一次寄存器写。 */
        s_act->ops->set_us(s_act->ctx, (uint8_t)i, st.servo[i].target_us);
        s_act->ops->set_enabled(s_act->ctx, (uint8_t)i, s_strat_cfg.ch_enabled[i]);
    }
    s_act->ops->step(s_act->ctx, now_ms);

    /* 填渲染快照：state 由这里推进，采集器的 sample 不再重复推一次 */
    servo_act_chan_state_t act[SERVO_ACT_CHANNELS];
    s_act->ops->get_state(s_act->ctx, act);
    servo_render_state_t *d = &s_servo_disp;
    /* src 报"实际在跑哪个执行器"，不是"配置想要哪个"：LEDC 起不来时两者会不一致，
     * 页面必须看得见这个不一致（sim 字段仍照配置回显，供设置表单回填）。 */
    d->src = (s_act == &s_act_ledc) ? "ledc" : "sim";
    d->sim = s_cfg.servo_sim;
    d->sim_speed_us_s = s_cfg.servo_sim_speed_us_s;
    d->cfg = s_strat_cfg;
    for (int i = 0; i < STRATEGY_SERVO_COUNT; i++) {
        d->ch[i].enabled = s_strat_cfg.ch_enabled[i];
        d->ch[i].reached = act[i].reached;
        d->ch[i].target_us = st.servo[i].target_us;
        d->ch[i].cur_us = act[i].cur_us;
        d->ch[i].phase = st.servo[i].phase;
        d->ch[i].hold_ms = st.servo[i].hold_ms;
        d->ch[i].ratio = st.servo[i].ratio;
        d->ch[i].slip_fast_left = st.servo[i].slip_fast_left;
    }
}

static esp_err_t servo_collector_render(void *ctx, char *buf, size_t len, size_t *used)
{
    (void)ctx;
    return servo_render_block(&s_servo_disp, buf, len, used);
}

/* sample 留空：状态由 servo_control_step 在同一采样任务里推进，框架不必再推一次 */
static const telem_collector_ops_t s_servo_ops = {
    .type = TELEM_TYPE_SERVO,
    .init = NULL,
    .sample = NULL,
    .render = servo_collector_render,
    .reset_baseline = NULL,
};

/* ---- 采样任务 ---- */

static void sampler_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    uint8_t gpio[WHEEL_COUNT];

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(WHEEL_SAMPLE_MS));
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        /* GPIO 电平（sys 块，不受 PCNT 影响） */
        wheel_sensor_read_gpio_levels(gpio);
        telem_set_gpio_levels(gpio);

        /* 采集器运算（帧渲染与其分离，由 ws_push 完成） */
        telem_sample_all(now_ms);

        /* 舵机控制链：读刚更新的轮速，算目标、驱动执行器、填快照 */
        servo_control_step(now_ms);
    }
}

/* ---- 遥测推送任务 ---- */

static void ws_push_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    char frame[2048];
    uint32_t seq = 0;

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(WHEEL_WS_PUSH_MS));
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        /* 组装各轮显示快照：全部写 s_units[0].disp（渲染端固定读它，
         * 此前误写各 unit 自己的 wheels[i]，导致只有 wheels[0] 有值、
         * wheels[1..3] 全零→页面显示禁用/0 RPM） */
        wheel_render_state_t *d = &s_units[0].disp;
        d->source = s_cfg.sim_on ? WHEEL_RENDER_SRC_SIM : WHEEL_RENDER_SRC_PCNT;
        d->magnets = s_cfg.magnets;
        d->wheel_diam_mm = s_cfg.wheel_diam_mm;
        d->alpha = s_cfg.alpha;
        d->debounce_ms = s_cfg.debounce_ms;
        d->sim_on = s_cfg.sim_on;
        for (int j = 0; j < WHEEL_COUNT; j++) {
            d->sim_rpm[j] = s_cfg.sim_rpm[j];
        }
        for (int i = 0; i < WHEEL_COUNT; i++) {
            wheel_unit_t *u = &s_units[i];
            wheel_render_wheel_t *w = &d->wheels[i];
            w->enabled = s_cfg.wheel_enabled[i];
            w->rpm = u->chan.math.rpm_ema;
            w->rps = u->chan.math.rpm_ema / 60.0f;
            w->speed_cm_s = u->chan.math.speed_cm_s;
            w->speed_km_h = u->chan.math.speed_km_h;
            w->freq_hz = u->chan.math.freq_hz;
            w->pulses = u->chan.pulses_total;
            w->trigger = u->chan.trigger;
        }

        size_t used = 0;
        esp_err_t e = telem_render_frame(frame, sizeof(frame), &used, seq, now_ms,
                                         (uint32_t)esp_get_free_heap_size());
        if (e == ESP_OK) {
            bench_http_server_broadcast(frame, used, seq++);
        }
    }
}

/* ---- WiFi AP ---- */

static void wifi_ap_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap = {
        .ap = {
            .ssid = WHEEL_AP_SSID,
            .password = WHEEL_AP_PASSWD,
            .ssid_len = 0,
            .channel = 1,
            .max_connection = 4, /* 验证台最多 4 台设备（与 sdkconfig CONFIG_ESP_WIFI_AP_MAX_STA_CONN=4 一致） */
            .authmode = WIFI_AUTH_OPEN,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* create_default_wifi_ap() 已默认 DHCP server 192.168.4.1，勿再 set_ip_info
     * （会因 DHCP 已启动返回 ESP_ERR_INVALID_STATE） */
    ESP_LOGI(TAG, "WiFi AP up: %s @ %s", WHEEL_AP_SSID, WHEEL_AP_IP);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    cfg_params_t cfg;
    ESP_ERROR_CHECK(config_store_load(&cfg));
    s_cfg = cfg;
    s_cfg_mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "config: magnets=%lu diam=%lu alpha=%.2f en=[%d,%d,%d,%d] sim=%d",
             (unsigned long)cfg.magnets, (unsigned long)cfg.wheel_diam_mm,
             (double)cfg.alpha,
             cfg.wheel_enabled[0], cfg.wheel_enabled[1],
             cfg.wheel_enabled[2], cfg.wheel_enabled[3], cfg.sim_on);
    /* 舵机配置值得单独打一行：升级后这里能直接看出新字段是否落到了预期值
     * （v1 blob 迁移后新字段取默认，一眼可辨） */
    ESP_LOGI(TAG, "servo: sim=%d mode=%lu en=[%d,%d] 解锁/锁定=[%lu/%lu,%lu/%lu] "
                  "手动=[%lu,%lu] engage=%.2f minrpm=%.1f hold=%lu/%lu probe=%lu speed=%lu",
             cfg.servo_sim, (unsigned long)cfg.servo_mode,
             cfg.servo_en[0], cfg.servo_en[1],
             (unsigned long)cfg.servo_unlock_us[0], (unsigned long)cfg.servo_lock_us[0],
             (unsigned long)cfg.servo_unlock_us[1], (unsigned long)cfg.servo_lock_us[1],
             (unsigned long)cfg.servo_manual_us[0], (unsigned long)cfg.servo_manual_us[1],
             (double)cfg.slip_engage_ratio, (double)cfg.slip_min_rpm,
             (unsigned long)cfg.lock_hold_ms, (unsigned long)cfg.lock_hold_max_ms,
             (unsigned long)cfg.probe_window_ms,
             (unsigned long)cfg.servo_sim_speed_us_s);

    /* collector 配置快照 */
    s_chan_cfg.window_ms = WHEEL_SAMPLE_MS;
    s_chan_cfg.magnets = cfg.magnets;
    s_chan_cfg.wheel_diam_mm = cfg.wheel_diam_mm;
    s_chan_cfg.alpha = cfg.alpha;
    s_chan_cfg.debounce_ms = cfg.debounce_ms;

    static const uint8_t hall_gpios[WHEEL_COUNT] = {
        WHEEL_HALL_GPIO_LF, WHEEL_HALL_GPIO_RF, WHEEL_HALL_GPIO_LR, WHEEL_HALL_GPIO_RR,
    };
    ESP_ERROR_CHECK(wheel_sensor_init(hall_gpios));
    for (int i = 0; i < WHEEL_COUNT; i++) {
        sim_source_init(&s_units[i].sim);
        memset(&s_units[i].disp, 0, sizeof(s_units[i].disp));
    }

    /* 舵机执行器：起步用模拟模型，起始位置取配置里的解锁档（上电差速应松开）。
     * 采样任务起来之前初始化，无竞争。 */
    servo_sim_init(&s_servo_sim, (float)cfg.servo_sim_speed_us_s, cfg.servo_unlock_us);
    strategy_init();

    /* 注册采集器到 telemetry 框架 */
    esp_err_t e = telem_register(&s_wheel_ops, NULL);
    ESP_LOGI(TAG, "wheel collector register: %d", e);
    e = telem_register(&s_servo_ops, NULL);
    ESP_LOGI(TAG, "servo collector register: %d", e);

    wifi_ap_start();

    bench_server_ctx_t ctx = {
        .cfg = &s_cfg,
        .cfg_mutex = s_cfg_mutex,
        .units = s_units,
    };
    ESP_ERROR_CHECK(bench_http_server_start(&ctx));

    /* httpd 与 mutex 就绪后再起推送任务，避免 xSemaphoreTake(NULL) */
    xTaskCreatePinnedToCore(sampler_task, "sampler", WHEEL_SAMPLER_STACK, NULL,
                            WHEEL_SAMPLER_PRIO, NULL, WHEEL_SAMPLER_CORE);
    xTaskCreatePinnedToCore(ws_push_task, "ws_push", WHEEL_WS_PUSH_STACK, NULL,
                            WHEEL_WS_PUSH_PRIO, NULL, WHEEL_WS_PUSH_CORE);
}
