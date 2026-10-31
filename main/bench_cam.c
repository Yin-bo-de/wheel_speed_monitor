/* bench_cam 实现：见头文件与 docs/CAMERA_DESIGN.md 4.5.1。 */
#include <string.h>

#include "bench_cam.h"

#include "cam_rate.h"
#include "camera_drv.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/task.h"

#include "wheel_config.h"

static const char *TAG = "benchcam";

/* 控制器是独立纯 C 组件，帧率边界必须和配置校验保持一致。 */
_Static_assert(CAM_RATE_FPS_MIN == CFG_CAM_FPS_MIN, "cam_rate min fps mismatch");
_Static_assert(CAM_RATE_FPS_MAX == CFG_CAM_FPS_MAX, "cam_rate max fps mismatch");

#define CAM_IDLE_POLL_MS 20
#define CAM_PREVIEW_HOLD_MS 30000
#define CAM_STREAM_GRACE_MS 30000
#define CAM_FAIL_LIMIT 3
#define CAM_FPS_CALIB_FRAMES 7
#define CAM_DEADLINE_NONE UINT32_MAX

typedef enum {
    CAM_OFF = 0,
    CAM_IDLE,
    CAM_INIT,
    CAM_ON,
    CAM_FAIL,
} cam_state_t;

typedef enum {
    ACT_NONE = 0,
    ACT_INIT,
    ACT_DEINIT,
    ACT_SET_FPS,
    ACT_GRAB,
} cam_action_t;

static cfg_params_t *s_cfg;
static SemaphoreHandle_t s_cfg_mutex;
static SemaphoreHandle_t s_lock;

/* 以下状态都归 s_lock 保护（相机驱动句柄本身只归 cam_publish_task）。 */
static cam_state_t s_state = CAM_IDLE;
static uint32_t s_applied_policy = CFG_CAM_POLICY_ON_DEMAND;
static uint32_t s_applied_res = CFG_CAM_RES_QVGA;
static uint32_t s_applied_quality = 12;
static uint32_t s_cached_overlay = CFG_CAM_OVERLAY_ALL;
static cam_rate_t s_rate;

static bool s_preview_active;
static bool s_stream_active;
static bool s_stop_stream;
static bool s_init_requested;
static bool s_reinit_requested;
static uint32_t s_preview_deadline = CAM_DEADLINE_NONE;
static uint32_t s_grace_deadline = CAM_DEADLINE_NONE;

static uint8_t *s_pub;
static size_t s_pub_cap;
static size_t s_frame_len;
static uint32_t s_frame_gen;
static uint32_t s_last_frame_ms;
static float s_fps;
static uint32_t s_fps_window_ms;
static uint32_t s_fps_frames;
static uint32_t s_fail_count;
static uint32_t s_big_frame_drops; /* 帧大于发布缓冲：限频打印，别刷日志 */

/* 传感器帧率控制。基准帧率在初始化后实测，之后按目标/基准换算寄存器；
 * 传感器不支持时退回"按目标节奏取帧"，至少不让 DMA/拷贝继续全速跑。 */
static uint32_t s_base_fps;
static uint32_t s_sensor_fps_applied;
static bool s_fps_calibrated;
static bool s_sensor_fps_ok;
static bool s_calib_started;
static uint32_t s_calib_frames;
static uint32_t s_calib_start_ms;
static uint32_t s_next_grab_ms;

static uint32_t now_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static void lock(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
}

static void unlock(void)
{
    xSemaphoreGive(s_lock);
}

static bool deadline_due(uint32_t now, uint32_t deadline)
{
    return deadline != CAM_DEADLINE_NONE && (int32_t)(now - deadline) >= 0;
}

static const char *state_str(cam_state_t st)
{
    switch (st) {
    case CAM_OFF:
        return "off";
    case CAM_IDLE:
        return "idle";
    case CAM_INIT:
        return "init";
    case CAM_ON:
        return "on";
    case CAM_FAIL:
    default:
        return "fail";
    }
}

static const char *policy_str(uint32_t policy)
{
    switch (policy) {
    case CFG_CAM_POLICY_OFF:
        return "off";
    case CFG_CAM_POLICY_ALWAYS:
        return "always";
    case CFG_CAM_POLICY_ON_DEMAND:
    default:
        return "on_demand";
    }
}

static const char *res_str(uint32_t res)
{
    switch (res) {
    case CFG_CAM_RES_VGA:
        return "vga";
    case CFG_CAM_RES_SVGA:
        return "svga";
    case CFG_CAM_RES_QVGA:
    default:
        return "qvga";
    }
}

/* 读配置只拷 cam 相关字段：相机任务不应该因为别的参数变化去碰 NVS/业务结构。 */
static bool read_cam_cfg(uint32_t *policy, uint32_t *res, uint32_t *quality,
                         uint32_t *fps_cap, uint32_t *overlay)
{
    if (s_cfg == NULL || s_cfg_mutex == NULL) {
        return false;
    }
    if (xSemaphoreTake(s_cfg_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }
    *policy = s_cfg->cam_policy;
    *res = s_cfg->cam_res;
    *quality = s_cfg->cam_quality;
    *fps_cap = s_cfg->cam_fps_cap;
    *overlay = s_cfg->overlay_mask;
    xSemaphoreGive(s_cfg_mutex);
    return true;
}

/* 调用时 s_lock 已持有。 */
static void free_pub_locked(void)
{
    if (s_pub != NULL) {
        heap_caps_free(s_pub);
        s_pub = NULL;
    }
    s_pub_cap = 0;
    s_frame_len = 0;
    s_frame_gen++;
    s_last_frame_ms = 0;
    s_fps = 0.0f;
    s_fps_window_ms = 0;
    s_fps_frames = 0;
}

/* 调用时 s_lock 已持有；只负责清运行时缓冲和状态，不碰相机句柄。 */
static void set_idle_locked(cam_state_t target)
{
    free_pub_locked();
    s_state = target;
    s_fail_count = 0;
    s_reinit_requested = false;
    s_preview_deadline = CAM_DEADLINE_NONE;
    s_grace_deadline = CAM_DEADLINE_NONE;
    if (target == CAM_OFF) {
        s_init_requested = false;
        s_stop_stream = true;
    }
}

static bool alloc_pub_locked(uint32_t res)
{
    size_t cap = camera_drv_max_jpeg(res);
    uint8_t *p = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == NULL) {
        ESP_LOGE(TAG, "PSRAM publish buffer alloc failed (%u B)", (unsigned)cap);
        return false;
    }
    s_pub = p;
    s_pub_cap = cap;
    s_frame_len = 0;
    s_frame_gen++;
    return true;
}

static void publish_frame(const camera_drv_frame_t *frame, uint32_t now)
{
    lock();
    /* 能拿到帧就说明传感器还在出数据，失败计数清零；帧太大只是发布缓冲
     * 不够，不该和"取帧超时"混成同一个故障。 */
    s_fail_count = 0;
    if (s_pub != NULL && frame->len <= s_pub_cap) {
        memcpy(s_pub, frame->data, frame->len);
        s_frame_len = frame->len;
        s_frame_gen++;
        s_last_frame_ms = now;

        if (s_fps_window_ms == 0) {
            s_fps_window_ms = now;
            s_fps_frames = 0;
        }
        s_fps_frames++;
        uint32_t elapsed = now - s_fps_window_ms;
        if (elapsed >= 1000) {
            s_fps = (float)s_fps_frames * 1000.0f / (float)elapsed;
            s_fps_window_ms = now;
            s_fps_frames = 0;
        }
    } else if (s_pub != NULL && frame->len > s_pub_cap) {
        if ((s_big_frame_drops++ % 60u) == 0) {
            ESP_LOGW(TAG, "frame %u B exceeds publish buffer %u B, dropped",
                     (unsigned)frame->len, (unsigned)s_pub_cap);
        }
    }
    unlock();
}

/* 丢掉 init 后第一帧，再用随后 7 帧测传感器默认帧率，然后改寄存器。放在相机任务里做，
 * 不额外增加 init 阻塞；校准期间先按原速出几帧，随后落到目标帧率。 */
static void calibrate_sensor_fps(uint32_t grab_ms)
{
    if (s_fps_calibrated || !s_sensor_fps_ok) {
        return;
    }
    /* 丢掉 init 后队列里的第一帧，避免陈旧帧把基准测高。 */
    if (!s_calib_started) {
        s_calib_started = true;
        return;
    }
    if (s_calib_frames == 0) {
        s_calib_start_ms = grab_ms;
    }
    s_calib_frames++;
    if (s_calib_frames < CAM_FPS_CALIB_FRAMES) {
        return;
    }

    uint32_t elapsed = grab_ms - s_calib_start_ms;
    if (elapsed == 0) {
        s_calib_frames = 0; /* 时间戳还没走，下一帧重新量 */
        return;
    }
    uint32_t base = (uint32_t)(s_calib_frames - 1) * 1000u / elapsed;
    if (base < 1) {
        base = 1;
    }
    if (base > 60) {
        base = 60;
    }
    s_base_fps = base;
    s_fps_calibrated = true;

    uint32_t target;
    lock();
    target = cam_rate_effective(&s_rate);
    unlock();

    esp_err_t e = camera_drv_set_fps(s_base_fps, target);
    if (e == ESP_OK) {
        s_sensor_fps_applied = target;
        ESP_LOGI(TAG, "sensor fps calibrated: base=%lu, target=%lu",
                 (unsigned long)s_base_fps, (unsigned long)target);
    } else {
        s_sensor_fps_ok = false;
        ESP_LOGW(TAG, "sensor fps set failed (%d), fall back to grab pacing", e);
    }
}

/* 传感器不支持控速时的兜底：按有效帧率决定下一次取帧时间。 */
static void pace_next_grab(void)
{
    if (s_sensor_fps_ok) {
        return;
    }
    uint32_t eff;
    lock();
    eff = cam_rate_effective(&s_rate);
    unlock();
    if (eff == 0) {
        eff = 1;
    }
    uint32_t interval = 1000u / eff;
    if (interval == 0) {
        interval = 1;
    }
    s_next_grab_ms = now_ms() + interval;
}

static void cam_task(void *arg)
{
    (void)arg;
    camera_drv_frame_t frame;

    for (;;) {
        uint32_t now = now_ms();
        uint32_t cfg_policy, cfg_res, cfg_quality, cfg_fps_cap, cfg_overlay;
        bool have_cfg = read_cam_cfg(&cfg_policy, &cfg_res, &cfg_quality,
                                     &cfg_fps_cap, &cfg_overlay);

        cam_action_t action = ACT_NONE;
        bool idle_to_off = false;
        uint32_t init_res = 0, init_quality = 0;
        uint32_t fps_to_apply = 0;

        lock();

        if (have_cfg) {
            s_cached_overlay = cfg_overlay;
            if (cfg_fps_cap != cam_rate_cap(&s_rate)) {
                cam_rate_set_cap(&s_rate, cfg_fps_cap);
                ESP_LOGI(TAG, "fps cap -> %lu", (unsigned long)cam_rate_cap(&s_rate));
            }

            if (cfg_policy != s_applied_policy) {
                ESP_LOGI(TAG, "policy %s -> %s", policy_str(s_applied_policy),
                         policy_str(cfg_policy));
                s_applied_policy = cfg_policy;
                if (cfg_policy == CFG_CAM_POLICY_OFF) {
                    s_stop_stream = true;
                    action = ACT_DEINIT;
                    idle_to_off = true;
                } else if (cfg_policy == CFG_CAM_POLICY_ALWAYS) {
                    s_preview_deadline = CAM_DEADLINE_NONE;
                    s_grace_deadline = CAM_DEADLINE_NONE;
                    cam_rate_set_cap(&s_rate, cam_rate_cap(&s_rate));
                    if (s_state != CAM_ON && s_state != CAM_INIT) {
                        s_init_requested = true;
                    }
                } else { /* on_demand */
                    if (s_state == CAM_OFF) {
                        s_state = CAM_IDLE;
                    }
                    if (s_state == CAM_ON) {
                        if (s_stream_active) {
                            if (!s_preview_active) {
                                s_preview_deadline = now + CAM_PREVIEW_HOLD_MS;
                            }
                        } else {
                            s_grace_deadline = now + CAM_STREAM_GRACE_MS;
                        }
                    }
                }
            }

            if (cfg_res != s_applied_res || cfg_quality != s_applied_quality) {
                s_applied_res = cfg_res;
                s_applied_quality = cfg_quality;
                /* 有流或 always：立即重 init，旧流由流任务结束；没有流且不在
                 * 采集中：先回 idle，等下一次请求按新参数 init。 */
                if (s_state == CAM_ON || s_state == CAM_INIT) {
                    if (s_stream_active || s_applied_policy == CFG_CAM_POLICY_ALWAYS) {
                        s_reinit_requested = true;
                        s_stop_stream = true;
                    } else if (action == ACT_NONE) {
                        action = ACT_DEINIT;
                        idle_to_off = false;
                    }
                }
            }
        }

        if (action == ACT_NONE && s_applied_policy == CFG_CAM_POLICY_OFF &&
            s_state != CAM_OFF) {
            action = ACT_DEINIT;
            idle_to_off = true;
        }

        if (action == ACT_NONE && s_reinit_requested &&
            s_applied_policy != CFG_CAM_POLICY_OFF && s_state != CAM_OFF) {
            s_reinit_requested = false;
            s_stop_stream = true;
            s_init_requested = true;
            action = ACT_DEINIT;
            idle_to_off = false;
        }

        if (action == ACT_NONE && s_init_requested &&
            s_applied_policy != CFG_CAM_POLICY_OFF &&
            (s_state == CAM_IDLE || s_state == CAM_FAIL || s_state == CAM_OFF)) {
            s_init_requested = false;
            s_state = CAM_INIT;
            init_res = s_applied_res;
            init_quality = s_applied_quality;
            action = ACT_INIT;
        }

        if (action == ACT_NONE && s_state == CAM_ON &&
            s_applied_policy == CFG_CAM_POLICY_ON_DEMAND) {
            if (deadline_due(now, s_preview_deadline)) {
                ESP_LOGI(TAG, "preview hold expired, stop stream and camera");
                s_preview_deadline = CAM_DEADLINE_NONE;
                s_grace_deadline = CAM_DEADLINE_NONE;
                s_stop_stream = true;
                action = ACT_DEINIT;
                idle_to_off = false;
            } else if (deadline_due(now, s_grace_deadline) && !s_stream_active) {
                ESP_LOGI(TAG, "stream grace expired, stop camera");
                s_grace_deadline = CAM_DEADLINE_NONE;
                action = ACT_DEINIT;
                idle_to_off = false;
            }
        }

        if (action == ACT_NONE && s_state == CAM_ON && s_fps_calibrated &&
            s_sensor_fps_ok) {
            uint32_t eff = cam_rate_effective(&s_rate);
            if (eff != s_sensor_fps_applied) {
                fps_to_apply = eff;
                s_sensor_fps_applied = eff; /* 失败时下面的分支会关掉传感器控制 */
                action = ACT_SET_FPS;
            }
        }

        if (action == ACT_NONE && s_state == CAM_ON) {
            /* 传感器不支持寄存器控速时，用取帧节奏兜底：不取帧就不会把
             * JPEG 拷进 DMA/发布缓冲。支持控速时交给传感器自己降。 */
            if (!s_sensor_fps_ok && s_next_grab_ms != 0 &&
                (int32_t)(now - s_next_grab_ms) < 0) {
                /* 还没到下一拍，走底部的空闲延时 */
            } else {
                action = ACT_GRAB;
            }
        }

        unlock();

        switch (action) {
        case ACT_DEINIT:
            (void)camera_drv_deinit();
            s_fps_calibrated = false;
            s_sensor_fps_ok = false;
            s_sensor_fps_applied = 0;
            s_base_fps = 0;
            s_calib_started = false;
            s_calib_frames = 0;
            s_next_grab_ms = 0;
            lock();
            set_idle_locked(idle_to_off ? CAM_OFF : CAM_IDLE);
            cam_rate_stream_stop(&s_rate, now_ms());
            /* 新的相机会话（下一次 init）从用户上限重新起步。 */
            cam_rate_set_cap(&s_rate, cam_rate_cap(&s_rate));
            if (idle_to_off) {
                ESP_LOGI(TAG, "camera off");
            } else {
                ESP_LOGI(TAG, "camera idle");
            }
            unlock();
            break;

        case ACT_SET_FPS: {
            esp_err_t e = camera_drv_set_fps(s_base_fps, fps_to_apply);
            if (e != ESP_OK) {
                ESP_LOGW(TAG, "sensor fps %lu failed (%d), fall back to grab pacing",
                         (unsigned long)fps_to_apply, e);
                s_sensor_fps_ok = false;
                s_next_grab_ms = 0;
            } else {
                ESP_LOGI(TAG, "sensor fps -> %lu (base %lu)",
                         (unsigned long)fps_to_apply, (unsigned long)s_base_fps);
            }
            break;
        }

        case ACT_INIT: {
            bool ok = false;
            camera_drv_cfg_t dcfg = {
                .res = init_res,
                .quality = init_quality,
            };

            lock();
            bool alloc_ok = alloc_pub_locked(init_res);
            unlock();

            if (alloc_ok) {
                esp_err_t e = camera_drv_init(&dcfg);
                if (e == ESP_OK) {
                    ok = true;
                } else {
                    ESP_LOGE(TAG, "camera init failed: %d", e);
                }
            }

            lock();
            if (ok) {
                uint32_t ready_ms = now_ms();
                s_state = CAM_ON;
                s_fail_count = 0;
                s_fps_window_ms = ready_ms;
                s_fps_frames = 0;
                s_last_frame_ms = 0;
                s_init_requested = false; /* 本次 init 已满足发起它的请求 */
                s_fps_calibrated = false;
                s_calib_started = false;
                s_calib_frames = 0;
                s_calib_start_ms = 0;
                s_sensor_fps_applied = 0;
                s_next_grab_ms = 0;
                s_base_fps = 0;
                s_sensor_fps_ok = camera_drv_fps_control_supported();
                if (!s_sensor_fps_ok) {
                    ESP_LOGW(TAG, "sensor fps control unsupported, use grab pacing");
                }
                if (s_applied_policy == CFG_CAM_POLICY_ON_DEMAND) {
                    if (!s_preview_active && s_preview_deadline == CAM_DEADLINE_NONE) {
                        s_preview_deadline = ready_ms + CAM_PREVIEW_HOLD_MS;
                    } else if (s_preview_active && !s_stream_active &&
                               s_grace_deadline == CAM_DEADLINE_NONE) {
                        s_grace_deadline = ready_ms + CAM_STREAM_GRACE_MS;
                    }
                } else {
                    s_preview_deadline = CAM_DEADLINE_NONE;
                    s_grace_deadline = CAM_DEADLINE_NONE;
                }
                ESP_LOGI(TAG, "camera on: %s, q=%lu, target=%lufps",
                         res_str(init_res), (unsigned long)init_quality,
                         (unsigned long)cam_rate_effective(&s_rate));
            } else {
                free_pub_locked();
                s_state = CAM_FAIL;
                s_fail_count = 0;
                s_stop_stream = true;
                s_init_requested = false;
                s_preview_deadline = CAM_DEADLINE_NONE;
                s_grace_deadline = CAM_DEADLINE_NONE;
                s_fps_calibrated = false;
                s_sensor_fps_ok = false;
                s_sensor_fps_applied = 0;
                s_base_fps = 0;
                s_calib_started = false;
                s_calib_frames = 0;
                s_next_grab_ms = 0;
                ESP_LOGE(TAG, "camera state=fail; next stream/snapshot retries");
            }
            unlock();
            break;
        }

        case ACT_GRAB: {
            esp_err_t e = camera_drv_grab(&frame);
            if (e == ESP_OK) {
                /* 取帧会阻塞，时间戳按拿到帧的时刻算，不能用循环开头的 now。 */
                uint32_t grab_ms = now_ms();
                publish_frame(&frame, grab_ms);
                camera_drv_release(&frame);
                calibrate_sensor_fps(grab_ms);
                pace_next_grab();
            } else {
                bool fail_now = false;
                lock();
                s_fail_count++;
                if (s_fail_count >= CAM_FAIL_LIMIT) {
                    fail_now = true;
                }
                unlock();
                if (fail_now) {
                    ESP_LOGE(TAG, "frame grab failed %lu times, camera state=fail",
                             (unsigned long)CAM_FAIL_LIMIT);
                    (void)camera_drv_deinit();
                    lock();
                    s_state = CAM_FAIL;
                    s_stop_stream = true;
                    s_init_requested = false;
                    free_pub_locked();
                    s_preview_deadline = CAM_DEADLINE_NONE;
                    s_grace_deadline = CAM_DEADLINE_NONE;
                    cam_rate_stream_stop(&s_rate, now_ms());
                    unlock();
                    s_fps_calibrated = false;
                    s_sensor_fps_ok = false;
                    s_sensor_fps_applied = 0;
                    s_base_fps = 0;
                    s_calib_started = false;
                    s_calib_frames = 0;
                    s_next_grab_ms = 0;
                }
            }
            break;
        }

        case ACT_NONE:
        default:
            vTaskDelay(pdMS_TO_TICKS(CAM_IDLE_POLL_MS));
            break;
        }
    }
}

esp_err_t bench_cam_start(cfg_params_t *cfg, SemaphoreHandle_t cfg_mutex)
{
    if (cfg == NULL || cfg_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = cfg;
    s_cfg_mutex = cfg_mutex;
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_applied_policy = cfg->cam_policy;
    s_applied_res = cfg->cam_res;
    s_applied_quality = cfg->cam_quality;
    cam_rate_init(&s_rate, cfg->cam_fps_cap);
    s_cached_overlay = cfg->overlay_mask;
    s_state = (cfg->cam_policy == CFG_CAM_POLICY_OFF) ? CAM_OFF : CAM_IDLE;
    s_init_requested = (cfg->cam_policy == CFG_CAM_POLICY_ALWAYS);

    BaseType_t ok = xTaskCreatePinnedToCore(cam_task, "cam_publish", WHEEL_CAM_PUB_STACK,
                                            NULL, WHEEL_CAM_PUB_PRIO, NULL,
                                            WHEEL_CAM_PUB_CORE);
    if (ok != pdPASS) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "camera service started: policy=%s res=%s q=%lu fps_cap=%lu",
             policy_str(s_applied_policy), res_str(s_applied_res),
             (unsigned long)s_applied_quality, (unsigned long)cam_rate_cap(&s_rate));
    return ESP_OK;
}

bench_cam_stream_result_t bench_cam_stream_acquire(void)
{
    if (s_lock == NULL) {
        return BENCH_CAM_STREAM_BUSY;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        return BENCH_CAM_STREAM_BUSY;
    }

    bench_cam_stream_result_t result = BENCH_CAM_STREAM_OK;
    if (s_applied_policy == CFG_CAM_POLICY_OFF) {
        result = BENCH_CAM_STREAM_OFF;
    } else if (s_stream_active) {
        result = BENCH_CAM_STREAM_BUSY;
    } else {
        s_stream_active = true;
        s_stop_stream = false;
        s_preview_active = true;
        s_preview_deadline = CAM_DEADLINE_NONE;
        s_grace_deadline = CAM_DEADLINE_NONE;
        cam_rate_stream_start(&s_rate, now_ms());
        if (s_state != CAM_ON) {
            s_init_requested = true;
        }
    }
    xSemaphoreGive(s_lock);
    return result;
}

void bench_cam_stream_release(void)
{
    if (s_lock == NULL) {
        return;
    }
    lock();
    if (!s_stream_active) {
        unlock();
        return;
    }
    s_stream_active = false;
    /* 断流后保留降速结果：网络可能还没恢复，下一次请求先按低帧率起步，
     * 由 cam_rate 的 recovery 慢慢升回去；相机 deinit 时才恢复用户上限。
     * always 没有流时不存在网络压力，直接回到用户上限。 */
    cam_rate_stream_stop(&s_rate, now_ms());
    if (s_applied_policy == CFG_CAM_POLICY_ALWAYS) {
        cam_rate_set_cap(&s_rate, cam_rate_cap(&s_rate));
    }
    if (s_applied_policy == CFG_CAM_POLICY_ON_DEMAND && s_state == CAM_ON) {
        s_grace_deadline = now_ms() + CAM_STREAM_GRACE_MS;
    }
    unlock();
}

void bench_cam_report_send_time(uint32_t send_ms)
{
    if (s_lock == NULL) {
        return;
    }
    lock();
    uint32_t before = cam_rate_effective(&s_rate);
    uint32_t after = cam_rate_report_send(&s_rate, send_ms, now_ms());
    if (after < before) {
        ESP_LOGW(TAG, "network throttle: %lu -> %lu fps (send %lu ms)",
                 (unsigned long)before, (unsigned long)after, (unsigned long)send_ms);
    } else if (after > before) {
        ESP_LOGI(TAG, "network recovered: %lu -> %lu fps",
                 (unsigned long)before, (unsigned long)after);
    }
    unlock();
}

bool bench_cam_stream_should_stop(void)
{
    if (s_lock == NULL) {
        return true;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }
    bool stop = s_stop_stream;
    xSemaphoreGive(s_lock);
    return stop;
}

esp_err_t bench_cam_copy_latest(uint8_t *dst, size_t cap, size_t *len, uint32_t *gen)
{
    if (dst == NULL || len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t e = ESP_OK;
    if (s_frame_len == 0 || s_pub == NULL ||
        (gen != NULL && *gen == s_frame_gen)) {
        /* gen 相同说明没有新帧：不要每次轮询都白拷一份几十 KB 的 JPEG。 */
        e = ESP_ERR_NOT_FOUND;
    } else if (s_frame_len > cap) {
        e = ESP_ERR_INVALID_SIZE;
    } else {
        memcpy(dst, s_pub, s_frame_len);
        *len = s_frame_len;
        if (gen != NULL) {
            *gen = s_frame_gen;
        }
    }
    xSemaphoreGive(s_lock);
    return e;
}

size_t bench_cam_max_frame(void)
{
    if (s_lock == NULL) {
        return camera_drv_max_jpeg(CFG_CAM_RES_SVGA);
    }
    lock();
    size_t cap = s_pub_cap != 0 ? s_pub_cap : camera_drv_max_jpeg(s_applied_res);
    unlock();
    return cap;
}

bench_cam_snap_result_t bench_cam_snapshot(uint8_t **out, size_t *len)
{
    if (out == NULL || len == NULL || s_lock == NULL) {
        return BENCH_CAM_SNAP_NO_MEM;
    }
    *out = NULL;
    *len = 0;

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        return BENCH_CAM_SNAP_WARMING;
    }

    if (s_applied_policy == CFG_CAM_POLICY_OFF) {
        xSemaphoreGive(s_lock);
        return BENCH_CAM_SNAP_OFF;
    }

    if (s_state != CAM_ON) {
        bench_cam_snap_result_t r = (s_state == CAM_FAIL) ? BENCH_CAM_SNAP_FAIL
                                                          : BENCH_CAM_SNAP_WARMING;
        s_init_requested = true;
        xSemaphoreGive(s_lock);
        return r;
    }

    if (s_frame_len == 0 || s_pub == NULL) {
        /* 已经 on 但第一帧还没出来：也算快照消费者在场，起 30 秒宽限，
         * 免得页面刚请求完就关掉、相机没人管一直开着。 */
        if (s_applied_policy == CFG_CAM_POLICY_ON_DEMAND && !s_stream_active) {
            s_grace_deadline = now_ms() + CAM_STREAM_GRACE_MS;
        }
        xSemaphoreGive(s_lock);
        return BENCH_CAM_SNAP_WARMING;
    }

    /* 快照罕见，拷贝缓冲按当前帧长分配；成功即证明消费者在场，起 30 秒宽限。 */
    uint8_t *p = heap_caps_malloc(s_frame_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == NULL) {
        xSemaphoreGive(s_lock);
        return BENCH_CAM_SNAP_NO_MEM;
    }
    memcpy(p, s_pub, s_frame_len);
    *len = s_frame_len;
    if (s_applied_policy == CFG_CAM_POLICY_ON_DEMAND && !s_stream_active) {
        s_grace_deadline = now_ms() + CAM_STREAM_GRACE_MS;
    }
    xSemaphoreGive(s_lock);
    *out = p;
    return BENCH_CAM_SNAP_OK;
}

void bench_cam_set_preview(bool active)
{
    if (s_lock == NULL) {
        return;
    }
    lock();
    if (s_preview_active == active) {
        /* 重复的 active=false 不重置计时；active=true 的重复也无事可做。 */
        unlock();
        return;
    }
    s_preview_active = active;
    if (active) {
        s_preview_deadline = CAM_DEADLINE_NONE;
        s_grace_deadline = CAM_DEADLINE_NONE;
    } else if (s_applied_policy == CFG_CAM_POLICY_ON_DEMAND &&
               (s_state == CAM_ON || s_state == CAM_INIT)) {
        s_preview_deadline = now_ms() + CAM_PREVIEW_HOLD_MS;
    }
    unlock();
}

void bench_cam_on_ws_disconnect(void)
{
    bench_cam_set_preview(false);
}

void bench_cam_get_status(telem_cam_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (s_lock == NULL) {
        out->policy = "off";
        out->state = "off";
        out->res = "qvga";
        return;
    }

    lock();
    uint32_t now = now_ms();
    out->policy = policy_str(s_applied_policy);
    out->state = state_str(s_state);
    out->preview_active = s_preview_active;
    out->streaming = s_stream_active;
    out->clients = s_stream_active ? 1 : 0;
    out->res = res_str(s_applied_res);
    out->quality = (uint8_t)s_applied_quality;
    out->fps_cap = (uint8_t)cam_rate_cap(&s_rate);
    out->fps_eff = (uint8_t)cam_rate_effective(&s_rate);
    out->overlay_mask = s_cached_overlay;
    out->bytes = (uint32_t)s_frame_len;

    if (s_last_frame_ms != 0 && now - s_last_frame_ms > 2000) {
        out->fps = 0.0f;
    } else {
        out->fps = s_fps;
    }

    if (s_applied_policy == CFG_CAM_POLICY_ON_DEMAND &&
        (s_state == CAM_ON || s_state == CAM_INIT)) {
        bool have = false;
        uint32_t best = 0;
        uint32_t deadlines[2] = {s_preview_deadline, s_grace_deadline};
        for (int i = 0; i < 2; i++) {
            if (deadlines[i] == CAM_DEADLINE_NONE) {
                continue;
            }
            int32_t rem = (int32_t)(deadlines[i] - now);
            if (rem < 0) {
                rem = 0;
            }
            uint32_t sec = ((uint32_t)rem + 999u) / 1000u;
            if (!have || sec < best) {
                best = sec;
                have = true;
            }
        }
        out->stop_in_s = have ? best : 0;
    }
    unlock();
}

bench_cam_state_t bench_cam_state(void)
{
    if (s_lock == NULL) {
        return BENCH_CAM_STATE_OFF;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return BENCH_CAM_STATE_INIT;
    }
    cam_state_t st = s_state;
    xSemaphoreGive(s_lock);
    switch (st) {
    case CAM_OFF:
        return BENCH_CAM_STATE_OFF;
    case CAM_IDLE:
        return BENCH_CAM_STATE_IDLE;
    case CAM_ON:
        return BENCH_CAM_STATE_ON;
    case CAM_FAIL:
        return BENCH_CAM_STATE_FAIL;
    case CAM_INIT:
    default:
        return BENCH_CAM_STATE_INIT;
    }
}
