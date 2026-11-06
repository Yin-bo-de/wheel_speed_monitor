/* telemetry 实现：见头文件注释。注册表为静态数组，启动期写、运行期只读。 */
#include <stdio.h>
#include <string.h>

#include "telemetry.h"

typedef struct {
    /* 快照 type 与函数指针，不再保存调用方 ops 指针：
     * 调用方在注册后修改自己的 ops 对象不应影响注册表，
     * 且栈/静态对象复用地址也不会造成"后来者篡改先注册者" */
    telem_type_t type;
    esp_err_t (*init)(void *ctx);
    esp_err_t (*sample)(void *ctx, uint32_t now_ms);
    esp_err_t (*render)(void *ctx, char *buf, size_t len, size_t *used);
    esp_err_t (*reset_baseline)(void *ctx);
    void *ctx;
    bool in_use;
} telem_slot_t;

static telem_slot_t g_slots[TELEM_MAX_COLLECTORS];
static uint8_t g_gpio_levels[4]; /* sys 块原始 IO 电平（由采样任务喂值） */
static uint32_t g_internal_heap; /* sys 块内部 RAM 余量（PSRAM 不在此列） */

/* 相机状态用内部定长数组存：调用方传的是指针，框架不能假设它一直有效。 */
typedef struct {
    char policy[16];
    char state[8];
    bool preview_active;
    bool streaming;
    uint8_t clients;
    uint32_t stop_in_s;
    char res[8];
    uint8_t quality;
    uint8_t fps_cap;
    uint8_t fps_eff;
    uint32_t overlay_mask;
    float fps;
    uint32_t bytes;
} cam_status_copy_t;

static cam_status_copy_t g_cam;

static void copy_str(char *dst, size_t cap, const char *src)
{
    if (src == NULL) {
        dst[0] = '\0';
        return;
    }
    snprintf(dst, cap, "%s", src);
}

esp_err_t telem_register(const telem_collector_ops_t *ops, void *ctx)
{
    if (ops == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < TELEM_MAX_COLLECTORS; i++) {
        if (g_slots[i].in_use && g_slots[i].type == ops->type) {
            return ESP_ERR_INVALID_ARG; /* type 唯一，拒绝重复注册 */
        }
    }
    for (int i = 0; i < TELEM_MAX_COLLECTORS; i++) {
        if (!g_slots[i].in_use) {
            g_slots[i].type = ops->type;
            g_slots[i].init = ops->init;
            g_slots[i].sample = ops->sample;
            g_slots[i].render = ops->render;
            g_slots[i].reset_baseline = ops->reset_baseline;
            g_slots[i].ctx = ctx;
            g_slots[i].in_use = true;
            if (ops->init != NULL) {
                return ops->init(ctx);
            }
            return ESP_OK;
        }
    }
    return ESP_ERR_NO_MEM;
}

esp_err_t telem_sample_all(uint32_t now_ms)
{
    esp_err_t first_err = ESP_OK;
    for (int i = 0; i < TELEM_MAX_COLLECTORS; i++) {
        if (!g_slots[i].in_use) {
            continue;
        }
        if (g_slots[i].sample == NULL) {
            continue;
        }
        esp_err_t e = g_slots[i].sample(g_slots[i].ctx, now_ms);
        /* 容错隔离：记录首个错误，继续遍历其余采集器 */
        if (e != ESP_OK && first_err == ESP_OK) {
            first_err = e;
        }
    }
    return first_err;
}

esp_err_t telem_render_frame(char *buf, size_t len, size_t *used,
                             uint32_t seq, uint32_t now_ms, uint32_t free_heap)
{
    if (buf == NULL || used == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *used = 0;

    size_t off = 0;

#define TELEM_SNPRINTF(...) do { \
        int n_ = snprintf((buf) + off, (len) - off, __VA_ARGS__); \
        if (n_ < 0 || (size_t)n_ >= (len) - off) { return ESP_ERR_INVALID_SIZE; } \
        off += (size_t)n_; \
    } while (0)

    TELEM_SNPRINTF("{\"type\":\"telemetry\",\"seq\":%lu,\"t\":%lu,",
                   (unsigned long)seq, (unsigned long)now_ms);

    TELEM_SNPRINTF("\"sys\":{\"gpio\":[%u,%u,%u,%u],\"free_heap\":%lu,"
                   "\"heap_int\":%lu,\"uptime_s\":%lu,",
                   g_gpio_levels[0], g_gpio_levels[1], g_gpio_levels[2],
                   g_gpio_levels[3], (unsigned long)free_heap,
                   (unsigned long)g_internal_heap, (unsigned long)(now_ms / 1000));

    TELEM_SNPRINTF("\"cam\":{\"policy\":\"%s\",\"state\":\"%s\",\"preview\":\"%s\","
                   "\"streaming\":%s,\"clients\":%u,\"stop_in_s\":%lu,"
                   "\"res\":\"%s\",\"quality\":%u,\"fps_cap\":%u,"
                   "\"fps_eff\":%u,\"overlay\":%lu,"
                   "\"fps\":%.1f,\"bytes\":%lu}},",
                   g_cam.policy, g_cam.state,
                   g_cam.preview_active ? "active" : "inactive",
                   g_cam.streaming ? "true" : "false", (unsigned)g_cam.clients,
                   (unsigned long)g_cam.stop_in_s, g_cam.res,
                   (unsigned)g_cam.quality, (unsigned)g_cam.fps_cap,
                   (unsigned)g_cam.fps_eff,
                   (unsigned long)g_cam.overlay_mask, (double)g_cam.fps,
                   (unsigned long)g_cam.bytes);

    TELEM_SNPRINTF("\"collectors\":[");

    bool first = true;
    for (int i = 0; i < TELEM_MAX_COLLECTORS; i++) {
        if (!g_slots[i].in_use || g_slots[i].render == NULL) {
            continue;
        }
        char block[1024]; /* 单个采集器块缓冲：轮速块随字段增长（rps 后约 541B），
                           与 telem_render_frame 的帧缓冲/采集器协议一起定 */

        size_t blk_used = 0;
        esp_err_t e = g_slots[i].render(g_slots[i].ctx, block, sizeof(block), &blk_used);
        if (e != ESP_OK) {
            continue; /* 渲染失败的采集器跳过，不吹整帧 */
        }
        if (!first) {
            buf[off++] = ',';
        }
        first = false;
        if (off + blk_used + 2 > len) {
            return ESP_ERR_INVALID_SIZE;
        }
        memcpy(buf + off, block, blk_used);
        off += blk_used;
    }
    if (off + 3 > len) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[off++] = ']';
    buf[off++] = '}';
    buf[off] = '\0';
    *used = off;
    return ESP_OK;
}

esp_err_t telem_reset_all_baselines(void)
{
    esp_err_t first_err = ESP_OK;
    for (int i = 0; i < TELEM_MAX_COLLECTORS; i++) {
        if (!g_slots[i].in_use || g_slots[i].reset_baseline == NULL) {
            continue;
        }
        esp_err_t e = g_slots[i].reset_baseline(g_slots[i].ctx);
        if (e != ESP_OK && first_err == ESP_OK) {
            first_err = e;
        }
    }
    return first_err;
}

void telem_set_gpio_levels(const uint8_t levels[4])
{
    memcpy(g_gpio_levels, levels, sizeof(g_gpio_levels));
}

void telem_set_internal_heap(uint32_t bytes)
{
    g_internal_heap = bytes;
}

void telem_set_cam_status(const telem_cam_status_t *status)
{
    if (status == NULL) {
        memset(&g_cam, 0, sizeof(g_cam));
        return;
    }
    copy_str(g_cam.policy, sizeof(g_cam.policy), status->policy);
    copy_str(g_cam.state, sizeof(g_cam.state), status->state);
    g_cam.preview_active = status->preview_active;
    g_cam.streaming = status->streaming;
    g_cam.clients = status->clients;
    g_cam.stop_in_s = status->stop_in_s;
    copy_str(g_cam.res, sizeof(g_cam.res), status->res);
    g_cam.quality = status->quality;
    g_cam.fps_cap = status->fps_cap;
    g_cam.fps_eff = status->fps_eff;
    g_cam.overlay_mask = status->overlay_mask;
    g_cam.fps = status->fps;
    g_cam.bytes = status->bytes;
}

void telem_reset_for_test(void)
{
    memset(g_slots, 0, sizeof(g_slots));
    memset(g_gpio_levels, 0, sizeof(g_gpio_levels));
    memset(&g_cam, 0, sizeof(g_cam));
    g_internal_heap = 0;
}
