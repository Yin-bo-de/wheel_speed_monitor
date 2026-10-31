/*
 * telemetry：车载数据采集框架，纯 C 无 IDF 依赖。
 * 每个采集器是一个自包含对象（内部状态+配置+数据源），实现统一的 ops 接口后
 * 注册进框架；框架负责采样泵、遥测帧拼装，却完全不知道各采集器的内部细节。
 * 框架外新增一种采集器（如 IMU）：在 telem_type_t 里加枚举 → 实现 ops →
 * main 注册 → 前端加渲染函数。注册表启动期完成，运行期只读，无锁。
 *
 * ops 的 sample 可以为 NULL：状态由别处（如采样任务的控制路径）推进的采集器
 * 不需要框架再推一次，只要有人填好快照、render 能读到即可。
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 采集器类型：真实类型顺序排列，末尾 AUTO_COUNT 是哨兵。
 * TELEM_TYPE_MOCK_BASE 起预留为宿主机测试的 mock 类型（固件不注册）。 */
typedef enum {
    TELEM_TYPE_WHEEL_SPEED = 0,
    TELEM_TYPE_SERVO = 1,        /* 差速舵机状态（策略阶段 + 指令 vs 到达） */
    TELEM_TYPE_RC = 2,           /* 遥控三路通道 + 每轴目标来源 */
    TELEM_TYPE_MOCK_BASE = 100,  /* 宿主机测试用，固件构建不出现 */
    TELEM_TYPE_COUNT,            /* 非哨兵：不在前端渲染，只占位 */
} telem_type_t;

#define TELEM_MAX_COLLECTORS 8

/* sys.cam 的只读状态快照。相机不走采集器框架（30KB 级二进制帧不适合
 * 1024B 的渲染块），但页面要显示策略、状态和实际参数，由 main 每帧喂进来。
 * 字符串用 const char*，框架内部另存一份，调用方不必保证生命周期。 */
typedef struct {
    const char *policy;     /* off / on_demand / always */
    const char *state;      /* off / idle / init / on / fail */
    bool preview_active;    /* 预览 tab 是否可见 */
    bool streaming;         /* 当前是否有 MJPEG 流 */
    uint8_t clients;        /* 流客户端数（单流限制下只可能是 0/1） */
    uint32_t stop_in_s;     /* 距离停采集还剩几秒，0 = 没有计时 */
    const char *res;        /* qvga / vga / svga */
    uint8_t quality;        /* JPEG quality 4..63 */
    uint8_t fps_cap;        /* 用户配置的采集/发送帧率上限 */
    uint8_t fps_eff;        /* 网络自适应后的当前有效帧率 */
    uint32_t overlay_mask;  /* 叠加位图 */
    float fps;              /* 实测发布帧率 */
    uint32_t bytes;         /* 最近一帧大小 */
} telem_cam_status_t;

typedef struct {
    telem_type_t type;  /* 唯一标识，也是前端渲染分发的键 */
    esp_err_t (*init)(void *ctx);
    /* 推进内部状态：读硬件 + 计算链。每采样窗口调用一次，now_ms 单调递增。 */
    esp_err_t (*sample)(void *ctx, uint32_t now_ms);
    /* 把本类型最新状态渲染为 JSON 对象文本（不含外层包裹）。
     * used 返回写入字节数；buffer 不足返回 ESP_ERR_INVALID_SIZE。 */
    esp_err_t (*render)(void *ctx, char *buf, size_t len, size_t *used);
    /* 换源/启停/配置变更后重置基准，避免切换瞬间的差值毛刺。 */
    esp_err_t (*reset_baseline)(void *ctx);
} telem_collector_ops_t;

/* 注册采集器。type 重复 → ESP_ERR_INVALID_ARG；注册表满 → ESP_ERR_NO_MEM。 */
esp_err_t telem_register(const telem_collector_ops_t *ops, void *ctx);

/* 依次调用注册表里所有采集器的 sample()；返回第一个非 OK 错误（容错隔离：
 * 单个采集器出错不影响其余遍历）。 */
esp_err_t telem_sample_all(uint32_t now_ms);

/* 渲染完整遥测帧：{"type":"telemetry","seq":..,"t":..,"sys":{..},"collectors":[...]}。
 * seq 由调用方传入（保证帧间序号递增）；free_heap 为 sys 块调试信息（宿主机
 * 测试传 0），uptime_s 由 now_ms 换算。 */
esp_err_t telem_render_frame(char *buf, size_t len, size_t *used,
                             uint32_t seq, uint32_t now_ms, uint32_t free_heap);

/* 通知所有采集器重置基准（换源/启停/配置变更后由上层调用）。 */
esp_err_t telem_reset_all_baselines(void);

/* 喂 sys 块原始 IO 电平（4 路 GPIO，采样任务每窗口调用）。 */
void telem_set_gpio_levels(const uint8_t levels[4]);

/* 喂 sys 块内部 RAM 余量。开 PSRAM 后 esp_get_free_heap_size() 会把外部
 * PSRAM 算进去，页面拿它看不出内部堆压力，所以单独报这一项。 */
void telem_set_internal_heap(uint32_t bytes);

/* 喂 sys.cam 快照（ws_push_task 每帧调用，和 render 同一任务，无需加锁）。 */
void telem_set_cam_status(const telem_cam_status_t *status);

/* 宿主机测试专用：清空注册表。固件代码不得调用。 */
void telem_reset_for_test(void);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
