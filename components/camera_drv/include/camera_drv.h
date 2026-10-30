/*
 * camera_drv：板载 DVP 摄像头的 esp32-camera 薄封装（依赖 IDF，不做宿主机测试）。
 *
 * 上层只看见"初始化 / 取一帧 / 归还 / 反初始化"四件事，esp_camera 的类型和
 * 引脚细节留在本组件里。帧缓冲的所有权规则：grab 成功后必须 release，
 * 中间不要把 data 指针传到归还之后——发布缓冲的拷贝由上层负责。
 */
#ifndef CAMERA_DRV_H
#define CAMERA_DRV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 与 cfg_params 的 CAM_RES_* 顺序一致，main 层直接强转使用。 */
typedef enum {
    CAMERA_DRV_RES_QVGA = 0,
    CAMERA_DRV_RES_VGA = 1,
    CAMERA_DRV_RES_SVGA = 2,
} camera_drv_res_t;

typedef struct {
    uint32_t res;     /* camera_drv_res_t */
    uint32_t quality; /* JPEG quality 4..63，值小画质高 */
} camera_drv_cfg_t;

/* handle 是 esp32-camera 的 camera_fb_t；上层不解释它，只原样传回 release。 */
typedef struct {
    void *handle;
    const uint8_t *data;
    size_t len;
    uint16_t width;
    uint16_t height;
} camera_drv_frame_t;

/* 初始化相机并应用分辨率/画质。重复调用会先反初始化，便于配置变更后重 init。
 * PSRAM 未开启或模组不在线时返回错误，调用方只记日志，不重启。 */
esp_err_t camera_drv_init(const camera_drv_cfg_t *cfg);

esp_err_t camera_drv_deinit(void);

/* 取一帧，阻塞到驱动自己的取帧超时（esp32-camera 约 4 秒）或拿到帧。
 * 成功返回 ESP_OK，out->handle 必须交给 camera_drv_release。 */
esp_err_t camera_drv_grab(camera_drv_frame_t *out);

/* 归还 grab 得到的帧。允许传 NULL 或空句柄，重复调用无副作用。 */
void camera_drv_release(camera_drv_frame_t *frame);

/* 当前分辨率下给 esp32-camera 的 JPEG 缓冲大小，也是发布/发送缓冲的上限。 */
size_t camera_drv_max_jpeg(uint32_t res);

uint16_t camera_drv_width(uint32_t res);
uint16_t camera_drv_height(uint32_t res);

bool camera_drv_is_inited(void);

/* 上电读到的传感器名（OV2640/OV5640/...），未初始化或未知时返回 "unknown"。 */
const char *camera_drv_sensor_name(void);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_DRV_H */
