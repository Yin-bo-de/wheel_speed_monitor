/* feed_ws：WebSocket 遥测接收（IDF 依赖，不宿主机测）。
 * 连车端（或模拟脚本）的 /ws 端点收下行遥测帧，解析成 telem_frame_view_t。
 * 车端零改动：C3 只是又一个 WS 客户端，与浏览器并存。
 * 二期 IMU 上行帧从这里扩展（同一条 WS 连接反向发送），一期只收不发。 */
#ifndef FEED_WS_H
#define FEED_WS_H

#include <stdbool.h>
#include <stdint.h>

#include "dev_cfg.h"
#include "esp_err.h"
#include "telem_frame.h"

typedef struct {
    bool ws_connected;
    uint32_t last_frame_ms; /* 最后一帧到达时刻（tick ms），0 = 还没收到过 */
    telem_frame_view_t view;
} feed_state_t;

/* 启动接收任务：等拿到 IP 后建 WS 连接，断线自动重连（重连退避自愈，
 * 车端重启/OTA 几十秒也不需要人工干预） */
esp_err_t feed_ws_start(const dev_cfg_t *cfg);

/* 状态快照拷贝（显示任务读，WS 任务写，内部互斥） */
void feed_ws_snapshot(feed_state_t *out);

#endif /* FEED_WS_H */
