/* bench_cam_stream 实现：见头文件与 docs/CAMERA_DESIGN.md 4.1/4.2。 */
#include <stdio.h>
#include <string.h>

#include "bench_cam_stream.h"

#include "bench_cam.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "wheel_config.h"

static const char *TAG = "camstream";

#define CAM_STREAM_QUEUE_LEN 2
#define CAM_STREAM_START_WAIT_MS 5000
#define CAM_STREAM_IDLE_TIMEOUT_MS 10000
#define CAM_PART_HDR_CAP 128
/* 边界串故意长一点：JPEG 里偶尔出现 "frame" 这种短串不至于被误判成边界。 */
#define CAM_BOUNDARY "wheelcamframe"

static QueueHandle_t s_req_q;
static bool s_started;

static esp_err_t send_503(httpd_req_t *req, const char *why, bool retry)
{
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (retry) {
        httpd_resp_set_hdr(req, "Retry-After", "1");
    }
    return httpd_resp_sendstr(req, why);
}

/* handler 只负责抢名额、转异步、入队；相机启动和发送都在流任务里。 */
static esp_err_t handle_cam_stream(httpd_req_t *req)
{
    bench_cam_stream_result_t r = bench_cam_stream_acquire();
    if (r == BENCH_CAM_STREAM_BUSY) {
        return send_503(req, "camera stream busy", false);
    }
    if (r == BENCH_CAM_STREAM_OFF) {
        return send_503(req, "camera off", false);
    }

    httpd_req_t *async = NULL;
    esp_err_t e = httpd_req_async_handler_begin(req, &async);
    if (e != ESP_OK) {
        bench_cam_stream_release();
        ESP_LOGE(TAG, "async handler begin failed: %d", e);
        return e;
    }

    if (xQueueSend(s_req_q, &async, pdMS_TO_TICKS(100)) != pdTRUE) {
        /* 入队失败时已经不能再用原 req 发响应；完成异步句柄让连接关闭。
         * 仍返回 ESP_OK：异步句柄已接管这个请求，httpd 不该再碰原 req。 */
        bench_cam_stream_release();
        (void)httpd_req_async_handler_complete(async);
        ESP_LOGE(TAG, "stream request queue full");
        return ESP_OK;
    }
    return ESP_OK;
}

static esp_err_t handle_cam_snapshot(httpd_req_t *req)
{
    uint8_t *buf = NULL;
    size_t len = 0;
    bench_cam_snap_result_t r = bench_cam_snapshot(&buf, &len);

    switch (r) {
    case BENCH_CAM_SNAP_OK:
        httpd_resp_set_status(req, "200 OK");
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        {
            esp_err_t e = httpd_resp_send(req, (const char *)buf, len);
            heap_caps_free(buf);
            return e;
        }
    case BENCH_CAM_SNAP_OFF:
        return send_503(req, "camera off", false);
    case BENCH_CAM_SNAP_FAIL:
        return send_503(req, "camera failed, retrying", true);
    case BENCH_CAM_SNAP_NO_MEM:
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "snapshot buffer alloc failed");
    case BENCH_CAM_SNAP_WARMING:
    default:
        return send_503(req, "camera starting", true);
    }
}

static void stream_finish(httpd_req_t *req, uint8_t *buf, uint32_t frames, uint32_t bytes)
{
    if (frames == 0) {
        /* 还没发过任何 chunk，响应头也还没出去，这里可以照实回 503。 */
        (void)send_503(req, "camera stream ended before first frame", true);
    } else {
        /* 结束 multipart 的 chunked 响应；连接已断时这次发送会失败，忽略即可。 */
        (void)httpd_resp_send_chunk(req, NULL, 0);
    }
    (void)httpd_req_async_handler_complete(req);
    if (buf != NULL) {
        heap_caps_free(buf);
    }
    bench_cam_stream_release();
    ESP_LOGI(TAG, "stream end: frames=%lu bytes=%lu", (unsigned long)frames,
             (unsigned long)bytes);
}

static void cam_stream_task(void *arg)
{
    (void)arg;
    httpd_req_t *req = NULL;

    for (;;) {
        if (xQueueReceive(s_req_q, &req, portMAX_DELAY) != pdTRUE || req == NULL) {
            continue;
        }

        bool ready = false;
        for (int i = 0; i < CAM_STREAM_START_WAIT_MS / 50; i++) {
            bench_cam_state_t st = bench_cam_state();
            if (st == BENCH_CAM_STATE_ON) {
                ready = true;
                break;
            }
            if (st == BENCH_CAM_STATE_FAIL || st == BENCH_CAM_STATE_OFF ||
                bench_cam_stream_should_stop()) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (!ready) {
            (void)send_503(req, "camera not ready", true);
            (void)httpd_req_async_handler_complete(req);
            bench_cam_stream_release();
            continue;
        }

        telem_cam_status_t st;
        bench_cam_get_status(&st);
        uint32_t fps_cap = st.fps_cap != 0 ? st.fps_cap : 15;
        size_t frame_cap = bench_cam_max_frame();
        /* +2 给 JPEG 尾部的 CRLF，别在 frame_cap 正好用满时越界一字节 */
        uint8_t *send_buf = heap_caps_malloc(frame_cap + CAM_PART_HDR_CAP + 2,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (send_buf == NULL) {
            (void)send_503(req, "stream buffer alloc failed", false);
            (void)httpd_req_async_handler_complete(req);
            bench_cam_stream_release();
            continue;
        }

        httpd_resp_set_status(req, "200 OK");
        httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=" CAM_BOUNDARY);
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        httpd_resp_set_hdr(req, "Pragma", "no-cache");

        uint32_t frames = 0, bytes = 0, last_gen = 0;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t last_send_ms = now;
        uint32_t next_send_ms = now;
        uint32_t last_cfg_ms = now;

        while (1) {
            if (bench_cam_stream_should_stop() ||
                bench_cam_state() != BENCH_CAM_STATE_ON) {
                break;
            }

            now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            /* fps 上限只影响发送，不需要重 init；每秒重读一次配置，
             * 设置页改完立即生效，不要求用户等流重连。 */
            if (now - last_cfg_ms >= 1000) {
                bench_cam_get_status(&st);
                if (st.fps_cap != 0) {
                    fps_cap = st.fps_cap;
                }
                last_cfg_ms = now;
            }

            /* 帧率上限只管发送：没到下一个发送窗口就先等，到点后才从发布
             * 缓冲取最新帧。先取再等会把刚拷的帧白白丢掉。 */
            if ((int32_t)(now - next_send_ms) < 0) {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }

            size_t len = 0;
            esp_err_t e = bench_cam_copy_latest(send_buf + CAM_PART_HDR_CAP,
                                                frame_cap, &len, &last_gen);
            now = xTaskGetTickCount() * portTICK_PERIOD_MS;

            if (e == ESP_ERR_NOT_FOUND || e == ESP_ERR_TIMEOUT) {
                if (now - last_send_ms > CAM_STREAM_IDLE_TIMEOUT_MS) {
                    ESP_LOGW(TAG, "stream idle timeout");
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            if (e != ESP_OK) {
                ESP_LOGW(TAG, "copy latest failed: %d", e);
                break;
            }

            int hlen = snprintf((char *)send_buf, CAM_PART_HDR_CAP,
                                "--" CAM_BOUNDARY "\r\nContent-Type: image/jpeg\r\n"
                                "Content-Length: %u\r\n\r\n",
                                (unsigned)len);
            if (hlen <= 0 || hlen >= CAM_PART_HDR_CAP) {
                ESP_LOGW(TAG, "part header overflow");
                break;
            }
            /* 帧先拷在 staging 偏移 128，这里把它左移到正文头后面；不移动的话
             * hlen..127 的填充会被当成 JPEG 正文，浏览器收到的是花屏。 */
            if (hlen != CAM_PART_HDR_CAP) {
                memmove(send_buf + hlen, send_buf + CAM_PART_HDR_CAP, len);
            }
            send_buf[hlen + len] = '\r';
            send_buf[hlen + len + 1] = '\n';

            e = httpd_resp_send_chunk(req, (const char *)send_buf,
                                      (ssize_t)(hlen + len + 2));
            if (e != ESP_OK) {
                ESP_LOGW(TAG, "send chunk failed: %d", e);
                break;
            }

            last_send_ms = now;
            frames++;
            bytes += (uint32_t)len;
            uint32_t interval = 1000u / fps_cap;
            if (interval == 0) {
                interval = 1;
            }
            next_send_ms = now + interval;
        }

        stream_finish(req, send_buf, frames, bytes);
        req = NULL;
    }
}

esp_err_t bench_cam_stream_start(httpd_handle_t hd)
{
    if (hd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_started) {
        return ESP_OK;
    }

    s_req_q = xQueueCreate(CAM_STREAM_QUEUE_LEN, sizeof(httpd_req_t *));
    if (s_req_q == NULL) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(cam_stream_task, "cam_stream", WHEEL_CAM_STREAM_STACK,
                                            NULL, WHEEL_CAM_STREAM_PRIO, NULL,
                                            WHEEL_CAM_STREAM_CORE);
    if (ok != pdPASS) {
        vQueueDelete(s_req_q);
        s_req_q = NULL;
        return ESP_ERR_NO_MEM;
    }

    httpd_uri_t uri = {
        .uri = "/cam/stream",
        .method = HTTP_GET,
        .handler = handle_cam_stream,
        .user_ctx = NULL,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "stream uri");

    uri.uri = "/cam/snapshot";
    uri.handler = handle_cam_snapshot;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "snapshot uri");

    s_started = true;
    ESP_LOGI(TAG, "camera endpoints: /cam/stream, /cam/snapshot");
    return ESP_OK;
}
