/*
 * wheel_config.h：板级编译期常量。
 * 板卡：Freenove ESP32-S3 WROOM CAM（FNK0085），OV5640 已接线但 v1 不启用。
 */
#ifndef WHEEL_CONFIG_H
#define WHEEL_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 4 路霍尔传感器 GPIO。
 * 板 GPIO 占用核对：摄像头 4,5,6,7,8,9,10,11,12,13,15,16,17,18；SD 38,39,40；
 * PSRAM 35,36,37；USB 19,20；CH343 串口 43,44；WS2812 48；GPIO2 板载 LED；
 * strapping 0,3,45,46。唯一净空的是 1,14,21,47，按轮顺序分配。 */
#define WHEEL_HALL_GPIO_LF 1   /* 左前 */
#define WHEEL_HALL_GPIO_RF 14  /* 右前 */
#define WHEEL_HALL_GPIO_LR 21  /* 左后 */
#define WHEEL_HALL_GPIO_RR 47  /* 右后 */

#define WHEEL_COUNT 4

/* 验证台 WiFi AP */
#define WHEEL_AP_SSID "wheel-bench"
#define WHEEL_AP_PASSWD ""        /* 开放热点；接外网/量产再设密码 */
#define WHEEL_AP_IP "192.168.4.1"

/* 采样/遥测周期 */
#define WHEEL_SAMPLE_MS 50        /* sampler 周期（采样窗口） */
#define WHEEL_TELEMETRY_MS 100    /* telemetry 聚合推送周期 = 2 × sample */
#define WHEEL_WS_PUSH_MS 100      /* WebSocket 推送周期 */

/* PCNT：A3144 开集极输出，磁铁触发拉低 = 负沿计数。
 * 毛刺滤波 200ns 只滤电气噪声；磁铁双脉冲用 collector 去抖（默认关）。 */
#define WHEEL_PCNT_GLITCH_NS 200

/* 任务参数（双核：重逻辑 Core1、轮询/分发 Core0） */
#define WHEEL_SAMPLER_CORE 0
#define WHEEL_SAMPLER_PRIO 22
#define WHEEL_SAMPLER_STACK 3072
#define WHEEL_CALC_CORE 0
#define WHEEL_CALC_PRIO 20
#define WHEEL_CALC_STACK 4096
#define WHEEL_WS_PUSH_CORE 0
#define WHEEL_WS_PUSH_PRIO 15
#define WHEEL_WS_PUSH_STACK 4096
#define WHEEL_HTTPD_CORE 1
#define WHEEL_HTTPD_STACK 8192

/* 队列（全静态内容，OOM 豁免：定长小结构） */
#define WHEEL_RAW_Q_LEN 4
#define WHEEL_TELE_Q_LEN 4

/* 二期 PWM 引脚预留（v1 只留常量与注释，不接线）：
 * + 舵机输出候选 GPIO41/42，默认 JTAG，启用时需关 JTAG 重配普通 IO；
 * + RC 接收机输入脚二期再定；此表在动硬件前核对 CLAUDE.md 引脚表。 */
#define WHEEL_SERVO_GPIO_PLANNED 42

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_CONFIG_H */
