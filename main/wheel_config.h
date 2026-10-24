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
/* 栈 8192：任务内帧缓冲 2048 + snprintf/WS/WiFi 调用面（4096 实测栈溢出） */
#define WHEEL_WS_PUSH_STACK 8192
#define WHEEL_HTTPD_CORE 1
#define WHEEL_HTTPD_STACK 8192

/* 队列（全静态内容，OOM 豁免：定长小结构） */
#define WHEEL_RAW_Q_LEN 4
#define WHEEL_TELE_Q_LEN 4

/* 差速舵机输出（LEDC，50Hz 标准舵机）。
 *
 * 41/42 是 ESP32-S3 的 JTAG 脚（MTDI/MTMS），但那是"外部 JTAG 探针"用的引脚：
 * 默认的调试通道是芯片内置 USB-Serial-JTAG，它走内部 TAP、不占用这四个脚
 * （IDF 文档 jtag-debugging/tips-and-quirks：用内置 USJ 调试时 GPIO39-42
 * 可另作他用）。所以这里直接拿来用，不必关 JTAG、不必切 console、不必烧 efuse。
 * 唯一的冲突来源是有人烧了 DIS_USB_JTAG efuse 把 JTAG 引到引脚上。
 *
 * RC 接收机输入脚二期再定；动硬件前核对 CLAUDE.md 引脚表。 */
#define WHEEL_SERVO_GPIO_FRONT 41 /* 前差速 */
#define WHEEL_SERVO_GPIO_REAR 42  /* 后差速 */

/* 遥控接收机三路 PWM 输入（GPIO 双沿中断测脉宽，见 components/rc_capture）。
 *
 * 38/39/40 原属板载 microSD 卡座：本项目从不用 SD，卡座让位；39/40 兼外部
 * JTAG 脚（MTCK/MTDO），与已在用的 41/42 同理——默认调试走内置
 * USB-Serial-JTAG，不占这四根脚。
 *
 * 没选另外三个"净空"脚 3/45/46：它们都是 strapping，45 在上电瞬间被采到
 * 高电平会把 flash 供电切成 1.8V 直接起不来；46 与 BOOT 键的组合会影响
 * 下载模式；3 只在烧过 efuse 时才影响调试源。让一路持续发脉冲的信号去踩
 * 上电敏感脚不划算。
 *
 * 现状提醒：接真接收机之前，这三个脚上什么都没有（悬空，内部下拉）。 */
#define WHEEL_RC_GPIO_MODE 38       /* 模式通道：≥1500µs 手动 / <1500µs 自动 */
#define WHEEL_RC_GPIO_DIFF_FRONT 39 /* 前差速通道 */
#define WHEEL_RC_GPIO_DIFF_REAR 40  /* 后差速通道 */

/* RC 通道超时：接收机 50Hz 帧周期 20ms，连丢 5 帧即判信号丢失。
 * 采样节拍 50ms，最迟 3 拍（150ms）内通告回落。 */
#define WHEEL_RC_TIMEOUT_MS 100

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_CONFIG_H */
