/* telem_frame：遥测帧契约（车载端与 OLED 显示子项目共享）。
 *
 * 契约的唯一事实来源是车端的渲染代码（components/telemetry +
 * wheel_speed/render.c + strategy/strategy_render.c），本组件只做解析侧的
 * 镜像：字段名与取值以那几个 render 函数的输出为准，改任何一侧都要同步。
 *
 * 纯 C、不依赖 IDF，宿主机可测。一期只启用下行遥测帧解析；
 * 上行 IMU 帧只定契约，不实现、不发送。 */
#ifndef TELEM_FRAME_H
#define TELEM_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 契约版本：帧结构（字段增删/语义变化）时 +1，两侧按版本兼容。 */
#define TELEM_FRAME_CONTRACT_VER 1

/* 帧类型（顶层 type 字段） */
#define TELEM_FRAME_TYPE_TELEMETRY "telemetry"
#define TELEM_FRAME_TYPE_IMU       "imu" /* 二期预留：C3→车端上行姿态帧 */

#define TELEM_WHEEL_COUNT 4
#define TELEM_DIFF_COUNT  2 /* 0 = 前差速，1 = 后差速 */

/* 差速策略阶段，与车端 strategy_phase_name() 的字符串一一对应 */
typedef enum {
    TELEM_PHASE_UNKNOWN = 0,
    TELEM_PHASE_IDLE,         /* "idle"         解锁 */
    TELEM_PHASE_PENDING_SYNC, /* "pending_sync" 等待同步 */
    TELEM_PHASE_PROBE,        /* "probe"        试探 */
    TELEM_PHASE_LOCKED,       /* "locked"       锁定 */
} telem_phase_t;

/* 下行遥测帧里 OLED 需要的字段视图（车端帧字段多，这里只摘用得到的） */
typedef struct {
    bool has_wheel;                        /* 本帧含 wheel_speed 块 */
    bool wheel_en[TELEM_WHEEL_COUNT];      /* 轮启用（en=false 视为无效轮） */
    float wheel_rpm[TELEM_WHEEL_COUNT];
    bool has_mode;                         /* 本帧含 servo 块（生效模式） */
    bool mode_auto;                        /* servo.mode："auto" / "manual" */
    telem_phase_t phase[TELEM_DIFF_COUNT]; /* servos[i].phase */
    bool phase_en[TELEM_DIFF_COUNT];       /* servos[i].en */
} telem_frame_view_t;

/* 解析车端 telem_render_frame() 输出的下行遥测帧。
 * 至少解析出一个有效块才返回 true；块缺失不视为错误（车端版本演进容忍
 * 字段增减），由调用方按 has_* 判断数据是否齐。 */
bool telem_frame_parse(const char *json, size_t len, telem_frame_view_t *out);

/* "idle"/"pending_sync"/"probe"/"locked" → 枚举，其余归 UNKNOWN */
telem_phase_t telem_phase_from_str(const char *s);

/* ---- 二期预留：上行 IMU 帧契约（一期不实现、不发送） ----
 * MPU6050 接 C3 后，C3 周期性发此结构（序列化格式随契约版本一起定），
 * 车端 S3 用姿态数据驱动舵机云台。 */
typedef struct {
    uint32_t t_ms;       /* C3 本地时间戳 */
    float gyro_dps[3];   /* 角速度 */
    float accel_g[3];    /* 加速度 */
} telem_imu_frame_t;

#ifdef __cplusplus
}
#endif

#endif /* TELEM_FRAME_H */
