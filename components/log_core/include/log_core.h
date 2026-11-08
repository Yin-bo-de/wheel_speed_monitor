/*
 * log_core：日志旁路的纯逻辑部分（无 IDF 依赖，宿主机可测）。
 *
 * 三块内容（行为规格见 docs/LOGGING_DESIGN.md 1~3 节）：
 *   1. RAM 环形缓冲：实时视图的行存储，行带全局单调 seq，前端按 cursor 增量拉取；
 *   2. 日志文件名生成与严格解析：日期命名 log_YYYYMMDD_HH.log / 开机相对命名
 *      log_bBBBB_HHH.log，解析即 /api/logs/file 的文件名白名单校验（防路径穿越）；
 *   3. 轮转判定与配额删除规划：刷盘任务每 2s 问一次 should_rotate，
 *      启动/轮转时用 plan_deletes 得到待删文件清单（b 文件在前按 boot 升序，
 *      date 文件在后按文件名升序），由调用方执行删除。
 *
 * 本组件只做纯计算：不碰 vprintf 钩子、文件系统和时间源，那些在 IDF 侧的
 * bench_log 里组装。所有 API 无锁——并发由上层（vprintf 钩子加锁）保证。
 */
#ifndef LOG_CORE_H
#define LOG_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

/* 宿主机测试命中 tests/native/include/esp_err.h（shim），IDF 构建命中 esp_common 的 esp_err.h */
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 单行上限（不含结尾 '\0'）。224 是因为项目里有 200+ 字符的 ESP_LOG 配置
 * dump 行，160 会截断丢信息；环形缓冲按 256 行 × 232B ≈ 58KB RAM 预算。
 */
#define LOG_LINE_MAX 224

/* 环形缓冲的一行。len 为实际字符数（不含 '\0'），截断后 ≤ LOG_LINE_MAX-1。 */
typedef struct {
    uint32_t seq;
    uint16_t len;
    char text[LOG_LINE_MAX];
} log_ring_entry_t;

/*
 * 环形缓冲控制块。slots 由调用方提供静态数组（避免堆分配/碎片）；
 * seq 从 1 开始单调递增，seq 为 s 的行物理上固定放在 slots[(s-1) % count]，
 * 挤掉旧行只推进 oldest_seq，不改变这个映射。
 */
typedef struct {
    log_ring_entry_t *slots;
    size_t count;        /* 槽位数（设计值 256） */
    uint32_t next_seq;   /* 下一条将写入的 seq；也用于写入定位 */
    uint32_t oldest_seq; /* 环内现存最老行 seq；空环时 == next_seq */
} log_ring_t;

/*
 * 初始化为空环。slots==NULL 或 count==0 时环不可用（push 恒失败返回 0），
 * 便于在配置异常时安全降级为不缓存。
 */
void log_ring_init(log_ring_t *r, log_ring_entry_t *slots, size_t count);

/*
 * 压入一行（不含结尾 '\n'；超长内部截断到 LOG_LINE_MAX-1 并保证 '\0' 结尾）。
 * 参数错误（r/slots 未初始化或 line 为 NULL）返回 0，成功返回该行 seq。
 * len 为 0 的空行是合法行（如空 progress 行），正常分配 seq。
 */
uint32_t log_ring_push(log_ring_t *r, const char *line, size_t len);

/*
 * 拷贝 seq > cursor 的行到 out（容量 cap），按 seq 升序，返回实际写入行数。
 * *oldest = 环内现存最老 seq（空环时 = 下一行 seq），*next = 下一条将写入的 seq。
 * 丢行判定归调用方：cursor < oldest 说明 cursor+1..oldest-1 已被挤掉；
 * 环被 cap 截断时用 next-oldest 与返回行数比对。
 * oldest/next/out 允许传 NULL（只查状态或只查 oldest/next 时用）。
 */
size_t log_ring_copy_from(const log_ring_t *r, uint32_t cursor,
                          log_ring_entry_t *out, size_t cap,
                          uint32_t *oldest, uint32_t *next);

/* 文件名类型：日期命名 / 开机相对命名（未对时期间的文件） */
typedef enum {
    LOG_FN_DATE = 0,
    LOG_FN_BOOT = 1,
} log_fn_type_t;

/*
 * 日期命名：log_20261108_15.log（年月日_24 小时制小时，零填充）。mon 取 1~12。
 * 字段饱和而非取模：year 收敛到 0..9999，mon/day/hour 收敛到 0..99，负数
 * 收敛到 0。保证位宽固定、生成的名字永远能被 log_name_parse 接受；不做日历
 * 合法性校验（如 mon=13），与白名单正则的语义一致。
 */
void log_name_date(char *buf, size_t len, int year, int mon, int day, int hour);

/*
 * 开机相对命名：log_b0037_005.log（boot 序号 4 位 + 开机后第几小时 3 位，零填充）。
 * 字段饱和而非取模：boot > 9999 收敛到 9999，hour > 999 收敛到 999，负数
 * 收敛到 0。否则会生成 5 位/4 位字段，被 log_name_parse 拒绝后轮转永久失效。
 */
void log_name_boot(char *buf, size_t len, uint32_t boot, int hour);

/*
 * 严格解析：整体匹配设计文档 3 节的白名单正则
 * ^log_(b\d{4}_\d{3}|\d{8}_\d{2})\.log$，多一个字符、大小写不符都算失败。
 * 成功时全部输出填充：date 型填 y/mon/d/h、boot 填 0；boot 型填 boot/h、
 * 其余填 0；失败时返回 false 且不改写任何输出（解析只做结构匹配，
 * 不校验 mon≤12 等取值范围，与白名单正则语义一致）。
 * 所有输出指针允许为 NULL（只关心是否合法/类型时用）。
 */
bool log_name_parse(const char *name, log_fn_type_t *type,
                    int *y, int *mon, int *d, int *h, uint32_t *boot);

/*
 * 轮转判定：open_name 为 NULL/空 = 尚无文件，返回 true（必须先开一个）；
 * date 文件完整比较文件名里的 年/月/日/时 与 now_lt（tm_year+1900、tm_mon+1、
 * tm_mday、tm_hour），任一不同 → true。只比小时会漏掉系统时间恰好跳变
 * 24 小时（同小时不同天）的场景；boot 文件比较文件名中的 hour 与
 * uptime_hour 按同样规则饱和后的值（0..999），任一不同 → true——不饱和会
 * 在 uptime_hour > 999 后永不相等，导致无限轮转；
 * 名字无法解析 → false（不认识的文件不动它，避免误轮转）。
 */
bool log_should_rotate(const char *open_name, const struct tm *now_lt, int uptime_hour);

/* 待删文件条目（name 指向调用方持有的字符串，本组件不拷贝） */
typedef struct {
    const char *name;
    size_t size;
} log_file_info_t;

/*
 * 配额删除规划：在 files 里按删除顺序挑出使「删完后总大小 ≤ quota」的最少
 * 文件，把 name 指针按删除顺序写进 out_names（最多 cap 个），返回数量。
 * 删除顺序：b 文件按 boot 序号升序在前（同 boot 按文件名升序），
 * date 文件按文件名升序在后；cur_name（当前正在写的文件）永不入选。
 * 无法解析的文件无法排序，跳过不删；删完所有可删文件仍超配额时返回已达
 * 到的数量（尽力而为，调用方收到清单后逐个删即可）。已在配额内返回 0。
 */
size_t log_plan_deletes(const log_file_info_t *files, size_t n, size_t quota,
                        const char *cur_name, const char **out_names, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* LOG_CORE_H */
