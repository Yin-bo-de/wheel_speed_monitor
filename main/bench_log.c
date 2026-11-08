/* bench_log 实现：环形缓冲 + vprintf 钩子 + esp_timer 刷盘 + 轮转/配额。
 * 刷盘全部在 esp_timer 任务上下文（不新增 FreeRTOS 任务）；OTA 联动由
 * bench_ota 在 httpd 任务里调 flush_now/suspend（flush_now 与定时器互斥）。 */
#include "bench_log.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h> /* struct timeval / settimeofday：浏览器对时写系统时钟 */
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_vfs.h"
#include "esp_partition.h" /* 挂载失败时判断分区是否出厂空（全 0xFF） */
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_littlefs.h"

#include "log_core.h"
#include "config_store.h"

static const char *TAG = "blog";

#define LOG_MOUNT_POINT "/logfs"
#define LOG_PART_LABEL  "storage"
#define LOG_RING_COUNT  256
#define LOG_FLUSH_MS    2000   /* 设计文档：崩溃最多丢最后 2s 日志 */
#define LOG_DIRTY_TRIGGER (4u * 1024u) /* 设计文档：脏字节 > 4KB 提前刷 */
#define LOG_FILE_MAX_BYTES (128u * 1024u) /* 设计文档 3 节：单小时文件封顶 */
#define LOG_QUOTA       (6u * 1024u * 1024u)
#define LOG_SNAP_LINES  32     /* 每批刷盘行数（~7KB 写入） */
#define LOG_NAME_MAX    32
#define LOG_LIVE_BATCH  16     /* live 每次在锁内最多拷 16 行，放锁后再发送 */
#define LOG_LIVE_MAX    256    /* live 单次最多覆盖整个环 */
#define LOG_LIVE_DEFAULT 50    /* 无 cursor：返回环内最新 50 行 */
#define LOG_MAX_FILES   2048   /* 文件数上限（防目录异常撑爆内存）；实际按需动态分配 */

/* ---- 状态 ---- */

static log_ring_entry_t *s_ring_slots; /* PSRAM 优先，分配失败退内部 RAM */
static log_ring_t s_ring;
static SemaphoreHandle_t s_lock;       /* 环形缓冲 + dirty/cursor + s_cur/名字 */
static SemaphoreHandle_t s_flush_lock; /* flush_once 串行化（定时器 vs OTA） */
static volatile uint32_t s_flush_cursor;
static volatile size_t s_dirty;        /* 待刷字节数（含每行换行），不是行数 */
static FILE *s_cur;
static char s_cur_name[LOG_NAME_MAX];
static size_t s_cur_size;              /* 当前文件已写字节，用于 128KiB 封顶 */
static bool s_cap_warned;              /* 当前文件封顶告警只打一次 */
static esp_timer_handle_t s_urgent_timer; /* 脏字节超阈值时的提前刷（一次性） */
static volatile bool s_clock_synced;
static volatile bool s_rotate_pending; /* 首次对时：b 文件 → 日期文件 */
static volatile bool s_suspended;
static bool s_fs_ok;                   /* LittleFS 可用；false 时降级为只留实时视图 */
static bool s_files_cap_warned;        /* 文件数超上限只告警一次 */
static uint32_t s_boot;

/* 用 esp_timer_get_time（int64 微秒）而不是 esp_log_timestamp：后者受
 * CONFIG_LOG_TIMESTAMP_SOURCE 影响（SYSTEM 源下是 epoch 时间），且 32 位
 * 毫秒在 49.7 天回绕，会让 b 文件的小时数倒退 */
static int uptime_hour(void)
{
    return (int)(esp_timer_get_time() / 3600000000ULL);
}

static uint64_t uptime_seconds(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000000ULL);
}

static void log_path(const char *name, char *out, size_t len)
{
    /* 文件名都过 log_name_parse（最长 19）；%.*s 让编译器能证明不会截断，
     * 也顺手切掉 dirent 里可能出现的超长脏名字 */
    snprintf(out, len, LOG_MOUNT_POINT "/%.*s", LOG_NAME_MAX - 1, name);
}

/* ---- 文件扫描（配额清理与 /api/logs/list 共用） ----
 * 典型负载每小时约 20KB，6MiB 配额下会有 ~300 个文件；固定 64 项会让配额
 * 删不干净、列表也看不全。改为先计数再按实际数量动态分配（PSRAM 优先），
 * 名字直接内嵌在条目里，避免再开一份定长二维数组。 */

typedef struct {
    char name[LOG_NAME_MAX];
    size_t size;
    bool is_boot;
    uint32_t boot;
    uint32_t key; /* date: y*10000+mo*100+d；boot: 0（纯排序键） */
    int hour;
} log_list_entry_t;

/* 大数组优先放 PSRAM；PSRAM 不可用/耗尽时退内部 RAM */
static void *alloc_prefer_psram(size_t size)
{
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (p == NULL) {
        p = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    }
    return p;
}

static void warn_files_cap(void)
{
    if (!s_files_cap_warned) {
        s_files_cap_warned = true;
        ESP_LOGW(TAG, "more than %u log files, extra ones ignored",
                 (unsigned)LOG_MAX_FILES);
    }
}

/* 第一遍：只按文件名白名单计数（不做 stat，快） */
static size_t count_log_files(void)
{
    DIR *d = opendir(LOG_MOUNT_POINT);
    if (d == NULL) {
        return 0;
    }
    size_t n = 0;
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        log_fn_type_t type;
        int y, mo, dd, h;
        uint32_t b;
        if (log_name_parse(ent->d_name, &type, &y, &mo, &dd, &h, &b)) {
            n++;
        }
    }
    closedir(d);
    return n;
}

/* 第二遍：填充名字/大小/排序键，最多 cap 项。cap 来自第一遍计数，
 * 中途新增的文件装不下就留到下一次（不为此误报“超过上限”） */
static size_t fill_log_files(log_list_entry_t *ents, size_t cap)
{
    DIR *d = opendir(LOG_MOUNT_POINT);
    if (d == NULL) {
        return 0;
    }
    size_t n = 0;
    struct dirent *ent;
    while (n < cap && (ent = readdir(d)) != NULL) {
        log_fn_type_t type;
        int y, mo, dd, h;
        uint32_t b;
        if (!log_name_parse(ent->d_name, &type, &y, &mo, &dd, &h, &b)) {
            continue;
        }
        char path[48];
        log_path(ent->d_name, path, sizeof(path));
        struct stat st;
        if (stat(path, &st) != 0) {
            continue;
        }
        /* log_name_parse 已保证长度 ≤19，直接 memcpy 避免 snprintf 的
         * format-truncation 告警（-Werror=all 下是硬错误） */
        size_t name_len = strlen(ent->d_name);
        if (name_len >= LOG_NAME_MAX) {
            continue; /* 理论不可达，防脏目录项 */
        }
        memcpy(ents[n].name, ent->d_name, name_len + 1);
        ents[n].size = (size_t)st.st_size;
        ents[n].is_boot = (type == LOG_FN_BOOT);
        ents[n].boot = b;
        ents[n].key = (type == LOG_FN_DATE)
                          ? (uint32_t)(y * 10000 + mo * 100 + dd)
                          : 0;
        ents[n].hour = h;
        n++;
    }
    closedir(d);
    return n;
}

/* qsort 比较器：boot 文件在前（按 boot 升序），date 文件按 日期+小时 升序 */
static int log_list_entry_cmp(const void *pa, const void *pb)
{
    const log_list_entry_t *a = (const log_list_entry_t *)pa;
    const log_list_entry_t *b = (const log_list_entry_t *)pb;
    if (a->is_boot != b->is_boot) {
        return a->is_boot ? -1 : 1;
    }
    if (a->is_boot) {
        return (a->boot < b->boot) ? -1 : (a->boot > b->boot);
    }
    if (a->key != b->key) {
        return (a->key < b->key) ? -1 : 1;
    }
    return (a->hour < b->hour) ? -1 : (a->hour > b->hour);
}

/* 在锁内快照当前文件状态：s_cur / s_cur_name 会被 httpd 的 list 处理器与
 * 刷盘定时器并发访问。这里只拷名字和"是否打开"，FILE* 不暴露到锁外使用。
 * 锁顺序约束：调用方可能持有 s_flush_lock（刷盘路径），这里只短暂拿 s_lock；
 * 任何路径都不允许反向（持 s_lock 去拿 s_flush_lock）。 */
static bool cur_file_snapshot(char *name, size_t len)
{
    if (s_lock == NULL) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool open = (s_cur != NULL);
    if (open && name != NULL && len > 0) {
        snprintf(name, len, "%s", s_cur_name);
    }
    xSemaphoreGive(s_lock);
    return open;
}

/* s_rotate_pending 用"取走"语义：刷盘线程消费后清掉，开文件失败再放回。
 * 这样与 httpd 对时线程的 set 竞争时，首次对时的轮转请求不会丢。 */
static bool take_rotate_pending(void)
{
    if (s_lock == NULL) {
        bool pending = s_rotate_pending;
        s_rotate_pending = false;
        return pending;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool pending = s_rotate_pending;
    s_rotate_pending = false;
    xSemaphoreGive(s_lock);
    return pending;
}

static void set_rotate_pending(void)
{
    if (s_lock == NULL) {
        s_rotate_pending = true;
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_rotate_pending = true;
    xSemaphoreGive(s_lock);
}

/* ---- 配额清理 ---- */

static void cleanup_quota(void)
{
    if (!s_fs_ok) {
        return;
    }
    /* 删除计划的 cur 保护名在锁内取最新值（本函数与 open/close 都在
     * s_flush_lock 下串行，锁只用于和 httpd list 的并发读隔离） */
    char cur_name[LOG_NAME_MAX];
    bool cur_open = cur_file_snapshot(cur_name, sizeof(cur_name));

    size_t found = count_log_files();
    if (found == 0) {
        return;
    }
    if (found > LOG_MAX_FILES) {
        found = LOG_MAX_FILES;
        warn_files_cap();
    }

    log_list_entry_t *ents = alloc_prefer_psram(found * sizeof(*ents));
    log_file_info_t *infos = alloc_prefer_psram(found * sizeof(*infos));
    const char **del = alloc_prefer_psram(found * sizeof(*del));
    if (ents == NULL || infos == NULL || del == NULL) {
        /* 配额清理是尽力而为：本轮跳过，历史文件留着总比崩溃好 */
        ESP_LOGE(TAG, "quota: no mem for %u files, skip cleanup", (unsigned)found);
        free(ents);
        free(infos);
        free(del);
        return;
    }

    size_t n = fill_log_files(ents, found);
    for (size_t i = 0; i < n; i++) {
        infos[i].name = ents[i].name; /* 名字内嵌在 ents 里，不另拷贝 */
        infos[i].size = ents[i].size;
    }

    /* 删除清单上限取文件总数：文件多时一轮就能删到配额内 */
    size_t nd = log_plan_deletes(infos, n, LOG_QUOTA,
                                 cur_open ? cur_name : NULL, del, n);
    for (size_t i = 0; i < nd; i++) {
        char path[48];
        log_path(del[i], path, sizeof(path));
        if (unlink(path) == 0) {
            ESP_LOGI(TAG, "quota: dropped %s", del[i]);
        }
    }

    free(del);
    free(infos);
    free(ents);
}

/* ---- 轮转与刷盘 ---- */

/* 打开当前时钟/开机小时对应的文件；成功返回 true。调用方持有 s_flush_lock。
 * 命名与 fopen 都在锁外做（I/O 不占 s_lock），只把指针/名字的发布放进锁。 */
static bool open_log_file(void)
{
    char name[LOG_NAME_MAX];
    if (s_clock_synced) {
        time_t now = time(NULL);
        struct tm lt;
        localtime_r(&now, &lt);
        log_name_date(name, sizeof(name), lt.tm_year + 1900, lt.tm_mon + 1,
                      lt.tm_mday, lt.tm_hour);
    } else {
        log_name_boot(name, sizeof(name), s_boot, uptime_hour());
    }
    char path[48];
    log_path(name, path, sizeof(path));
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "open %s failed", path);
        return false; /* 下个刷盘周期重试 */
    }

    /* 追加模式可能接着已有文件写（同小时重启）：记录当前大小，
     * 128KiB 封顶要从已有字节继续累加 */
    size_t size = 0;
    if (fseek(f, 0, SEEK_END) == 0) {
        long sz = ftell(f);
        if (sz > 0) {
            size = (size_t)sz;
        }
    }
    s_cur_size = size;
    s_cap_warned = false;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    FILE *old = s_cur;
    s_cur = f;
    snprintf(s_cur_name, sizeof(s_cur_name), "%s", name);
    xSemaphoreGive(s_lock);
    if (old != NULL) {
        fclose(old); /* 句柄已从共享状态摘除，锁外关闭不影响 list 读名字 */
    }

    ESP_LOGI(TAG, "logging to %s (%u KiB)", name, (unsigned)(size / 1024));
    return true;
}

static void close_log_file(void)
{
    FILE *f = NULL;
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        f = s_cur;
        s_cur = NULL;
        s_cur_name[0] = '\0';
        xSemaphoreGive(s_lock);
    } else {
        f = s_cur;
        s_cur = NULL;
        s_cur_name[0] = '\0';
    }
    if (f != NULL) {
        fclose(f);
    }
    s_cur_size = 0;
    s_cap_warned = false;
}

/* 拷贝一批脏行 → 轮转检查 → 追加写盘。调用方持有 s_flush_lock。 */
static void flush_once(void)
{
    /* 降级模式（LittleFS 挂载失败）不碰任何文件操作：环只服务实时视图 */
    if (!s_fs_ok || s_suspended || s_lock == NULL) {
        return;
    }
    /* 仅 esp_timer/OTA 调用，静态缓冲省 7KB 栈；必须是 log_ring_entry_t
     * （含 seq/len 头），不能按 char[LOG_LINE_MAX] 声明，否则拷贝越界 */
    static log_ring_entry_t snap[LOG_SNAP_LINES];

    for (;;) {
        size_t n;
        size_t bytes = 0;
        uint32_t oldest, next;

        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_ring.slots != NULL && s_ring.count > 0 &&
            s_flush_cursor + 1 < s_ring.oldest_seq) {
            /* 环把没来得及刷的行挤掉了：跳过空洞，并按环内实际未消费行重算
             * 脏字节。否则 s_dirty 只增不减，提前刷定时器会持续空转 */
            s_flush_cursor = s_ring.oldest_seq - 1;
            size_t d = 0;
            for (uint32_t s = s_ring.oldest_seq; s < s_ring.next_seq; s++) {
                d += s_ring.slots[(s - 1) % s_ring.count].len + 1;
            }
            s_dirty = d;
        }
        n = log_ring_copy_from(&s_ring, s_flush_cursor, snap, LOG_SNAP_LINES,
                               &oldest, &next);
        if (n > 0) {
            for (size_t i = 0; i < n; i++) {
                bytes += (size_t)snap[i].len + 1; /* 落盘每行补 '\n' */
            }
            s_flush_cursor = next;
            s_dirty = (s_dirty > bytes) ? s_dirty - bytes : 0;
        }
        xSemaphoreGive(s_lock);

        bool pending = take_rotate_pending();

        if (n == 0) {
            /* 没有新行也要把首次对时的轮转做掉，否则 b 文件会一直写到
             * 下一小时；无文件时不必现在开空文件，下次有行时按新时钟命名 */
            if (pending && cur_file_snapshot(NULL, 0)) {
                close_log_file();
                cleanup_quota();
                if (!open_log_file()) {
                    set_rotate_pending();
                }
            }
            break;
        }

        bool have_file = cur_file_snapshot(NULL, 0);
        bool rotate = pending;
        if (have_file && !rotate) {
            char cur[LOG_NAME_MAX];
            (void)cur_file_snapshot(cur, sizeof(cur));
            time_t now = time(NULL);
            struct tm lt;
            localtime_r(&now, &lt);
            rotate = log_should_rotate(cur, &lt, uptime_hour());
        }

        if (!have_file) {
            /* 无文件 → 开；时钟若已对时，这里直接开日期文件 */
            (void)open_log_file();
        } else if (rotate) {
            close_log_file();
            cleanup_quota();
            (void)open_log_file();
        }

        if (!cur_file_snapshot(NULL, 0)) {
            if (pending) {
                set_rotate_pending(); /* 开失败：保留轮转请求，下个周期重试 */
            }
            /* 打不开文件（flash 异常）：丢弃这批，避免每批空转重试 */
            ESP_LOGE(TAG, "log file unavailable, drop %u lines", (unsigned)n);
            continue;
        }
        /* 开成功后 pending 已消费：新文件就是按当前时钟/开机小时命名的，
         * 下一批不用再白关白开一次 */

        for (size_t i = 0; i < n; i++) {
            size_t line_bytes = (size_t)snap[i].len + 1;
            if (s_cur_size + line_bytes > LOG_FILE_MAX_BYTES) {
                /* 单文件封顶：停止写盘但行已从 dirty 游标消费（丢弃），
                 * RAM 环不会因此堆积；每文件只告警一次 */
                if (!s_cap_warned) {
                    s_cap_warned = true;
                    ESP_LOGW(TAG, "log file reached %u KiB, dropping until rotate",
                             (unsigned)(LOG_FILE_MAX_BYTES / 1024));
                }
                break; /* 本批剩余行同样丢弃 */
            }
            fputs(snap[i].text, s_cur);
            fputc('\n', s_cur);
            s_cur_size += line_bytes;
        }
        fflush(s_cur);
    }
}

static void flush_timer_cb(void *arg)
{
    (void)arg;
    if (!s_fs_ok || s_suspended) {
        /* 降级模式没有可写文件；OTA 写 flash 期间也不刷，避免与升级交错 */
        return;
    }
    if (s_dirty > 0 || s_rotate_pending) {
        xSemaphoreTake(s_flush_lock, portMAX_DELAY);
        flush_once();
        xSemaphoreGive(s_flush_lock);
    }
}

/* ---- vprintf 钩子 ---- */

/* 把一段 [src,len) 去掉 ANSI 颜色序列后压进环形缓冲。
 * 控制台保留原始输出（含颜色），落盘/实时视图用干净行。 */
static void hook_push_clean(const char *src, size_t len)
{
    /* 普通栈缓冲：_Thread_local 依赖 sdkconfig 的 TLS 支持，多 224B 栈
     * 代价可接受，换掉这个隐式配置依赖 */
    char clean[LOG_LINE_MAX];
    size_t o = 0;
    for (size_t i = 0; i < len && o < sizeof(clean) - 1; i++) {
        char c = src[i];
        if (c == 0x1b && i + 1 < len && src[i + 1] == '[') {
            i += 2;
            while (i < len && (src[i] == ';' || (src[i] >= '0' && src[i] <= '9'))) {
                i++;
            }
            if (i < len && src[i] == 'm') {
                continue; /* 序列消费完毕 */
            }
        }
        clean[o++] = c;
    }
    clean[o] = '\0';
    if (o == 0 || s_lock == NULL) {
        return;
    }

    /* 落盘按"行 + 换行"计字节；push 内部会截断到 LOG_LINE_MAX-1 */
    size_t stored = (o < LOG_LINE_MAX - 1) ? o : LOG_LINE_MAX - 1;
    size_t dirty_bytes = stored + 1;
    bool trigger = false;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (log_ring_push(&s_ring, clean, o) > 0 && s_fs_ok) {
        /* 降级模式（无法落盘）不累计脏字节：flush 直接返回，累计只会让
         * 提前刷定时器空转；行本身仍进环，实时视图照常可用 */
        s_dirty += dirty_bytes;
        trigger = (s_dirty >= LOG_DIRTY_TRIGGER);
    }
    xSemaphoreGive(s_lock);

    if (trigger && !s_suspended && s_urgent_timer != NULL) {
        /* 脏字节超阈值：一次性定时器提前刷。已武装时返回 INVALID_STATE，
         * 忽略即可；钩子只在任务上下文运行（ISR 里不能打 ESP_LOG）。
         * 定时器创建失败时退化为 2s 周期刷 */
        (void)esp_timer_start_once(s_urgent_timer, 0);
    }
}

static int log_vprintf_hook(const char *fmt, va_list args)
{
    char line[LOG_LINE_MAX];
    /* vsnprintf 会消耗 args；给控制台留一份拷贝。超长行环里只能截断，
     * 但串口仍按默认 vprintf 完整输出，不改变原来的串口行为 */
    va_list console_args;
    va_copy(console_args, args);
    int n = vsnprintf(line, sizeof(line), fmt, args);
    if (n < 0) {
        va_end(console_args);
        return n;
    }
    size_t len = ((size_t)n > sizeof(line) - 1) ? sizeof(line) - 1 : (size_t)n;

    /* 按 \n 拆行（ESP_LOG 消息体内可能带换行），逐行入环 */
    size_t start = 0;
    for (size_t i = 0; i <= len; i++) {
        if (i == len || line[i] == '\n') {
            if (i > start) {
                hook_push_clean(line + start, i - start);
            }
            start = i + 1;
        }
    }

    if (n >= (int)sizeof(line)) {
        vprintf(fmt, console_args); /* 截断只发生在环/文件，串口保持完整 */
    } else {
        fputs(line, stdout); /* 控制台原样输出（含 ANSI 颜色） */
    }
    va_end(console_args);
    return n;
}

/* ---- 生命周期 ---- */

/* 分区是否为出厂空（探测区全是擦除值 0xFF）。只有这种情况才允许格式化：
 * 非空但挂载失败说明可能有历史日志，绝不能清 */
static bool storage_partition_erased(void)
{
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, LOG_PART_LABEL);
    if (part == NULL) {
        return false;
    }
    static uint8_t probe[256]; /* init 单次调用，静态缓冲省 app_main 栈 */
    size_t n = (part->size < sizeof(probe)) ? (size_t)part->size : sizeof(probe);
    if (n == 0 || esp_partition_read(part, 0, probe, n) != ESP_OK) {
        return false;
    }
    for (size_t i = 0; i < n; i++) {
        if (probe[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

esp_err_t bench_log_init(void)
{
    const esp_vfs_littlefs_conf_t conf = {
        .base_path = LOG_MOUNT_POINT,
        .partition_label = LOG_PART_LABEL,
        /* false：设计文档 2 节要求挂载失败绝不清历史日志（日志场景掉电概率高）。
         * 仅当分区是出厂空（整段 0xFF）时才格式化后重试一次 */
        .format_if_mount_failed = false,
    };
    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK && storage_partition_erased()) {
        ESP_LOGW(TAG, "storage partition blank, format once");
        err = esp_littlefs_format(LOG_PART_LABEL);
        if (err == ESP_OK) {
            err = esp_vfs_littlefs_register(&conf);
        }
    }
    s_fs_ok = (err == ESP_OK);
    if (!s_fs_ok) {
        /* 非空分区挂载失败：保留历史数据（绝不清），降级为“只留实时视图”。
         * 这样文件系统坏掉时还能打开日志 tab 看实时日志、排查挂载问题。
         * 此时钩子还没装，日志直接走串口 */
        ESP_LOGW(TAG, "littlefs mount %s failed (%s), live-only mode",
                 LOG_PART_LABEL, esp_err_to_name(err));
    }

    s_boot = config_store_next_boot_seq();

    s_ring_slots = heap_caps_calloc(LOG_RING_COUNT, sizeof(log_ring_entry_t),
                                    MALLOC_CAP_SPIRAM);
    if (s_ring_slots == NULL) {
        s_ring_slots = heap_caps_calloc(LOG_RING_COUNT, sizeof(log_ring_entry_t),
                                        MALLOC_CAP_8BIT);
    }
    if (s_ring_slots == NULL) {
        return ESP_ERR_NO_MEM;
    }
    log_ring_init(&s_ring, s_ring_slots, LOG_RING_COUNT);

    s_lock = xSemaphoreCreateMutex();
    s_flush_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL || s_flush_lock == NULL) {
        /* 钩子/刷盘/端点都依赖这两个锁；宁可不装日志，也不能让钩子
         * 里对 NULL 信号量取锁直接崩 */
        return ESP_ERR_NO_MEM;
    }

    if (s_fs_ok) {
        cleanup_quota(); /* 降级模式没有文件系统可清理 */
    }

    /* 定时器先建好再装钩子：建失败直接返回，不留"钩子在跑但没人刷盘"的
     * 半初始化状态 */
    const esp_timer_create_args_t flush_args = {
        .callback = flush_timer_cb,
        .name = "logflush",
    };
    esp_timer_handle_t flush_timer = NULL;
    ESP_RETURN_ON_ERROR(esp_timer_create(&flush_args, &flush_timer), TAG,
                        "timer create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(flush_timer, LOG_FLUSH_MS * 1000),
                        TAG, "timer start");

    const esp_timer_create_args_t urgent_args = {
        .callback = flush_timer_cb,
        .name = "logflush_urgent",
    };
    if (esp_timer_create(&urgent_args, &s_urgent_timer) != ESP_OK) {
        s_urgent_timer = NULL;
        ESP_LOGW(TAG, "urgent flush timer unavailable, use periodic only");
    }

    /* 钩子最后装：此时环/锁/定时器都已就绪（存储可用与否都能服务实时
     * 视图）；之前的启动日志只有串口，不会积压无法处理的行 */
    esp_log_set_vprintf(log_vprintf_hook);

    if (s_fs_ok) {
        ESP_LOGI(TAG, "log fs ready (boot #%lu, quota %u KiB)",
                 (unsigned long)s_boot, (unsigned)(LOG_QUOTA / 1024));
    } else {
        ESP_LOGW(TAG, "log live view ready (boot #%lu, no storage)",
                 (unsigned long)s_boot);
    }
    return ESP_OK;
}

void bench_log_clock_sync(int64_t client_epoch_ms)
{
    if (client_epoch_ms <= 0) {
        return;
    }
    setenv("TZ", "CST-8", 1); /* 验证台固定东八区 */
    tzset();
    struct timeval tv = {
        .tv_sec = (time_t)(client_epoch_ms / 1000),
        .tv_usec = (suseconds_t)((client_epoch_ms % 1000) * 1000),
    };
    settimeofday(&tv, NULL);
    if (!s_clock_synced) {
        /* 与刷盘线程的 pending 消费互斥，保证首次对时的强制轮转不丢 */
        if (s_lock != NULL) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
        }
        s_clock_synced = true;
        s_rotate_pending = true; /* b 文件立刻转日期命名 */
        if (s_lock != NULL) {
            xSemaphoreGive(s_lock);
        }
        ESP_LOGI(TAG, "clock synced from browser");
    }
}

void bench_log_flush_now(void)
{
    if (s_flush_lock == NULL) {
        return;
    }
    xSemaphoreTake(s_flush_lock, portMAX_DELAY);
    flush_once();
    xSemaphoreGive(s_flush_lock);
}

void bench_log_suspend(bool susp)
{
    if (!susp) {
        s_suspended = false;
        return;
    }
    /* 暂停时先等正在刷的那批写完，再在刷盘锁内立旗：这样函数返回后
     * 定时器既不会接着写旧批，也不会开始新批，OTA 的 flash 写入不会和
     * 日志追加交错。flush_timer_cb 也会先看这个标志，不会白等锁。 */
    if (s_flush_lock != NULL) {
        xSemaphoreTake(s_flush_lock, portMAX_DELAY);
        s_suspended = true;
        xSemaphoreGive(s_flush_lock);
    } else {
        s_suspended = true;
    }
}

/* ---- HTTP 端点 ---- */

/* JSON 分块写入器：转义内容攒进小缓冲、满了才发 chunk，避免整段响应
 * malloc（200 行 JSON 约 46KB，内部 RAM 峰值接近 100KB，风暴时容易失败） */
typedef struct {
    httpd_req_t *req;
    char *buf;
    size_t cap;
    size_t used;
    esp_err_t err;
} json_writer_t;

static void jw_flush(json_writer_t *w)
{
    if (w->err == ESP_OK && w->used > 0) {
        w->err = httpd_resp_send_chunk(w->req, w->buf, (ssize_t)w->used);
    }
    w->used = 0;
}

static void jw_put(json_writer_t *w, const char *data, size_t len)
{
    while (len > 0 && w->err == ESP_OK) {
        if (w->used == w->cap) {
            jw_flush(w);
            continue;
        }
        size_t take = w->cap - w->used;
        if (take > len) {
            take = len;
        }
        memcpy(w->buf + w->used, data, take);
        w->used += take;
        data += take;
        len -= take;
    }
}

/* JSON 字符串转义：引号/反斜杠加 '\'，<0x20 控制字符转 \u00XX；
 * 非 ASCII 字节按 UTF-8 原样透传（日志本体就是字节流） */
static void jw_put_escaped(json_writer_t *w, const char *s, size_t len)
{
    size_t run = 0;
    for (size_t i = 0; i < len && w->err == ESP_OK; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c >= 0x20 && c != '"' && c != '\\') {
            run++;
            continue;
        }
        if (run > 0) {
            jw_put(w, s + i - run, run);
            run = 0;
        }
        if (c == '"' || c == '\\') {
            char esc[2] = {'\\', (char)c};
            jw_put(w, esc, sizeof(esc));
        } else {
            char esc[7];
            int el = snprintf(esc, sizeof(esc), "\\u%04X", (unsigned)c);
            jw_put(w, esc, (size_t)el);
        }
    }
    if (run > 0) {
        jw_put(w, s + len - run, run);
    }
}

/* GET /api/logs/live?cursor=N：分块流式返回 seq>cursor 的行；cursor 缺省
 * 返回最新 50 行。响应 {"boot":B,"oldest":N,"lines":[...],"seq":M}：M 是本次
 * 实际返回最后一行的 seq（没返回行时是环内最后一行 next_seq-1，空环为 0）。
 * 前端把 M 作为下次请求的 cursor（仍取 seq>cursor），因此不能给 next_seq，
 * 否则每轮会跳过下一条新日志。boot 用于识别重启（seq 从 1 重数，单看 seq
 * 可能恰好等于旧 cursor）。最多覆盖整个环（256 行）。 */
static esp_err_t handle_logs_live(httpd_req_t *req)
{
    uint32_t cursor = 0;
    bool have_cursor = false;

    /* IDF 5.5 没有 httpd_req_get_url_value_str：先取 query 串再按键解析。
     * 无 query（ESP_ERR_NOT_FOUND）按缺省"最新 50 行"处理 */
    char qry[64];
    if (httpd_req_get_url_query_str(req, qry, sizeof(qry)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(qry, "cursor", val, sizeof(val)) == ESP_OK) {
            cursor = (uint32_t)strtoul(val, NULL, 10);
            have_cursor = true;
        }
    }

    if (s_lock == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "log not ready");
    }

    /* httpd 单任务，静态批缓冲既省 httpd 栈（16×232B）也不会与其他请求并发 */
    static log_ring_entry_t batch[LOG_LIVE_BATCH];

    uint32_t oldest = 0, next = 0;
    uint32_t expect = 0;   /* 下一行应具有的 seq（= 已发最后一行 + 1） */
    uint32_t resp_seq = 0; /* 响应里的 seq 字段 */

    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint32_t start = cursor;
    if (!have_cursor) {
        uint32_t avail = (s_ring.next_seq > s_ring.oldest_seq)
                             ? (s_ring.next_seq - s_ring.oldest_seq)
                             : 0;
        uint32_t want = (avail < LOG_LIVE_DEFAULT) ? avail : LOG_LIVE_DEFAULT;
        start = s_ring.next_seq - want - 1; /* copy_from 返回 seq > start 的行 */
    }
    size_t n = log_ring_copy_from(&s_ring, start, batch, LOG_LIVE_BATCH,
                                  &oldest, &next);
    xSemaphoreGive(s_lock);

    /* 没返回行时给环内最后一行 seq（next_seq-1；空环 next=1 → 0）。
     * 给 next_seq 会让前端下轮从 >next_seq 开始，跳过新出现的这一行 */
    resp_seq = (next > 0) ? next - 1 : 0;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char head[64];
    int hl = snprintf(head, sizeof(head), "{\"boot\":%lu,\"oldest\":%lu,\"lines\":[",
                      (unsigned long)s_boot, (unsigned long)oldest);
    esp_err_t err = httpd_resp_send_chunk(req, head, hl);
    if (err != ESP_OK) {
        return err;
    }

    /* httpd 单任务，静态 2KB 缓冲把多行攒进一个 chunk：128B 时一行 223B
     * 就要拆成多次 send_chunk，日志风暴时大量小包会堵住 httpd */
    static char s_live_jbuf[2048];
    json_writer_t w = {
        .req = req,
        .buf = s_live_jbuf,
        .cap = sizeof(s_live_jbuf),
        .err = ESP_OK,
    };

    size_t total = 0;
    bool have_sent = false;
    while (n > 0) {
        if (have_sent && batch[0].seq != expect) {
            /* 响应中途环回绕：与已发内容之间出现 seq 空洞。立即收尾让前端
             * 下次轮询按 oldest 提示丢行，绝不跨空洞继续发 */
            break;
        }
        for (size_t i = 0; i < n && total < LOG_LIVE_MAX; i++) {
            jw_put(&w, have_sent ? ",\"" : "\"", have_sent ? 2 : 1);
            jw_put_escaped(&w, batch[i].text, batch[i].len);
            jw_put(&w, "\"", 1);
            have_sent = true;
            expect = batch[i].seq + 1; /* 供下一批无缝衔接判断 */
            resp_seq = batch[i].seq;   /* 最后实际发行的 seq 给前端当 cursor */
            total++;
        }
        jw_flush(&w);
        if (w.err != ESP_OK) {
            return w.err;
        }
        if (n < LOG_LIVE_BATCH || total >= LOG_LIVE_MAX) {
            break;
        }

        xSemaphoreTake(s_lock, portMAX_DELAY);
        n = log_ring_copy_from(&s_ring, expect - 1, batch, LOG_LIVE_BATCH,
                               &oldest, &next);
        xSemaphoreGive(s_lock);
    }

    char tail[32];
    int tl = snprintf(tail, sizeof(tail), "],\"seq\":%lu}",
                      (unsigned long)resp_seq);
    err = httpd_resp_send_chunk(req, tail, tl);
    if (err != ESP_OK) {
        return err;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

/* GET /api/logs/list：按时间序（旧→新）返回有效日志文件。
 * 文件数典型 300+，逐项 cJSON 会占大量内部堆；这里手写分块 JSON。文件名
 * 已由 log_name_parse 白名单校验（只含 ASCII 字母数字/下划线/点），cur_file
 * 来自同一命名规则，因此无需 JSON 转义。
 * 响应 {boot, clock_synced, cur_file, uptime_s, files:[{f,size,cur,unsynced}]} */
static esp_err_t handle_logs_list(httpd_req_t *req)
{
    /* s_cur/s_cur_name 与刷盘定时器并发：锁内只拷名字与打开状态 */
    char cur_name[LOG_NAME_MAX] = "";
    bool cur_open = s_fs_ok && cur_file_snapshot(cur_name, sizeof(cur_name));

    log_list_entry_t *ents = NULL;
    size_t n = 0;
    if (s_fs_ok) {
        size_t found = count_log_files();
        if (found > LOG_MAX_FILES) {
            found = LOG_MAX_FILES;
            warn_files_cap();
        }
        if (found > 0) {
            ents = alloc_prefer_psram(found * sizeof(*ents));
            if (ents == NULL) {
                /* 分配失败：退回空 files 但仍给元数据（200），比 500 更可用 */
                ESP_LOGE(TAG, "list: no mem for %u files, return empty",
                         (unsigned)found);
            } else {
                n = fill_log_files(ents, found);
            }
        }
    }

    /* b 文件按 boot 升序在前，date 文件按日期+小时升序在后：与删除顺序
     * 一致，也符合真实时间序 */
    if (n > 1) {
        qsort(ents, n, sizeof(*ents), log_list_entry_cmp);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char jbuf[2048];
    json_writer_t w = {
        .req = req,
        .buf = jbuf,
        .cap = sizeof(jbuf),
        .err = ESP_OK,
    };
    char buf[160];
    int l = snprintf(buf, sizeof(buf),
                     "{\"boot\":%lu,\"clock_synced\":%s,\"cur_file\":\"%s\","
                     "\"uptime_s\":%llu,\"files\":[",
                     (unsigned long)s_boot, s_clock_synced ? "true" : "false",
                     cur_open ? cur_name : "",
                     (unsigned long long)uptime_seconds());
    jw_put(&w, buf, (size_t)l);

    for (size_t i = 0; i < n; i++) {
        l = snprintf(buf, sizeof(buf), "%s{\"f\":\"%s\",\"size\":%lu",
                     i > 0 ? "," : "", ents[i].name,
                     (unsigned long)ents[i].size);
        jw_put(&w, buf, (size_t)l);
        if (cur_open && strcmp(ents[i].name, cur_name) == 0) {
            static const char cur_flag[] = ",\"cur\":true";
            jw_put(&w, cur_flag, sizeof(cur_flag) - 1);
        }
        if (ents[i].is_boot) {
            l = snprintf(buf, sizeof(buf),
                         ",\"unsynced\":true,\"boot\":%lu,\"h\":%d",
                         (unsigned long)ents[i].boot, ents[i].hour);
            jw_put(&w, buf, (size_t)l);
        }
        jw_put(&w, "}", 1);
    }
    jw_put(&w, "]}", 2);
    jw_flush(&w);
    free(ents);
    if (w.err != ESP_OK) {
        return w.err;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

/* GET /api/logs/file?f=<name>：整文件原始文本流式下发（单文件 ≤128KiB，
 * 搜索/过滤放前端）。文件名经 log_name_parse 白名单校验，防路径穿越。 */
static esp_err_t handle_logs_file(httpd_req_t *req)
{
    char f[LOG_NAME_MAX];
    char qry[64];
    /* IDF 5.5 无 httpd_req_get_url_value_str：query 串 + key 两步解析 */
    if (httpd_req_get_url_query_str(req, qry, sizeof(qry)) != ESP_OK ||
        httpd_query_key_value(qry, "f", f, sizeof(f)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "f required");
    }
    log_fn_type_t type;
    int y, mo, dd, h;
    uint32_t b;
    if (!log_name_parse(f, &type, &y, &mo, &dd, &h, &b)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad name");
    }
    char path[48];
    log_path(f, path, sizeof(path));
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
    }

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    /* 带上文件名，浏览器下载得到 log_20261108_15.log 而不是 "file"。
     * httpd 只保存指针，disp 在本 handler 返回前一直有效。f 已过白名单。 */
    char disp[64];
    snprintf(disp, sizeof(disp), "attachment; filename=\"%.*s\"",
             LOG_NAME_MAX - 1, f);
    httpd_resp_set_hdr(req, "Content-Disposition", disp);

    char buf[2048];
    esp_err_t e = ESP_OK;
    size_t r;
    while ((r = fread(buf, 1, sizeof(buf), fp)) > 0) {
        e = httpd_resp_send_chunk(req, buf, r);
        if (e != ESP_OK) {
            break;
        }
    }
    fclose(fp);
    if (e == ESP_OK) {
        e = httpd_resp_send_chunk(req, NULL, 0);
    }
    return e;
}

esp_err_t bench_log_start(httpd_handle_t hd)
{
    httpd_uri_t uri;

    uri = (httpd_uri_t){
        .uri = "/api/logs/live",
        .method = HTTP_GET,
        .handler = handle_logs_live,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "live");

    uri = (httpd_uri_t){
        .uri = "/api/logs/list",
        .method = HTTP_GET,
        .handler = handle_logs_list,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "list");

    uri = (httpd_uri_t){
        .uri = "/api/logs/file",
        .method = HTTP_GET,
        .handler = handle_logs_file,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(hd, &uri), TAG, "file");

    ESP_LOGI(TAG, "log endpoints up");
    return ESP_OK;
}
