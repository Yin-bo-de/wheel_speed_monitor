/*
 * log_core 实现：环形缓冲 / 文件名生成与解析 / 轮转判定 / 配额删除规划。
 * 纯 C，无 IDF 依赖；时间与文件系统由上层组装（见 docs/LOGGING_DESIGN.md 5 节）。
 */
#include "log_core.h"

#include <stdio.h>
#include <string.h>

/* ---------------- 环形缓冲 ---------------- */

void log_ring_init(log_ring_t *r, log_ring_entry_t *slots, size_t count)
{
    if (r == NULL) {
        return;
    }
    r->slots = slots;
    r->count = count;
    r->next_seq = 1;
    r->oldest_seq = 1; /* 空环：oldest == next */
}

uint32_t log_ring_push(log_ring_t *r, const char *line, size_t len)
{
    if (r == NULL || r->slots == NULL || r->count == 0 || line == NULL) {
        return 0;
    }
    /* 截断到 LOG_LINE_MAX-1，留 1 字节给 '\0'，保证 text 永远可当 C 字符串读 */
    size_t n = (len < LOG_LINE_MAX - 1) ? len : LOG_LINE_MAX - 1;

    /* seq 为 s 的行固定落在 slots[(s-1) % count]；环满时这里写的正是
     * oldest 那一行的槽位，写完推进 oldest_seq 即完成一次挤掉 */
    log_ring_entry_t *e = &r->slots[(r->next_seq - 1) % r->count];
    memcpy(e->text, line, n);
    e->text[n] = '\0';
    e->len = (uint16_t)n;
    e->seq = r->next_seq++;

    if (r->next_seq - r->oldest_seq > r->count) {
        r->oldest_seq = r->next_seq - r->count;
    }
    return e->seq;
}

size_t log_ring_copy_from(const log_ring_t *r, uint32_t cursor,
                          log_ring_entry_t *out, size_t cap,
                          uint32_t *oldest, uint32_t *next)
{
    if (r == NULL || r->slots == NULL || r->count == 0) {
        return 0;
    }
    if (oldest != NULL) {
        *oldest = r->oldest_seq;
    }
    if (next != NULL) {
        *next = r->next_seq;
    }
    if (out == NULL || cap == 0) {
        return 0;
    }

    /* cursor 在最老行之前（丢行场景）时从现存最老行开始给 */
    uint32_t start = (cursor + 1 > r->oldest_seq) ? cursor + 1 : r->oldest_seq;
    size_t n = 0;
    for (uint32_t s = start; s < r->next_seq && n < cap; s++) {
        out[n++] = r->slots[(s - 1) % r->count];
    }
    return n;
}

/* ---------------- 文件名生成与解析 ---------------- */

/* 字段值收敛到 [lo, hi]。饱和而非取模：取模会把大值折成小值造成时间序错乱，
 * 饱和至少保持单调，且位宽固定、生成的名字永远能通过 log_name_parse。 */
static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void log_name_date(char *buf, size_t len, int year, int mon, int day, int hour)
{
    if (buf == NULL || len == 0) {
        return;
    }
    /* 不做日历合法性校验，与白名单正则的语义一致；只保证 2/4 位位宽 */
    snprintf(buf, len, "log_%04d%02d%02d_%02d.log",
             clamp_int(year, 0, 9999), clamp_int(mon, 0, 99),
             clamp_int(day, 0, 99), clamp_int(hour, 0, 99));
}

void log_name_boot(char *buf, size_t len, uint32_t boot, int hour)
{
    if (buf == NULL || len == 0) {
        return;
    }
    /* boot 超过 9999 或 hour 超过 999 若不饱和会生成更长的字段，解析拒绝后
     * 轮转判定不认识当前文件，将永久不再轮转 */
    unsigned b = (boot > 9999u) ? 9999u : (unsigned)boot;
    snprintf(buf, len, "log_b%04u_%03d.log", b, clamp_int(hour, 0, 999));
}

/* s[0..n) 是否全为十进制数字 */
static bool all_digits(const char *s, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (s[i] < '0' || s[i] > '9') {
            return false;
        }
    }
    return true;
}

/* s[0..n) 的十进制值（n ≤ 4，无溢出问题） */
static int digits_val(const char *s, size_t n)
{
    int v = 0;
    for (size_t i = 0; i < n; i++) {
        v = v * 10 + (s[i] - '0');
    }
    return v;
}

bool log_name_parse(const char *name, log_fn_type_t *type,
                    int *y, int *mon, int *d, int *h, uint32_t *boot)
{
    if (name == NULL) {
        return false;
    }
    size_t slen = strlen(name);

    /* 日期型：log_ + 8 位 + _ + 2 位 + .log = 19 字符，整体匹配 */
    if (slen == 19 &&
        memcmp(name, "log_", 4) == 0 &&
        all_digits(name + 4, 8) &&
        name[12] == '_' &&
        all_digits(name + 13, 2) &&
        memcmp(name + 15, ".log", 4) == 0) {
        if (type != NULL) {
            *type = LOG_FN_DATE;
        }
        if (y != NULL) {
            *y = digits_val(name + 4, 4);
        }
        if (mon != NULL) {
            *mon = digits_val(name + 8, 2);
        }
        if (d != NULL) {
            *d = digits_val(name + 10, 2);
        }
        if (h != NULL) {
            *h = digits_val(name + 13, 2);
        }
        if (boot != NULL) {
            *boot = 0;
        }
        return true;
    }

    /* 开机相对型：log_b + 4 位 + _ + 3 位 + .log = 17 字符，整体匹配 */
    if (slen == 17 &&
        memcmp(name, "log_b", 5) == 0 &&
        all_digits(name + 5, 4) &&
        name[9] == '_' &&
        all_digits(name + 10, 3) &&
        memcmp(name + 13, ".log", 4) == 0) {
        if (type != NULL) {
            *type = LOG_FN_BOOT;
        }
        if (y != NULL) {
            *y = 0;
        }
        if (mon != NULL) {
            *mon = 0;
        }
        if (d != NULL) {
            *d = 0;
        }
        if (h != NULL) {
            *h = digits_val(name + 10, 3);
        }
        if (boot != NULL) {
            *boot = (uint32_t)digits_val(name + 5, 4);
        }
        return true;
    }
    return false;
}

/* ---------------- 轮转判定 ---------------- */

bool log_should_rotate(const char *open_name, const struct tm *now_lt, int uptime_hour)
{
    if (open_name == NULL || open_name[0] == '\0') {
        return true; /* 尚无文件，必须先开一个 */
    }
    log_fn_type_t type;
    int y, mon, d, h;
    uint32_t boot;
    if (!log_name_parse(open_name, &type, &y, &mon, &d, &h, &boot)) {
        return false; /* 不认识的文件不动它 */
    }
    if (type == LOG_FN_DATE) {
        if (now_lt == NULL) {
            return false;
        }
        /* 完整比较年月日时：只比小时会漏掉系统时间恰好跳变 24 小时
         * （同小时不同天）的场景，导致一直往旧文件里追加 */
        return y != now_lt->tm_year + 1900 ||
               mon != now_lt->tm_mon + 1 ||
               d != now_lt->tm_mday ||
               h != now_lt->tm_hour;
    }
    /* 生成侧把 hour 饱和到 999，这里必须用同样的饱和值比较：否则 uptime_hour
     * 超过 999 后永远不相等，每个刷盘周期都会关/开同一个文件（无限轮转） */
    return h != clamp_int(uptime_hour, 0, 999);
}

/* ---------------- 配额删除规划 ---------------- */

size_t log_plan_deletes(const log_file_info_t *files, size_t n, size_t quota,
                        const char *cur_name, const char **out_names, size_t cap)
{
    if (files == NULL || n == 0 || out_names == NULL || cap == 0) {
        return 0;
    }

    size_t total = 0;
    for (size_t i = 0; i < n; i++) {
        total += files[i].size;
    }
    if (total <= quota) {
        return 0; /* 已在配额内 */
    }
    /* 需要释放的字节数；按固定删除顺序累积到这么多即停，不超额多删 */
    size_t need = total - quota;

    size_t picked = 0;
    while (need > 0 && picked < cap) {
        /* 在未入选文件里找删除顺序中的下一个（选择排序式逐个挑选，
         * 文件数量级 ≤ 百个，无需引入排序缓冲） */
        const log_file_info_t *best = NULL;
        log_fn_type_t best_type = LOG_FN_DATE;
        uint32_t best_boot = 0;
        for (size_t i = 0; i < n; i++) {
            const log_file_info_t *f = &files[i];
            if (f->name == NULL) {
                continue;
            }
            if (cur_name != NULL && strcmp(f->name, cur_name) == 0) {
                continue; /* 当前正在写的文件永不删 */
            }
            bool already = false;
            for (size_t k = 0; k < picked; k++) {
                if (out_names[k] == f->name) {
                    already = true;
                    break;
                }
            }
            if (already) {
                continue;
            }
            log_fn_type_t type;
            int y, mon, d, h;
            uint32_t boot;
            if (!log_name_parse(f->name, &type, &y, &mon, &d, &h, &boot)) {
                continue; /* 无法排序的文件不纳入删除计划 */
            }
            bool better;
            if (best == NULL) {
                better = true;
            } else if (type != best_type) {
                better = (type == LOG_FN_BOOT); /* b 文件整体排在 date 前 */
            } else if (type == LOG_FN_BOOT) {
                better = (boot < best_boot) ||
                         (boot == best_boot && strcmp(f->name, best->name) < 0);
            } else {
                better = strcmp(f->name, best->name) < 0;
            }
            if (better) {
                best = f;
                best_type = type;
                best_boot = boot;
            }
        }
        if (best == NULL) {
            break; /* 剩下的全被 cur 保护或不可解析 */
        }
        out_names[picked++] = best->name;
        need = (best->size >= need) ? 0 : need - best->size;
    }
    return picked;
}
