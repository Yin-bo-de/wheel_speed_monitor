/* telem_frame 实现：见头文件注释。
 *
 * 不引 cJSON：帧是自家 render 出来的机器生成 JSON，结构固定（对象/数组/
 * 数字/布尔/短字符串），用一个定向的极简扫描器就够了，且不给宿主机测试
 * 增加第三方依赖。扫描器只认目标字段、跳过不认识的值，天然容忍字段增减。 */
#include <stdlib.h>
#include <string.h>

#include "telem_frame.h"

static const char *skip_ws(const char *p, const char *end)
{
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) {
        p++;
    }
    return p;
}

/* p 指向字符串起始引号，返回闭引号之后的位置 */
static const char *skip_string(const char *p, const char *end)
{
    p++;
    while (p < end) {
        if (*p == '\\') {
            p += 2;
            continue;
        }
        if (*p == '"') {
            return p + 1;
        }
        p++;
    }
    return NULL;
}

/* 跳过一个 JSON 值，返回值之后的位置；结构非法返回 NULL */
static const char *skip_value(const char *p, const char *end)
{
    p = skip_ws(p, end);
    if (p >= end) {
        return NULL;
    }
    if (*p == '"') {
        return skip_string(p, end);
    }
    if (*p == '{' || *p == '[') {
        char open = *p;
        char close = (*p == '{') ? '}' : ']';
        int depth = 0;
        while (p < end) {
            if (*p == '"') {
                p = skip_string(p, end);
                if (p == NULL) {
                    return NULL;
                }
                continue;
            }
            if (*p == open) {
                depth++;
            } else if (*p == close) {
                depth--;
                if (depth == 0) {
                    return p + 1;
                }
            }
            p++;
        }
        return NULL;
    }
    /* 标量：数字 / true / false / null，吃到分隔符为止 */
    while (p < end && *p != ',' && *p != '}' && *p != ']' &&
           *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r') {
        p++;
    }
    return p;
}

/* obj 指向 '{'，在对象顶层找 key，返回值起始位置；找不到返回 NULL。
 * 只看顶层：嵌套对象里的同名 key 会被 skip_value 跳掉。 */
static const char *find_key(const char *obj, const char *end, const char *key)
{
    if (obj == NULL || obj >= end || *obj != '{') {
        return NULL;
    }
    size_t klen = strlen(key);
    const char *p = skip_ws(obj + 1, end);
    while (p < end && *p != '}') {
        if (*p != '"') {
            return NULL;
        }
        const char *ks = p + 1;
        p = skip_string(p, end);
        if (p == NULL) {
            return NULL;
        }
        size_t len = (size_t)(p - 1 - ks);
        p = skip_ws(p, end);
        if (p >= end || *p != ':') {
            return NULL;
        }
        p = skip_ws(p + 1, end);
        if (p >= end) {
            return NULL;
        }
        if (len == klen && strncmp(ks, key, klen) == 0) {
            return p;
        }
        p = skip_value(p, end);
        if (p == NULL) {
            return NULL;
        }
        p = skip_ws(p, end);
        if (p < end && *p == ',') {
            p = skip_ws(p + 1, end);
        }
    }
    return NULL;
}

/* 数组遍历状态：arr 指向 '[' */
typedef struct {
    const char *next;
    const char *end;
    bool started;
} arr_walk_t;

static bool arr_walk_init(arr_walk_t *w, const char *arr, const char *end)
{
    if (arr == NULL || arr >= end || *arr != '[') {
        return false;
    }
    w->next = arr + 1;
    w->end = end;
    w->started = false;
    return true;
}

static bool arr_walk_next(arr_walk_t *w, const char **elem, const char **elem_end)
{
    const char *p = skip_ws(w->next, w->end);
    if (p >= w->end || *p == ']') {
        return false;
    }
    if (w->started) {
        if (*p != ',') {
            return false;
        }
        p = skip_ws(p + 1, w->end);
        if (p >= w->end || *p == ']') {
            return false;
        }
    }
    w->started = true;
    *elem = p;
    p = skip_value(p, w->end);
    if (p == NULL) {
        return false;
    }
    *elem_end = p;
    w->next = p;
    return true;
}

static bool get_string(const char *v, const char *end, char *out, size_t cap)
{
    if (v == NULL || v >= end || *v != '"' || cap == 0) {
        return false;
    }
    const char *p = v + 1;
    size_t n = 0;
    while (p < end && *p != '"') {
        char c = *p;
        if (c == '\\') {
            p++;
            if (p >= end) {
                return false;
            }
            c = *p;
            if (c == 'n') {
                c = '\n';
            } else if (c == 't') {
                c = '\t';
            }
        }
        if (n + 1 < cap) {
            out[n++] = c;
        }
        p++;
    }
    if (p >= end) {
        return false;
    }
    out[n] = '\0';
    return true;
}

static bool get_number(const char *v, double *out)
{
    if (v == NULL) {
        return false;
    }
    char *endp = NULL;
    double d = strtod(v, &endp);
    if (endp == v) {
        return false;
    }
    *out = d;
    return true;
}

static bool get_bool(const char *v, bool *out)
{
    if (v == NULL) {
        return false;
    }
    if (strncmp(v, "true", 4) == 0) {
        *out = true;
        return true;
    }
    if (strncmp(v, "false", 5) == 0) {
        *out = false;
        return true;
    }
    return false;
}

static bool str_eq(const char *v, const char *end, const char *want)
{
    char buf[32];
    return get_string(v, end, buf, sizeof(buf)) && strcmp(buf, want) == 0;
}

telem_phase_t telem_phase_from_str(const char *s)
{
    if (s == NULL) {
        return TELEM_PHASE_UNKNOWN;
    }
    if (strcmp(s, "idle") == 0) {
        return TELEM_PHASE_IDLE;
    }
    if (strcmp(s, "pending_sync") == 0) {
        return TELEM_PHASE_PENDING_SYNC;
    }
    if (strcmp(s, "probe") == 0) {
        return TELEM_PHASE_PROBE;
    }
    if (strcmp(s, "locked") == 0) {
        return TELEM_PHASE_LOCKED;
    }
    return TELEM_PHASE_UNKNOWN;
}

/* wheel_speed 块：wheels[{i,en,rpm}] */
static void parse_wheel_block(const char *blk, const char *blk_end,
                              telem_frame_view_t *out)
{
    const char *arr = find_key(blk, blk_end, "wheels");
    arr_walk_t w;
    if (!arr_walk_init(&w, arr, blk_end)) {
        return;
    }
    const char *el, *el_end;
    while (arr_walk_next(&w, &el, &el_end)) {
        double idx = -1;
        const char *v = find_key(el, el_end, "i");
        if (v != NULL) {
            (void)get_number(v, &idx);
        }
        if (idx < 0 || idx >= TELEM_WHEEL_COUNT) {
            continue;
        }
        int i = (int)idx;
        bool b;
        v = find_key(el, el_end, "en");
        if (v != NULL && get_bool(v, &b)) {
            out->wheel_en[i] = b;
        }
        double rpm;
        v = find_key(el, el_end, "rpm");
        if (v != NULL && get_number(v, &rpm)) {
            out->wheel_rpm[i] = (float)rpm;
        }
    }
    out->has_wheel = true;
}

/* 两档落点 [a,b]（cfg.unlock_us / cfg.lock_us），供指令落点判定 */
static void parse_us_pair(const char *obj, const char *end, const char *key,
                          uint16_t out[TELEM_DIFF_COUNT])
{
    const char *arr = find_key(obj, end, key);
    arr_walk_t w;
    if (!arr_walk_init(&w, arr, end)) {
        return;
    }
    const char *el, *el_end;
    int i = 0;
    while (i < TELEM_DIFF_COUNT && arr_walk_next(&w, &el, &el_end)) {
        double d;
        if (get_number(el, &d)) {
            out[i] = (uint16_t)d;
        }
        i++;
    }
}

/* servo 块：顶层 mode（生效模式）+ servos[{i,en,phase,target_us}] + cfg 两档落点 */
static void parse_servo_block(const char *blk, const char *blk_end,
                              telem_frame_view_t *out)
{
    char mode[16];
    const char *v = find_key(blk, blk_end, "mode");
    if (v != NULL && get_string(v, blk_end, mode, sizeof(mode))) {
        out->mode_auto = (strcmp(mode, "auto") == 0);
        out->has_mode = true;
    }
    const char *arr = find_key(blk, blk_end, "servos");
    arr_walk_t w;
    if (!arr_walk_init(&w, arr, blk_end)) {
        return;
    }
    const char *el, *el_end;
    while (arr_walk_next(&w, &el, &el_end)) {
        double idx = -1;
        v = find_key(el, el_end, "i");
        if (v != NULL) {
            (void)get_number(v, &idx);
        }
        if (idx < 0 || idx >= TELEM_DIFF_COUNT) {
            continue;
        }
        int i = (int)idx;
        bool b;
        v = find_key(el, el_end, "en");
        if (v != NULL && get_bool(v, &b)) {
            out->phase_en[i] = b;
        }
        char phase[16];
        v = find_key(el, el_end, "phase");
        if (v != NULL && get_string(v, el_end, phase, sizeof(phase))) {
            out->phase[i] = telem_phase_from_str(phase);
        }
        double us;
        v = find_key(el, el_end, "target_us");
        if (v != NULL && get_number(v, &us)) {
            out->diff_target_us[i] = (uint16_t)us;
        }
    }
    const char *cfgv = find_key(blk, blk_end, "cfg");
    if (cfgv != NULL) {
        parse_us_pair(cfgv, blk_end, "unlock_us", out->diff_unlock_us);
        parse_us_pair(cfgv, blk_end, "lock_us", out->diff_lock_us);
    }
    out->has_servo_cmd = true;
}

bool telem_frame_parse(const char *json, size_t len, telem_frame_view_t *out)
{
    if (json == NULL || out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));

    const char *end = json + len;
    const char *arr = find_key(json, end, "collectors");
    arr_walk_t w;
    if (!arr_walk_init(&w, arr, end)) {
        return false;
    }
    const char *el, *el_end;
    while (arr_walk_next(&w, &el, &el_end)) {
        const char *v = find_key(el, el_end, "type");
        if (v == NULL) {
            continue;
        }
        if (str_eq(v, el_end, "wheel_speed")) {
            parse_wheel_block(el, el_end, out);
        } else if (str_eq(v, el_end, "servo")) {
            parse_servo_block(el, el_end, out);
        }
    }
    return out->has_wheel || out->has_mode;
}
