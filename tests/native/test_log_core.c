/*
 * log_core 行为测试：环形缓冲（回绕/oldest/next/截断）、文件名生成与严格
 * 解析（正反例）、轮转判定、配额删除规划（顺序/cur 保护/配额不足）。
 */
#include <string.h>

#include "unity.h"
#include "unity_runner.h"
#include "log_core.h"

void setUp(void) {}
void tearDown(void) {}

/* ---------------- 环形缓冲 ---------------- */

/* 1. 空环状态：oldest == next == 1，拷不到东西；out 传 NULL 只查状态 */
static void test_ring_empty_state(void)
{
    log_ring_entry_t slots[4];
    log_ring_t r;
    log_ring_init(&r, slots, 4);

    log_ring_entry_t out[4];
    uint32_t oldest = 999, next = 999;
    TEST_ASSERT_EQUAL_INT(0, log_ring_copy_from(&r, 0, out, 4, &oldest, &next));
    TEST_ASSERT_EQUAL_UINT32(1, oldest);
    TEST_ASSERT_EQUAL_UINT32(1, next); /* 空环时 oldest = 下一行 seq */

    /* 只要 oldest/next：out/cap 不给也行 */
    oldest = 0;
    next = 0;
    TEST_ASSERT_EQUAL_INT(0, log_ring_copy_from(&r, 0, NULL, 0, &oldest, &next));
    TEST_ASSERT_EQUAL_UINT32(1, oldest);
    TEST_ASSERT_EQUAL_UINT32(1, next);
}

/* 2. 基本 push：seq 从 1 单调递增，内容和 len 原样保留 */
static void test_ring_push_basic(void)
{
    log_ring_entry_t slots[4];
    log_ring_t r;
    log_ring_init(&r, slots, 4);

    TEST_ASSERT_EQUAL_UINT32(1, log_ring_push(&r, "hello", 5));
    TEST_ASSERT_EQUAL_UINT32(2, log_ring_push(&r, "world!", 6));
    TEST_ASSERT_EQUAL_UINT32(3, log_ring_push(&r, "", 0)); /* 空行合法 */

    TEST_ASSERT_EQUAL_UINT16(5, slots[0].len);
    TEST_ASSERT_EQUAL_STRING("hello", slots[0].text);
    TEST_ASSERT_EQUAL_UINT16(6, slots[1].len);
    TEST_ASSERT_EQUAL_STRING("world!", slots[1].text);
    TEST_ASSERT_EQUAL_UINT16(0, slots[2].len);
    TEST_ASSERT_EQUAL_STRING("", slots[2].text);

    uint32_t oldest, next;
    log_ring_entry_t out[4];
    TEST_ASSERT_EQUAL_INT(3, log_ring_copy_from(&r, 0, out, 4, &oldest, &next));
    TEST_ASSERT_EQUAL_UINT32(1, oldest);
    TEST_ASSERT_EQUAL_UINT32(4, next);
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_EQUAL_UINT32((uint32_t)(i + 1), out[i].seq);
    }
    TEST_ASSERT_EQUAL_STRING("hello", out[0].text);
    TEST_ASSERT_EQUAL_STRING("", out[2].text);
}

/* 3. push 参数错误一律返回 0，且不破坏状态 */
static void test_ring_push_invalid_args(void)
{
    log_ring_entry_t slots[4];
    log_ring_t r;
    log_ring_init(&r, slots, 4);

    TEST_ASSERT_EQUAL_UINT32(0, log_ring_push(NULL, "x", 1));
    TEST_ASSERT_EQUAL_UINT32(0, log_ring_push(&r, NULL, 0));
    TEST_ASSERT_EQUAL_UINT32(0, log_ring_push(&r, NULL, 3));

    log_ring_t uninit;
    uninit.slots = NULL;
    uninit.count = 0;
    uninit.next_seq = 1;
    uninit.oldest_seq = 1;
    TEST_ASSERT_EQUAL_UINT32(0, log_ring_push(&uninit, "x", 1));

    /* 失败后环仍可用：seq 没有被跳过 */
    TEST_ASSERT_EQUAL_UINT32(1, log_ring_push(&r, "ok", 2));
}

/* 4. 截断边界：223 恰好不截；224 与 300 都截到 LOG_LINE_MAX-1 */
static void test_ring_push_truncation_boundary(void)
{
    log_ring_entry_t slots[4];
    log_ring_t r;
    log_ring_init(&r, slots, 4);

    char big[300];
    memset(big, 'x', sizeof(big));

    /* 恰好 LOG_LINE_MAX-1 = 223 字符：完整保留 */
    memset(big, 'a', 223);
    uint32_t s1 = log_ring_push(&r, big, 223);
    TEST_ASSERT_EQUAL_UINT32(1, s1);
    TEST_ASSERT_EQUAL_UINT16(223, slots[0].len);
    TEST_ASSERT_EQUAL_UINT8('a', (uint8_t)slots[0].text[0]);
    TEST_ASSERT_EQUAL_UINT8('a', (uint8_t)slots[0].text[222]);
    TEST_ASSERT_EQUAL_UINT8('\0', (uint8_t)slots[0].text[223]);

    /* 224 字符：截断到 223，长度字段不含 '\0' */
    memset(big, 'b', 224);
    TEST_ASSERT_EQUAL_UINT32(2, log_ring_push(&r, big, 224));
    TEST_ASSERT_EQUAL_UINT16(223, slots[1].len);
    TEST_ASSERT_EQUAL_UINT8('b', (uint8_t)slots[1].text[222]);
    TEST_ASSERT_EQUAL_UINT8('\0', (uint8_t)slots[1].text[223]);

    /* 远超上限：同样截到 223 */
    memset(big, 'c', sizeof(big));
    TEST_ASSERT_EQUAL_UINT32(3, log_ring_push(&r, big, 300));
    TEST_ASSERT_EQUAL_UINT16(223, slots[2].len);
    TEST_ASSERT_EQUAL_UINT8('\0', (uint8_t)slots[2].text[223]);
}

/* 5. 回绕：4 槽压 6 行，最老两行被挤掉；oldest/next/定位正确 */
static void test_ring_wraparound_oldest_next(void)
{
    log_ring_entry_t slots[4];
    log_ring_t r;
    log_ring_init(&r, slots, 4);
    char label[8];
    for (int i = 1; i <= 6; i++) {
        label[0] = 'l';
        label[1] = (char)('0' + i);
        label[2] = '\0';
        TEST_ASSERT_EQUAL_UINT32((uint32_t)i, log_ring_push(&r, label, 2));
    }

    uint32_t oldest, next;
    log_ring_entry_t out[8];
    /* cursor=0：只能拿到现存 4 行（seq 3..6） */
    TEST_ASSERT_EQUAL_INT(4, log_ring_copy_from(&r, 0, out, 8, &oldest, &next));
    TEST_ASSERT_EQUAL_UINT32(3, oldest);
    TEST_ASSERT_EQUAL_UINT32(7, next);
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_UINT32((uint32_t)(i + 3), out[i].seq);
        TEST_ASSERT_EQUAL_INT(2, out[i].len);
        TEST_ASSERT_EQUAL_UINT8((uint8_t)('3' + i), (uint8_t)out[i].text[1]);
    }

    /* cursor 落在环内：增量只给新行 */
    TEST_ASSERT_EQUAL_INT(2, log_ring_copy_from(&r, 4, out, 8, &oldest, &next));
    TEST_ASSERT_EQUAL_UINT32(5, out[0].seq);
    TEST_ASSERT_EQUAL_UINT32(6, out[1].seq);
    TEST_ASSERT_EQUAL_INT(0, log_ring_copy_from(&r, 6, out, 8, &oldest, &next));

    /* cursor 落后被挤掉区间（丢行场景）：从现存最老行开始给，
     * 调用方用 cursor < oldest 判定丢了 cursor+1..oldest-1 */
    TEST_ASSERT_EQUAL_INT(4, log_ring_copy_from(&r, 1, out, 8, &oldest, &next));
    TEST_ASSERT_EQUAL_UINT32(3, out[0].seq);
}

/* 6. 恰好填满不回绕：一行都不丢 */
static void test_ring_exact_fill_no_drop(void)
{
    log_ring_entry_t slots[4];
    log_ring_t r;
    log_ring_init(&r, slots, 4);
    for (int i = 1; i <= 4; i++) {
        TEST_ASSERT_EQUAL_UINT32((uint32_t)i, log_ring_push(&r, "row", 3));
    }
    uint32_t oldest, next;
    log_ring_entry_t out[4];
    TEST_ASSERT_EQUAL_INT(4, log_ring_copy_from(&r, 0, out, 4, &oldest, &next));
    TEST_ASSERT_EQUAL_UINT32(1, oldest);
    TEST_ASSERT_EQUAL_UINT32(5, next);
}

/* 7. 拷贝容量截断：从最老行开始只给前 cap 行，oldest/next 仍反映全环 */
static void test_ring_copy_cap_limit(void)
{
    log_ring_entry_t slots[4];
    log_ring_t r;
    log_ring_init(&r, slots, 4);
    for (int i = 1; i <= 4; i++) {
        TEST_ASSERT_EQUAL_UINT32((uint32_t)i, log_ring_push(&r, "row", 3));
    }
    uint32_t oldest, next;
    log_ring_entry_t out[4];
    TEST_ASSERT_EQUAL_INT(2, log_ring_copy_from(&r, 0, out, 2, &oldest, &next));
    TEST_ASSERT_EQUAL_UINT32(1, oldest);
    TEST_ASSERT_EQUAL_UINT32(5, next);
    TEST_ASSERT_EQUAL_UINT32(1, out[0].seq);
    TEST_ASSERT_EQUAL_UINT32(2, out[1].seq);
}

/* ---------------- 文件名生成与解析 ---------------- */

/* 8. 日期命名生成：年月日与小时全部零填充 */
static void test_name_date_format(void)
{
    char buf[32];
    log_name_date(buf, sizeof(buf), 2026, 11, 8, 15);
    TEST_ASSERT_EQUAL_STRING("log_20261108_15.log", buf);
    log_name_date(buf, sizeof(buf), 2026, 3, 1, 5);
    TEST_ASSERT_EQUAL_STRING("log_20260301_05.log", buf);
    log_name_date(buf, sizeof(buf), 2026, 1, 2, 0);
    TEST_ASSERT_EQUAL_STRING("log_20260102_00.log", buf);
}

/* 9. 开机相对命名生成：boot 4 位、小时 3 位零填充 */
static void test_name_boot_format(void)
{
    char buf[32];
    log_name_boot(buf, sizeof(buf), 37, 5);
    TEST_ASSERT_EQUAL_STRING("log_b0037_005.log", buf);
    log_name_boot(buf, sizeof(buf), 0, 0);
    TEST_ASSERT_EQUAL_STRING("log_b0000_000.log", buf);
    log_name_boot(buf, sizeof(buf), 9999, 23);
    TEST_ASSERT_EQUAL_STRING("log_b9999_023.log", buf);
}

/* 9b. 字段饱和：超出位宽收敛到上限（不是取模），保证生成的名字永远能被
 *     本组件解析——否则 5 位/4 位字段会被解析拒绝，轮转永久失效。
 *     负数收敛到 0，日期字段不做日历合法性校验（与白名单正则一致）。 */
static void test_name_field_saturation(void)
{
    char buf[32];
    log_fn_type_t type;
    int y, mon, d, h;
    uint32_t boot;

    /* boot 上限 9999、小时上限 999 */
    log_name_boot(buf, sizeof(buf), 10000, 5);
    TEST_ASSERT_EQUAL_STRING("log_b9999_005.log", buf);
    TEST_ASSERT_TRUE(log_name_parse(buf, &type, &y, &mon, &d, &h, &boot));
    TEST_ASSERT_EQUAL_INT(LOG_FN_BOOT, type);
    TEST_ASSERT_EQUAL_UINT32(9999, boot);
    TEST_ASSERT_EQUAL_INT(5, h);

    log_name_boot(buf, sizeof(buf), 123456, 1000);
    TEST_ASSERT_EQUAL_STRING("log_b9999_999.log", buf);
    TEST_ASSERT_TRUE(log_name_parse(buf, NULL, NULL, NULL, NULL, &h, &boot));
    TEST_ASSERT_EQUAL_UINT32(9999, boot);
    TEST_ASSERT_EQUAL_INT(999, h);

    log_name_boot(buf, sizeof(buf), 37, -5); /* 负小时收敛到 0 */
    TEST_ASSERT_EQUAL_STRING("log_b0037_000.log", buf);

    /* date：year 0..9999，mon/day/hour 0..99 */
    log_name_date(buf, sizeof(buf), 10000, 12, 31, 0);
    TEST_ASSERT_EQUAL_STRING("log_99991231_00.log", buf);
    log_name_date(buf, sizeof(buf), -1, -1, -1, -1);
    TEST_ASSERT_EQUAL_STRING("log_00000000_00.log", buf);
    log_name_date(buf, sizeof(buf), 2026, 100, 100, 100);
    TEST_ASSERT_EQUAL_STRING("log_20269999_99.log", buf);

    /* 饱和值同样可解析，roundtrip 回饱和后的值 */
    log_name_boot(buf, sizeof(buf), 10000, 1000);
    TEST_ASSERT_TRUE(log_name_parse(buf, &type, &y, &mon, &d, &h, &boot));
    TEST_ASSERT_EQUAL_INT(LOG_FN_BOOT, type);
    TEST_ASSERT_EQUAL_UINT32(9999, boot);
    TEST_ASSERT_EQUAL_INT(999, h);

    log_name_date(buf, sizeof(buf), 12345, 42, 7, 88);
    TEST_ASSERT_TRUE(log_name_parse(buf, &type, &y, &mon, &d, &h, &boot));
    TEST_ASSERT_EQUAL_INT(LOG_FN_DATE, type);
    TEST_ASSERT_EQUAL_INT(9999, y);
    TEST_ASSERT_EQUAL_INT(42, mon);
    TEST_ASSERT_EQUAL_INT(7, d);
    TEST_ASSERT_EQUAL_INT(88, h);
}

/* 10. 解析正例：date 填 y/mon/d/h 且 boot=0；boot 填 boot/h 其余 0；
 *     生成→解析 roundtrip 一致 */
static void test_parse_positive(void)
{
    log_fn_type_t type;
    int y, mon, d, h;
    uint32_t boot;

    TEST_ASSERT_TRUE(log_name_parse("log_20261108_15.log", &type,
                                    &y, &mon, &d, &h, &boot));
    TEST_ASSERT_EQUAL_INT(LOG_FN_DATE, type);
    TEST_ASSERT_EQUAL_INT(2026, y);
    TEST_ASSERT_EQUAL_INT(11, mon);
    TEST_ASSERT_EQUAL_INT(8, d);
    TEST_ASSERT_EQUAL_INT(15, h);
    TEST_ASSERT_EQUAL_UINT32(0, boot);

    TEST_ASSERT_TRUE(log_name_parse("log_b0037_005.log", &type,
                                    &y, &mon, &d, &h, &boot));
    TEST_ASSERT_EQUAL_INT(LOG_FN_BOOT, type);
    TEST_ASSERT_EQUAL_INT(0, y);
    TEST_ASSERT_EQUAL_INT(0, mon);
    TEST_ASSERT_EQUAL_INT(0, d);
    TEST_ASSERT_EQUAL_INT(5, h);
    TEST_ASSERT_EQUAL_UINT32(37, boot);

    /* roundtrip */
    char buf[32];
    log_name_boot(buf, sizeof(buf), 123, 7);
    TEST_ASSERT_TRUE(log_name_parse(buf, &type, &y, &mon, &d, &h, &boot));
    TEST_ASSERT_EQUAL_INT(LOG_FN_BOOT, type);
    TEST_ASSERT_EQUAL_UINT32(123, boot);
    TEST_ASSERT_EQUAL_INT(7, h);

    log_name_date(buf, sizeof(buf), 1999, 12, 31, 0);
    TEST_ASSERT_TRUE(log_name_parse(buf, &type, &y, &mon, &d, &h, &boot));
    TEST_ASSERT_EQUAL_INT(LOG_FN_DATE, type);
    TEST_ASSERT_EQUAL_INT(1999, y);
    TEST_ASSERT_EQUAL_INT(12, mon);
    TEST_ASSERT_EQUAL_INT(31, d);
    TEST_ASSERT_EQUAL_INT(0, h);

    /* 只关心合法性/类型：其余输出指针允许 NULL */
    TEST_ASSERT_TRUE(log_name_parse("log_20261108_15.log", &type,
                                    NULL, NULL, NULL, NULL, NULL));
    TEST_ASSERT_EQUAL_INT(LOG_FN_DATE, type);
}

/* 11. 解析反例：整体匹配，长度/大小写/字符集/多余字符都不放过；
 *     失败时不改写任何输出 */
static void test_parse_negative(void)
{
    static const char *bad[] = {
        "",                                /* 空串 */
        "log",                             /* 残缺 */
        "log_20261108_15.txt",             /* 后缀不对 */
        "log_20261108_155.log",            /* 多一个字符 */
        "log_2026110_15.log",              /* 少一个字符 */
        "log_2026110815.log",              /* 缺下划线 */
        "LOG_20261108_15.log",             /* 大小写 */
        "log_202611a8_15.log",             /* 数字位混字母 */
        "log_20261108-15.log",             /* 分隔符不对 */
        "xlog_20261108_15.log",            /* 前缀不对 */
        "log_20261108_15.logx",            /* 尾巴多余 */
        " log_20261108_15.log",            /* 前导空格 */
        "log_20261108_15.log ",            /* 尾随空格 */
        "/log_20261108_15.log",            /* 路径穿越 */
        "../log_20261108_15.log",
        "log_b037_005.log",                /* boot 少一位 */
        "log_b00370_05.log",               /* boot 多一位 */
        "log_b0037_05.log",                /* 小时少一位 */
        "log_b0037_0055.log",              /* 小时多一位 */
        "log_b0037_005.txt",               /* 后缀不对 */
        "log_b00x7_005.log",               /* 数字位混字母 */
        "log_b0037_005.log.bak",           /* 尾巴多余 */
        "log_c0037_005.log",               /* b 前缀变 c */
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        log_fn_type_t type = LOG_FN_DATE;
        int y = -1, mon = -1, d = -1, h = -1;
        uint32_t boot = 99;
        TEST_ASSERT_FALSE_MESSAGE(log_name_parse(bad[i], &type, &y, &mon, &d, &h, &boot),
                                  bad[i]);
        /* 失败不改写输出 */
        TEST_ASSERT_EQUAL_INT(-1, y);
        TEST_ASSERT_EQUAL_INT(-1, mon);
        TEST_ASSERT_EQUAL_INT(-1, d);
        TEST_ASSERT_EQUAL_INT(-1, h);
        TEST_ASSERT_EQUAL_UINT32(99, boot);
    }
    TEST_ASSERT_FALSE(log_name_parse(NULL, NULL, NULL, NULL, NULL, NULL, NULL));
}

/* ---------------- 轮转判定 ---------------- */

/* 构造 2026-11-08 的本地时间。日期型轮转做全年月日时比较，helper 必须给全
 * 日期，否则既有「同小时不轮转」用例会因为日期不匹配而失效。 */
static struct tm make_tm(int hour)
{
    struct tm t;
    memset(&t, 0, sizeof(t));
    t.tm_year = 2026 - 1900;
    t.tm_mon = 11 - 1;
    t.tm_mday = 8;
    t.tm_hour = hour;
    return t;
}

/* 12. 尚无文件必须开新文件；date 按本地小时；boot 按开机小时；不认识不动 */
static void test_should_rotate(void)
{
    struct tm now15 = make_tm(15);
    struct tm now00 = make_tm(0);

    TEST_ASSERT_TRUE(log_should_rotate(NULL, &now15, 3));
    TEST_ASSERT_TRUE(log_should_rotate("", &now15, 3));

    /* date：跨小时轮转；同小时不轮转 */
    TEST_ASSERT_FALSE(log_should_rotate("log_20261108_15.log", &now15, 0));
    TEST_ASSERT_TRUE(log_should_rotate("log_20261108_15.log", &now00, 0));
    /* 跨天场景：23 时文件在 0 点必然轮转（小时值变化即覆盖） */
    TEST_ASSERT_TRUE(log_should_rotate("log_20261108_23.log", &now00, 0));

    /* boot：按开机小时 */
    TEST_ASSERT_FALSE(log_should_rotate("log_b0037_005.log", &now15, 5));
    TEST_ASSERT_TRUE(log_should_rotate("log_b0037_005.log", &now15, 6));

    /* 无法解析的文件名：不动它 */
    TEST_ASSERT_FALSE(log_should_rotate("junk.log", &now15, 5));
    TEST_ASSERT_FALSE(log_should_rotate("log_20261108_155.log", &now15, 5));
    TEST_ASSERT_FALSE(log_should_rotate("log_20261108_15.log", NULL, 5));
}

/* 12b. 日期文件按完整 年/月/日/时 比较：时间恰好跳变 24 小时（同小时不同天）
 *      必须轮转，否则会一直往昨天打开的文件里追加 */
static void test_should_rotate_date_full_compare(void)
{
    struct tm t = make_tm(15); /* 2026-11-08 15:59:59 */

    /* 同日同小时、仅分秒不同：不轮转 */
    t.tm_min = 59;
    t.tm_sec = 59;
    TEST_ASSERT_FALSE(log_should_rotate("log_20261108_15.log", &t, 0));

    /* 次日同小时：必须轮转（旧实现只比小时会漏掉这种 24 小时跳变） */
    t.tm_mday = 9;
    TEST_ASSERT_TRUE(log_should_rotate("log_20261108_15.log", &t, 0));

    /* 同日不同月、同月日不同年：同样轮转 */
    t.tm_mday = 8;
    t.tm_mon = 12 - 1;
    TEST_ASSERT_TRUE(log_should_rotate("log_20261108_15.log", &t, 0));
    t.tm_mon = 11 - 1;
    t.tm_year = 2027 - 1900;
    TEST_ASSERT_TRUE(log_should_rotate("log_20261108_15.log", &t, 0));
}

/* 12c. boot 分支的小时也按生成侧同样的饱和规则比较：uptime_hour 超过 999
 *      时文件名里的小时最多是 999，若拿原始值比较会永远不相等，导致每个
 *      刷盘周期都把同一个文件关掉再开（无限轮转） */
static void test_should_rotate_boot_hour_saturation(void)
{
    struct tm now = make_tm(15);

    TEST_ASSERT_FALSE(log_should_rotate("log_b0037_999.log", &now, 1000));
    TEST_ASSERT_FALSE(log_should_rotate("log_b0037_999.log", &now, 5000));
    TEST_ASSERT_FALSE(log_should_rotate("log_b0037_999.log", &now, 999));
    TEST_ASSERT_TRUE(log_should_rotate("log_b0037_999.log", &now, 998));

    /* 负数 uptime 按 0 比较（与生成侧负数收敛到 0 一致） */
    TEST_ASSERT_FALSE(log_should_rotate("log_b0037_000.log", &now, -5));
    TEST_ASSERT_TRUE(log_should_rotate("log_b0037_000.log", &now, 1));
}

/* ---------------- 配额删除规划 ---------------- */

/* 13. 删除顺序：b 按 boot 升序在前（同 boot 按文件名），date 按文件名在后；
 *     释放量到达配额即停，不超额多删；cur 永不入选 */
static void test_plan_deletes_order_and_stop(void)
{
    const char *b0001 = "log_b0001_003.log";
    const char *b0005a = "log_b0005_003.log";
    const char *b0005b = "log_b0005_010.log";
    const char *date07 = "log_20261107_10.log";
    const char *date08 = "log_20261108_09.log";
    const char *cur = "log_20261108_15.log";

    log_file_info_t files[] = {
        { b0001, 100 },
        { b0005b, 200 },
        { b0005a, 300 },
        { date07, 400 },
        { date08, 500 },
        { cur,    600 },
    }; /* 总 2100 */
    const char *plan[6] = { 0 };
    size_t n = log_plan_deletes(files, 6, 1200, cur, plan, 6);
    /* need = 900：b0001(100) + b0005_003(300) + b0005_010(200) = 600，
     * 再删 date07(400) 到 1000 ≥ 900 即停，date08 留下 */
    TEST_ASSERT_EQUAL_INT(4, n);
    TEST_ASSERT_EQUAL_STRING(b0001, plan[0]);
    TEST_ASSERT_EQUAL_STRING(b0005a, plan[1]); /* 同 boot 按文件名序 */
    TEST_ASSERT_EQUAL_STRING(b0005b, plan[2]);
    TEST_ASSERT_EQUAL_STRING(date07, plan[3]);
    for (size_t i = 0; i < n; i++) {
        TEST_ASSERT_NOT_EQUAL(plan[i], cur);
    }
}

/* 14. cur 保护：排在删除顺序最前面的 cur 被跳过；cur 一个文件时无可删返回 0；
 *     无法解析的文件跳过；配额内 / 无文件返回 0 */
static void test_plan_deletes_cur_and_edge(void)
{
    const char *b0001 = "log_b0001_003.log";
    const char *date08 = "log_20261108_09.log";
    const char *cur = "log_20261108_15.log";

    /* cur 是删除顺序第一个：跳过它删下一个，尽力达到配额 */
    log_file_info_t f1[] = {
        { b0001, 100 },
        { date08, 500 },
        { cur,    600 },
    }; /* 总 1200，quota 1000 → need 200；b0001 是 cur 跳过，删 date08 */
    const char *plan[3] = { 0 };
    TEST_ASSERT_EQUAL_INT(1, log_plan_deletes(f1, 3, 1000, b0001, plan, 3));
    TEST_ASSERT_EQUAL_STRING(date08, plan[0]);

    /* 只剩 cur：无可删 */
    log_file_info_t f2[] = { { cur, 600 } };
    TEST_ASSERT_EQUAL_INT(0, log_plan_deletes(f2, 1, 100, cur, plan, 3));

    /* 总量在配额内：不删 */
    log_file_info_t f3[] = { { b0001, 100 }, { date08, 500 } };
    TEST_ASSERT_EQUAL_INT(0, log_plan_deletes(f3, 2, 600, NULL, plan, 3));
    /* 总 600 超配额 599 一个字节：也要删，按顺序删 b0001(100) 后 500 ≤ 599 即停 */
    TEST_ASSERT_EQUAL_INT(1, log_plan_deletes(f3, 2, 599, NULL, plan, 3));
    TEST_ASSERT_EQUAL_STRING(b0001, plan[0]);

    /* 不可解析的文件无法排序：跳过，只删可解析的 */
    log_file_info_t f4[] = {
        { "junk.log", 1000 },
        { b0001,      100 },
    };
    TEST_ASSERT_EQUAL_INT(1, log_plan_deletes(f4, 2, 500, NULL, plan, 3));
    TEST_ASSERT_EQUAL_STRING(b0001, plan[0]);

    /* 参数退化 */
    TEST_ASSERT_EQUAL_INT(0, log_plan_deletes(NULL, 0, 0, NULL, plan, 3));
    TEST_ASSERT_EQUAL_INT(0, log_plan_deletes(f3, 0, 0, NULL, plan, 3));
    TEST_ASSERT_EQUAL_INT(0, log_plan_deletes(f3, 2, 0, NULL, NULL, 3));
    TEST_ASSERT_EQUAL_INT(0, log_plan_deletes(f3, 2, 0, NULL, plan, 0));
}

/* 15. cap 截断：需要删 3 个但只给 2 个槽位，返回 2 且按删除顺序取前两个 */
static void test_plan_deletes_cap_limit(void)
{
    const char *b0001 = "log_b0001_003.log";
    const char *b0002 = "log_b0002_004.log";
    const char *b0003 = "log_b0003_005.log";
    const char *date09 = "log_20261108_09.log";

    log_file_info_t files[] = {
        { b0001, 100 },
        { b0002, 100 },
        { b0003, 100 },
        { date09, 100 },
    }; /* 总 400，quota 100 → need 300 = 3 个文件 */
    const char *plan[3] = { 0 };
    TEST_ASSERT_EQUAL_INT(3, log_plan_deletes(files, 4, 100, NULL, plan, 3));
    TEST_ASSERT_EQUAL_STRING(b0001, plan[0]);
    TEST_ASSERT_EQUAL_STRING(b0002, plan[1]);
    TEST_ASSERT_EQUAL_STRING(b0003, plan[2]);

    const char *plan2[2] = { 0 };
    TEST_ASSERT_EQUAL_INT(2, log_plan_deletes(files, 4, 100, NULL, plan2, 2));
    TEST_ASSERT_EQUAL_STRING(b0001, plan2[0]);
    TEST_ASSERT_EQUAL_STRING(b0002, plan2[1]);
}

NATIVE_TEST_MAIN(
    UnityDefaultTestRun(test_ring_empty_state, "test_ring_empty_state", __LINE__);
    UnityDefaultTestRun(test_ring_push_basic, "test_ring_push_basic", __LINE__);
    UnityDefaultTestRun(test_ring_push_invalid_args, "test_ring_push_invalid_args", __LINE__);
    UnityDefaultTestRun(test_ring_push_truncation_boundary, "test_ring_push_truncation_boundary", __LINE__);
    UnityDefaultTestRun(test_ring_wraparound_oldest_next, "test_ring_wraparound_oldest_next", __LINE__);
    UnityDefaultTestRun(test_ring_exact_fill_no_drop, "test_ring_exact_fill_no_drop", __LINE__);
    UnityDefaultTestRun(test_ring_copy_cap_limit, "test_ring_copy_cap_limit", __LINE__);
    UnityDefaultTestRun(test_name_date_format, "test_name_date_format", __LINE__);
    UnityDefaultTestRun(test_name_boot_format, "test_name_boot_format", __LINE__);
    UnityDefaultTestRun(test_name_field_saturation, "test_name_field_saturation", __LINE__);
    UnityDefaultTestRun(test_parse_positive, "test_parse_positive", __LINE__);
    UnityDefaultTestRun(test_parse_negative, "test_parse_negative", __LINE__);
    UnityDefaultTestRun(test_should_rotate, "test_should_rotate", __LINE__);
    UnityDefaultTestRun(test_should_rotate_date_full_compare, "test_should_rotate_date_full_compare", __LINE__);
    UnityDefaultTestRun(test_should_rotate_boot_hour_saturation, "test_should_rotate_boot_hour_saturation", __LINE__);
    UnityDefaultTestRun(test_plan_deletes_order_and_stop, "test_plan_deletes_order_and_stop", __LINE__);
    UnityDefaultTestRun(test_plan_deletes_cur_and_edge, "test_plan_deletes_cur_and_edge", __LINE__);
    UnityDefaultTestRun(test_plan_deletes_cap_limit, "test_plan_deletes_cap_limit", __LINE__);
)
