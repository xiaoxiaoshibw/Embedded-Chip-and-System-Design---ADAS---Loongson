/**
 * ESP32 ADAS 主机端单元测试
 *
 * 测试纯逻辑函数（CRC-8、解析、限幅），不依赖 FreeRTOS 或 ESP-IDF。
 * 编译：gcc -o test_main test_main.c -lm && ./test_main
 * 或在 ESP-IDF 中通过 unity 框架运行。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>

/* ── 从 main.c 提取的纯函数（static 去掉以便独立编译） ── */

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float parsef(const char *s) {
    if (!s || !*s) return 0.0f;
    char *endp;
    float v = strtof(s, &endp);
    if (endp == s) return 0.0f;
    return v;
}

static bool parse_tag_float(const char *line, const char *tag, float *out) {
    const char *p = strstr(line, tag);
    if (!p) return false;
    p += strlen(tag);
    char *endp;
    float v = strtof(p, &endp);
    if (endp == p) return false;
    *out = v;
    return true;
}

/**
 * CRC-8/MAXIM (polynomial 0x31, init 0x00)
 * 与 Jetson serial_protocol.py 中的 _crc8_dallas() 算法完全一致。
 */
static uint8_t crc8_dallas(const char *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint8_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ── 测试框架（极简 assert 风格） ── */

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) \
    do { tests_run++; printf("  %-50s", name); } while(0)

#define PASS() \
    do { tests_passed++; printf("PASS\n"); } while(0)

#define FAIL(msg) \
    do { printf("FAIL: %s\n", msg); } while(0)

#define ASSERT_FLOAT_EQ(a, b, eps) \
    do { if (fabsf((a) - (b)) > (eps)) { \
        char buf[128]; snprintf(buf, sizeof(buf), \
            "expected %.6f, got %.6f", (double)(b), (double)(a)); \
        FAIL(buf); return; } } while(0)

#define ASSERT_UINT8_EQ(a, b) \
    do { if ((a) != (b)) { \
        char buf[128]; snprintf(buf, sizeof(buf), \
            "expected 0x%02X, got 0x%02X", (unsigned)(b), (unsigned)(a)); \
        FAIL(buf); return; } } while(0)

#define ASSERT_TRUE(v) \
    do { if (!(v)) { FAIL("expected true"); return; } } while(0)

#define ASSERT_FALSE(v) \
    do { if (v) { FAIL("expected false"); return; } } while(0)

/* ── 测试用例 ── */

/* CRC-8 测试 */
void test_crc8_empty(void) {
    TEST("crc8_dallas: empty string");
    uint8_t crc = crc8_dallas("", 0);
    ASSERT_UINT8_EQ(crc, 0x00);
    PASS();
}

void test_crc8_known_ascii(void) {
    TEST("crc8_dallas: known ASCII '123456789'");
    /* CRC-8/MAXIM (poly=0x31, init=0x00) of "123456789" */
    uint8_t crc = crc8_dallas("123456789", 9);
    ASSERT_UINT8_EQ(crc, 0xA2);
    PASS();
}

void test_crc8_single_byte_zero(void) {
    TEST("crc8_dallas: single byte 0x00");
    uint8_t crc = crc8_dallas("\x00", 1);
    ASSERT_UINT8_EQ(crc, 0x00);
    PASS();
}

void test_crc8_single_byte_ff(void) {
    TEST("crc8_dallas: single byte 0xFF");
    uint8_t crc = crc8_dallas("\xFF", 1);
    ASSERT_UINT8_EQ(crc, 0xAC);
    PASS();
}

void test_crc8_deterministic(void) {
    TEST("crc8_dallas: deterministic");
    const char *data = "TTC:8.00 DIST:15.50";
    uint8_t crc1 = crc8_dallas(data, strlen(data));
    uint8_t crc2 = crc8_dallas(data, strlen(data));
    ASSERT_UINT8_EQ(crc1, crc2);
    PASS();
}

void test_crc8_different_data(void) {
    TEST("crc8_dallas: different data different CRC");
    uint8_t crc1 = crc8_dallas("hello", 5);
    uint8_t crc2 = crc8_dallas("world", 5);
    ASSERT_TRUE(crc1 != crc2);
    PASS();
}

/* clampf 测试 */
void test_clampf_within(void) {
    TEST("clampf: value within range");
    ASSERT_FLOAT_EQ(clampf(5.0f, 0.0f, 10.0f), 5.0f, 1e-6f);
    PASS();
}

void test_clampf_below(void) {
    TEST("clampf: value below range");
    ASSERT_FLOAT_EQ(clampf(-1.0f, 0.0f, 10.0f), 0.0f, 1e-6f);
    PASS();
}

void test_clampf_above(void) {
    TEST("clampf: value above range");
    ASSERT_FLOAT_EQ(clampf(11.0f, 0.0f, 10.0f), 10.0f, 1e-6f);
    PASS();
}

void test_clampf_negative_range(void) {
    TEST("clampf: negative range");
    ASSERT_FLOAT_EQ(clampf(-0.5f, -9.99f, 9.99f), -0.5f, 1e-6f);
    PASS();
}

void test_clampf_nan(void) {
    TEST("clampf: NaN returns NaN");
    float result = clampf(NAN, 0.0f, 10.0f);
    /* clampf 不特殊处理 NaN，返回 NaN（与 C 标准一致） */
    ASSERT_TRUE(isnan(result));
    PASS();
}

/* parsef 测试 */
void test_parsef_valid(void) {
    TEST("parsef: valid number");
    ASSERT_FLOAT_EQ(parsef("3.14"), 3.14f, 1e-4f);
    PASS();
}

void test_parsef_negative(void) {
    TEST("parsef: negative number");
    ASSERT_FLOAT_EQ(parsef("-2.5"), -2.5f, 1e-4f);
    PASS();
}

void test_parsef_empty(void) {
    TEST("parsef: empty string");
    ASSERT_FLOAT_EQ(parsef(""), 0.0f, 1e-6f);
    PASS();
}

void test_parsef_null(void) {
    TEST("parsef: NULL pointer");
    ASSERT_FLOAT_EQ(parsef(NULL), 0.0f, 1e-6f);
    PASS();
}

void test_parsef_invalid(void) {
    TEST("parsef: invalid string");
    ASSERT_FLOAT_EQ(parsef("abc"), 0.0f, 1e-6f);
    PASS();
}

/* parse_tag_float 测试 */
void test_parse_tag_float_found(void) {
    TEST("parse_tag_float: tag found");
    float out = 0.0f;
    ASSERT_TRUE(parse_tag_float("TTC:8.00 DIST:15.50", "TTC:", &out));
    ASSERT_FLOAT_EQ(out, 8.0f, 1e-4f);
    PASS();
}

void test_parse_tag_float_not_found(void) {
    TEST("parse_tag_float: tag not found");
    float out = 0.0f;
    ASSERT_FALSE(parse_tag_float("TTC:8.00", "DIST:", &out));
    PASS();
}

void test_parse_tag_float_negative(void) {
    TEST("parse_tag_float: negative value");
    float out = 0.0f;
    ASSERT_TRUE(parse_tag_float("ACC:-2.50", "ACC:", &out));
    ASSERT_FLOAT_EQ(out, -2.5f, 1e-4f);
    PASS();
}

void test_parse_tag_float_multiple(void) {
    TEST("parse_tag_float: multiple tags");
    float ttc = 0.0f, dist = 0.0f, psi = 0.0f;
    const char *line = "TTC:8.00 DIST:15.50 PSI:0.1234";
    ASSERT_TRUE(parse_tag_float(line, "TTC:", &ttc));
    ASSERT_TRUE(parse_tag_float(line, "DIST:", &dist));
    ASSERT_TRUE(parse_tag_float(line, "PSI:", &psi));
    ASSERT_FLOAT_EQ(ttc, 8.0f, 1e-4f);
    ASSERT_FLOAT_EQ(dist, 15.5f, 1e-4f);
    ASSERT_FLOAT_EQ(psi, 0.1234f, 1e-4f);
    PASS();
}

/* ── 主函数 ── */

int main(void) {
    printf("=== ESP32 ADAS Unit Tests ===\n\n");

    printf("[CRC-8/MAXIM]\n");
    test_crc8_empty();
    test_crc8_known_ascii();
    test_crc8_single_byte_zero();
    test_crc8_single_byte_ff();
    test_crc8_deterministic();
    test_crc8_different_data();

    printf("\n[clampf]\n");
    test_clampf_within();
    test_clampf_below();
    test_clampf_above();
    test_clampf_negative_range();
    test_clampf_nan();

    printf("\n[parsef]\n");
    test_parsef_valid();
    test_parsef_negative();
    test_parsef_empty();
    test_parsef_null();
    test_parsef_invalid();

    printf("\n[parse_tag_float]\n");
    test_parse_tag_float_found();
    test_parse_tag_float_not_found();
    test_parse_tag_float_negative();
    test_parse_tag_float_multiple();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
