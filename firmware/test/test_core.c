// Tests for the parts with no hardware in them. Runs on a laptop: test/run.sh
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mecanum.h"
#include "ms200.h"
#include "wheel_ctl.h"

#define NEAR(a, b) (fabsf((a) - (b)) < 1e-4f)
static const float K = 0.16f;

static void test_wheel_patterns(void)
{
    float w[4];
    mecanum_inverse(&(twist_t){0.2f, 0, 0}, K, 1.0f, w);        // forward: all forward
    for (int i = 0; i < 4; i++) assert(NEAR(w[i], 0.2f));

    mecanum_inverse(&(twist_t){0, 0.2f, 0}, K, 1.0f, w);        // left: LF-, LR+, RF+, RR-
    assert(NEAR(w[0], -0.2f) && NEAR(w[1], 0.2f) && NEAR(w[2], 0.2f) && NEAR(w[3], -0.2f));

    mecanum_inverse(&(twist_t){0, -0.2f, 0}, K, 1.0f, w);       // right: the opposite
    assert(NEAR(w[0], 0.2f) && NEAR(w[1], -0.2f) && NEAR(w[2], -0.2f) && NEAR(w[3], 0.2f));

    mecanum_inverse(&(twist_t){0, 0, 1.0f}, K, 1.0f, w);        // anticlockwise: left back
    assert(NEAR(w[0], -K) && NEAR(w[1], -K) && NEAR(w[2], K) && NEAR(w[3], K));

    // Diagonal: two opposite corners stop.
    mecanum_inverse(&(twist_t){0.2f, 0.2f, 0}, K, 1.0f, w);
    assert(NEAR(w[0], 0) && NEAR(w[1], 0.4f) && NEAR(w[2], 0.4f) && NEAR(w[3], 0));
}

static void test_round_trip(void)
{
    const twist_t cases[] = {{0.3f, 0, 0}, {0, 0.3f, 0}, {0, 0, 1.0f}, {0.1f, -0.2f, 0.5f}};
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        float w[4];
        twist_t out;
        mecanum_inverse(&cases[i], K, 10.0f, w);
        mecanum_forward(w, K, &out);
        assert(NEAR(out.vx, cases[i].vx) && NEAR(out.vy, cases[i].vy) && NEAR(out.wz, cases[i].wz));
    }
}

static void test_saturation_keeps_direction(void)
{
    float w[4];
    twist_t out;
    mecanum_inverse(&(twist_t){1.0f, 1.0f, 0}, K, 1.0f, w);
    for (int i = 0; i < 4; i++) assert(fabsf(w[i]) <= 1.0f + 1e-4f);
    mecanum_forward(w, K, &out);
    assert(NEAR(out.vx, out.vy));
}

static const wheel_cfg_t CFG = {6.0f, 1.0f, 100, 240, 400, 100};

static void test_wheel_control(void)
{
    wheel_state_t s = {0};
    assert(wheel_step(&CFG, &s, 0.0f, 0) == 0);                 // zero target is off
    int pwm = wheel_step(&CFG, &s, 5.0f, 0);
    assert(pwm > 100 && pwm <= 240);                            // past the dead zone
    s = (wheel_state_t){0};
    assert(wheel_step(&CFG, &s, -5.0f, 0) < -100);              // and in reverse

    s = (wheel_state_t){0};
    assert(wheel_step(&CFG, &s, 1000.0f, 0) == 240);            // limited
    for (int i = 0; i < 1000; i++) wheel_step(&CFG, &s, 5.0f, 5);
    assert(s.fault == WHEEL_OK);
}

static void test_backwards_encoder_faults(void)
{
    wheel_state_t s = {0};
    int steps = 0;
    while (s.fault == WHEEL_OK && steps < 100) { wheel_step(&CFG, &s, 10.0f, -20); steps++; }
    assert(s.fault == WHEEL_RUNAWAY && steps == 20);
    assert(wheel_step(&CFG, &s, 10.0f, 10) == 0);               // and it stays off
}

static void test_reversing_at_speed_is_not_a_fault(void)
{
    wheel_state_t s = {0};
    for (int i = 0; i < 5; i++) wheel_step(&CFG, &s, 10.0f, -30);
    for (int i = 0; i < 100; i++) wheel_step(&CFG, &s, 10.0f, 10);
    for (int i = 0; i < 5; i++) wheel_step(&CFG, &s, 10.0f, -30);
    assert(s.fault == WHEEL_OK);
}

static void test_stall_faults(void)
{
    wheel_state_t s = {0};
    int steps = 0;
    while (s.fault == WHEEL_OK && steps < 1000) { wheel_step(&CFG, &s, 10.0f, 0); steps++; }
    assert(s.fault == WHEEL_STALL && steps < 150);
}

static void make_packet(uint8_t *p, uint16_t start, uint16_t end, uint16_t first_mm)
{
    memset(p, 0, MS200_PACKET_LEN);
    p[0] = 0x54; p[1] = 0x2C;
    p[4] = start & 0xFF; p[5] = start >> 8;
    for (int i = 0; i < MS200_POINTS; i++) {
        uint16_t d = (uint16_t)(first_mm + i);
        p[6 + 3 * i] = d & 0xFF; p[7 + 3 * i] = d >> 8; p[8 + 3 * i] = 200;
    }
    p[42] = end & 0xFF; p[43] = end >> 8;
    p[46] = ms200_crc8(p, 46);
}

static int feed_all(ms200_parser_t *parser, const uint8_t *data, int len)
{
    int good = 0;
    for (int i = 0; i < len; i++) good += ms200_feed(parser, data[i]);
    return good;
}

static void test_crc_matches_oradar_table(void)
{
    // First entries of CrcTable in Oradar's driver: 0x00, 0x4d, 0x9a, 0xd7.
    for (int i = 0; i < 4; i++) {
        const uint8_t b = (uint8_t)i, expected[] = {0x00, 0x4D, 0x9A, 0xD7};
        assert(ms200_crc8(&b, 1) == expected[i]);
    }
}

static void test_parser(void)
{
    uint8_t p[MS200_PACKET_LEN];
    make_packet(p, 1000, 2100, 500);
    ms200_parser_t parser = {0};

    assert(feed_all(&parser, p, sizeof(p)) == 1);

    const uint8_t junk[] = {0x00, 0x54, 0x11, 0xFF, 0x54};      // resyncs after noise
    assert(feed_all(&parser, junk, sizeof(junk)) == 0);
    assert(feed_all(&parser, p + 1, sizeof(p) - 1) == 1);

    uint8_t bad[MS200_PACKET_LEN];
    memcpy(bad, p, sizeof(bad));
    bad[20] ^= 1;
    parser = (ms200_parser_t){0};
    assert(feed_all(&parser, bad, sizeof(bad)) == 0);           // bad CRC rejected
}

static void test_angles_match_laserscan(void)
{
    uint8_t p[MS200_PACKET_LEN];
    uint16_t d[MS200_BINS];

    // Lidar 0 deg is ahead: LaserScan index 180, angle 0.
    memset(d, 0, sizeof(d));
    make_packet(p, 0, 0, 1000);                                 // all points at 0 deg
    ms200_store(p, d);
    assert(d[180] != 0);

    // Lidar angles run clockwise, so 90 deg is the right: index 90, angle -90.
    memset(d, 0, sizeof(d));
    make_packet(p, 9000, 9000, 1000);
    ms200_store(p, d);
    assert(d[90] != 0 && d[270] == 0);

    // 270 deg is the left: index 270, angle +90.
    memset(d, 0, sizeof(d));
    make_packet(p, 27000, 27000, 1000);
    ms200_store(p, d);
    assert(d[270] != 0 && d[90] == 0);

    // A packet across the 360 to 0 wrap must not lose points or overflow.
    memset(d, 0, sizeof(d));
    make_packet(p, 35950, 500, 1000);
    ms200_store(p, d);
    assert(d[180] != 0 && d[175] != 0);
}

int main(void)
{
    test_wheel_patterns();
    test_round_trip();
    test_saturation_keeps_direction();
    test_wheel_control();
    test_backwards_encoder_faults();
    test_reversing_at_speed_is_not_a_fault();
    test_stall_faults();
    test_crc_matches_oradar_table();
    test_parser();
    test_angles_match_laserscan();
    puts("all firmware tests passed");
    return 0;
}
