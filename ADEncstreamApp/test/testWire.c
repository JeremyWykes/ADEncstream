/* testWire.c - the vendored encstream packet codec against encstream's hand-written golden packets
 * (vendor/encstream/golden). Expected values are written from the golden files' comments,
 * independently of wire.c, as in encstream's brick/tests/test_wire.c. Checks that the copy decodes
 * and encodes identically on this host's compiler (gcc, MSVC, MinGW).
 */
#include <stdio.h>
#include <string.h>

#include <epicsUnitTest.h>
#include <testMain.h>

#include "wire.h"

#ifndef ES_GOLDEN_DIR
#error "ES_GOLDEN_DIR must name the golden packet directory"
#endif

/* read a golden .hex file: hex byte pairs, '#' comments; returns the byte count, 0 on error */
static size_t load(const char *name, unsigned char *buf, size_t cap)
{
    char path[1024], line[512];
    size_t n = 0;
    FILE *f;
    sprintf(path, "%s/%s", ES_GOLDEN_DIR, name);   /* both short, fixed strings */
    f = fopen(path, "r");
    if (!f) {
        testAbort("cannot open %s", path);
        return 0;
    }
    while (fgets(line, sizeof line, f)) {
        char *p = line, *hash = strchr(line, '#');
        if (hash)
            *hash = '\0';
        for (;;) {
            unsigned v;
            int used;
            if (sscanf(p, " %2x%n", &v, &used) != 1)
                break;
            if (n >= cap) {
                fclose(f);
                testAbort("%s too long", path);
                return 0;
            }
            buf[n++] = (unsigned char)v;
            p += used;
        }
    }
    fclose(f);
    return n;
}

/* the layout of the golden DATA/CONTEXT packets */
static void golden_context(struct es_context *c, uint8_t order)
{
    memset(c, 0, sizeof *c);
    c->byte_order = order;
    c->n_items = 3;
    c->row_bytes = 16;
    c->servo_period_s = 0.0009765625;
    c->gather_period = 1;
    c->max_lines = 262144;
    c->id_len = 10;
    strcpy(c->id, "host=brick");
    c->items[0].role = ES_ROLE_CLOCK; c->items[0].type = 0; c->items[0].words = 1;
    c->items[0].addr = 0x1000; strcpy(c->items[0].name, "Sys.ServoCount");
    c->items[1].role = ES_ROLE_GATE; c->items[1].gate_bit = 11; c->items[1].type = 0;
    c->items[1].words = 1; c->items[1].addr = 0x4000;
    strcpy(c->items[1].name, "PowerBrick[0].Chan[0].Status");
    c->items[2].role = ES_ROLE_POSITION; c->items[2].axis = 1; c->items[2].type = 5;
    c->items[2].words = 2; c->items[2].addr = 0x3000; strcpy(c->items[2].name, "Motor[1].ActPos");
}

static void check_rows(const struct es_data *d, uint8_t order, const char *what)
{
    uint32_t w[2];
    double v = 0;
    const uint8_t *r0 = d->rows, *r1 = d->rows + 16;
    testOk(es_row_word(r0, 0, order) == 0x01020304u, "%s row 0 ServoCount", what);
    testOk(es_row_word(r0, 1, order) == 0xA5A508A5u, "%s row 0 gate word", what);
    testOk(es_row_word(r1, 0, order) == 0x01020305u, "%s row 1 ServoCount", what);
    testOk(es_row_word(r1, 1, order) == 0xA5A500A5u, "%s row 1 gate word", what);
    testOk(((es_row_word(r0, 1, order) >> 11) & 1u) == 1, "%s row 0 gate bit 11 set", what);
    testOk(((es_row_word(r1, 1, order) >> 11) & 1u) == 0, "%s row 1 gate bit 11 clear", what);
    w[0] = es_row_word(r0, 2, order); w[1] = es_row_word(r0, 3, order);
    testOk(es_decode_value(5, w, order, &v) == ES_OK && v == 123.5, "%s row 0 position 123.5", what);
    w[0] = es_row_word(r1, 2, order); w[1] = es_row_word(r1, 3, order);
    testOk(es_decode_value(5, w, order, &v) == ES_OK && v == -1234.5678,
           "%s row 1 position -1234.5678", what);
}

static void test_golden_data(void)
{
    static const char *const names[2] = { "data_le.hex", "data_be.hex" };
    static const uint8_t orders[2] = { ES_LITTLE, ES_BIG };
    unsigned char b[256];
    struct es_packet p;
    unsigned k;
    for (k = 0; k < 2; ++k) {
        size_t n = load(names[k], b, sizeof b);
        const struct es_data *d = &p.u.data;
        testOk(n == 72, "%s: 72 bytes", names[k]);
        testOk(es_decode(b, n, &p) == ES_OK, "%s decodes", names[k]);
        testOk(p.h.type == ES_DATA && p.h.stream_id == 0x11223344u && p.h.seq == 5,
               "%s common header", names[k]);
        testOk(d->first_row == 0x0000000100000002ull && d->n_rows == 2 && d->row_bytes == 16,
               "%s first row, rows, row size", names[k]);
        testOk(d->cal_generation == 1 && d->flags == ES_DATA_SKIPPED && d->rows_skipped == 7,
               "%s generation, flags, rows skipped", names[k]);
        testOk(d->rows == b + 40, "%s rows borrowed from the datagram", names[k]);
        check_rows(d, orders[k], names[k]);
    }
}

static void test_golden_context(void)
{
    unsigned char b[256];
    struct es_packet p;
    const struct es_context *c = &p.u.context;
    size_t n = load("context_be.hex", b, sizeof b);
    testOk(n == 182, "context_be.hex: 182 bytes");
    testOk(es_decode(b, n, &p) == ES_OK, "CONTEXT decodes");
    testOk(p.h.type == ES_CONTEXT && p.h.seq == 6, "CONTEXT common header");
    testOk(c->byte_order == ES_BIG && c->n_items == 3 && c->row_bytes == 16,
           "CONTEXT byte order, items, row size");
    testOk(c->servo_period_s == 0.0009765625 && c->gather_period == 1 && c->max_lines == 262144,
           "CONTEXT servo period, Gather.Period, MaxLines");
    testOk(c->id_len == 10 && strcmp(c->id, "host=brick") == 0, "CONTEXT identification");
    testOk(c->items[0].role == ES_ROLE_CLOCK && c->items[0].type == 0 && c->items[0].words == 1
           && c->items[0].addr == 0x1000 && strcmp(c->items[0].name, "Sys.ServoCount") == 0,
           "CONTEXT item 0: sample clock");
    testOk(c->items[1].role == ES_ROLE_GATE && c->items[1].gate_bit == 11
           && c->items[1].addr == 0x4000
           && strcmp(c->items[1].name, "PowerBrick[0].Chan[0].Status") == 0,
           "CONTEXT item 1: gate word, bit 11");
    testOk(c->items[2].role == ES_ROLE_POSITION && c->items[2].axis == 1 && c->items[2].type == 5
           && c->items[2].words == 2 && c->items[2].addr == 0x3000
           && strcmp(c->items[2].name, "Motor[1].ActPos") == 0,
           "CONTEXT item 2: position of motor 1");
}

static void test_golden_small(void)
{
    unsigned char b[256];
    struct es_packet p;
    size_t n;

    n = load("calibration.hex", b, sizeof b);
    testOk(n == 64 && es_decode(b, n, &p) == ES_OK && p.h.type == ES_CALIBRATION,
           "CALIBRATION decodes");
    testOk(p.u.calibration.generation == 3 && p.u.calibration.n_axes == 2,
           "CALIBRATION generation 3, two axes");
    testOk(p.u.calibration.axes[0].motor == 1 && p.u.calibration.axes[0].home_pos == 123.5
           && p.u.calibration.axes[0].pos_sf == 1.0, "CALIBRATION motor 1");
    testOk(p.u.calibration.axes[1].motor == 2 && p.u.calibration.axes[1].home_pos == -2.0
           && p.u.calibration.axes[1].pos_sf == 0.5, "CALIBRATION motor 2");

    n = load("sender_report.hex", b, sizeof b);
    testOk(es_decode(b, n, &p) == ES_OK && p.h.type == ES_SENDER_REPORT, "SENDER_REPORT decodes");
    testOk(p.u.report.servo_count == 0xDEADBEEFu && p.u.report.flags == ES_REPORT_NTP_SYNC
           && p.u.report.wall_ns == 0x18E1A2B3C4D5E6F7ll && p.u.report.row == 0x123456789Aull,
           "SENDER_REPORT fields");

    n = load("status.hex", b, sizeof b);
    testOk(es_decode(b, n, &p) == ES_OK && p.h.type == ES_STATUS, "STATUS decodes");
    testOk(p.u.status.state == ES_STATE_FAULT_GATHER && p.u.status.enable == 3
           && p.u.status.fault == ES_FAULT_CLOCK_DISCONTINUITY, "STATUS state, enable, fault");
    testOk(p.u.status.overruns == 0x11 && p.u.status.rows_skipped == 0x0102030405060708ull
           && p.u.status.rows_sent == 0x1000 && p.u.status.packets_sent == 0x200,
           "STATUS counters");
    testOk(p.u.status.rows_rejected == 0x21 && p.u.status.clock_violations == 1
           && p.u.status.send_failures == 0x22 && p.u.status.report_failures == 0x23,
           "STATUS failure counters");
    testOk(p.u.status.worst_loop_us == 1500 && p.u.status.backlog == 13
           && p.u.status.vmrss_kb == 2048 && p.u.status.open_fds == 7, "STATUS loop and resources");
    testOk(p.u.status.next_row == 10000 && p.u.status.flags == ES_STATUS_FINAL,
           "STATUS next row and final flag");

    n = load("subscribe.hex", b, sizeof b);
    testOk(es_decode(b, n, &p) == ES_OK && p.h.type == ES_SUBSCRIBE, "SUBSCRIBE decodes");
    testOk(p.h.stream_id == 0 && p.h.seq == 42 && p.u.subscribe.flags == ES_SUB_SEND_META
           && p.u.subscribe.id_len == 5 && strcmp(p.u.subscribe.id, "ioc=X") == 0,
           "SUBSCRIBE fields");
}

/* the encoders reproduce the golden bytes: the copy encodes exactly as encstream does */
static void test_encode_golden(void)
{
    unsigned char g[256], b[256];
    size_t n, w = 0;
    struct es_common h = { ES_DATA, 0x11223344u, 0 };
    struct es_data d;
    struct es_context c;
    struct es_calibration cal;
    struct es_subscribe sub;

    n = load("data_be.hex", g, sizeof g);
    memset(&d, 0, sizeof d);
    d.first_row = 0x0000000100000002ull; d.n_rows = 2; d.row_bytes = 16; d.cal_generation = 1;
    d.flags = ES_DATA_SKIPPED; d.rows_skipped = 7;
    h.type = ES_DATA; h.seq = 5;
    memset(b, 0xEE, sizeof b);
    memcpy(b + 40, g + 40, 32);               /* rows placed by the caller */
    testOk(es_encode_data_header(b, sizeof b, &h, &d, &w) == ES_OK && w == n
           && memcmp(b, g, n) == 0, "DATA header encodes to the golden bytes");

    n = load("context_be.hex", g, sizeof g);
    golden_context(&c, ES_BIG);
    h.type = ES_CONTEXT; h.seq = 6;
    memset(b, 0xEE, sizeof b);
    testOk(es_encode_context(b, sizeof b, &h, &c, &w) == ES_OK && w == n && memcmp(b, g, n) == 0,
           "CONTEXT encodes to the golden bytes");

    n = load("calibration.hex", g, sizeof g);
    memset(&cal, 0, sizeof cal);
    cal.generation = 3; cal.n_axes = 2;
    cal.axes[0].motor = 1; cal.axes[0].home_pos = 123.5; cal.axes[0].pos_sf = 1.0;
    cal.axes[1].motor = 2; cal.axes[1].home_pos = -2.0; cal.axes[1].pos_sf = 0.5;
    h.type = ES_CALIBRATION; h.seq = 7;
    memset(b, 0xEE, sizeof b);
    testOk(es_encode_calibration(b, sizeof b, &h, &cal, &w) == ES_OK && w == n
           && memcmp(b, g, n) == 0, "CALIBRATION encodes to the golden bytes");

    /* the IOC sends SUBSCRIBE: its encoder matters most on this side */
    n = load("subscribe.hex", g, sizeof g);
    memset(&sub, 0, sizeof sub);
    sub.flags = ES_SUB_SEND_META; sub.id_len = 5; strcpy(sub.id, "ioc=X");
    h.type = ES_SUBSCRIBE; h.stream_id = 0; h.seq = 42;
    memset(b, 0xEE, sizeof b);
    testOk(es_encode_subscribe(b, sizeof b, &h, &sub, &w) == ES_OK && w == n
           && memcmp(b, g, n) == 0, "SUBSCRIBE encodes to the golden bytes");
}

/* every golden packet one byte short, and with one byte too many, is rejected */
static void test_exact_lengths(void)
{
    static const char *const names[] = {
        "data_le.hex", "data_be.hex", "context_be.hex", "calibration.hex",
        "sender_report.hex", "status.hex", "subscribe.hex"
    };
    unsigned char b[260];
    struct es_packet p;
    unsigned k;
    for (k = 0; k < sizeof names / sizeof names[0]; ++k) {
        size_t n = load(names[k], b, sizeof b - 1);
        b[n] = 0;
        testOk(es_decode(b, n - 1, &p) < 0 && es_decode(b, n + 1, &p) < 0,
               "%s: one byte short or long is rejected", names[k]);
    }
}

/* encoder status items (role 4) are the raw word, Type 0; anything else is refused */
static void test_d37(void)
{
    struct es_context c;
    golden_context(&c, ES_BIG);
    c.items[1].role = ES_ROLE_ENCODER_STATUS; c.items[1].axis = 1; c.items[1].gate_bit = 0;
    testOk(es_check_layout(&c) == ES_OK, "role-4 Type 0 accepted");
    c.items[1].type = 5; c.items[1].words = 2; c.row_bytes = 20;
    testOk(es_check_layout(&c) == ES_E_LAYOUT, "role-4 Type 5 refused");
    c.items[1].type = 4; c.items[1].words = 1; c.row_bytes = 16;
    testOk(es_check_layout(&c) == ES_E_LAYOUT, "role-4 Type 4 refused");
}

MAIN(testWire)
{
    testPlan(65);
    test_golden_data();
    test_golden_context();
    test_golden_small();
    test_encode_golden();
    test_exact_lengths();
    test_d37();
    return testDone();
}
