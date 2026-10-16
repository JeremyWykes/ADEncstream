/* wire.h - encstream packet codec (protocol version 1; the Wire Protocol guide).
 *
 * Pure encoding and decoding: caller-owned buffers with explicit lengths and capacities, no
 * sockets, no allocation, no logging, no controller access. Header fields are little-endian on the
 * wire on every host. DATA row payloads are opaque here: they are the gather ring's bytes in the
 * Brick's native byte order; es_row_word() and es_decode_value() interpret them, guided by
 * CONTEXT. Every decode checks the exact datagram length and all counts before any field is used
 *, and a CONTEXT is accepted only if its layout passes es_check_layout().
 *
 * C99 (-std=gnu99), builds with gcc 4.6.3 (PowerPC Brick) up to current gcc.
 */
#ifndef ENCSTREAM_WIRE_H
#define ENCSTREAM_WIRE_H

#include <stddef.h>
#include <stdint.h>

#define ES_MAGIC        0x53434E45u   /* bytes 45 4E 43 53, "ENCS" */
#define ES_VERSION      1u

#define ES_MAX_ITEMS    24u
#define ES_MAX_AXES     16u
#define ES_MAX_ID       128u          /* CONTEXT identification */
#define ES_MAX_SUB_ID   64u           /* SUBSCRIBE identification */
#define ES_NAME_LEN     32u           /* CONTEXT item name field */

/* exact header lengths (common 16 bytes + type-specific) */
#define ES_COMMON_LEN          16u
#define ES_DATA_HDR_LEN        40u
#define ES_CONTEXT_HDR_LEN     40u
#define ES_CONTEXT_ITEM_LEN    44u
#define ES_CALIBRATION_HDR_LEN 24u
#define ES_CALIBRATION_REC_LEN 20u
#define ES_SENDER_REPORT_LEN   40u
#define ES_STATUS_LEN          96u
#define ES_SUBSCRIBE_HDR_LEN   24u

enum es_type {
    ES_DATA = 1,
    ES_CONTEXT = 2,
    ES_SENDER_REPORT = 3,
    ES_STATUS = 4,
    ES_CALIBRATION = 5,
    ES_SUBSCRIBE = 16
};

enum es_role {
    ES_ROLE_CLOCK = 1,
    ES_ROLE_GATE = 2,
    ES_ROLE_POSITION = 3,
    ES_ROLE_ENCODER_STATUS = 4,
    ES_ROLE_OTHER = 5
};

enum es_byte_order { ES_LITTLE = 0, ES_BIG = 1 };

/* DATA flags */
#define ES_DATA_SKIPPED    0x1u   /* rows were skipped (catch-up) since the previous DATA */
/* SENDER_REPORT flags */
#define ES_REPORT_NTP_SYNC 0x1u
/* STATUS flags */
#define ES_STATUS_FINAL    0x1u   /* encstream is stopping: no further packets on this stream */
/* SUBSCRIBE flags */
#define ES_SUB_SEND_META   0x1u   /* send CONTEXT and CALIBRATION now */

/* STATUS state and fault codes */
enum es_state { ES_STATE_STREAMING = 1, ES_STATE_FAULT_GATHER = 2, ES_STATE_FAULT_OTHER = 3 };
enum es_fault {
    ES_FAULT_NONE = 0,
    ES_FAULT_CONFIG_DIFFERS = 1,
    ES_FAULT_SAMPLES_DECREASED = 2,
    ES_FAULT_NOT_ENABLED = 3,
    ES_FAULT_INTERNAL = 4,
    ES_FAULT_CLOCK_DISCONTINUITY = 5
};

/* Codec results. ES_OK is 0; every failure is negative. */
enum es_result {
    ES_OK = 0,
    ES_E_SHORT = -1,           /* shorter than the common header */
    ES_E_MAGIC = -2,           /* not an encstream packet */
    ES_E_MAGIC_SWAPPED = -3,   /* magic in the wrong byte order: a sender byte-order bug */
    ES_E_VERSION = -4,         /* protocol version not understood */
    ES_E_UNKNOWN_TYPE = -5,    /* valid common header, unknown packet type: skip and count */
    ES_E_HDR_LEN = -6,         /* header-length field differs from the type's exact value */
    ES_E_LENGTH = -7,          /* datagram length differs from what its counts imply */
    ES_E_FIELD = -8,           /* a field value out of its allowed range */
    ES_E_LAYOUT = -9,          /* CONTEXT layout fails es_check_layout() */
    ES_E_CAPACITY = -10,       /* encode: output buffer too small */
    ES_E_ARG = -11             /* invalid argument (NULL, count beyond a maximum) */
};

struct es_common {
    uint8_t type;
    uint32_t stream_id;
    uint32_t seq;
};

struct es_data {
    uint64_t first_row;      /* sample ordinal of the first row */
    uint16_t n_rows;         /* >= 1 */
    uint16_t row_bytes;      /* multiple of 4, >= 4 */
    uint32_t cal_generation;
    uint32_t flags;          /* ES_DATA_SKIPPED */
    uint32_t rows_skipped;   /* immediately before first_row */
    const uint8_t *rows;     /* decode: points into the caller's datagram (borrowed) */
};

struct es_item {
    uint8_t role;            /* enum es_role */
    uint8_t axis;            /* motor number for roles 3 and 4, else 0 */
    uint8_t gate_bit;        /* role 2 */
    uint16_t type;           /* raw Gather.Type */
    uint16_t words;          /* 1, or 2 for type 5 */
    uint32_t addr;           /* raw Gather.Addr, diagnostic */
    char name[ES_NAME_LEN + 1];   /* NUL-terminated copy of the NUL-padded field */
};

struct es_context {
    uint8_t byte_order;      /* enum es_byte_order */
    uint8_t n_items;
    uint16_t row_bytes;
    double servo_period_s;
    uint16_t gather_period;
    uint32_t max_lines;
    uint16_t id_len;
    char id[ES_MAX_ID + 1];  /* NUL-terminated copy */
    struct es_item items[ES_MAX_ITEMS];
};

struct es_cal_axis {
    uint32_t motor;
    double home_pos;
    double pos_sf;
};

struct es_calibration {
    uint32_t generation;
    uint16_t n_axes;
    struct es_cal_axis axes[ES_MAX_AXES];
};

struct es_sender_report {
    uint32_t servo_count;
    uint32_t flags;          /* ES_REPORT_NTP_SYNC */
    int64_t wall_ns;         /* ns since the Unix epoch */
    uint64_t row;            /* sample ordinal current at the clock read */
};

struct es_status {
    uint8_t state;           /* enum es_state */
    uint8_t enable;          /* Gather.Enable as last read */
    uint16_t fault;          /* enum es_fault */
    uint32_t overruns;
    uint64_t rows_skipped;
    uint64_t rows_sent;
    uint64_t packets_sent;
    uint32_t rows_rejected;
    uint32_t clock_violations;
    uint32_t send_failures;
    uint32_t report_failures;
    uint32_t worst_loop_us;
    uint32_t backlog;
    uint32_t vmrss_kb;
    uint32_t open_fds;
    uint64_t next_row;       /* every row below it was sent in DATA, skipped or discarded */
    uint32_t flags;          /* ES_STATUS_FINAL; other bits ignored on receipt */
};

struct es_subscribe {
    uint32_t flags;          /* ES_SUB_SEND_META */
    uint16_t id_len;
    char id[ES_MAX_SUB_ID + 1];
};

struct es_packet {
    struct es_common h;
    union {
        struct es_data data;
        struct es_context context;
        struct es_calibration calibration;
        struct es_sender_report report;
        struct es_status status;
        struct es_subscribe subscribe;
    } u;
};

/* ---- encoding: each writes one complete packet into buf[0..cap) and sets *written ---------- */

/* DATA: writes the 40-byte header only. The caller places n_rows * row_bytes row bytes at
 * buf + ES_DATA_HDR_LEN (e.g. copied straight from the ring); cap must cover header and rows,
 * and *written is the whole datagram length. d->rows is ignored. */
int es_encode_data_header(uint8_t *buf, size_t cap, const struct es_common *h,
                          const struct es_data *d, size_t *written);
int es_encode_context(uint8_t *buf, size_t cap, const struct es_common *h,
                      const struct es_context *c, size_t *written);
int es_encode_calibration(uint8_t *buf, size_t cap, const struct es_common *h,
                          const struct es_calibration *c, size_t *written);
int es_encode_sender_report(uint8_t *buf, size_t cap, const struct es_common *h,
                            const struct es_sender_report *r, size_t *written);
int es_encode_status(uint8_t *buf, size_t cap, const struct es_common *h,
                     const struct es_status *st, size_t *written);
int es_encode_subscribe(uint8_t *buf, size_t cap, const struct es_common *h,
                        const struct es_subscribe *sub, size_t *written);

/* ---- decoding ---------------------------------------------------------------------------- */

/* Decode one datagram. On ES_OK *out is complete and valid. On ES_E_UNKNOWN_TYPE out->h is
 * valid (skip and count; its header length was at least 16 and within the datagram). On any other failure *out is unspecified and must not be used.
 * DATA's rows pointer borrows buf: keep buf unchanged while using it. */
int es_decode(const uint8_t *buf, size_t len, struct es_packet *out);

/* layout checks (sender at start-up, receiver on every CONTEXT). ES_OK or ES_E_LAYOUT. */
int es_check_layout(const struct es_context *c);

/* ---- row interpretation (receiver side) --------------------------------------------------- */

/* Number of 32-bit words of a Gather.Type, or 0 if the type is not in the decode table. */
unsigned es_type_words(uint16_t type);

/* 32-bit word `word` of a row stored in byte order `order` (enum es_byte_order). */
uint32_t es_row_word(const uint8_t *row, unsigned word, uint8_t order);

/* Value of one item from its raw words (w[0], and w[1] for doubles, each already converted from
 * the row byte order with es_row_word). For doubles, w[0] is the word that comes first in the
 * row: the high half for big-endian rows, the low half for little-endian rows. Returns ES_OK and
 * *value, or ES_E_FIELD for a type outside the decode table. */
int es_decode_value(uint16_t type, const uint32_t *w, uint8_t order, double *value);

/* Bytes needed for packets of the given counts (no overflow: counts are bounded by the maxima). */
size_t es_data_len(uint16_t n_rows, uint16_t row_bytes);
size_t es_context_len(unsigned n_items, unsigned id_len);
size_t es_calibration_len(unsigned n_axes);
size_t es_subscribe_len(unsigned id_len);

const char *es_strerror(int result);

#endif
