/* wire.c - encstream packet codec, protocol version 1. See wire.h and the Wire Protocol guide. */
#include "wire.h"

#include <string.h>

typedef char es_check_double_size[(sizeof(double) == 8) ? 1 : -1];
typedef char es_check_float_size[(sizeof(float) == 4) ? 1 : -1];

/* ---- little-endian field access: byte operations only, any alignment, any host ---------- */

static void put_u8(uint8_t *p, unsigned v) { p[0] = (uint8_t)v; }

static void put_u16(uint8_t *p, unsigned v)
{
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
    p[2] = (uint8_t)((v >> 16) & 0xffu);
    p[3] = (uint8_t)((v >> 24) & 0xffu);
}

static void put_u64(uint8_t *p, uint64_t v)
{
    put_u32(p, (uint32_t)(v & 0xffffffffu));
    put_u32(p + 4, (uint32_t)(v >> 32));
}

static void put_f64(uint8_t *p, double d)
{
    uint64_t bits;
    memcpy(&bits, &d, sizeof bits);   /* IEEE 754 bit pattern; endianness handled by put_u64 */
    put_u64(p, bits);
}

static unsigned get_u8(const uint8_t *p) { return p[0]; }

static unsigned get_u16(const uint8_t *p) { return (unsigned)p[0] | ((unsigned)p[1] << 8); }

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t get_u64(const uint8_t *p)
{
    return (uint64_t)get_u32(p) | ((uint64_t)get_u32(p + 4) << 32);
}

static double get_f64(const uint8_t *p)
{
    uint64_t bits = get_u64(p);
    double d;
    memcpy(&d, &bits, sizeof d);
    return d;
}

static int is_finite(double d) { return d == d && d - d == 0.0; }   /* not NaN, not infinite */

/* ---- sizes -------------------------------------------------------------------------------- */

size_t es_data_len(uint16_t n_rows, uint16_t row_bytes)
{
    return ES_DATA_HDR_LEN + (size_t)n_rows * (size_t)row_bytes;
}

size_t es_context_len(unsigned n_items, unsigned id_len)
{
    return ES_CONTEXT_HDR_LEN + (size_t)n_items * ES_CONTEXT_ITEM_LEN + (size_t)id_len;
}

size_t es_calibration_len(unsigned n_axes)
{
    return ES_CALIBRATION_HDR_LEN + (size_t)n_axes * ES_CALIBRATION_REC_LEN;
}

size_t es_subscribe_len(unsigned id_len) { return ES_SUBSCRIBE_HDR_LEN + (size_t)id_len; }

/* ---- types and values --------------------------------------------------------------------- */

/* Bit-field descriptor (DESIGN decode table; simulator Runs 7, 8, 20): bits 0-2 = 6 (unsigned)
 * or 7 (signed), bits 3-5 must be 0, width = 32 - bits 6-10, start bit = bits 11-15. */
static int bitfield(uint16_t type, unsigned *start, unsigned *width, int *is_signed)
{
    unsigned mode = type & 7u;
    if (type <= 5u || (type & 0x38u) != 0 || (mode != 6u && mode != 7u))
        return 0;
    *width = 32u - ((type >> 6) & 31u);
    *start = (unsigned)(type >> 11) & 31u;
    *is_signed = mode == 7u;
    return *start + *width <= 32u;
}

unsigned es_type_words(uint16_t type)
{
    unsigned start, width;
    int sgn;
    if (type <= 5u)
        return type == 5u ? 2u : 1u;
    return bitfield(type, &start, &width, &sgn) ? 1u : 0u;
}

uint32_t es_row_word(const uint8_t *row, unsigned word, uint8_t order)
{
    const uint8_t *p = row + 4u * (size_t)word;
    if (order == ES_BIG)
        return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
    return get_u32(p);
}

/* two's-complement interpretation of the low `width` bits of v, without implementation-defined
 * conversions or shifts of negative values */
static double sign_extend(uint32_t v, unsigned width)
{
    uint64_t top = (uint64_t)1 << (width - 1u);
    if ((uint64_t)v & top)
        return (double)((int64_t)v - (int64_t)(top << 1));
    return (double)v;
}

int es_decode_value(uint16_t type, const uint32_t *w, uint8_t order, double *value)
{
    unsigned start, width;
    int sgn;
    switch (type) {
    case 0: *value = (double)w[0]; return ES_OK;
    case 1: *value = sign_extend(w[0], 32u); return ES_OK;
    case 2: *value = (double)(w[0] >> 8); return ES_OK;
    case 3: *value = sign_extend(w[0] >> 8, 24u); return ES_OK;
    case 4: {
        float f;
        memcpy(&f, &w[0], sizeof f);
        *value = (double)f;
        return ES_OK;
    }
    case 5: {
        uint64_t hi = order == ES_BIG ? w[0] : w[1], lo = order == ES_BIG ? w[1] : w[0];
        uint64_t bits = (hi << 32) | lo;
        double d;
        memcpy(&d, &bits, sizeof d);
        *value = d;
        return ES_OK;
    }
    default:
        break;
    }
    if (!bitfield(type, &start, &width, &sgn))
        return ES_E_FIELD;
    {
        uint32_t v = w[0] >> start;
        if (width < 32u)
            v &= ((uint32_t)1 << width) - 1u;
        *value = sgn ? sign_extend(v, width) : (double)v;
    }
    return ES_OK;
}

/* ---- layout checks ------------------------------------------------------------ */

int es_check_layout(const struct es_context *c)
{
    unsigned words = 0, gates = 0, positions = 0, i, j;
    if (!c || c->n_items < 1 || c->n_items > ES_MAX_ITEMS)
        return ES_E_LAYOUT;
    if (c->byte_order != ES_LITTLE && c->byte_order != ES_BIG)
        return ES_E_LAYOUT;
    if (c->max_lines < 1 || c->gather_period < 1 || !is_finite(c->servo_period_s) ||
        c->servo_period_s <= 0.0)
        return ES_E_LAYOUT;
    for (i = 0; i < c->n_items; ++i) {
        const struct es_item *it = &c->items[i];
        unsigned n = es_type_words(it->type);
        if (n == 0 || it->words != n)
            return ES_E_LAYOUT;
        words += n;
        if ((i == 0) != (it->role == ES_ROLE_CLOCK))
            return ES_E_LAYOUT;           /* exactly one clock, and it is item 0 */
        switch (it->role) {
        case ES_ROLE_CLOCK:
            if (it->type != 0)
                return ES_E_LAYOUT;
            break;
        case ES_ROLE_GATE:              /* the raw status word; gate_bit indexes it */
            if (++gates > 1 || it->gate_bit > 31 || it->type != 0)
                return ES_E_LAYOUT;
            break;
        case ES_ROLE_POSITION:          /* CALIBRATION holds at most ES_MAX_AXES axes */
            if (it->type != 5 || ++positions > ES_MAX_AXES)
                return ES_E_LAYOUT;
            break;
        case ES_ROLE_ENCODER_STATUS:    /* the raw status word, like the gate */
            if (it->type != 0)
                return ES_E_LAYOUT;
            break;
        case ES_ROLE_OTHER:
            break;
        default:
            return ES_E_LAYOUT;
        }
        if (it->role == ES_ROLE_POSITION || it->role == ES_ROLE_ENCODER_STATUS) {
            if (it->axis < 1)
                return ES_E_LAYOUT;
            for (j = 0; j < i; ++j)       /* one item per axis within each role */
                if (c->items[j].role == it->role && c->items[j].axis == it->axis)
                    return ES_E_LAYOUT;
        }
    }
    if ((size_t)words * 4u != c->row_bytes)
        return ES_E_LAYOUT;
    return ES_OK;
}

/* STATUS: known state and fault, consistent with each other (fail closed on anything else) */
static int status_ok(const struct es_status *st)
{
    if (st->enable > 3)
        return 0;
    switch (st->state) {
    case ES_STATE_STREAMING:
        return st->fault == ES_FAULT_NONE;
    case ES_STATE_FAULT_GATHER:
        return st->fault == ES_FAULT_CONFIG_DIFFERS || st->fault == ES_FAULT_SAMPLES_DECREASED ||
               st->fault == ES_FAULT_NOT_ENABLED || st->fault == ES_FAULT_CLOCK_DISCONTINUITY;
    case ES_STATE_FAULT_OTHER:
        return st->fault == ES_FAULT_INTERNAL;
    default:
        return 0;
    }
}

/* CALIBRATION: motors numbered from 1, each once, values finite */
static int calibration_ok(const struct es_calibration *c)
{
    unsigned i, j;
    for (i = 0; i < c->n_axes; ++i) {
        if (c->axes[i].motor < 1 || !is_finite(c->axes[i].home_pos) || !is_finite(c->axes[i].pos_sf))
            return 0;
        for (j = 0; j < i; ++j)
            if (c->axes[j].motor == c->axes[i].motor)
                return 0;
    }
    return 1;
}

/* ---- common header ------------------------------------------------------------------------ */

static unsigned hdr_len_of(unsigned type)
{
    switch (type) {
    case ES_DATA: return ES_DATA_HDR_LEN;
    case ES_CONTEXT: return ES_CONTEXT_HDR_LEN;
    case ES_SENDER_REPORT: return ES_SENDER_REPORT_LEN;
    case ES_STATUS: return ES_STATUS_LEN;
    case ES_CALIBRATION: return ES_CALIBRATION_HDR_LEN;
    case ES_SUBSCRIBE: return ES_SUBSCRIBE_HDR_LEN;
    default: return 0;
    }
}

/* stream ID: never 0 from the Brick; always 0 in SUBSCRIBE */
static int stream_id_ok(unsigned type, uint32_t id)
{
    return type == ES_SUBSCRIBE ? id == 0 : id != 0;
}

static int begin(uint8_t *buf, size_t cap, size_t len, const struct es_common *h, unsigned type)
{
    if (!buf || !h)
        return ES_E_ARG;
    if (!stream_id_ok(type, h->stream_id))
        return ES_E_FIELD;
    if (len > cap)
        return ES_E_CAPACITY;
    memset(buf, 0, hdr_len_of(type));   /* reserved fields are zero on send */
    put_u32(buf, ES_MAGIC);
    put_u8(buf + 4, ES_VERSION);
    put_u8(buf + 5, type);
    put_u16(buf + 6, hdr_len_of(type));
    put_u32(buf + 8, h->stream_id);
    put_u32(buf + 12, h->seq);
    return ES_OK;
}

/* identification strings: printable ASCII, others replaced by '?' */
static void put_text(uint8_t *p, const char *s, unsigned n)
{
    unsigned i;
    for (i = 0; i < n; ++i) {
        unsigned char ch = (unsigned char)s[i];
        p[i] = (ch >= 0x20 && ch < 0x7f) ? ch : (uint8_t)'?';
    }
}

/* ---- encoders ----------------------------------------------------------------------------- */

int es_encode_data_header(uint8_t *buf, size_t cap, const struct es_common *h,
                          const struct es_data *d, size_t *written)
{
    size_t len;
    int r;
    if (!d || !written)
        return ES_E_ARG;
    if (d->n_rows < 1 || d->row_bytes < 4 || d->row_bytes % 4 != 0 ||
        d->first_row > UINT64_MAX - d->n_rows)
        return ES_E_FIELD;
    len = es_data_len(d->n_rows, d->row_bytes);
    if ((r = begin(buf, cap, len, h, ES_DATA)) != ES_OK)
        return r;
    put_u64(buf + 16, d->first_row);
    put_u16(buf + 24, d->n_rows);
    put_u16(buf + 26, d->row_bytes);
    put_u32(buf + 28, d->cal_generation);
    put_u32(buf + 32, d->flags);
    put_u32(buf + 36, d->rows_skipped);
    *written = len;
    return ES_OK;
}

int es_encode_context(uint8_t *buf, size_t cap, const struct es_common *h,
                      const struct es_context *c, size_t *written)
{
    size_t len;
    unsigned i;
    int r;
    if (!c || !written || c->id_len > ES_MAX_ID)
        return ES_E_ARG;
    if (es_check_layout(c) != ES_OK)
        return ES_E_LAYOUT;
    for (i = 0; i < c->n_items; ++i)
        if (memchr(c->items[i].name, 0, sizeof c->items[i].name) == NULL)
            return ES_E_ARG;          /* names are NUL-terminated, at most ES_NAME_LEN chars */
    len = es_context_len(c->n_items, c->id_len);
    if ((r = begin(buf, cap, len, h, ES_CONTEXT)) != ES_OK)
        return r;
    put_u8(buf + 16, c->byte_order);
    put_u8(buf + 17, c->n_items);
    put_u16(buf + 18, c->row_bytes);
    put_f64(buf + 20, c->servo_period_s);
    put_u16(buf + 28, c->gather_period);
    put_u32(buf + 32, c->max_lines);
    put_u16(buf + 36, c->id_len);
    for (i = 0; i < c->n_items; ++i) {
        const struct es_item *it = &c->items[i];
        uint8_t *p = buf + ES_CONTEXT_HDR_LEN + (size_t)i * ES_CONTEXT_ITEM_LEN;
        size_t n = strlen(it->name);
        memset(p, 0, ES_CONTEXT_ITEM_LEN);
        put_u8(p, it->role);
        put_u8(p + 1, it->axis);
        put_u8(p + 2, it->gate_bit);
        put_u16(p + 4, it->type);
        put_u16(p + 6, it->words);
        put_u32(p + 8, it->addr);
        put_text(p + 12, it->name, (unsigned)n);   /* n <= ES_NAME_LEN; rest stays NUL */
    }
    put_text(buf + ES_CONTEXT_HDR_LEN + (size_t)c->n_items * ES_CONTEXT_ITEM_LEN, c->id, c->id_len);
    *written = len;
    return ES_OK;
}

int es_encode_calibration(uint8_t *buf, size_t cap, const struct es_common *h,
                          const struct es_calibration *c, size_t *written)
{
    size_t len;
    unsigned i;
    int r;
    if (!c || !written || c->n_axes > ES_MAX_AXES)
        return ES_E_ARG;
    if (!calibration_ok(c))
        return ES_E_FIELD;
    len = es_calibration_len(c->n_axes);
    if ((r = begin(buf, cap, len, h, ES_CALIBRATION)) != ES_OK)
        return r;
    put_u32(buf + 16, c->generation);
    put_u16(buf + 20, c->n_axes);
    for (i = 0; i < c->n_axes; ++i) {
        uint8_t *p = buf + ES_CALIBRATION_HDR_LEN + (size_t)i * ES_CALIBRATION_REC_LEN;
        put_u32(p, c->axes[i].motor);
        put_f64(p + 4, c->axes[i].home_pos);
        put_f64(p + 12, c->axes[i].pos_sf);
    }
    *written = len;
    return ES_OK;
}

int es_encode_sender_report(uint8_t *buf, size_t cap, const struct es_common *h,
                            const struct es_sender_report *rep, size_t *written)
{
    int r;
    if (!rep || !written)
        return ES_E_ARG;
    if ((r = begin(buf, cap, ES_SENDER_REPORT_LEN, h, ES_SENDER_REPORT)) != ES_OK)
        return r;
    put_u32(buf + 16, rep->servo_count);
    put_u32(buf + 20, rep->flags);
    put_u64(buf + 24, (uint64_t)rep->wall_ns);
    put_u64(buf + 32, rep->row);
    *written = ES_SENDER_REPORT_LEN;
    return ES_OK;
}

int es_encode_status(uint8_t *buf, size_t cap, const struct es_common *h,
                     const struct es_status *st, size_t *written)
{
    int r;
    if (!st || !written)
        return ES_E_ARG;
    if (!status_ok(st))
        return ES_E_FIELD;
    if ((r = begin(buf, cap, ES_STATUS_LEN, h, ES_STATUS)) != ES_OK)
        return r;
    put_u8(buf + 16, st->state);
    put_u8(buf + 17, st->enable);
    put_u16(buf + 18, st->fault);
    put_u32(buf + 20, st->overruns);
    put_u64(buf + 24, st->rows_skipped);
    put_u64(buf + 32, st->rows_sent);
    put_u64(buf + 40, st->packets_sent);
    put_u32(buf + 48, st->rows_rejected);
    put_u32(buf + 52, st->clock_violations);
    put_u32(buf + 56, st->send_failures);
    put_u32(buf + 60, st->report_failures);
    put_u32(buf + 64, st->worst_loop_us);
    put_u32(buf + 68, st->backlog);
    put_u32(buf + 72, st->vmrss_kb);
    put_u32(buf + 76, st->open_fds);
    put_u64(buf + 80, st->next_row);
    put_u32(buf + 88, st->flags);
    put_u32(buf + 92, 0);
    *written = ES_STATUS_LEN;
    return ES_OK;
}

int es_encode_subscribe(uint8_t *buf, size_t cap, const struct es_common *h,
                        const struct es_subscribe *sub, size_t *written)
{
    size_t len;
    int r;
    if (!sub || !written || sub->id_len > ES_MAX_SUB_ID)
        return ES_E_ARG;
    len = es_subscribe_len(sub->id_len);
    if ((r = begin(buf, cap, len, h, ES_SUBSCRIBE)) != ES_OK)
        return r;
    put_u32(buf + 16, sub->flags);
    put_u16(buf + 20, sub->id_len);
    put_text(buf + ES_SUBSCRIBE_HDR_LEN, sub->id, sub->id_len);
    *written = len;
    return ES_OK;
}

/* ---- decoder ------------------------------------------------------------------------------ */

static void get_text(char *dst, const uint8_t *src, unsigned n)
{
    memcpy(dst, src, n);
    dst[n] = '\0';
}

int es_decode(const uint8_t *buf, size_t len, struct es_packet *out)
{
    uint32_t magic;
    unsigned type, hlen, i;
    if (!buf || !out)
        return ES_E_ARG;
    if (len < ES_COMMON_LEN)
        return ES_E_SHORT;
    magic = get_u32(buf);
    if (magic != ES_MAGIC)
        return magic == 0x454E4353u ? ES_E_MAGIC_SWAPPED : ES_E_MAGIC;
    if (get_u8(buf + 4) != ES_VERSION)
        return ES_E_VERSION;
    type = get_u8(buf + 5);
    hlen = get_u16(buf + 6);
    out->h.type = (uint8_t)type;
    out->h.stream_id = get_u32(buf + 8);
    out->h.seq = get_u32(buf + 12);
    if (hdr_len_of(type) == 0)
        return hlen >= ES_COMMON_LEN && hlen <= len ? ES_E_UNKNOWN_TYPE : ES_E_HDR_LEN;
    if (hlen != hdr_len_of(type))
        return ES_E_HDR_LEN;
    if (len < hlen)
        return ES_E_LENGTH;
    if (!stream_id_ok(type, out->h.stream_id))
        return ES_E_FIELD;

    switch (type) {
    case ES_DATA: {
        struct es_data *d = &out->u.data;
        d->first_row = get_u64(buf + 16);
        d->n_rows = (uint16_t)get_u16(buf + 24);
        d->row_bytes = (uint16_t)get_u16(buf + 26);
        d->cal_generation = get_u32(buf + 28);
        d->flags = get_u32(buf + 32);
        d->rows_skipped = get_u32(buf + 36);
        if (d->n_rows < 1 || d->row_bytes < 4 || d->row_bytes % 4 != 0 ||
            d->first_row > UINT64_MAX - d->n_rows)
            return ES_E_FIELD;
        if (len != es_data_len(d->n_rows, d->row_bytes))
            return ES_E_LENGTH;
        d->rows = buf + ES_DATA_HDR_LEN;
        return ES_OK;
    }
    case ES_CONTEXT: {
        struct es_context *c = &out->u.context;
        c->byte_order = (uint8_t)get_u8(buf + 16);
        c->n_items = (uint8_t)get_u8(buf + 17);
        c->row_bytes = (uint16_t)get_u16(buf + 18);
        c->servo_period_s = get_f64(buf + 20);
        c->gather_period = (uint16_t)get_u16(buf + 28);
        c->max_lines = get_u32(buf + 32);
        c->id_len = (uint16_t)get_u16(buf + 36);
        if (c->n_items < 1 || c->n_items > ES_MAX_ITEMS || c->id_len > ES_MAX_ID)
            return ES_E_FIELD;
        if (len != es_context_len(c->n_items, c->id_len))
            return ES_E_LENGTH;
        for (i = 0; i < c->n_items; ++i) {
            const uint8_t *p = buf + ES_CONTEXT_HDR_LEN + (size_t)i * ES_CONTEXT_ITEM_LEN;
            struct es_item *it = &c->items[i];
            it->role = (uint8_t)get_u8(p);
            it->axis = (uint8_t)get_u8(p + 1);
            it->gate_bit = (uint8_t)get_u8(p + 2);
            it->type = (uint16_t)get_u16(p + 4);
            it->words = (uint16_t)get_u16(p + 6);
            it->addr = get_u32(p + 8);
            get_text(it->name, p + 12, ES_NAME_LEN);   /* a shorter name ends at its first NUL */
        }
        get_text(c->id, buf + ES_CONTEXT_HDR_LEN + (size_t)c->n_items * ES_CONTEXT_ITEM_LEN, c->id_len);
        return es_check_layout(c) == ES_OK ? ES_OK : ES_E_LAYOUT;
    }
    case ES_CALIBRATION: {
        struct es_calibration *c = &out->u.calibration;
        c->generation = get_u32(buf + 16);
        c->n_axes = (uint16_t)get_u16(buf + 20);
        if (c->n_axes > ES_MAX_AXES)
            return ES_E_FIELD;
        if (len != es_calibration_len(c->n_axes))
            return ES_E_LENGTH;
        for (i = 0; i < c->n_axes; ++i) {
            const uint8_t *p = buf + ES_CALIBRATION_HDR_LEN + (size_t)i * ES_CALIBRATION_REC_LEN;
            c->axes[i].motor = get_u32(p);
            c->axes[i].home_pos = get_f64(p + 4);
            c->axes[i].pos_sf = get_f64(p + 12);
        }
        return calibration_ok(c) ? ES_OK : ES_E_FIELD;
    }
    case ES_SENDER_REPORT: {
        struct es_sender_report *r = &out->u.report;
        if (len != ES_SENDER_REPORT_LEN)
            return ES_E_LENGTH;
        r->servo_count = get_u32(buf + 16);
        r->flags = get_u32(buf + 20);
        r->wall_ns = (int64_t)get_u64(buf + 24);   /* two's complement on every target */
        r->row = get_u64(buf + 32);
        return ES_OK;
    }
    case ES_STATUS: {
        struct es_status *st = &out->u.status;
        if (len != ES_STATUS_LEN)
            return ES_E_LENGTH;
        st->state = (uint8_t)get_u8(buf + 16);
        st->enable = (uint8_t)get_u8(buf + 17);
        st->fault = (uint16_t)get_u16(buf + 18);
        st->overruns = get_u32(buf + 20);
        st->rows_skipped = get_u64(buf + 24);
        st->rows_sent = get_u64(buf + 32);
        st->packets_sent = get_u64(buf + 40);
        st->rows_rejected = get_u32(buf + 48);
        st->clock_violations = get_u32(buf + 52);
        st->send_failures = get_u32(buf + 56);
        st->report_failures = get_u32(buf + 60);
        st->worst_loop_us = get_u32(buf + 64);
        st->backlog = get_u32(buf + 68);
        st->vmrss_kb = get_u32(buf + 72);
        st->open_fds = get_u32(buf + 76);
        st->next_row = get_u64(buf + 80);
        st->flags = get_u32(buf + 88);
        return status_ok(st) ? ES_OK : ES_E_FIELD;
    }
    case ES_SUBSCRIBE: {
        struct es_subscribe *s = &out->u.subscribe;
        s->flags = get_u32(buf + 16);
        s->id_len = (uint16_t)get_u16(buf + 20);
        if (s->id_len > ES_MAX_SUB_ID)
            return ES_E_FIELD;
        if (len != es_subscribe_len(s->id_len))
            return ES_E_LENGTH;
        get_text(s->id, buf + ES_SUBSCRIBE_HDR_LEN, s->id_len);
        return ES_OK;
    }
    default:
        return ES_E_UNKNOWN_TYPE;
    }
}

const char *es_strerror(int result)
{
    switch (result) {
    case ES_OK: return "ok";
    case ES_E_SHORT: return "shorter than the common header";
    case ES_E_MAGIC: return "not an encstream packet (magic)";
    case ES_E_MAGIC_SWAPPED: return "magic in the wrong byte order (sender byte-order bug)";
    case ES_E_VERSION: return "unsupported protocol version";
    case ES_E_UNKNOWN_TYPE: return "unknown packet type";
    case ES_E_HDR_LEN: return "header length differs from the packet type's";
    case ES_E_LENGTH: return "datagram length differs from its counts";
    case ES_E_FIELD: return "field value out of range";
    case ES_E_LAYOUT: return "gather layout fails the layout checks";
    case ES_E_CAPACITY: return "output buffer too small";
    case ES_E_ARG: return "invalid argument";
    default: return "unknown result";
    }
}
