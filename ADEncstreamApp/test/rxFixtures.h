/* rxFixtures.h - test fixtures for the receiver core: a Brick that builds encstream packets with
 * the vendored encoder, and a sink that records every call in order. Test-only. */
#ifndef ENCSTREAM_RX_FIXTURES_H
#define ENCSTREAM_RX_FIXTURES_H

#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "esReceiver.h"

namespace fx {

using namespace encstream;
typedef std::vector<uint8_t> Bytes;

inline Endpoint endpoint(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint16_t port)
{
    Endpoint e;
    e.addr[0] = a; e.addr[1] = b; e.addr[2] = c; e.addr[3] = d;
    e.port = port;
    return e;
}

inline Endpoint brickEndpoint() { return endpoint(192, 0, 2, 1, 5199); }

const uint64_t MS = 1000000ull, S = 1000000000ull;

/* One stream's packets: item 0 clock (Type 0), optional gate word (Type 0, bit 11), positions
 * (Type 5) for `motors`, optional status words (Type 0, role 4) and other items; rows little-
 * endian unless `big`. Row r carries ServoCount clock0 + r * period, gate bit = r % 2, position
 * of motor m = r + m / 10, status word of motor m = 0xA0000000 + m * 256 + r. */
struct Brick {
    uint32_t stream, seq;
    std::vector<uint8_t> motors, statusMotors;
    std::vector<uint16_t> otherTypes;          /* role-5 items: Gather.Type each */
    bool gate, big;
    uint16_t period;
    uint32_t clock0;

    explicit Brick(uint32_t id = 0x11111111u)
        : stream(id), seq(0), gate(false), big(false), period(1), clock0(1000)
    {
        motors.push_back(1);
    }

    es_common header(uint8_t type) { es_common h; h.type = type; h.stream_id = stream; h.seq = seq++; return h; }

    es_context contextStruct()
    {
        es_context c;
        memset(&c, 0, sizeof c);
        c.byte_order = big ? ES_BIG : ES_LITTLE;
        c.servo_period_s = 1e-4;
        c.gather_period = period;
        c.max_lines = 1000;
        c.id_len = 9;
        memcpy(c.id, "host=test", 9);
        unsigned n = 0, words = 0;
        es_item *it = &c.items[n++];
        it->role = ES_ROLE_CLOCK; it->type = 0; it->words = 1; strcpy(it->name, "Sys.ServoCount");
        words += 1;
        if (gate) {
            it = &c.items[n++];
            it->role = ES_ROLE_GATE; it->gate_bit = 11; it->type = 0; it->words = 1;
            strcpy(it->name, "PowerBrick[0].Chan[0].Status");
            words += 1;
        }
        for (size_t i = 0; i < motors.size(); ++i) {
            it = &c.items[n++];
            it->role = ES_ROLE_POSITION; it->axis = motors[i]; it->type = 5; it->words = 2;
            sprintf(it->name, "Motor[%u].ActPos", (unsigned)motors[i]);
            words += 2;
        }
        for (size_t i = 0; i < statusMotors.size(); ++i) {
            it = &c.items[n++];
            it->role = ES_ROLE_ENCODER_STATUS; it->axis = statusMotors[i]; it->type = 0; it->words = 1;
            sprintf(it->name, "Acc84B[0].Chan[%u].SerialEncDataB", (unsigned)i);
            words += 1;
        }
        for (size_t i = 0; i < otherTypes.size(); ++i) {
            it = &c.items[n++];
            it->role = ES_ROLE_OTHER; it->type = otherTypes[i];
            it->words = (uint16_t)es_type_words(otherTypes[i]);
            sprintf(it->name, "Sys.Udata[%u]", (unsigned)i);
            words += it->words;
        }
        c.n_items = (uint8_t)n;
        c.row_bytes = (uint16_t)(4 * words);
        return c;
    }

    unsigned rowBytes() { return contextStruct().row_bytes; }

    Bytes context()
    {
        es_context c = contextStruct();
        es_common h = header(ES_CONTEXT);
        Bytes b(2048);
        size_t w = 0;
        es_encode_context(&b[0], b.size(), &h, &c, &w);
        b.resize(w);
        return b;
    }

    Bytes calibration(uint32_t gen = 1, double home = 0.0, const std::vector<uint8_t> *ms = 0)
    {
        const std::vector<uint8_t> &m = ms ? *ms : motors;
        es_calibration c;
        memset(&c, 0, sizeof c);
        c.generation = gen;
        c.n_axes = (uint16_t)m.size();
        for (size_t i = 0; i < m.size(); ++i) {
            c.axes[i].motor = m[i];
            c.axes[i].home_pos = home + m[i];
            c.axes[i].pos_sf = 1.0;
        }
        es_common h = header(ES_CALIBRATION);
        Bytes b(2048);
        size_t w = 0;
        es_encode_calibration(&b[0], b.size(), &h, &c, &w);
        b.resize(w);
        return b;
    }

    void putWord(Bytes &b, uint32_t v)
    {
        for (int i = 0; i < 4; ++i)
            b.push_back((uint8_t)(big ? v >> (24 - 8 * i) : v >> (8 * i)));
    }

    void putDouble(Bytes &b, double d)
    {
        uint64_t u;
        memcpy(&u, &d, 8);
        if (big) {                             /* high word first, each word big-endian */
            putWord(b, (uint32_t)(u >> 32));
            putWord(b, (uint32_t)u);
        } else {
            putWord(b, (uint32_t)u);
            putWord(b, (uint32_t)(u >> 32));
        }
    }

    /* rows [first, first + n); breakAt >= 0 puts a ServoCount jump at that row of the packet */
    Bytes data(uint64_t first, unsigned n = 2, uint32_t gen = 1, uint32_t skipped = 0,
               int breakAt = -1, int rowBytesOverride = -1)
    {
        Bytes rows;
        for (unsigned i = 0; i < n; ++i) {
            uint64_t r = first + i;
            uint32_t clock = (uint32_t)(clock0 + r * period);
            if (breakAt >= 0 && (int)i >= breakAt)
                clock += 7;
            putWord(rows, clock);
            if (gate)
                putWord(rows, (uint32_t)((r % 2) << 11) | 0xA5000000u);
            for (size_t m = 0; m < motors.size(); ++m)
                putDouble(rows, (double)r + motors[m] / 10.0);
            for (size_t m = 0; m < statusMotors.size(); ++m)
                putWord(rows, 0xA0000000u + statusMotors[m] * 256u + (uint32_t)r);
            for (size_t k = 0; k < otherTypes.size(); ++k) {
                if (otherTypes[k] == 5)
                    putDouble(rows, -0.5 * (double)r);
                else
                    putWord(rows, (uint32_t)(0xFFFFFF00u + r));
            }
        }
        es_data d;
        memset(&d, 0, sizeof d);
        d.first_row = first;
        d.n_rows = (uint16_t)n;
        d.row_bytes = (uint16_t)(rowBytesOverride >= 0 ? rowBytesOverride : (int)rowBytes());
        d.cal_generation = gen;
        d.flags = skipped ? ES_DATA_SKIPPED : 0;
        d.rows_skipped = skipped;
        es_common h = header(ES_DATA);
        Bytes b(ES_DATA_HDR_LEN + rows.size());
        memcpy(&b[ES_DATA_HDR_LEN], &rows[0], rows.size());
        size_t w = 0;
        if (es_encode_data_header(&b[0], b.size(), &h, &d, &w) != ES_OK)
            b.clear();
        return b;
    }

    Bytes status(uint16_t fault = 0, uint64_t nextRow = 0, bool final = false, uint64_t rowsSent = 0)
    {
        es_status s;
        memset(&s, 0, sizeof s);
        s.fault = fault;
        s.state = fault == 0 ? ES_STATE_STREAMING : fault == ES_FAULT_INTERNAL ? ES_STATE_FAULT_OTHER
                                                                              : ES_STATE_FAULT_GATHER;
        s.enable = fault == 0 ? 3 : 0;
        s.rows_sent = rowsSent;
        s.next_row = nextRow;
        s.flags = final ? ES_STATUS_FINAL : 0;
        es_common h = header(ES_STATUS);
        Bytes b(256);
        size_t w = 0;
        es_encode_status(&b[0], b.size(), &h, &s, &w);
        b.resize(w);
        return b;
    }

    Bytes report(uint32_t servo = 5000)
    {
        es_sender_report r;
        memset(&r, 0, sizeof r);
        r.servo_count = servo;
        r.wall_ns = 1700000000000000000ll;
        r.row = 10;
        es_common h = header(ES_SENDER_REPORT);
        Bytes b(64);
        size_t w = 0;
        es_encode_sender_report(&b[0], b.size(), &h, &r, &w);
        b.resize(w);
        return b;
    }
};

struct RowsCopy {
    uint32_t stream, gen;
    uint64_t first;
    uint32_t n;
    std::vector<uint64_t> tick;
    std::vector<uint32_t> servo, gateWord;
    std::vector<uint8_t> gate;
    std::vector<std::vector<double> > pos, other;
    std::vector<std::vector<uint32_t> > status;
    std::vector<uint8_t> axisMotor, statusMotor, otherItem;
    bool hasGate;
};

/* Records every sink call; `calls` gives their order: "gap", "rows", or the event type name. */
struct Sink : public RowSink {
    std::vector<GapReport> gaps;
    std::vector<RowsCopy> rowBlocks;
    std::vector<StreamEvent> events;
    std::vector<std::string> calls;

    void gap(const GapReport &g) { gaps.push_back(g); calls.push_back("gap"); }

    void rows(const RowBlock &b)
    {
        RowsCopy c;
        c.stream = b.stream; c.gen = b.calGeneration; c.first = b.firstRow; c.n = b.nRows;
        c.tick.assign(b.tick, b.tick + b.nRows);
        c.servo.assign(b.servoCount, b.servoCount + b.nRows);
        c.hasGate = b.gate != 0;
        if (b.gate) {
            c.gate.assign(b.gate, b.gate + b.nRows);
            c.gateWord.assign(b.gateWord, b.gateWord + b.nRows);
        }
        c.axisMotor.assign(b.axisMotor, b.axisMotor + b.nAxes);
        for (unsigned a = 0; a < b.nAxes; ++a)
            c.pos.push_back(std::vector<double>(b.position[a], b.position[a] + b.nRows));
        c.statusMotor.assign(b.statusMotor, b.statusMotor + b.nStatus);
        for (unsigned k = 0; k < b.nStatus; ++k)
            c.status.push_back(std::vector<uint32_t>(b.status[k], b.status[k] + b.nRows));
        c.otherItem.assign(b.otherItem, b.otherItem + b.nOther);
        for (unsigned k = 0; k < b.nOther; ++k)
            c.other.push_back(std::vector<double>(b.other[k], b.other[k] + b.nRows));
        rowBlocks.push_back(c);
        calls.push_back("rows");
    }

    void event(const StreamEvent &e)
    {
        static const char *const names[] = {
            "Replaced", "Started", "Calibration", "Faulted", "Ended", "TailComplete",
            "ContactLost", "ContactRestored", "DataStalled", "DataStallEnded"
        };
        events.push_back(e);
        calls.push_back(names[e.type]);
    }

    unsigned count(const char *what) const
    {
        unsigned n = 0;
        for (size_t i = 0; i < calls.size(); ++i)
            n += calls[i] == what;
        return n;
    }

    std::string sequence() const
    {
        std::string s;
        for (size_t i = 0; i < calls.size(); ++i)
            s += (i ? "," : "") + calls[i];
        return s;
    }

    void clear() { gaps.clear(); rowBlocks.clear(); events.clear(); calls.clear(); }
};

inline ReceiverConfig config()
{
    ReceiverConfig c;
    c.brick = brickEndpoint();
    c.identification = "ioc=test";
    return c;
}

/* A receiver, its sink and a clock; feed() delivers datagrams from the Brick 1 ms apart. */
struct Rig {
    Sink sink;
    Receiver rx;
    uint64_t t;

    explicit Rig(const ReceiverConfig &c = config()) : rx(c, sink), t(0) {}

    Outcome feed(const Bytes &b, uint64_t dt = MS)
    {
        t += dt;
        return rx.onDatagram(b.empty() ? (const uint8_t *)"" : &b[0], b.size(), false,
                             brickEndpoint(), t);
    }

    void start(Brick &br, uint32_t gen = 1) { feed(br.context()); feed(br.calibration(gen)); }

    /* SUBSCRIBEs handed out so far: their metadata flags, oldest first */
    std::vector<uint32_t> subscribes;
    void takeSends()
    {
        const uint8_t *b;
        size_t n;
        while (rx.pendingSend(&b, &n)) {
            es_packet p;
            if (es_decode(b, n, &p) == ES_OK && p.h.type == ES_SUBSCRIBE)
                subscribes.push_back(p.u.subscribe.flags);
            rx.sent(true, t);
        }
    }
};

}

#endif
