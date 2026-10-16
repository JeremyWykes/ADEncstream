/* replay.h - replay a receiver trace through the receiver core.
 * Test-only; used by testReplay and the esReplay tool. The facts printed are the named mapping
 * that encstream's host/encstream_replay.py prints for the Python receiver. */
#ifndef ENCSTREAM_REPLAY_H
#define ENCSTREAM_REPLAY_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <utility>
#include <vector>

#include "esReceiver.h"

namespace replay {

using namespace encstream;

struct NullSink : public RowSink {
    void gap(const GapReport &) {}
    void rows(const RowBlock &) {}
    void event(const StreamEvent &) {}
};

typedef std::vector<std::pair<std::string, std::string> > Facts;

inline std::string num(uint64_t v)
{
    char b[32];
    sprintf(b, "%llu", (unsigned long long)v);
    return b;
}

inline const std::string *get(const TraceConfig &c, const char *key)
{
    for (size_t i = 0; i < c.size(); ++i)
        if (c[i].first == key)
            return &c[i].second;
    return 0;
}

/* the receiver configuration a trace was recorded with */
inline bool configFrom(const TraceConfig &c, ReceiverConfig *out, std::string *error)
{
    const std::string *brick = get(c, "brick");
    unsigned a, b, cc, d, port;
    char extra;
    if (!brick || sscanf(brick->c_str(), "%u.%u.%u.%u:%u%c", &a, &b, &cc, &d, &port, &extra) != 5 ||
        a > 255 || b > 255 || cc > 255 || d > 255 || port > 65535) {
        *error = "CONFIG has no valid brick=a.b.c.d:port";
        return false;
    }
    out->brick.addr[0] = (uint8_t)a; out->brick.addr[1] = (uint8_t)b;
    out->brick.addr[2] = (uint8_t)cc; out->brick.addr[3] = (uint8_t)d;
    out->brick.port = (uint16_t)port;
    if (const std::string *s = get(c, "identification"))
        out->identification = *s;
    if (const std::string *s = get(c, "subscribe_period_ns"))
        out->subscribePeriodNs = strtoull(s->c_str(), 0, 10);
    if (const std::string *s = get(c, "contact_timeout_ns"))
        out->contactTimeoutNs = strtoull(s->c_str(), 0, 10);
    if (const std::string *s = get(c, "data_timeout_ns"))
        out->dataTimeoutNs = strtoull(s->c_str(), 0, 10);
    *error = checkConfig(*out);
    return error->empty();
}

/* Replays a trace: RECEIVED -> onDatagram, TIMER and END -> onTimer, SENT -> the receiver's
 * pending SUBSCRIBE must equal the recorded bytes, then sent(ok). A SUBSCRIBE the core prepared
 * but the recorder never sent is a mismatch too. Returns false (with *error) if the trace cannot
 * be replayed; mismatches are listed separately. */
inline bool run(TraceReader &reader, Facts *facts, std::vector<std::string> *mismatches,
                std::string *error)
{
    TraceRecord x;
    if (!reader.next(&x) || x.kind != TRACE_CONFIG) {
        *error = reader.error().empty() ? "no CONFIG" : reader.error();
        return false;
    }
    ReceiverConfig cfg;
    if (!configFrom(x.config, &cfg, error))
        return false;
    NullSink sink;
    Receiver rx(cfg, sink);
    uint64_t matched = 0;
    const uint8_t *buf;
    size_t len;
    while (reader.next(&x)) {
        if (x.kind == TRACE_RECEIVED || x.kind == TRACE_TIMER || x.kind == TRACE_END) {
            if (rx.pendingSend(&buf, &len))
                mismatches->push_back("a SUBSCRIBE prepared before " + num(x.tNs) +
                                      " ns was never sent by the recorder");
            if (x.kind == TRACE_RECEIVED)
                rx.onDatagram(x.data, x.dataLength, (x.flags & TRACE_TRUNCATED) != 0, x.peer, x.tNs);
            else
                rx.onTimer(x.tNs);
        } else if (x.kind == TRACE_SENT) {
            if (!rx.pendingSend(&buf, &len))
                mismatches->push_back("SENT at " + num(x.tNs) + " ns: no SUBSCRIBE pending");
            else if (len != x.dataLength || memcmp(buf, x.data, len) != 0)
                mismatches->push_back("SENT at " + num(x.tNs) + " ns: bytes differ");
            else
                ++matched;
            rx.sent((x.flags & TRACE_SEND_FAILED) == 0, x.tNs);
        }
    }
    if (!reader.error().empty()) {
        *error = reader.error();
        return false;
    }
    const Counters &c = rx.counters();
    StreamView v = rx.current();
    uint64_t rejected = c.unknownType + c.unexpectedType + c.oversize + c.truncated + c.rejectedRowSize;
    for (size_t i = 0; i < sizeof c.rejected / sizeof c.rejected[0]; ++i)
        rejected += c.rejected[i];
    char stream[16];
    sprintf(stream, "%08x", (unsigned)v.stream);
    Facts &f = *facts;
    f.clear();
    f.push_back(std::make_pair("rows", num(c.rowsAdmitted)));
    f.push_back(std::make_pair("data_packets", num(c.dataPackets)));
    f.push_back(std::make_pair("rows_lost", num(c.rowsLost)));
    f.push_back(std::make_pair("rows_skipped", num(c.rowsSkippedByBrick)));
    f.push_back(std::make_pair("late_or_duplicate", num(c.lateOrDuplicate)));
    f.push_back(std::make_pair("stream_changes", num(c.streamChanges)));
    f.push_back(std::make_pair("sequence_gaps", num(c.sequenceGaps)));
    f.push_back(std::make_pair("retired_stream", num(c.retiredStream)));
    f.push_back(std::make_pair("before_context", num(c.beforeContext)));
    f.push_back(std::make_pair("data_before_calibration", num(c.dataBeforeCalibration)));
    f.push_back(std::make_pair("data_after_fault", num(c.dataAfterFault)));
    f.push_back(std::make_pair("unknown_generation", num(c.unknownGeneration)));
    f.push_back(std::make_pair("foreign_source", num(c.foreignSource)));
    f.push_back(std::make_pair("rejected", num(rejected)));
    f.push_back(std::make_pair("subscribes", num(matched)));
    f.push_back(std::make_pair("stream", v.hasStream ? std::string(stream) : std::string("-")));
    f.push_back(std::make_pair("faulted", std::string(v.fault != FAULT_NONE ? "1" : "0")));
    f.push_back(std::make_pair("first_row", v.hasRows ? num(v.firstRow) : std::string("-")));
    f.push_back(std::make_pair("cursor", v.hasRows ? num(v.cursor) : std::string("-")));
    f.push_back(std::make_pair("final_next_row", v.hasFinal ? num(v.final.next_row) : std::string("-")));
    return true;
}

/* key=value lines (as encstream_replay.py prints them) */
inline bool readFacts(const char *path, Facts *out)
{
    FILE *f = fopen(path, "r");
    if (!f)
        return false;
    char line[256];
    out->clear();
    while (fgets(line, sizeof line, f)) {
        char *eq = strchr(line, '=');
        if (!eq)
            continue;
        *eq = '\0';
        char *v = eq + 1;
        v[strcspn(v, "\r\n")] = '\0';
        out->push_back(std::make_pair(std::string(line), std::string(v)));
    }
    fclose(f);
    return true;
}

}

#endif
