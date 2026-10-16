/* testReplay.cpp - recorded traffic through the receiver core.
 *
 * traces/mock_stream.estrace was recorded by encstream_rx.py --record from encstream running
 * against encstream's mock controller (10 kHz, about 0.8 s, ending with encstream's final STATUS);
 * traces/mock_stream.expected holds the facts encstream's host/encstream_replay.py printed for it.
 * The core must reproduce them exactly, including every recorded SUBSCRIBE byte for byte.
 *
 * Variants of that trace with loss, duplication, reordering and corruption are derived here in
 * memory, with expectations computed from the derivation itself. tools/differential_replay.sh
 * runs the same derivations through both receivers on a development host. */
#include <string.h>

#include <string>
#include <vector>

#include <epicsUnitTest.h>
#include <testMain.h>

#include "replay.h"

#ifndef ES_TRACE_DIR
#error "ES_TRACE_DIR must name the trace fixture directory"
#endif

using namespace encstream;

namespace {

typedef std::vector<uint8_t> Bytes;

std::string path(const char *name) { return std::string(ES_TRACE_DIR) + "/" + name; }

Bytes readAll(const std::string &p)
{
    Bytes b;
    FILE *f = fopen(p.c_str(), "rb");
    if (!f) {
        testAbort("cannot open %s", p.c_str());
        return b;
    }
    unsigned char chunk[65536];
    size_t n;
    while ((n = fread(chunk, 1, sizeof chunk, f)) > 0)
        b.insert(b.end(), chunk, chunk + n);
    fclose(f);
    return b;
}

/* a trace as a list of whole records (12-byte header + payload), after the 8-byte magic */
struct Rec { Bytes bytes; };

std::vector<Rec> split(const Bytes &b)
{
    std::vector<Rec> out;
    size_t pos = 8;
    while (pos + 12 <= b.size()) {
        size_t n = b[pos + 10] | (b[pos + 11] << 8);
        Rec r;
        r.bytes.assign(b.begin() + pos, b.begin() + pos + 12 + n);
        out.push_back(r);
        pos += 12 + n;
    }
    return out;
}

Bytes join(const std::vector<Rec> &rs)
{
    Bytes b(8);
    memcpy(&b[0], "ESTRACE1", 8);
    for (size_t i = 0; i < rs.size(); ++i)
        b.insert(b.end(), rs[i].bytes.begin(), rs[i].bytes.end());
    return b;
}

uint8_t kind(const Rec &r) { return r.bytes[8]; }

/* a RECEIVED record carrying an encstream DATA packet */
bool isData(const Rec &r)
{
    return kind(r) == TRACE_RECEIVED && r.bytes.size() > 12 + 10 + 16 && r.bytes[12 + 10 + 5] == ES_DATA;
}

unsigned dataRows(const Rec &r)
{
    const uint8_t *p = &r.bytes[12 + 10];      /* the datagram */
    return p[24] | (p[25] << 8);
}

/* the same record turned into a TIMER at the same time: the datagram never arrived */
Rec asTimer(const Rec &r)
{
    Rec t;
    t.bytes.assign(r.bytes.begin(), r.bytes.begin() + 12);
    t.bytes[8] = TRACE_TIMER;
    t.bytes[9] = 0;
    t.bytes[10] = t.bytes[11] = 0;
    return t;
}

bool replayBuffer(const Bytes &b, replay::Facts *facts, std::vector<std::string> *mismatches)
{
    TraceReader reader;
    std::string error;
    if (!reader.openBuffer(&b[0], b.size(), &error) || !replay::run(reader, facts, mismatches, &error)) {
        testDiag("replay failed: %s", error.c_str());
        return false;
    }
    return true;
}

std::string fact(const replay::Facts &f, const char *key)
{
    for (size_t i = 0; i < f.size(); ++i)
        if (f[i].first == key)
            return f[i].second;
    return "?";
}

uint64_t factNum(const replay::Facts &f, const char *key)
{
    return strtoull(fact(f, key).c_str(), 0, 10);
}

void testRecordedTrace()
{
    replay::Facts expected, got;
    std::vector<std::string> mismatches;
    testOk(replay::readFacts(path("mock_stream.expected").c_str(), &expected) && expected.size() == 20,
           "the Python receiver's facts for the recorded trace");
    Bytes b = readAll(path("mock_stream.estrace"));
    testOk1(replayBuffer(b, &got, &mismatches));
    testOk(mismatches.empty(), "every recorded SUBSCRIBE reproduced byte for byte (%u mismatches)",
           (unsigned)mismatches.size());
    for (size_t i = 0; i < mismatches.size() && i < 5; ++i)
        testDiag("%s", mismatches[i].c_str());
    bool same = got.size() == expected.size();
    for (size_t i = 0; same && i < got.size(); ++i) {
        same = got[i] == expected[i];
        if (!same)
            testDiag("%s: core %s, Python %s", got[i].first.c_str(), got[i].second.c_str(),
                     expected[i].second.c_str());
    }
    testOk(same, "the core's facts equal the Python receiver's on the recorded trace");
    testOk(fact(got, "cursor") == fact(got, "final_next_row") && factNum(got, "rows_lost") == 0 &&
           factNum(got, "rows") == factNum(got, "cursor") - factNum(got, "first_row"),
           "complete: %s rows, cursor %s = final next row", fact(got, "rows").c_str(),
           fact(got, "cursor").c_str());
}

void testDerived()
{
    const std::vector<Rec> base = split(readAll(path("mock_stream.estrace")));
    replay::Facts f0, f;
    std::vector<std::string> mm;
    replayBuffer(join(base), &f0, &mm);
    const uint64_t rows0 = factNum(f0, "rows"), first0 = factNum(f0, "first_row");

    /* indices of DATA records; skip the first few so the stream is established */
    std::vector<size_t> data;
    for (size_t i = 0; i < base.size(); ++i)
        if (isData(base[i]))
            data.push_back(i);
    testOk(data.size() > 500, "%u DATA records in the recorded trace", (unsigned)data.size());

    /* loss: every 50th DATA (never the first or the last) becomes a receive timeout */
    {
        std::vector<Rec> v = base;
        uint64_t lost = 0;
        unsigned dropped = 0;
        for (size_t k = 25; k + 1 < data.size(); k += 50) {
            lost += dataRows(v[data[k]]);
            v[data[k]] = asTimer(v[data[k]]);
            ++dropped;
        }
        mm.clear();
        testOk1(replayBuffer(join(v), &f, &mm));
        testOk(factNum(f, "rows_lost") == lost && factNum(f, "rows") == rows0 - lost &&
               factNum(f, "data_packets") == data.size() - dropped && fact(f, "faulted") == "0" &&
               fact(f, "cursor") == fact(f0, "cursor") && mm.empty(),
               "loss: %u packets, %llu rows lost and counted, end reached", dropped,
               (unsigned long long)lost);
    }
    /* duplication: every 40th DATA arrives twice */
    {
        std::vector<Rec> v;
        unsigned dups = 0;
        for (size_t i = 0, k = 0; i < base.size(); ++i) {
            v.push_back(base[i]);
            if (k < data.size() && i == data[k]) {
                if (k % 40 == 20) {
                    v.push_back(base[i]);
                    ++dups;
                }
                ++k;
            }
        }
        mm.clear();
        testOk1(replayBuffer(join(v), &f, &mm));
        testOk(factNum(f, "late_or_duplicate") == dups && factNum(f, "rows") == rows0 &&
               factNum(f, "rows_lost") == 0 && factNum(f, "sequence_gaps") == dups &&
               fact(f, "faulted") == "0" && mm.empty(),
               "duplication: %u duplicates dropped, rows unchanged", dups);
    }
    /* reordering: every 30th DATA swaps contents with the next one (times stay in place) */
    {
        std::vector<Rec> v = base;
        uint64_t lateRows = 0;
        unsigned swaps = 0;
        for (size_t k = 15; k + 2 < data.size(); k += 30) {
            Rec &a = v[data[k]], &b = v[data[k + 1]];
            Bytes ah(a.bytes.begin(), a.bytes.begin() + 8), bh(b.bytes.begin(), b.bytes.begin() + 8);
            std::swap(a.bytes, b.bytes);
            memcpy(&a.bytes[0], &ah[0], 8);    /* keep each slot's time */
            memcpy(&b.bytes[0], &bh[0], 8);
            lateRows += dataRows(b);           /* the earlier packet now arrives second: late */
            ++swaps;
        }
        mm.clear();
        testOk1(replayBuffer(join(v), &f, &mm));
        testOk(factNum(f, "late_or_duplicate") == swaps && factNum(f, "rows_lost") == lateRows &&
               factNum(f, "rows") == rows0 - lateRows && fact(f, "faulted") == "0" && mm.empty(),
               "reordering: %u swaps, each a late packet plus a counted gap (no reordering)", swaps);
    }
    /* corruption: the magic of every 60th received datagram damaged */
    {
        std::vector<Rec> v = base;
        uint64_t lost = 0;
        unsigned corrupted = 0, lastData = (unsigned)data.back();
        for (size_t i = 30; i < base.size(); i += 60) {
            if (kind(v[i]) != TRACE_RECEIVED || i == lastData || i == data.front())
                continue;
            if (isData(v[i]))
                lost += dataRows(v[i]);
            v[i].bytes[12 + 10] ^= 0xFF;
            ++corrupted;
        }
        mm.clear();
        testOk1(replayBuffer(join(v), &f, &mm));
        testOk(factNum(f, "rejected") == corrupted && factNum(f, "rows_lost") == lost &&
               factNum(f, "rows") == rows0 - lost && fact(f, "faulted") == "0" &&
               factNum(f, "first_row") == first0 && mm.empty(),
               "corruption: %u datagrams rejected, %llu rows lost", corrupted, (unsigned long long)lost);
    }
}

}

MAIN(testReplay)
{
    testPlan(0);
    testRecordedTrace();
    testDerived();
    return testDone();
}
