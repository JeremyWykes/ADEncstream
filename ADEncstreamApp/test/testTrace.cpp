/* testTrace.cpp - esTrace reader and writer against the hand-written golden trace
 * (traces/golden.hex, the ADEncstream guide). encstream's Python writer reproduces the same
 * bytes (host/tests/test_trace.py), so the two implementations agree through the format text. */
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include <epicsUnitTest.h>
#include <testMain.h>

#include "esTrace.h"

#ifndef ES_TRACE_DIR
#error "ES_TRACE_DIR must name the trace fixture directory"
#endif

using namespace encstream;

namespace {

std::vector<uint8_t> loadHex(const char *name)
{
    std::vector<uint8_t> b;
    std::string path = std::string(ES_TRACE_DIR) + "/" + name;
    FILE *f = fopen(path.c_str(), "r");
    if (!f) {
        testAbort("cannot open %s", path.c_str());
        return b;
    }
    char line[512];
    while (fgets(line, sizeof line, f)) {
        char *p = line, *hash = strchr(line, '#');
        if (hash)
            *hash = '\0';
        unsigned v;
        int used;
        while (sscanf(p, " %2x%n", &v, &used) == 1) {
            b.push_back((uint8_t)v);
            p += used;
        }
    }
    fclose(f);
    return b;
}

std::vector<uint8_t> readFile(const char *path)
{
    std::vector<uint8_t> b;
    FILE *f = fopen(path, "rb");
    if (!f)
        return b;
    int c;
    while ((c = fgetc(f)) != EOF)
        b.push_back((uint8_t)c);
    fclose(f);
    return b;
}

Endpoint ep(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint16_t port)
{
    Endpoint e;
    e.addr[0] = a; e.addr[1] = b; e.addr[2] = c; e.addr[3] = d;
    e.port = port;
    return e;
}

/* true if reading the whole buffer ends in an error (a rejected trace) */
bool rejected(const std::vector<uint8_t> &b)
{
    TraceReader r;
    if (!r.openBuffer(b.empty() ? (const uint8_t *)"" : &b[0], b.size(), 0))
        return true;
    TraceRecord rec;
    while (r.next(&rec)) {
    }
    return !r.error().empty();
}

void putRecord(std::vector<uint8_t> *b, uint64_t t, uint8_t kind, uint8_t flags,
               const std::vector<uint8_t> &payload)
{
    for (int i = 0; i < 8; ++i)
        b->push_back((uint8_t)(t >> (8 * i)));
    b->push_back(kind);
    b->push_back(flags);
    b->push_back((uint8_t)payload.size());
    b->push_back((uint8_t)(payload.size() >> 8));
    b->insert(b->end(), payload.begin(), payload.end());
}

void testGoldenRead()
{
    std::vector<uint8_t> g = loadHex("golden.hex");
    testOk(g.size() == 154, "golden trace: 154 bytes");
    TraceReader r;
    std::string err;
    testOk(r.openBuffer(&g[0], g.size(), &err), "golden trace opens (%s)", err.c_str());
    TraceRecord x;

    testOk(r.next(&x) && x.kind == TRACE_CONFIG && x.tNs == 0 && x.config.size() == 2 &&
           x.config[0].first == "brick" && x.config[0].second == "192.0.2.1:5199" &&
           x.config[1].first == "local_port" && x.config[1].second == "5198",
           "CONFIG: brick and local_port, in order");
    testOk(r.config().size() == 2, "config() keeps the CONFIG");
    testOk(r.next(&x) && x.kind == TRACE_TIMER && x.tNs == 0 && x.dataLength == 0, "TIMER at 0");
    testOk(r.next(&x) && x.kind == TRACE_RECEIVED && x.tNs == 1000000 && x.flags == 0 &&
           x.peer == ep(192, 0, 2, 1, 5199) && x.originalLength == 4 && x.dataLength == 4 &&
           memcmp(x.data, "ENCS", 4) == 0, "RECEIVED at 1 ms from 192.0.2.1:5199, 'ENCS'");
    testOk(r.next(&x) && x.kind == TRACE_SENT && x.tNs == 2000000 && x.flags == TRACE_SEND_FAILED &&
           x.peer == ep(192, 0, 2, 1, 5199) && x.dataLength == 2 && x.data[0] == 0xAA &&
           x.data[1] == 0xBB, "SENT at 2 ms, failed, AA BB");
    testOk(r.next(&x) && x.kind == TRACE_RECEIVED && x.tNs == 3000000 &&
           x.flags == TRACE_TRUNCATED && x.peer == ep(10, 0, 0, 7, 40000) &&
           x.originalLength == 1500 && x.dataLength == 3, "RECEIVED at 3 ms, truncated, 1500 -> 3");
    testOk(r.next(&x) && x.kind == TRACE_END && x.tNs == 4000000, "END at 4 ms");
    testOk(!r.next(&x) && r.error().empty(), "then the end, without error");
}

void testWriterReproducesGolden()
{
    std::vector<uint8_t> g = loadHex("golden.hex");
    const char *path = "testTrace.estrace";
    TraceWriter w;
    TraceConfig cfg;
    cfg.push_back(std::make_pair(std::string("brick"), std::string("192.0.2.1:5199")));
    cfg.push_back(std::make_pair(std::string("local_port"), std::string("5198")));
    std::string err;
    bool ok = w.open(path, cfg, 0, &err);
    ok = ok && w.timer(0);
    ok = ok && w.received(1000000, ep(192, 0, 2, 1, 5199), (const uint8_t *)"ENCS", 4, false, 0);
    const uint8_t ab[2] = { 0xAA, 0xBB };
    ok = ok && w.sent(2000000, ep(192, 0, 2, 1, 5199), ab, 2, false);
    const uint8_t ones[3] = { 1, 1, 1 };
    ok = ok && w.received(3000000, ep(10, 0, 0, 7, 40000), ones, 3, true, 1500);
    ok = ok && w.end(4000000);
    testOk(ok, "writer: every record written (%s)", err.c_str());
    testOk(readFile(path) == g, "writer output equals the golden trace byte for byte");
    testOk(!w.timer(5000000), "no record after END");
    remove(path);
}

void testWriterRules()
{
    TraceWriter w;
    TraceConfig bad;
    bad.push_back(std::make_pair(std::string("a=b"), std::string("c")));
    testOk(!w.open("testTrace2.estrace", bad, 0, 0), "CONFIG key with '=' refused");
    TraceWriter w2;
    TraceConfig cfg;
    std::string err;
    testOk(w2.open("testTrace2.estrace", cfg, 100, &err) && w2.timer(50) && w2.end(60),
           "times below the previous one are written");
    TraceReader r;
    testOk(r.openFile("testTrace2.estrace", &err), "and read back");
    TraceRecord x;
    bool monotonic = true;
    uint64_t last = 0;
    while (r.next(&x)) {
        monotonic = monotonic && x.tNs >= last && x.tNs == 100;
        last = x.tNs;
    }
    testOk(monotonic && r.error().empty(), "stored non-decreasing (all at 100 ns)");
    TraceWriter w3;
    std::vector<uint8_t> big(70000, 0);
    testOk(w3.open("testTrace2.estrace", cfg, 0, 0) &&
           !w3.received(1, ep(1, 2, 3, 4, 5), &big[0], big.size(), false, 0),
           "a datagram over 65535 bytes is refused");
    remove("testTrace2.estrace");
}

void testRejections()
{
    std::vector<uint8_t> g = loadHex("golden.hex");
    const size_t end = g.size() - 12;         // offset of the END record
    std::vector<uint8_t> b;

    testOk(!rejected(g), "the golden trace is accepted");
    b = g; b[7] = '2';
    testOk(rejected(b), "wrong magic");
    b.assign(g.begin(), g.begin() + end);
    testOk(rejected(b), "no END");
    b.assign(g.begin(), g.end() - 1);
    testOk(rejected(b), "truncated last record");
    b = g; putRecord(&b, 9000000, TRACE_TIMER, 0, std::vector<uint8_t>());
    testOk(rejected(b), "record after END");
    b.assign(g.begin(), g.begin() + 8);
    b.insert(b.end(), g.begin() + 8 + 12 + 37, g.end());
    testOk(rejected(b), "CONFIG missing");
    b.assign(g.begin(), g.begin() + end);
    putRecord(&b, 3999999, TRACE_TIMER, 0, std::vector<uint8_t>());   // after the 3 ms record
    b.insert(b.end(), g.begin() + end, g.end());
    testOk(!rejected(b), "a TIMER between 3 ms and END is fine");
    b.assign(g.begin(), g.begin() + end);
    putRecord(&b, 2999999, TRACE_TIMER, 0, std::vector<uint8_t>());   // earlier than 3 ms record
    b.insert(b.end(), g.begin() + end, g.end());
    testOk(rejected(b), "time going backwards");
    b.assign(g.begin(), g.begin() + end);
    putRecord(&b, 4000000, TRACE_CONFIG, 0, std::vector<uint8_t>(3, 'a'));
    b.insert(b.end(), g.begin() + end, g.end());
    testOk(rejected(b), "a second CONFIG");
    b.assign(g.begin(), g.begin() + end);
    putRecord(&b, 4000000, 9, 0, std::vector<uint8_t>());
    b.insert(b.end(), g.begin() + end, g.end());
    testOk(rejected(b), "unknown record kind");
    b.assign(g.begin(), g.begin() + end);
    putRecord(&b, 4000000, TRACE_TIMER, 1, std::vector<uint8_t>());
    b.insert(b.end(), g.begin() + end, g.end());
    testOk(rejected(b), "TIMER with flags");
    b.assign(g.begin(), g.begin() + end);
    std::vector<uint8_t> sent(8, 0);
    sent[6] = 1;                              // reserved field
    putRecord(&b, 4000000, TRACE_SENT, 0, sent);
    b.insert(b.end(), g.begin() + end, g.end());
    testOk(rejected(b), "SENT with a nonzero reserved field");
    b.assign(g.begin(), g.begin() + end);
    putRecord(&b, 4000000, TRACE_RECEIVED, 2, std::vector<uint8_t>(10, 0));
    b.insert(b.end(), g.begin() + end, g.end());
    testOk(rejected(b), "RECEIVED with an unknown flag");
    b.assign(g.begin(), g.begin() + end);
    putRecord(&b, 4000000, TRACE_RECEIVED, 0, std::vector<uint8_t>(9, 0));
    b.insert(b.end(), g.begin() + end, g.end());
    testOk(rejected(b), "RECEIVED shorter than its fixed part");
    TraceReader r;
    testOk(!r.openFile("no-such-trace.estrace", 0), "a missing file is an error");
}

}

MAIN(testTrace)
{
    testPlan(33);
    testGoldenRead();
    testWriterReproducesGolden();
    testWriterRules();
    testRejections();
    return testDone();
}
