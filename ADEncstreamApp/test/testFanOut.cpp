/* testFanOut.cpp - the fan-out and the independence of the two consumers: the recorded trace replayed through receiver -> fan-out -> framer (Free Running) and
 * monitor, with different window lengths, gives exactly what each consumer gives alone. */
#include <string.h>

#include <string>
#include <vector>

#include <epicsUnitTest.h>
#include <testMain.h>

#include "esFanOut.h"
#include "esFramer.h"
#include "esMonitor.h"

#ifndef ES_TRACE_DIR
#error "ES_TRACE_DIR must name the trace fixture directory"
#endif

using namespace encstream;

namespace {

struct Frames : public FrameOutput {
    std::vector<Frame> pool;
    std::vector<Frame *> freeList;
    std::vector<std::string> log;             /* number:samples:mean of axis 0 */
    Framer *fr;
    Frames() : fr(0)
    {
        pool.resize(4);
        for (size_t i = 0; i < pool.size(); ++i) {
            pool[i].capacity = 2000;
            pool[i].position.assign((size_t)FRAMER_MAX_AXES * 2000, 0);
            freeList.push_back(&pool[i]);
        }
    }
    Frame *acquire()
    {
        if (freeList.empty())
            return 0;
        Frame *f = freeList.back();
        freeList.pop_back();
        return f;
    }
    bool push(Frame *f)                       /* published at once: an instant worker */
    {
        double s = 0;
        for (uint32_t i = 0; i < f->samples; ++i)
            s += f->position[i];
        char b[96];
        sprintf(b, "%llu:%u:%.9g", (unsigned long long)f->number, f->samples, f->samples ? s / f->samples : 0.0);
        log.push_back(b);
        freeList.push_back(f);
        if (fr)
            fr->published(f->epoch, 1, true);
        return true;
    }
    bool pushEmptyRun(const EmptyRun &r)
    {
        char b[64];
        sprintf(b, "run %llu+%llu", (unsigned long long)r.firstNumber, (unsigned long long)r.count);
        log.push_back(b);
        return true;
    }
    void release(Frame *f) { freeList.push_back(f); }
    void stopped(uint64_t) {}
};

struct Updates : public MonitorOutput {
    std::vector<std::string> log;
    void publish(const MonitorUpdate &u)
    {
        if (!u.values)
            return;
        char b[128];
        sprintf(b, "%llu/%.12g/%.12g/%d", (unsigned long long)u.samples, u.slot[0].mean, u.slot[0].stddev,
                (int)u.health);
        log.push_back(b);
    }
};

struct Lookup : public CalibrationLookup {
    const Receiver *rx;
    bool calibration(uint32_t g, Calibration *out) const { return rx && rx->calibration(g, out); }
};

struct NullSink : public RowSink {
    void gap(const GapReport &) {}
    void rows(const RowBlock &) {}
    void event(const StreamEvent &) {}
};

/* replay the recorded trace; arm the framer and start the monitor once the stream is ready */
struct Run {
    Frames frames;
    Updates updates;
    Lookup lookup;
    Framer framer;
    Monitor monitor;
    Run() : framer(frames, lookup), monitor(updates, lookup) { frames.fr = &framer; lookup.rx = 0; }
};

void replay(RowSink &sink, Run &run, bool useFramer, bool useMonitor)
{
    std::vector<uint8_t> raw;
    std::string p = std::string(ES_TRACE_DIR) + "/mock_stream.estrace";
    FILE *f = fopen(p.c_str(), "rb");
    if (!f) {
        testAbort("cannot open %s", p.c_str());
        return;
    }
    int c;
    while ((c = fgetc(f)) != EOF)
        raw.push_back((uint8_t)c);
    fclose(f);
    TraceReader reader;
    std::string err;
    reader.openBuffer(&raw[0], raw.size(), &err);
    TraceRecord x;
    reader.next(&x);
    ReceiverConfig cfg;
    cfg.brick.addr[0] = 127; cfg.brick.addr[1] = 0; cfg.brick.addr[2] = 0; cfg.brick.addr[3] = 1;
    cfg.brick.port = 25199;
    cfg.identification = "rx=encstream_rx.py";
    Receiver rx(cfg, sink);
    run.lookup.rx = &rx;
    bool armed = false;
    while (reader.next(&x)) {
        if (x.kind == TRACE_RECEIVED)
            rx.onDatagram(x.data, x.dataLength, false, x.peer, x.tNs);
        else if (x.kind == TRACE_TIMER || x.kind == TRACE_END)
            rx.onTimer(x.tNs);
        const uint8_t *b; size_t n;
        while (rx.pendingSend(&b, &n))
            rx.sent(true, x.tNs);
        if (!armed && rx.current().ready) {
            armed = true;
            StreamView v = rx.current();
            if (useFramer) {
                FramerConfig fc;
                fc.trigger = TRIGGER_FREE_RUNNING;
                fc.framePoints = 1000;        /* 100 ms frames */
                fc.maxFrameSamples = 2000;
                std::string why = run.framer.acquire(fc, v);
                if (!why.empty())
                    testDiag("acquire: %s", why.c_str());
            }
            if (useMonitor) {
                MonitorConfig mc;
                mc.averagingTimeS = 0.03;     /* 30 ms, sliding every 10 ms */
                mc.updatePeriodS = 0.01;
                run.monitor.configure(mc, v);
                run.monitor.start(v);
            }
        }
    }
    run.lookup.rx = 0;
}

void testIndependence()
{
    Run both, framerOnly, monitorOnly;
    FanOut fan(both.framer, both.monitor);
    replay(fan, both, true, true);
    replay(framerOnly.framer, framerOnly, true, false);
    replay(monitorOnly.monitor, monitorOnly, false, true);
    testOk(both.frames.log.size() > 5 && both.updates.log.size() > 50,
           "the recorded trace gives %u frames and %u monitor updates", (unsigned)both.frames.log.size(),
           (unsigned)both.updates.log.size());
    testOk(both.frames.log == framerOnly.frames.log, "frames with the monitor running = frames alone");
    testOk(both.updates.log == monitorOnly.updates.log, "monitor updates with Free Running = monitor alone");
    testOk(both.framer.status().numAcquired == both.frames.log.size() && both.framer.status().state == ACQ_ACQUIRING,
           "every frame published; still acquiring (Continuous)");
}

}

MAIN(testFanOut)
{
    testPlan(0);
    testIndependence();
    return testDone();
}
