/* testEngine.cpp - the receiver thread's work (esEngine, 5.1-5.3) driven by the
 * recorded mock stream (traces/mock_stream.estrace: 8 axes, a gate, 10 kHz rows, 7010 rows, then a
 * final STATUS), with the callback thread's role played inline: take work, report it. */
#include <string.h>

#include <cmath>
#include <string>
#include <vector>

#include <epicsEvent.h>
#include <epicsUnitTest.h>
#include <testMain.h>

#include "esEngine.h"
#include "replay.h"

using namespace encstream;

namespace {

struct Rig {
    epicsEvent wake;
    FrameQueue q;
    CommandSlots cmds;
    Mailbox mb;
    TraceReader reader;
    EngineConfig cfg;
    Engine *eng;
    uint64_t now;
    bool ended;
    /* the callback thread's view */
    std::vector<uint64_t> numbers;
    std::vector<uint32_t> samples;
    std::vector<double> firstPos;
    uint64_t emptyFrames;
    bool failNext, paused;

    Rig(uint32_t cap = 2000, unsigned depth = 8)
        : q(cap, depth, wake), cmds(8), mb(wake), eng(0), now(0), ended(false), emptyFrames(0),
          failNext(false), paused(false)
    {
        std::string err;
        bool ok = reader.openFile(ES_TRACE_DIR "/mock_stream.estrace", &err);
        TraceRecord x;
        ok = ok && reader.next(&x) && x.kind == TRACE_CONFIG && replay::configFrom(x.config, &cfg.receiver, &err);
        if (!ok)
            testAbort("cannot open the mock trace: %s", err.c_str());
        eng = new Engine(cfg, q, cmds, mb);
    }
    ~Rig() { delete eng; }

    /* the callback thread: publish everything queued (or fail the next frame) */
    void drain()
    {
        Work w;
        while (!paused && q.take(&w)) {
            if (w.kind == Work::FRAME) {
                if (failNext) {
                    failNext = false;
                    q.done(w, 0, 1);
                    continue;
                }
                numbers.push_back(w.frame->number);
                samples.push_back(w.frame->samples);
                firstPos.push_back(w.frame->position[0]);
                q.done(w, 1, 0);
            } else {
                emptyFrames += w.run.count;
                q.done(w, w.run.count, 0);
            }
        }
    }
    /* one trace record, then the engine's service and the callback thread */
    bool step()
    {
        TraceRecord x;
        if (ended || !reader.next(&x)) {
            ended = true;
            return false;
        }
        now = x.tNs;
        if (x.kind == TRACE_RECEIVED)
            eng->datagram(x.data, x.dataLength, (x.flags & TRACE_TRUNCATED) != 0, x.peer, x.tNs);
        else if (x.kind == TRACE_SENT)
            eng->sent(true, x.tNs);
        else
            eng->timer(x.tNs);
        eng->service(now);
        drain();
        return true;
    }
    void run(unsigned records) { for (unsigned i = 0; i < records && step(); ++i) {} }
    void runToEnd() { while (step()) {} }
    void runUntilReady() { while (!eng->receiver().current().ready && step()) {} }
    /* a command applied at once (the receiver is this thread): submit, service, collect */
    CommandResult command(const Command &c)
    {
        int id = cmds.submit(c);
        eng->service(now);
        drain();
        CommandResult r;
        r.applied = false;
        if (id < 0 || cmds.wait(id, 0.0, &r) != WAIT_DONE)
            r.message = "no result";
        return r;
    }
    CommandResult acquire(const FramerConfig &f)
    {
        Command c;
        c.type = CMD_ACQUIRE;
        c.framer = f;
        return command(c);
    }
    CommandResult stop()
    {
        Command c;
        c.type = CMD_STOP;
        return command(c);
    }
    Snapshot latest()
    {
        Snapshot s = Snapshot();
        mb.takeSnapshot(&s);
        return s;
    }
};

FramerConfig freeRun(uint64_t points, AcquireMode mode = ACQUIRE_CONTINUOUS, uint64_t n = 1)
{
    FramerConfig c;
    c.trigger = TRIGGER_FREE_RUNNING;
    c.mode = mode;
    c.numAcquire = n;
    c.framePoints = points;
    c.maxFrameSamples = 2000;
    return c;
}

void testRefusedBeforeStream()
{
    Rig r;
    CommandResult a = r.acquire(freeRun(100));
    testOk(!a.applied && a.message == "no stream", "Acquire before the stream: refused (%s)", a.message.c_str());
    Snapshot s = r.latest();
    testOk(std::string(s.acqMessage).find("Acquire refused: no stream") == 0 && !s.active &&
           s.display == DISPLAY_IDLE, "the refusal is in AcqMessage; state idle, Acquire not held");
    FramerConfig big = freeRun(100);
    big.maxFrameSamples = 5000;
    r.runUntilReady();
    a = r.acquire(big);
    testOk(!a.applied && a.message.find("pool capacity") != std::string::npos,
           "MaxFrameSamples above the pool capacity: refused");
}

void testMultipleComplete()
{
    Rig r;
    r.runUntilReady();
    CommandResult a = r.acquire(freeRun(1000, ACQUIRE_MULTIPLE, 5));
    testOk(a.applied && r.eng->active() && r.eng->display() == DISPLAY_ACQUIRING, "Multiple N = 5 armed");
    Snapshot s = r.latest();
    testOk(s.revision >= a.revision && s.revision == r.eng->revision() && s.active,
           "R5: the state after applying is posted before the result (revision %llu)",
           (unsigned long long)a.revision);
    r.runToEnd();
    s = r.latest();
    bool full = r.samples.size() == 5;
    for (size_t i = 0; full && i < 5; ++i)
        full = r.samples[i] == 1000 && r.numbers[i] == i + 1;
    testOk(full, "5 frames of 1000 samples, numbered 1-5");
    testOk(s.acq.numAcquired == 5 && s.display == DISPLAY_COMPLETE && !s.active && !r.eng->active(),
           "complete: NumAcquired 5, Acquire released (4.4)");
    a = r.acquire(freeRun(1000, ACQUIRE_MULTIPLE, 1));
    testOk(a.applied && r.eng->display() == DISPLAY_ACQUIRING, "after completion a new Acquire arms");
}

void testStopWithOutputPending()
{
    Rig r;
    r.runUntilReady();
    r.acquire(freeRun(200));
    r.paused = true;                          /* the callback thread is busy: ~10 rows a record */
    r.run(100);
    unsigned entries;
    uint64_t frames;
    r.q.backlog(&entries, &frames);
    testOk(entries > 0, "frames queued while the callback thread is paused (%u)", entries);
    r.stop();
    testOk(r.eng->display() == DISPLAY_STOPPING && r.eng->active(),
           "Stop with queued frames: stopping, Acquire still held (4.4)");
    r.paused = false;
    r.drain();
    r.eng->service(r.now);
    testOk(r.eng->display() == DISPLAY_IDLE && !r.eng->active(), "queued frames published: idle, released");
    Snapshot s = r.latest();
    testOk(!s.active && s.display == DISPLAY_IDLE, "the snapshot shows it");
    CommandResult again = r.stop();
    testOk(again.applied && !r.eng->active(), "Stop when idle: a no-op");
}

void testStopThenRearm()
{
    Rig r;
    r.runUntilReady();
    r.acquire(freeRun(200));
    r.paused = true;
    r.run(300);
    r.stop();
    CommandResult a = r.acquire(freeRun(200));
    testOk(a.applied && r.eng->display() == DISPLAY_ACQUIRING && r.eng->active(),
           "Acquire while stopping arms at once");
    r.paused = false;
    r.run(300);
    testOk(r.eng->framer().status().numAcquired > 0 && r.eng->active(),
           "the new acquisition counts its own frames only (%llu)",
           (unsigned long long)r.eng->framer().status().numAcquired);
}

void testInvalidation()
{
    Rig r;
    r.runUntilReady();
    r.acquire(freeRun(100));
    r.run(50);
    r.paused = true;
    r.run(40);                                /* ~4 frames queued behind the failure */
    r.failNext = true;
    r.paused = false;
    r.drain();
    r.eng->service(r.now);
    testOk(r.eng->display() == DISPLAY_INVALID, "a failed publication invalidates");
    testOk(!r.eng->active(), "once queued outputs are published the driver ends it: Busy released");
    Snapshot s = r.latest();
    testOk(s.display == DISPLAY_INVALID && std::string(s.acqMessage).find("could not be published") !=
           std::string::npos && std::string(s.acqMessage).find("1 frame not delivered") != std::string::npos,
           "AcqMessage: %s", s.acqMessage);
    CommandResult a = r.acquire(freeRun(100));
    testOk(a.applied && r.eng->display() == DISPLAY_ACQUIRING && r.latest().acqMessage[0] == 0,
           "a new Acquire clears the invalid state");
}

void testQueueFull()
{
    Rig r(2000, 2);
    r.runUntilReady();
    r.acquire(freeRun(10));
    r.paused = true;
    r.run(100);
    testOk(r.eng->display() == DISPLAY_INVALID && r.eng->active(),
           "queue full: invalid, held while 2 frames are still queued");
    r.paused = false;
    r.drain();
    r.eng->service(r.now);
    Snapshot s = r.latest();
    testOk(!r.eng->active() && s.acq.notDelivered > 0 &&
           std::string(s.acqMessage).find("frame queue full") != std::string::npos, "then released: %s",
           s.acqMessage);
}

void testSourceStop()
{
    Rig r;
    r.runUntilReady();
    r.acquire(freeRun(1000, ACQUIRE_MULTIPLE, 100));
    r.runToEnd();
    Snapshot s = r.latest();
    testOk(s.acq.sourceStopped && s.acq.sourceStopRow == 12019 && s.view.hasFinal && s.view.tailComplete,
           "the final STATUS: source stopped at row 12019, tail complete");
    testOk(s.display == DISPLAY_ACQUIRING && s.active && s.acq.numAcquired == r.samples.size(),
           "reported, not invalidated: still acquiring, %llu frames published",
           (unsigned long long)s.acq.numAcquired);
}

void testMonitorCommands()
{
    Rig r;
    r.runUntilReady();
    Command c;
    c.type = CMD_MONITOR_CONFIGURE;
    c.monitor.averagingTimeS = 0.1;
    c.monitor.updatePeriodS = 0.05;
    CommandResult a = r.command(c);
    testOk(a.applied, "monitor T = 100 ms, U = 50 ms: accepted");
    c.monitor.updatePeriodS = 0.001;
    a = r.command(c);
    testOk(!a.applied && !a.message.empty(), "U = 1 ms: refused (%s)", a.message.c_str());
    /* F3: a write changes only its own field, on the accepted configuration */
    Command f1;
    f1.type = CMD_MONITOR_CONFIGURE;
    f1.monitor.averagingTimeS = 0.3;
    f1.monitorFields = MONITOR_FIELD_T;
    Command f2;
    f2.type = CMD_MONITOR_CONFIGURE;
    f2.monitor.averagingTimeS = 99;           /* stale: not a field of this write */
    f2.monitor.updatePeriodS = 0.1;
    f2.monitorFields = MONITOR_FIELD_U;
    testOk(r.command(f1).applied && r.command(f2).applied && r.eng->monitor().config().averagingTimeS == 0.3 &&
           r.eng->monitor().config().updatePeriodS == 0.1,
           "F3: T = 0.3 s then U alone = 0.1 s: both kept, the stale T of the second write ignored");
    c.type = CMD_MONITOR_START;
    r.command(c);
    MonitorUpdate u;
    uint64_t startRun = 0;
    if (r.mb.takeMonitor(&u))
        startRun = u.run;
    {
        epicsGuard<epicsMutex> g(r.mb.gate());
        testOk(startRun > 0 && r.mb.currentRun() == startRun, "Start posted its run before its result");
    }
    r.run(300);                               /* ~0.3 s of rows, before the stream ends */
    bool values = r.mb.takeMonitor(&u) && u.values && u.run == startRun && u.health == HEALTH_OK;
    testOk(values && u.slot[0].present && std::isfinite(u.slot[0].mean),
           "updates arrive in the start's run with values");
    c.type = CMD_MONITOR_STOP;
    r.command(c);
    epicsGuard<epicsMutex> g(r.mb.gate());
    testOk(r.mb.currentRun() > startRun && u.run != r.mb.currentRun(),
           "after Stop an update taken earlier is stale at the gate");
}

void testShortFrames()
{
    Rig r;
    r.runUntilReady();
    r.acquire(freeRun(5));
    r.runToEnd();
    bool all5 = true;
    for (size_t i = 0; i + 1 < r.samples.size(); ++i)
        all5 = all5 && r.samples[i] == 5 && r.numbers[i] == i + 1;
    testOk(r.samples.size() > 1300 && all5 && r.emptyFrames == 0 && r.eng->display() == DISPLAY_ACQUIRING,
           "5-sample windows: %u frames, consecutive, none empty", (unsigned)r.samples.size());
    Rig f;
    f.runUntilReady();
    f.acquire(freeRun(1));                    /* 10 frames a datagram into a queue of 8 */
    f.run(5);
    testOk(f.eng->display() == DISPLAY_INVALID, "1-sample windows: 10 frames a datagram overflow Q = 8: invalid");
}

}

MAIN(testEngine)
{
    testPlan(0);
    testRefusedBeforeStream();
    testMultipleComplete();
    testStopWithOutputPending();
    testStopThenRearm();
    testInvalidation();
    testQueueFull();
    testSourceStop();
    testMonitorCommands();
    testShortFrames();
    return testDone();
}
