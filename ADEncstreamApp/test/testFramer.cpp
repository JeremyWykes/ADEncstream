/* testFramer.cpp - the acquisition framer against the ADEncstream guide The framer is driven directly with constructed row
 * blocks; FrameOutput is a single-threaded pool and queue whose "worker" the test runs explicitly,
 * so ownership cases (a paused worker, a full queue) are deterministic. */
#include <math.h>
#include <string.h>

#include <deque>
#include <map>
#include <vector>

#include <epicsUnitTest.h>
#include <testMain.h>

#include "esFramer.h"

using namespace encstream;

namespace {

struct Cal : public CalibrationLookup {
    std::map<uint32_t, Calibration> gens;
    bool calibration(uint32_t g, Calibration *out) const
    {
        std::map<uint32_t, Calibration>::const_iterator it = gens.find(g);
        if (it == gens.end())
            return false;
        *out = it->second;
        return true;
    }
    void set(uint32_t g, const std::vector<uint8_t> &motors, double home)
    {
        Calibration c;
        memset(&c, 0, sizeof c);
        c.generation = g;
        c.nAxes = (uint16_t)motors.size();
        for (size_t i = 0; i < motors.size(); ++i) {
            c.axes[i].motor = motors[i];
            c.axes[i].home_pos = home * motors[i];
            c.axes[i].pos_sf = 1;
        }
        gens[g] = c;
    }
};

/* What the worker published: one entry per frame (empty ones included). */
struct Published {
    uint64_t epoch, number;
    bool empty;
    uint32_t samples;
    Frame copy;
};

/* Pool of Q + 2 frames and a queue of depth Q (4.9); the test runs the worker. */
struct Output : public FrameOutput {
    struct Entry { Frame *f; EmptyRun run; bool isRun; };
    std::vector<Frame> storage;
    std::vector<Frame *> freeList;
    std::deque<Entry> queue;
    size_t depth;
    Framer *framer;
    std::vector<Published> published;
    bool failNext;
    uint64_t discarded;

    Output(size_t q, uint32_t capacity, bool withIndex = false, bool withStatus = false)
        : depth(q), framer(0), failNext(false), discarded(0)
    {
        storage.resize(q + 2);
        for (size_t i = 0; i < storage.size(); ++i) {
            Frame &f = storage[i];
            f.capacity = capacity;
            f.position.assign((size_t)FRAMER_MAX_AXES * capacity, 0);
            if (withIndex)
                f.tick.assign(capacity, 0);
            if (withStatus)
                f.status.assign((size_t)FRAMER_MAX_STATUS * capacity, 0);
            freeList.push_back(&f);
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
    bool push(Frame *f)
    {
        if (queue.size() >= depth)
            return false;
        Entry e; e.f = f; e.isRun = false; memset(&e.run, 0, sizeof e.run);
        queue.push_back(e);
        return true;
    }
    bool pushEmptyRun(const EmptyRun &r)
    {
        if (queue.size() >= depth)
            return false;
        Entry e; e.f = 0; e.run = r; e.isRun = true;
        queue.push_back(e);
        return true;
    }
    void release(Frame *f) { freeList.push_back(f); }
    void stopped(uint64_t epoch)
    {
        for (std::deque<Entry>::iterator it = queue.begin(); it != queue.end();) {
            if (it->isRun && it->run.epoch == epoch) {
                discarded += it->run.count;            /* truncated (4.9.1) */
                it = queue.erase(it);
            } else {
                ++it;
            }
        }
    }
    /* the worker: publish up to n queue entries (an empty run is published whole) */
    void work(size_t n = 1000000)
    {
        while (n-- && !queue.empty()) {
            Entry e = queue.front();
            queue.pop_front();
            bool ok = !failNext;
            failNext = false;
            if (e.isRun) {
                for (uint64_t k = 0; ok && k < e.run.count; ++k) {
                    Published p;
                    p.copy = Frame();
                    p.epoch = e.run.epoch; p.number = e.run.firstNumber + k; p.empty = true; p.samples = 0;
                    published.push_back(p);
                }
                if (framer)
                    framer->published(e.run.epoch, e.run.count, ok);
            } else {
                if (ok) {
                    Published p;
                    p.epoch = e.f->epoch; p.number = e.f->number; p.empty = false; p.samples = e.f->samples;
                    p.copy = *e.f;
                    published.push_back(p);
                }
                if (framer)
                    framer->published(e.f->epoch, 1, ok);
                freeList.push_back(e.f);
            }
        }
    }
    size_t freeCount() const { return freeList.size(); }
};

/* A stream with one gate (bit) and motors; rows every Gather.Period ticks. */
struct Feed {
    uint32_t stream;
    LayoutSummary layout;
    std::vector<uint8_t> motors;
    uint64_t row, tick;
    uint32_t gen;
    std::vector<uint64_t> ticks;
    std::vector<uint32_t> servo, gateWord;
    std::vector<uint8_t> gate;
    std::vector<std::vector<double> > pos;
    std::vector<const double *> posPtr;
    std::vector<std::vector<uint32_t> > biss;  /* one column per layout status word */
    std::vector<const uint32_t *> bissPtr;

    Feed(uint32_t id, const std::vector<uint8_t> &m, bool withGate = true, uint16_t period = 1)
        : stream(id), motors(m), row(0), tick(5000), gen(1)
    {
        memset(&layout, 0, sizeof layout);
        layout.servoPeriodS = 1e-4;
        layout.gatherPeriod = period;
        layout.hasGate = withGate;
        layout.gateBit = 11;
        layout.nAxes = (uint8_t)m.size();
        for (size_t i = 0; i < m.size(); ++i)
            layout.axisMotor[i] = m[i];
    }
    StreamView view(bool fault = false, bool ready = true, bool final = false) const
    {
        StreamView v;
        memset(&v, 0, sizeof v);
        v.hasStream = true;
        v.stream = stream;
        v.layout = layout;
        v.ready = ready;
        v.fault = fault ? FAULT_BRICK_STATUS : FAULT_NONE;
        v.hasFinal = final;
        v.final.next_row = 777;
        v.latestGeneration = gen;
        return v;
    }
    StreamEvent ev(EventType t) const
    {
        StreamEvent e;
        memset(&e, 0, sizeof e);
        e.type = t;
        e.stream = stream;
        e.oldStream = stream;
        e.finalNextRow = 999;
        return e;
    }
    /* rows with the given gate values (one char per row: '0'/'1'), after `skip` lost rows */
    void send(RowSink &sink, const char *pattern, uint64_t skip = 0)
    {
        row += skip;
        tick += skip * layout.gatherPeriod;
        size_t n = strlen(pattern);
        ticks.assign(n, 0); servo.assign(n, 0); gate.assign(n, 0); gateWord.assign(n, 0);
        pos.assign(motors.size(), std::vector<double>(n));
        biss.assign(layout.nStatus ? layout.nStatus : 1, std::vector<uint32_t>(n, 0));
        uint64_t first = row;
        for (size_t i = 0; i < n; ++i) {
            ticks[i] = tick;
            servo[i] = (uint32_t)(1000 + tick);
            gate[i] = pattern[i] == '1';
            gateWord[i] = 0xA5000000u | ((uint32_t)gate[i] << 11);
            for (size_t a = 0; a < motors.size(); ++a)
                pos[a][i] = 100.0 * motors[a] + (double)row;
            for (size_t k = 0; k < biss.size(); ++k)
                biss[k][i] = row == 3 ? (0x4u << k) : 0;
            tick += layout.gatherPeriod;
            ++row;
        }
        posPtr.clear();
        for (size_t a = 0; a < pos.size(); ++a)
            posPtr.push_back(&pos[a][0]);
        bissPtr.clear();
        for (size_t k = 0; k < biss.size(); ++k)
            bissPtr.push_back(&biss[k][0]);
        RowBlock b;
        memset(&b, 0, sizeof b);
        b.stream = stream;
        b.firstRow = first;
        b.nRows = (uint32_t)n;
        b.calGeneration = gen;
        b.tick = &ticks[0];
        b.servoCount = &servo[0];
        b.gate = layout.hasGate ? &gate[0] : 0;
        b.gateWord = layout.hasGate ? &gateWord[0] : 0;
        b.nAxes = (unsigned)motors.size();
        b.axisMotor = layout.axisMotor;
        b.position = &posPtr[0];
        b.nStatus = layout.nStatus;
        b.statusMotor = layout.statusMotor;
        b.status = &bissPtr[0];
        sink.rows(b);
    }
    /* n rows of one gate value */
    void level(RowSink &sink, unsigned n, char v, uint64_t skip = 0)
    {
        std::string s(n, v);
        send(sink, s.c_str(), skip);
    }
};

std::vector<uint8_t> motors(int a, int b = -1)
{
    std::vector<uint8_t> v(1, (uint8_t)a);
    if (b >= 0) v.push_back((uint8_t)b);
    return v;
}

FramerConfig conf(TriggerMode t, AcquireMode m = ACQUIRE_CONTINUOUS, uint64_t n = 1, uint64_t fp = 0,
                  uint32_t maxS = 1000)
{
    FramerConfig c;
    c.trigger = t;
    c.mode = m;
    c.numAcquire = n;
    c.framePoints = fp;
    c.maxFrameSamples = maxS;
    return c;
}

struct Rig {
    Cal cal;
    Output out;
    Framer fr;
    Feed f;
    explicit Rig(size_t q = 8, uint32_t cap = 1000, bool withGate = true, bool idx = false, bool stat = false,
                 uint16_t period = 1)
        : out(q, cap, idx, stat), fr(out, cal), f(1, motors(1, 2), withGate, period)
    {
        cal.set(1, f.motors, 0.5);
        out.framer = &fr;
    }
    std::string arm(const FramerConfig &c) { return fr.acquire(c, f.view()); }
};

std::vector<uint32_t> sampleCounts(const Output &o)
{
    std::vector<uint32_t> v;
    for (size_t i = 0; i < o.published.size(); ++i)
        v.push_back(o.published[i].samples);
    return v;
}

std::string str(const std::vector<uint32_t> &v)
{
    std::string s;
    char b[16];
    for (size_t i = 0; i < v.size(); ++i) {
        sprintf(b, "%s%u", i ? "," : "", v[i]);
        s += b;
    }
    return s;
}

/* ---- arming and refusals (4.1, 4.2, F5, F7) --------------------------------------------------- */

void testRefusals()
{
    Rig r;
    StreamView none;
    memset(&none, 0, sizeof none);
    testOk(!r.fr.acquire(conf(TRIGGER_GATE_HIGH), none).empty(), "no stream: refused");
    testOk(!r.fr.acquire(conf(TRIGGER_GATE_HIGH), r.f.view(true)).empty(), "faulted stream: refused");
    testOk(!r.fr.acquire(conf(TRIGGER_GATE_HIGH), r.f.view(false, false)).empty(), "uncalibrated: refused");
    Rig ng(8, 1000, false);
    testOk(!ng.arm(conf(TRIGGER_RISING)).empty() && ng.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 10)).empty(),
           "no gate item: gate modes refused, Free Running allowed");
    Rig nine;
    nine.f.layout.nAxes = 9;
    testOk(!nine.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 10)).empty(), "9 positions: refused");
    nine.f.layout.nAxes = 0;
    testOk(!nine.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 10)).empty(), "no positions: refused");
    testOk(!r.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 2000, 1000)).empty(),
           "FramePoints > MaxFrameSamples: refused");
    {   /* position motors must be 1-8: the main array's rows and the addresses */
        const uint8_t bad[][3] = { { 9, 0, 0 }, { 17, 0, 0 }, { 1, 8, 9 }, { 0, 0, 0 } };
        const unsigned nBad[] = { 1, 1, 3, 1 };
        const char *name[] = { "Motor[9]", "Motor[17]", "Motors 1, 8, 9", "Motor[0]" };
        for (unsigned t = 0; t < 4; ++t) {
            Rig m;
            m.f.layout.nAxes = (uint8_t)nBad[t];
            memcpy(m.f.layout.axisMotor, bad[t], 3);
            std::string why = m.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 10));
            testOk(!why.empty() && why.find("Motor[1]-Motor[8]") != std::string::npos &&
                   m.fr.status().state == ACQ_IDLE, "R6: %s as positions: refused (%s)", name[t], why.c_str());
        }
        Rig o;
        uint8_t ord[] = { 2, 1, 8 };
        o.f.motors.assign(ord, ord + 3);
        o.f.layout.nAxes = 3;
        memcpy(o.f.layout.axisMotor, ord, 3);
        o.cal.set(1, o.f.motors, 0.5);
        testOk(o.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 10)).empty(),
               "R6: reordered Motors 2, 1, 8: accepted");
        Rig s;                                  /* status motors are not position indices */
        s.f.layout.nStatus = 1;
        s.f.layout.statusMotor[0] = 12;
        testOk(s.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 10)).empty(),
               "R6: a status word of Motor[12] does not refuse");
    }
    testOk1(r.arm(conf(TRIGGER_GATE_HIGH)).empty());
    testOk(!r.arm(conf(TRIGGER_GATE_HIGH)).empty(), "already acquiring: refused");
    Rig fin;
    testOk(fin.fr.acquire(conf(TRIGGER_GATE_HIGH), fin.f.view(false, true, true)).empty() &&
           fin.fr.status().sourceStopped && fin.fr.status().sourceStopRow == 777,
           "after a final STATUS: allowed, source stopped shown at once");
    Rig ft;
    FramerConfig c = conf(TRIGGER_FREE_RUNNING);
    c.frameTimeS = 0.00125;                     /* 12.5 rows at 0.1 ms: half-way rounds up */
    testOk(ft.arm(c).empty() && ft.fr.status().framePoints == 13, "FrameTime 1.25 ms: 13 slots");
}

void testArmingRow()
{
    Rig r;
    r.f.level(r.fr, 5, '0');                   /* before Acquire: never counted */
    r.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 10));
    r.f.level(r.fr, 21, '0');                  /* the next block: its first row is the arming row */
    r.out.work();
    testOk(r.out.published.size() == 2 && r.out.published[0].copy.firstTick == 5005 &&
           r.out.published[0].samples == 10, "Free Running starts at the arming row (rows before never count)");
}

/* ---- the six modes (4.4) ---------------------------------------------------------------------- */

void testGateHighLow()
{
    Rig r;
    r.arm(conf(TRIGGER_GATE_HIGH));
    r.f.send(r.fr, "11100000111111000011100");
    r.out.work();
    testOk(str(sampleCounts(r.out)) == "6,3", "Gate High: the gate active at Acquire skipped, then 6 and 3: %s",
           str(sampleCounts(r.out)).c_str());
    testOk(r.out.published[0].copy.coverage == 1.0 && r.out.published[0].copy.gateActive == 6 &&
           r.out.published[0].copy.gateEdges == 0, "full coverage; gate summary");
    Rig l;
    l.arm(conf(TRIGGER_GATE_LOW));
    l.f.send(l.fr, "0011100000111000001");
    l.out.work();
    testOk(str(sampleCounts(l.out)) == "5,5", "Gate Low: frames while the gate is 0: %s",
           str(sampleCounts(l.out)).c_str());
}

void testEdges()
{
    Rig r;
    r.arm(conf(TRIGGER_RISING));
    r.f.send(r.fr, "110001110000111000011");
    r.out.work();
    testOk(str(sampleCounts(r.out)) == "7,7", "Rising: rising edge to rising edge: %s", str(sampleCounts(r.out)).c_str());
    testOk(r.out.published[0].copy.gateEdges == 1 && r.out.published[0].copy.gateAtStart == 1 &&
           r.out.published[0].copy.gateAtEnd == 0, "a falling edge inside the frame is counted");
    Rig fa;
    fa.arm(conf(TRIGGER_FALLING));
    fa.f.send(fa.fr, "110001110000111000011");
    fa.out.work();
    testOk(str(sampleCounts(fa.out)) == "6,7", "Falling: %s", str(sampleCounts(fa.out)).c_str());
    Rig b;
    b.arm(conf(TRIGGER_BOTH));
    b.f.send(b.fr, "110001110000111000011");
    b.out.work();
    testOk(str(sampleCounts(b.out)) == "3,3,4,3,4", "Both: high and low periods: %s", str(sampleCounts(b.out)).c_str());
}

void testFreeRunning()
{
    Rig r;
    r.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 100));
    r.f.level(r.fr, 350, '0');
    r.out.work();
    testOk(r.out.published.size() == 3 && r.out.published[2].number == 3 && r.out.published[2].samples == 100 &&
           r.out.published[2].copy.windowStart == 5200 && r.out.published[2].copy.windowEnd == 5300,
           "100-slot windows: frames 1-3, window bounds on the sample clock");
    r.f.level(r.fr, 30, '0', 220);             /* rows 350..569 lost: window 3 cut, 4 empty, 5 partly */
    r.out.work();
    testOk(r.out.published.size() == 5 && !r.out.published[3].empty && r.out.published[3].number == 4 &&
           r.out.published[3].samples == 50 && r.out.published[4].number == 5 && r.out.published[4].empty,
           "frame 4 cut by the gap (50 rows), frame 5 Empty (an empty run of 1)");
    r.f.level(r.fr, 100, '0');
    r.out.work();
    const Frame &six = r.out.published[5].copy;
    testOk(r.out.published[5].number == 6 && six.samples == 30 && six.expected == 100 && six.missing == 70 &&
           six.coverage == 0.3, "frame 6: 30 of 100 slots (coverage)");
    testOk(r.fr.status().empty == 1 && r.fr.status().degraded, "empty frames counted; degraded");
}

void testHugeGap()
{
    Rig r(3);                                   /* a small queue */
    r.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 100));
    r.f.level(r.fr, 150, '0');
    r.f.level(r.fr, 10, '0', 1000000000ull);    /* ten million empty windows */
    testOk(r.out.queue.size() == 3 && r.out.queue[2].isRun && r.out.queue[2].run.count == 9999999 &&
           r.out.queue[2].run.firstNumber == 3, "a huge gap: one empty-run entry, numbers reserved");
    r.f.level(r.fr, 100, '0');                 /* the frame after the gap needs a queue slot: full */
    testOk(r.fr.status().state == ACQ_INVALID && r.fr.status().invalid == INVALID_QUEUE_FULL,
           "the worker never ran: queue full, invalidated");
    Rig m(4);
    m.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_MULTIPLE, 10, 100));
    m.f.level(m.fr, 150, '0');
    m.f.level(m.fr, 10, '0', 1000000000ull);
    testOk(m.out.queue.size() == 3 && m.out.queue[2].run.count == 8 && m.fr.status().state == ACQ_ASSEMBLED,
           "Multiple N=10: the run (frames 3-10) is cut at frame 10; all assembled");
    m.out.work();
    testOk(m.fr.status().state == ACQ_COMPLETE && m.fr.status().numAcquired == 10, "complete once published");
    /* Stop during a huge run: truncated, re-arm does not wait (4.9.1) */
    Rig s(4);
    s.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 100));
    s.f.level(s.fr, 150, '0');
    s.f.level(s.fr, 10, '0', 1000000000ull);
    s.fr.stop();
    testOk(s.out.discarded == 9999999 && s.out.queue.size() == 2, "Stop truncates the run");
    testOk1(s.fr.acquire(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 100), s.f.view()).empty());
    s.f.level(s.fr, 101, '0');
    s.out.work();
    testOk(s.out.published.size() == 3 && s.out.published[1].epoch == 1 && s.out.published[2].epoch == 2 &&
           s.out.published[2].number == 1 && s.fr.status().numAcquired == 1,
           "re-armed at once; the old real frame published under its epoch, not counted in the new run");
}

/* ---- gaps and edges (4.3), coverage (4.8) ----------------------------------------------------- */

void testGaps()
{
    Rig r;
    r.arm(conf(TRIGGER_GATE_HIGH));
    r.f.send(r.fr, "0001111");
    r.f.send(r.fr, "11110000", 3);              /* 3 high rows lost inside the frame */
    r.out.work();
    const Frame &a = r.out.published[0].copy;
    testOk(a.samples == 8 && a.expected == 11 && a.missing == 3 && !a.startEdgeInGap && !a.endEdgeInGap,
           "a gap inside a frame: 8 of 11, no edge flags");
    Rig e;
    e.arm(conf(TRIGGER_GATE_HIGH));
    e.f.send(e.fr, "000111111");
    e.f.send(e.fr, "0000", 2);                  /* the last two high rows lost before the low row */
    e.out.work();
    const Frame &b = e.out.published[0].copy;
    testOk(b.samples == 6 && b.coverage == 1.0 && b.endEdgeInGap && b.endGapRows == 2,
           "edge in a gap: full coverage inside the span, EndEdgeInGap with 2 rows (exact formula)");
    Rig s;
    s.arm(conf(TRIGGER_GATE_HIGH));
    s.f.send(s.fr, "0000");
    s.f.send(s.fr, "11110000", 5);              /* the rising edge inside a 5-row gap */
    s.out.work();
    testOk(s.out.published[0].copy.startEdgeInGap && s.out.published[0].copy.startGapRows == 5, "StartEdgeInGap");
    Rig w;
    w.arm(conf(TRIGGER_GATE_HIGH));
    w.f.send(w.fr, "00011100");
    w.f.send(w.fr, "0001110", 10);              /* a whole high period lost in the gap */
    w.f.send(w.fr, "0");
    w.out.work();
    testOk(w.out.published.size() == 2, "a whole gate in a gap: one frame fewer, never more");
    Rig g;
    g.arm(conf(TRIGGER_GATE_HIGH));
    g.f.send(g.fr, "000100011110");
    g.out.work();
    testOk(str(sampleCounts(g.out)) == "1,4", "a glitch: a 1-sample frame, emitted and visible");
}

void testOverlong()
{
    Rig r;
    r.arm(conf(TRIGGER_GATE_HIGH, ACQUIRE_CONTINUOUS, 1, 0, 20));
    r.f.send(r.fr, "000");
    r.f.level(r.fr, 50, '1');
    r.f.send(r.fr, "0001110");
    r.out.work();
    testOk(r.out.published.size() == 2 && r.out.published[0].samples == 20 && r.out.published[0].copy.overlong &&
           r.out.published[1].samples == 3 && r.fr.status().overlong == 1,
           "a stuck gate: one truncated Overlong frame, then the next activation");
    Rig e;
    e.arm(conf(TRIGGER_RISING, ACQUIRE_CONTINUOUS, 1, 0, 20));
    e.f.send(e.fr, "01");
    e.f.level(e.fr, 50, '1');
    e.f.send(e.fr, "01");
    e.out.work();
    testOk(e.out.published.size() == 1 && e.out.published[0].copy.overlong, "Rising: overlong, then waits for an edge");
}

/* ---- Multiple, publication, ownership (4.5, 4.9, F3, C1, C5) --------------------------------- */

void testMultipleAndPublication()
{
    Rig r;
    r.arm(conf(TRIGGER_GATE_HIGH, ACQUIRE_MULTIPLE, 2));
    r.f.send(r.fr, "0110110110");
    testOk(r.fr.status().state == ACQ_ASSEMBLED && r.fr.status().assembled == 2 && r.fr.status().numAcquired == 0,
           "N=2: assembled after the second frame; the third gate ignored");
    r.out.work(1);
    testOk(r.fr.status().state == ACQ_ASSEMBLED && r.fr.status().numAcquired == 1, "one published");
    r.out.work();
    testOk(r.fr.status().state == ACQ_COMPLETE && r.fr.status().numAcquired == 2, "complete when N are published");
    Rig p;
    p.arm(conf(TRIGGER_GATE_HIGH));
    p.f.send(p.fr, "0110110");
    p.out.failNext = true;
    p.out.work();
    testOk(p.fr.status().state == ACQ_INVALID && p.fr.status().invalid == INVALID_PUBLICATION_FAILED &&
           p.fr.status().notDelivered == 1, "a publication failure invalidates");
}

void testOwnership()
{
    Rig r(2);                                   /* pool of 4, queue of 2, worker paused */
    r.arm(conf(TRIGGER_GATE_HIGH));
    r.f.send(r.fr, "0110110");
    testOk(r.out.queue.size() == 2 && r.out.freeCount() == 2, "two frames queued, two objects free");
    r.f.send(r.fr, "11");                       /* a third frame assembling, one object still free */
    testOk(r.out.freeCount() == 1, "assembling one; one free");
    r.f.send(r.fr, "0");
    testOk(r.fr.status().state == ACQ_INVALID && r.fr.status().invalid == INVALID_QUEUE_FULL &&
           r.fr.status().notDelivered == 1 && r.out.freeCount() == 2,
           "queue full with a free object left: invalidated, the frame released, no leak");
    r.out.work();
    testOk(r.fr.status().numAcquired == 2 && r.out.freeCount() == 4, "queued frames still published; pool whole");
    Rig x(0);                                   /* queue of 0: pool of 2 */
    x.arm(conf(TRIGGER_GATE_HIGH));
    x.f.send(x.fr, "011");
    Frame *held[2] = { x.out.acquire(), x.out.acquire() };   /* the worker and the queue hold them */
    (void)held;
    testOk(x.out.freeCount() == 0, "every object taken");
    Rig y(1);
    y.arm(conf(TRIGGER_GATE_HIGH));
    while (y.out.freeCount())
        y.out.acquire();
    y.f.send(y.fr, "01");
    testOk(y.fr.status().state == ACQ_INVALID && y.fr.status().invalid == INVALID_NO_FREE_FRAME,
           "no free object when a frame must start: invalidated");
}

void testStopAndEpochs()
{
    Rig r;
    r.arm(conf(TRIGGER_GATE_HIGH));
    r.f.send(r.fr, "0111");
    size_t freeBefore = r.out.freeCount();
    r.fr.stop();
    testOk(r.fr.status().state == ACQ_IDLE && r.out.freeCount() == freeBefore + 1,
           "Stop drops the frame being assembled");
    r.arm(conf(TRIGGER_GATE_HIGH));
    r.fr.published(1, 5, true);                 /* a late report from the first acquisition */
    testOk(r.fr.status().numAcquired == 0 && r.fr.status().epoch == 2, "a late old-epoch report is ignored");
}

/* ---- stream events (4.7) ---------------------------------------------------------------------- */

void testStreamEvents()
{
    Rig r;
    r.arm(conf(TRIGGER_GATE_HIGH));
    r.f.send(r.fr, "0110111");
    r.fr.event(r.f.ev(EVENT_STREAM_FAULTED));
    testOk(r.fr.status().state == ACQ_INVALID && r.fr.status().invalid == INVALID_STREAM_FAULTED, "fault invalidates");
    r.out.work();
    testOk(r.fr.status().numAcquired == 1, "the frame already queued is still published");
    Rig p;
    p.arm(conf(TRIGGER_GATE_HIGH));
    p.fr.event(p.f.ev(EVENT_STREAM_REPLACED));
    testOk(p.fr.status().invalid == INVALID_STREAM_REPLACED, "replacement invalidates");
    Rig e;
    e.arm(conf(TRIGGER_GATE_HIGH, ACQUIRE_MULTIPLE, 3));
    e.f.send(e.fr, "0110");
    e.fr.event(e.f.ev(EVENT_STREAM_ENDED));
    testOk(e.fr.status().state == ACQ_ACQUIRING && e.fr.status().sourceStopped && e.fr.status().sourceStopRow == 999,
           "source stopped: reported, not invalidated");
    e.f.send(e.fr, "1101100");                  /* tail rows complete N */
    e.out.work();
    testOk(e.fr.status().state == ACQ_COMPLETE, "tail rows can still complete N");
}

/* ---- calibration, layout, optional arrays (4.6, 4.8) ------------------------------------- */

void testContents()
{
    Rig r(8, 1000, true, true, true);
    r.f.layout.nStatus = 1;
    r.f.layout.statusMotor[0] = 2;
    FramerConfig withArrays = conf(TRIGGER_GATE_HIGH);
    withArrays.withSampleIndex = withArrays.withStatus = true;
    r.arm(withArrays);
    r.f.send(r.fr, "0111");
    r.f.gen = 2;                                /* a new generation during the run */
    r.f.send(r.fr, "10");
    r.out.work();
    const Frame &f = r.out.published[0].copy;
    testOk(f.samples == 4 && f.position[0 * f.capacity + 0] == 100 + 1 - 0.5 &&
           f.position[1 * f.capacity + 0] == 200 + 1 - 1.0, "positions minus the frozen HomePos, motor order");
    testOk(f.calibrationChanged && r.fr.status().calibrationChanged == 1, "rows of another generation: flagged");
    testOk(f.tick[0] == 5001 && f.schema.nStatus == 2 && f.status[0 * f.capacity + 1] == (0xA5000000u | 0x800u) &&
           f.status[1 * f.capacity + 2] == 0x4u && f.statusOr == 0x4u,
           "optional sample index (tick) and status words (gate word first)");
    Rig p(8, 1000, true, false, false, 2);       /* Gather.Period 2 */
    p.arm(conf(TRIGGER_GATE_HIGH));
    p.f.send(p.fr, "01111");
    p.f.send(p.fr, "10", 1);
    p.out.work();
    testOk(p.out.published[0].copy.expected == 6 && p.out.published[0].copy.missing == 1,
           "Gather.Period 2: expected from the tick span in rows");
}

/* ---- code review regressions --------------------- */

void setStatus(Feed &f, unsigned n, bool reversed = false)
{
    f.layout.nStatus = (uint8_t)n;
    for (unsigned j = 0; j < n; ++j)
        f.layout.statusMotor[j] = (uint8_t)(reversed ? n - j : j + 1);
}

void testR1StatusStorage()
{
    Rig r(8, 1000, true, false, true);
    setStatus(r.f, 9);                          /* legal on the wire: status motors are independent */
    FramerConfig c = conf(TRIGGER_GATE_HIGH);
    c.withStatus = true;
    testOk(!r.arm(c).empty(), "R1: 9 encoder status words: Acquire refused");
    Rig e(8, 1000, true, false, true);
    setStatus(e.f, 8);
    e.f.motors.assign(1, 1);
    e.f.layout.nAxes = 1;
    testOk(e.arm(c).empty(), "8 status words with a gate (9 columns), more than the positions: accepted");
    e.f.send(e.fr, "0110");
    e.out.work();
    testOk(e.out.published.size() == 1 && e.out.published[0].copy.schema.nStatus == 9 &&
           e.out.published[0].copy.schema.statusMotor[0] == 0 && e.out.published[0].copy.schema.statusMotor[8] == 8,
           "all 9 columns written within storage; the schema names them (gate first)");
    Rig small(8, 10);                            /* pool objects of 10 samples */
    FramerConfig big = conf(TRIGGER_GATE_HIGH, ACQUIRE_CONTINUOUS, 1, 0, 1000);
    small.arm(big);
    small.f.send(small.fr, "011");
    testOk(small.fr.status().state == ACQ_INVALID && small.fr.status().invalid == INVALID_FRAME_STORAGE &&
           small.out.freeCount() == 10, "pool smaller than MaxFrameSamples: invalidated, nothing written, no leak");
    Rig nostat(8, 1000, true, false, false);    /* no status storage in the pool */
    setStatus(nostat.f, 2);
    nostat.arm(c);
    nostat.f.send(nostat.fr, "011");
    testOk(nostat.fr.status().invalid == INVALID_FRAME_STORAGE, "status array enabled but no storage: invalidated");
}

void testR2Schema()
{
    Rig r(8, 1000, true, true, true);
    setStatus(r.f, 2, true);                    /* status order (2, 1) differs from positions (1, 2) */
    FramerConfig on = conf(TRIGGER_GATE_HIGH);
    on.withSampleIndex = on.withStatus = true;
    r.arm(on);
    r.f.send(r.fr, "0110");                     /* a frame queued, the worker paused */
    r.fr.stop();
    r.arm(conf(TRIGGER_GATE_HIGH));             /* re-armed with the optional outputs off */
    r.f.send(r.fr, "0110");
    r.out.work();
    const OutputSchema &a = r.out.published[0].copy.schema, &b = r.out.published[1].copy.schema;
    testOk(a.withSampleIndex && a.withStatus && a.nStatus == 3 && a.statusMotor[1] == 2 && a.statusMotor[2] == 1 &&
           !b.withSampleIndex && !b.withStatus && b.nStatus == 0,
           "R2: an old queued frame keeps its own schema across Stop and re-arm");
    Rig e1(8, 1000, true, true, true), e2;     /* e1's pool has the optional storage */
    FramerConfig fon = conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 10);
    fon.withSampleIndex = fon.withStatus = true;
    e1.arm(fon);
    e2.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 10));
    e1.f.level(e1.fr, 15, '0'); e1.f.level(e1.fr, 5, '0', 100);
    e2.f.level(e2.fr, 15, '0'); e2.f.level(e2.fr, 5, '0', 100);
    const EmptyRun *ra = 0, *rb = 0;
    for (size_t i = 0; i < e1.out.queue.size(); ++i) if (e1.out.queue[i].isRun) ra = &e1.out.queue[i].run;
    for (size_t i = 0; i < e2.out.queue.size(); ++i) if (e2.out.queue[i].isRun) rb = &e2.out.queue[i].run;
    testOk(ra && rb && ra->count == rb->count && ra->schema.withStatus && !rb->schema.withStatus &&
           ra->schema.withSampleIndex && ra->schema.hasGate && ra->schema.framePoints == 10,
           "empty runs with the optional outputs on and off differ: each carries its schema");
}

void testR3Numbers()
{
    Rig r;
    FramerConfig c = conf(TRIGGER_FREE_RUNNING);
    c.frameTimeS = 1e300;
    testOk(!r.arm(c).empty(), "R3: FrameTime 1e300: refused before any conversion");
    c.frameTimeS = 0.1 + 1e-9;                  /* 1000 slots at 0.1 ms: exactly MaxFrameSamples */
    testOk(r.arm(c).empty() && r.fr.status().framePoints == 1000, "FrameTime at the limit: accepted");
    Rig w;
    w.f.tick = ~(uint64_t)0 - 250;              /* the sample clock near its end */
    w.arm(conf(TRIGGER_FREE_RUNNING, ACQUIRE_CONTINUOUS, 1, 100));
    w.f.level(w.fr, 250, '0');
    testOk(w.fr.status().state == ACQ_INVALID && w.fr.status().invalid == INVALID_CLOCK_RANGE,
           "a window whose end does not fit 64 bits: invalidated, not wrapped");
}

void testR4Cap()
{
    Rig a;
    a.arm(conf(TRIGGER_GATE_HIGH, ACQUIRE_SINGLE, 1, 0, 3));
    a.f.send(a.fr, "0111");                     /* reaches the cap and then silence */
    a.out.work();
    testOk(a.out.published.size() == 1 && a.out.published[0].samples == 3 && a.out.published[0].copy.overlong &&
           a.fr.status().state == ACQ_COMPLETE, "R4: Gate High, cap 3, '0111': emitted at the cap, Overlong");
    Rig b;
    b.arm(conf(TRIGGER_GATE_HIGH, ACQUIRE_CONTINUOUS, 1, 0, 3));
    b.f.send(b.fr, "0110");                     /* cap - 1 */
    b.f.send(b.fr, "01");                       /* reaches the cap across a block boundary */
    b.f.send(b.fr, "11");
    b.f.send(b.fr, "0");                        /* the closing row after an overlong frame: nothing */
    b.f.send(b.fr, "0111110");                  /* cap + 2 */
    b.out.work();
    testOk(str(sampleCounts(b.out)) == "2,3,3" && !b.out.published[0].copy.overlong &&
           b.out.published[1].copy.overlong && b.out.published[2].copy.overlong,
           "cap-1 normal; at the cap across blocks and beyond it: one Overlong frame each: %s",
           str(sampleCounts(b.out)).c_str());
    Rig e;
    e.arm(conf(TRIGGER_RISING, ACQUIRE_CONTINUOUS, 1, 0, 3));
    e.f.send(e.fr, "01110011");                 /* overlong at the cap; the falling edge ignored */
    e.out.work();
    testOk(e.out.published.size() == 1 && e.out.published[0].samples == 3 && e.out.published[0].copy.overlong,
           "Rising: overlong at the cap; the next rising edge opens the next frame");
    e.fr.event(e.f.ev(EVENT_STREAM_ENDED));
    e.fr.stop();
    testOk(e.out.published.size() == 1, "a final STATUS and Stop drop only the open (non-overlong) frame");
    Rig m;
    m.arm(conf(TRIGGER_GATE_HIGH, ACQUIRE_MULTIPLE, 2, 0, 3));
    m.f.send(m.fr, "011110110");
    m.out.work();
    testOk(m.fr.status().state == ACQ_COMPLETE && m.fr.status().numAcquired == 2 && m.fr.status().overlong == 1 &&
           str(sampleCounts(m.out)) == "3,2", "Multiple: an overlong frame counts as one of N");
}

}

MAIN(testFramer)
{
    testPlan(0);
    testRefusals();
    testArmingRow();
    testGateHighLow();
    testEdges();
    testFreeRunning();
    testHugeGap();
    testGaps();
    testOverlong();
    testMultipleAndPublication();
    testOwnership();
    testStopAndEpochs();
    testStreamEvents();
    testContents();
    testR1StatusStorage();
    testR2Schema();
    testR3Numbers();
    testR4Cap();
    return testDone();
}
