/* testMonitor.cpp - the continuous monitor against the ADEncstream guide The monitor is driven directly with constructed row blocks and events, so expected
 * values are computed independently here. */
#include <math.h>
#include <string.h>

#include <map>
#include <vector>

#include <epicsUnitTest.h>
#include <testMain.h>

#include "esMonitor.h"

using namespace encstream;

namespace {

struct Out : public MonitorOutput {
    std::vector<MonitorUpdate> all;
    void publish(const MonitorUpdate &u) { all.push_back(u); }
    unsigned valueUpdates() const
    {
        unsigned n = 0;
        for (size_t i = 0; i < all.size(); ++i)
            n += all[i].values;
        return n;
    }
    const MonitorUpdate *lastValues() const
    {
        for (size_t i = all.size(); i-- > 0;)
            if (all[i].values)
                return &all[i];
        return 0;
    }
};

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

/* A stream: positions for `motors` (CONTEXT order), status words for `statusMotors`; rows at
 * 0.1 ms (servo 0.1 ms, Gather.Period 1) unless changed. value(m, row) gives ActPos. */
struct Feed {
    uint32_t stream;
    LayoutSummary layout;
    std::vector<uint8_t> motors, statusMotors;
    uint64_t tick, row;
    uint32_t gen;
    std::vector<uint64_t> ticks;
    std::vector<std::vector<double> > pos;
    std::vector<std::vector<uint32_t> > st;
    std::vector<const double *> posPtr;
    std::vector<const uint32_t *> stPtr;
    std::vector<uint8_t> gateBuf;

    Feed(uint32_t id, const std::vector<uint8_t> &m, const std::vector<uint8_t> &s = std::vector<uint8_t>(),
         double servo = 1e-4, uint16_t period = 1, bool gate = false)
        : stream(id), motors(m), statusMotors(s), tick(1000), row(0), gen(1)
    {
        memset(&layout, 0, sizeof layout);
        layout.servoPeriodS = servo;
        layout.gatherPeriod = period;
        layout.nAxes = (uint8_t)m.size();
        for (size_t i = 0; i < m.size(); ++i)
            layout.axisMotor[i] = m[i];
        layout.nStatus = (uint8_t)s.size();
        for (size_t i = 0; i < s.size(); ++i)
            layout.statusMotor[i] = s[i];
        layout.hasGate = gate ? 1 : 0;
        layout.gateBit = 0;
    }

    StreamView view(bool fault = false, bool final = false) const
    {
        StreamView v;
        memset(&v, 0, sizeof v);
        v.hasStream = true;
        v.stream = stream;
        v.layout = layout;
        v.ready = true;
        v.fault = fault ? FAULT_BRICK_STATUS : FAULT_NONE;
        v.hasFinal = final;
        return v;
    }

    StreamEvent started() const
    {
        StreamEvent e;
        memset(&e, 0, sizeof e);
        e.type = EVENT_STREAM_STARTED;
        e.stream = stream;
        e.layout = layout;
        return e;
    }

    StreamEvent ev(EventType t) const
    {
        StreamEvent e;
        memset(&e, 0, sizeof e);
        e.type = t;
        e.stream = stream;
        return e;
    }

    /* n rows, each `step` row intervals after the previous; value = f(motor, row) */
    template <class F>
    void send(Monitor &mon, unsigned n, F f, uint64_t skipRows = 0)
    {
        tick += skipRows * layout.gatherPeriod;
        row += skipRows;
        ticks.assign(n, 0);
        pos.assign(motors.size(), std::vector<double>(n));
        st.assign(statusMotors.size(), std::vector<uint32_t>(n));
        for (unsigned i = 0; i < n; ++i) {
            ticks[i] = tick;
            for (size_t a = 0; a < motors.size(); ++a)
                pos[a][i] = f(motors[a], row);
            for (size_t k = 0; k < statusMotors.size(); ++k)
                st[k][i] = (row % 50 == 7) ? (1u << statusMotors[k]) : 0;
            tick += layout.gatherPeriod;
            ++row;
        }
        posPtr.clear();
        for (size_t a = 0; a < pos.size(); ++a)
            posPtr.push_back(&pos[a][0]);
        stPtr.clear();
        for (size_t k = 0; k < st.size(); ++k)
            stPtr.push_back(&st[k][0]);
        RowBlock b;
        memset(&b, 0, sizeof b);
        b.stream = stream;
        b.firstRow = row - n;
        b.nRows = n;
        b.calGeneration = gen;
        b.tick = &ticks[0];
        b.nAxes = (unsigned)motors.size();
        b.axisMotor = layout.axisMotor;
        b.position = posPtr.empty() ? 0 : &posPtr[0];
        b.nStatus = (unsigned)statusMotors.size();
        b.statusMotor = layout.statusMotor;
        b.status = stPtr.empty() ? 0 : &stPtr[0];
        mon.rows(b);
    }

    /* like send, but also a per-row gate g(row) -> 0/1 (the layout must have a gate item) */
    template <class F, class G>
    void sendG(Monitor &mon, unsigned n, F f, G g, uint64_t skipRows = 0)
    {
        tick += skipRows * layout.gatherPeriod;
        row += skipRows;
        ticks.assign(n, 0);
        pos.assign(motors.size(), std::vector<double>(n));
        st.assign(statusMotors.size(), std::vector<uint32_t>(n));
        gateBuf.assign(n, 0);
        for (unsigned i = 0; i < n; ++i) {
            ticks[i] = tick;
            for (size_t a = 0; a < motors.size(); ++a)
                pos[a][i] = f(motors[a], row);
            for (size_t k = 0; k < statusMotors.size(); ++k)
                st[k][i] = (row % 50 == 7) ? (1u << statusMotors[k]) : 0;
            gateBuf[i] = g(row) ? 1 : 0;
            tick += layout.gatherPeriod;
            ++row;
        }
        posPtr.clear();
        for (size_t a = 0; a < pos.size(); ++a)
            posPtr.push_back(&pos[a][0]);
        stPtr.clear();
        for (size_t k = 0; k < st.size(); ++k)
            stPtr.push_back(&st[k][0]);
        RowBlock b;
        memset(&b, 0, sizeof b);
        b.stream = stream;
        b.firstRow = row - n;
        b.nRows = n;
        b.calGeneration = gen;
        b.tick = &ticks[0];
        b.gate = layout.hasGate ? &gateBuf[0] : 0;
        b.nAxes = (unsigned)motors.size();
        b.axisMotor = layout.axisMotor;
        b.position = posPtr.empty() ? 0 : &posPtr[0];
        b.nStatus = (unsigned)statusMotors.size();
        b.statusMotor = layout.statusMotor;
        b.status = stPtr.empty() ? 0 : &stPtr[0];
        mon.rows(b);
    }
};

double lin(unsigned m, uint64_t r) { return 10.0 * m + 0.001 * (double)r; }

std::vector<uint8_t> motors(int a, int b = -1, int c = -1)
{
    std::vector<uint8_t> v(1, (uint8_t)a);
    if (b >= 0) v.push_back((uint8_t)b);
    if (c >= 0) v.push_back((uint8_t)c);
    return v;
}

MonitorConfig cfg(double t, double u = 0, double minCov = 0.9)
{
    MonitorConfig c;
    c.averagingTimeS = t;
    c.updatePeriodS = u;
    c.minCoverage = minCov;
    return c;
}

/* direct statistics of f(m, r) for rows [r0, r1), HomePos subtracted */
void direct(unsigned m, uint64_t r0, uint64_t r1, double home, double *mean, double *sd, double *mn, double *mx)
{
    double s = 0, n = (double)(r1 - r0);
    *mn = 1e300; *mx = -1e300;
    for (uint64_t r = r0; r < r1; ++r) {
        double x = lin(m, r) - home;
        s += x;
        if (x < *mn) *mn = x;
        if (x > *mx) *mx = x;
    }
    *mean = s / n;
    double v = 0;
    for (uint64_t r = r0; r < r1; ++r) {
        double d = lin(m, r) - home - *mean;
        v += d * d;
    }
    *sd = sqrt(v / n);
}

bool close(double a, double b, double tol = 1e-9) { return fabs(a - b) <= tol * (1 + fabs(b)); }

void testConfig()
{
    testOk1(checkMonitorConfig(cfg(1.0)).empty());
    testOk1(checkMonitorConfig(cfg(1.0, 0.01)).empty());
    testOk(!checkMonitorConfig(cfg(1.0, 0.009)).empty(), "U < 10 ms refused");
    testOk(!checkMonitorConfig(cfg(0.009)).empty(), "T = U < 10 ms refused");
    testOk(!checkMonitorConfig(cfg(1.0, 2.0)).empty(), "U > T refused");
    testOk(!checkMonitorConfig(cfg(20.0, 0.01)).empty(), "T/U = 2000 refused (at most 1000)");
    testOk1(checkMonitorConfig(cfg(10.0, 0.01)).empty());
    testOk(!checkMonitorConfig(cfg(-1)).empty() && !checkMonitorConfig(cfg(NAN)).empty() &&
           !checkMonitorConfig(cfg(1.0, INFINITY)).empty(), "non-finite or negative refused");
    testOk(!checkMonitorConfig(cfg(1.0, 0, 1.5)).empty(), "min coverage outside 0..1 refused");
}

void testEffective()
{
    Out out; Cal cal;
    Monitor mon(out, cal);
    Feed f(1, motors(1));
    mon.configure(cfg(0.1), f.view());
    testOk(mon.effective().rowsPerUpdate == 1000 && mon.effective().subBlocks == 1, "0.1 s at 0.1 ms rows: 1000 rows");
    Feed slow(2, motors(1), std::vector<uint8_t>(), 1e-3, 3);   /* 3 ms rows */
    mon.configure(cfg(0.01), slow.view());
    testOk(mon.effective().rowsPerUpdate == 4 && close(mon.effective().updatePeriodS, 0.012),
           "10 ms at 3 ms rows: 4 rows (12 ms), never below 10 ms");
    mon.configure(cfg(0.03, 0.01), f.view());
    testOk(mon.effective().subBlocks == 3 && mon.effective().rowsPerUpdate == 100 &&
           close(mon.effective().averagingTimeS, 0.03), "T 30 ms, U 10 ms: 3 sub-blocks of 100 rows");
}

void testBackToBack()
{
    Out out; Cal cal;
    Feed f(1, motors(1, 3));
    cal.set(1, f.motors, 5.0);                 /* HomePos = 5 x motor */
    Monitor mon(out, cal);
    mon.configure(cfg(0.01), f.view());
    mon.start(f.view());
    testOk(out.all.size() == 1 && !out.all[0].values && out.all[0].health == HEALTH_WAITING,
           "Start publishes 'waiting' at once");
    f.send(mon, 250, lin);
    testOk(out.valueUpdates() == 1 && out.lastValues()->updatesSkipped == 1,
           "250 rows in one call close 2 windows: one update published, one skipped");
    const MonitorUpdate &u = out.all[out.all.size() - 1];
    double mean, sd, mn, mx;
    direct(3, 100, 200, 15.0, &mean, &sd, &mn, &mx);
    testOk(u.values && u.samples == 100 && u.expected == 100 && u.coverage == 1.0 && u.health == HEALTH_OK,
           "second update: rows 100..199, full coverage, ok");
    testOk(close(u.slot[2].mean, mean) && close(u.slot[2].stddev, sd) && u.slot[2].min == mn &&
           u.slot[2].max == mx && u.slot[2].severity == SEVERITY_NONE,
           "Motor[3] (slot 3): mean %.9g sd %.9g, against direct %.9g %.9g", u.slot[2].mean, u.slot[2].stddev, mean, sd);
    testOk(!u.slot[1].present && isnan(u.slot[1].mean) && u.slot[1].severity == SEVERITY_INVALID,
           "slot 2 (no Motor[2] in the layout): NaN, INVALID");
}

void testSlidingAndCoverage()
{
    Out out; Cal cal;
    Feed f(1, motors(1));
    cal.set(1, f.motors, 0);
    Monitor mon(out, cal);
    mon.configure(cfg(0.03, 0.01), f.view());
    mon.start(f.view());
    for (int k = 0; k < 5; ++k)
        f.send(mon, 100, lin);
    f.send(mon, 1, lin);                      /* closes sub-block 4 */
    std::vector<const MonitorUpdate *> v;
    for (size_t i = 0; i < out.all.size(); ++i)
        if (out.all[i].values)
            v.push_back(&out.all[i]);
    testOk(v.size() == 5, "an update every U: %u", (unsigned)v.size());
    testOk(v[0]->samples == 100 && v[0]->expected == 300 && close(v[0]->coverage, 1.0 / 3) &&
           v[0]->slot[0].severity == SEVERITY_MINOR, "first update: 1/3 of T, MINOR");
    testOk(v[2]->samples == 300 && v[2]->coverage == 1.0 && v[2]->slot[0].severity == SEVERITY_NONE,
           "third: full window");
    double mean, sd, mn, mx;
    direct(1, 200, 500, 0, &mean, &sd, &mn, &mx);
    testOk(close(v[4]->slot[0].mean, mean) && close(v[4]->slot[0].stddev, sd) && v[4]->slot[0].min == mn &&
           v[4]->slot[0].max == mx, "fifth: rows 200..499 exactly (sliding)");
    /* rows missing inside a window */
    f.send(mon, 50, lin);
    f.send(mon, 51, lin, 49);                 /* 49 rows lost, then the rest of the sub-block */
    const MonitorUpdate *w = out.lastValues();
    testOk(w->samples == 251 && close(w->coverage, 251.0 / 300) && w->slot[0].severity == SEVERITY_MINOR,
           "49 rows lost: coverage %.3f, MINOR", w->coverage);
}

void testOneUpdatePerCall()
{
    Out out; Cal cal;
    Feed f(1, motors(1));
    cal.set(1, f.motors, 0);
    Monitor mon(out, cal);
    mon.configure(cfg(0.01), f.view());
    mon.start(f.view());
    f.send(mon, 50, lin);
    unsigned before = out.valueUpdates();
    f.send(mon, 358, lin);                    /* crosses 4 boundaries */
    testOk(out.valueUpdates() == before + 1 && out.lastValues()->updatesSkipped == 3,
           "a block crossing 4 boundaries: one update, 3 skipped");
    /* a gap of a million million sub-blocks: one call, one update, the window empty */
    size_t n0 = out.all.size();
    f.send(mon, 1, lin, 1000000000000ull);
    testOk(out.all.size() == n0 + 1 && out.lastValues()->samples == 0 &&
           isnan(out.lastValues()->slot[0].mean) && out.lastValues()->slot[0].severity == SEVERITY_INVALID,
           "a huge gap: one update, empty window, NaN/INVALID");
    f.send(mon, 200, lin);   /* the gap's row lands mid-window (windows keep the run's origin) */
    testOk(out.lastValues()->samples == 100 && out.lastValues()->coverage == 1.0,
           "the first whole window after the gap is complete again");
    /* a gap shorter than T in a sliding window */
    Out out2;
    Monitor m2(out2, cal);
    Feed g(1, motors(1));
    m2.configure(cfg(0.05, 0.01), g.view());
    m2.start(g.view());
    g.send(m2, 300, lin);
    g.send(m2, 100, lin, 200);                /* 2 sub-blocks empty */
    g.send(m2, 1, lin);
    testOk(out2.lastValues()->samples == 300 && close(out2.lastValues()->coverage, 0.6),
           "a gap shorter than T: two empty sub-blocks, the others kept (300 of 500 rows)");
}

void testPrecision()
{
    Out out; Cal cal;
    Feed f(1, motors(1));
    cal.set(1, f.motors, 0);
    Monitor mon(out, cal);
    mon.configure(cfg(0.03, 0.01), f.view());
    mon.start(f.view());
    struct Far {
        double operator()(unsigned, uint64_t r) const
        {
            if (r < 1000)
                return 0.0;                   /* the run starts near 0 ... */
            return 1e8 + ((r % 2) ? 0.001 : -0.001);   /* ... then sits far away, spread 0.001 */
        }
    } far;
    for (int k = 0; k < 20; ++k)
        f.send(mon, 100, far);
    f.send(mon, 1, far);
    const MonitorUpdate *u = out.lastValues();
    /* reference: centred two-pass over the same doubles (1e8 +- 0.001 is not exact in binary) */
    double s = 0, v = 0;
    for (uint64_t r = 1801; r < 2101; ++r)
        s += far(1, r);
    double m = s / 300;
    for (uint64_t r = 1801; r < 2101; ++r)
        v += (far(1, r) - m) * (far(1, r) - m);
    double ref = sqrt(v / 300);
    testOk(close(u->slot[0].mean, m, 1e-15) && fabs(u->slot[0].stddev - ref) < 1e-12,
           "large offset, small spread, after the start values expired: sd %.15g, centred %.15g",
           u->slot[0].stddev, ref);
}

void testHealth()
{
    Cal cal;
    {
        Out out;
        Monitor mon(out, cal);
        StreamView none;
        memset(&none, 0, sizeof none);
        mon.start(none);
        testOk(out.all.back().health == HEALTH_NO_STREAM, "Start with no stream");
    }
    {
        Out out;
        Feed f(1, motors(1));
        cal.set(1, f.motors, 0);
        Monitor mon(out, cal);
        mon.start(f.view(true));
        testOk(out.all.back().health == HEALTH_FAULTED, "Start on a faulted stream: faulted");
        f.send(mon, 300, lin);
        testOk(out.valueUpdates() == 0, "no updates from a faulted stream");
    }
    {
        Out out;
        Feed f(1, motors(1));
        Monitor mon(out, cal);
        mon.configure(cfg(0.01), f.view());
        mon.start(f.view(false, true));
        testOk(out.all.back().health == HEALTH_SOURCE_STOPPED, "Start after a final STATUS: source stopped");
        f.send(mon, 201, lin);
        testOk(out.valueUpdates() == 1 && out.lastValues()->health == HEALTH_SOURCE_STOPPED &&
               out.lastValues()->slot[0].severity == SEVERITY_INVALID && !isnan(out.lastValues()->slot[0].mean),
               "tail rows: values published, still source stopped, INVALID");
    }
    {
        Out out;
        Feed f(1, motors(1));
        Monitor mon(out, cal);
        mon.configure(cfg(0.01), f.view());
        mon.start(f.view());
        f.send(mon, 150, lin);
        mon.event(f.ev(EVENT_DATA_STALLED));
        testOk(out.all.back().health == HEALTH_STALLED && !out.all.back().values &&
               out.all.back().slot[0].severity == SEVERITY_INVALID, "stall: INVALID at once");
        f.send(mon, 100, lin, 10000);
        testOk(out.lastValues()->health == HEALTH_OK, "the next completed update clears a stall");
        mon.event(f.ev(EVENT_STREAM_ENDED));
        testOk(out.all.back().health == HEALTH_SOURCE_STOPPED && !out.all.back().values,
               "final STATUS: source stopped at once");
        mon.event(f.ev(EVENT_DATA_STALLED));
        testOk(out.all.back().health == HEALTH_SOURCE_STOPPED, "a later stall does not replace source stopped");
        f.send(mon, 200, lin);
        testOk(out.lastValues()->health == HEALTH_SOURCE_STOPPED, "an update does not clear source stopped");
        mon.event(f.ev(EVENT_STREAM_FAULTED));
        testOk(out.all.back().health == HEALTH_FAULTED, "fault");
        size_t n = out.all.size();
        f.send(mon, 300, lin);
        testOk(out.all.size() == n, "no updates after the fault");
    }
}

void testReplacementAndRuns()
{
    Out out; Cal cal;
    Feed a(1, motors(1)), b(2, motors(1, 2));
    cal.set(1, a.motors, 0);
    Monitor mon(out, cal);
    mon.configure(cfg(0.03, 0.01), a.view());
    mon.start(a.view());
    uint64_t run0 = out.all.back().run;
    a.send(mon, 250, lin);
    StreamEvent rep = b.ev(EVENT_STREAM_REPLACED);
    rep.oldStream = 1;
    mon.event(rep);
    testOk(out.all.back().health == HEALTH_REPLACED && out.all.back().run > run0,
           "replacement: published at once, new run");
    mon.event(b.started());
    testOk(out.all.back().health == HEALTH_WAITING && out.all.back().stream == 2 &&
           out.all.back().slot[1].present, "the new stream's layout (Motor[2] now present)");
    cal.set(1, b.motors, 0);
    b.send(mon, 101, lin);
    const MonitorUpdate *u = out.lastValues();
    testOk(u->stream == 2 && u->samples == 100 && close(u->coverage, 1.0 / 3),
           "history cleared: the first update of the new stream covers only its rows");
    uint64_t r1 = out.all.back().run;
    mon.configure(cfg(0.02, 0.01), b.view());
    testOk(out.all.back().run == r1 + 1 && out.all.back().health == HEALTH_WAITING,
           "T changed while running: internal Stop/Start, new run");
    mon.stop();
    testOk(out.all.back().health == HEALTH_STOPPED && out.all.back().run == r1 + 2, "Stop");
    size_t n = out.all.size();
    b.send(mon, 500, lin);
    testOk(out.all.size() == n, "nothing while stopped");
}

void testCalibrationAndLayout()
{
    Out out; Cal cal;
    Feed f(1, motors(3, 1, 9), motors(1, 3));
    cal.set(1, motors(3, 1, 9), 0.0);
    cal.set(2, motors(3, 1, 9), 1.0);           /* HomePos = motor number */
    Monitor mon(out, cal);
    mon.configure(cfg(0.01), f.view());
    mon.start(f.view());
    testOk(mon.ignoredMotors() == 1, "Motor[9] ignored and counted");
    f.send(mon, 50, lin);
    f.gen = 2;
    f.send(mon, 51, lin);
    const MonitorUpdate *u = out.lastValues();
    double expect = 0;
    for (uint64_t r = 0; r < 100; ++r)
        expect += lin(3, r) - (r < 50 ? 0.0 : 3.0);
    expect /= 100;
    testOk(u->calibrationChanged && u->slot[2].severity == SEVERITY_MINOR && close(u->slot[2].mean, expect),
           "a generation change in the window: each row with its own HomePos, flagged, MINOR");
    testOk(u->slot[0].present && u->slot[2].present && !u->slot[1].present, "slots by motor number");
    testOk(u->slot[0].hasStatus && u->slot[0].statusOr == (1u << 1) && u->slot[2].statusOr == (1u << 3),
           "status words matched by their own motor numbers");
}

void testR3Numbers()
{
    Out out; Cal cal;
    Monitor mon(out, cal);
    Feed f(1, motors(1));
    testOk(!mon.configure(cfg(1e300), f.view()).empty(), "R3: T = U = 1e300 at a known row rate: refused");
    StreamView none;
    memset(&none, 0, sizeof none);
    testOk(mon.configure(cfg(1e300), none).empty(), "with no stream yet it cannot be checked: accepted");
    mon.start(f.view());
    testOk(out.all.back().health == HEALTH_UNSUPPORTED, "Start on a stream where it is not representable: unsupported");
    f.send(mon, 500, lin);
    testOk(out.valueUpdates() == 0, "no windows, no updates");
    testOk(mon.configure(cfg(0.01), f.view()).empty() && out.all.back().health == HEALTH_WAITING,
           "a representable T while running: restarted, waiting");
}

/* a refused configuration changes nothing */
void testRefusedConfigure()
{
    Cal cal;
    Feed f(1, motors(1));
    cal.set(1, f.motors, 0);
    {
        Out out;
        Monitor mon(out, cal);
        mon.configure(cfg(0.01), f.view());
        mon.start(f.view());
        f.send(mon, 150, lin);
        uint64_t run = out.all.back().run;
        size_t published = out.all.size();
        testOk(!mon.configure(cfg(1e300), f.view()).empty(), "running: an unrepresentable T is refused");
        testOk(out.all.size() == published && mon.effective().known && mon.effective().rowsPerUpdate == 100 &&
               mon.running(), "nothing published, read-backs and run state unchanged");
        f.send(mon, 51, lin);                   /* completes the window the refusal must not have reset */
        const MonitorUpdate *u = out.lastValues();
        testOk(u->run == run && u->samples == 100 && u->coverage == 1.0 && u->health == HEALTH_OK,
               "same run, window history intact: the next update is whole");
    }
    {
        Out out;
        Monitor mon(out, cal);
        testOk1(mon.configure(cfg(0.01), f.view()).empty());
        testOk(!mon.configure(cfg(1e300), f.view()).empty() && mon.effective().known &&
               mon.effective().rowsPerUpdate == 100, "stopped: refused, the last accepted read-backs kept");
        mon.start(f.view());
        testOk(out.all.back().health == HEALTH_WAITING, "Start uses the last accepted configuration");
        f.send(mon, 101, lin);
        testOk(out.lastValues() && out.lastValues()->samples == 100, "and it runs");
    }
    {
        Out out;
        Monitor mon(out, cal);
        StreamView none;
        memset(&none, 0, sizeof none);
        mon.configure(cfg(0.01), f.view());
        testOk(mon.configure(cfg(0.02), none).empty() && !mon.effective().known,
               "no stream: accepted, read-backs unknown (not stale)");
    }
}

/* the gate over the window: last-sample level, edge count, active fraction (Option A) */
void testGate()
{
    struct Wave {   /* high [0,30) and [70,100) within each 100-row block */
        uint8_t operator()(uint64_t r) const { uint64_t p = r % 100; return (p < 30 || p >= 70) ? 1 : 0; }
    } wave;
    struct Block {  /* constant per 100-row sub-block, alternating 1,0,1,0,... */
        uint8_t operator()(uint64_t r) const { return (r / 100) % 2 == 0 ? 1 : 0; }
    } block;

    /* one 100-row window: level = last sample, internal edges, active fraction */
    {
        Out out; Cal cal;
        Feed f(1, motors(1), std::vector<uint8_t>(), 1e-4, 1, true);
        cal.set(1, f.motors, 0);
        Monitor mon(out, cal);
        mon.configure(cfg(0.01), f.view());
        mon.start(f.view());
        f.sendG(mon, 100, lin, wave);          /* rows 0..99 fill the sub-block */
        f.sendG(mon, 1, lin, wave);            /* row 100 closes it: update covers rows 0..99 */
        const MonitorUpdate *u = out.lastValues();
        testOk(u->hasGate && u->gateSeverity == SEVERITY_NONE, "gate present and valid");
        testOk(u->gateLevel == 1, "level = last sample in the window (High)");
        testOk(u->gateEdges == 2, "internal edges: 2 (at 30 and 70), got %u", u->gateEdges);
        testOk(close(u->gateActiveFraction, 0.6), "active fraction 60/100 = %.3f", u->gateActiveFraction);
    }
    /* three sub-blocks (gate 1,0,1), constant within each: boundary edges counted */
    {
        Out out; Cal cal;
        Feed f(1, motors(1), std::vector<uint8_t>(), 1e-4, 1, true);
        cal.set(1, f.motors, 0);
        Monitor mon(out, cal);
        mon.configure(cfg(0.03, 0.01), f.view());
        mon.start(f.view());
        for (int k = 0; k < 4; ++k)
            f.sendG(mon, 100, lin, block);     /* update now covers rows 0..299 (sub 1/0/1) */
        const MonitorUpdate *u = out.lastValues();
        testOk(u->gateLevel == 1 && u->gateEdges == 2 && close(u->gateActiveFraction, 2.0 / 3),
               "3 sub-blocks (1,0,1): 2 boundary edges, level High, fraction %.3f", u->gateActiveFraction);
    }
    /* no gate item in the layout: invalid */
    {
        Out out; Cal cal;
        Feed f(1, motors(1));
        cal.set(1, f.motors, 0);
        Monitor mon(out, cal);
        mon.configure(cfg(0.01), f.view());
        mon.start(f.view());
        f.send(mon, 101, lin);
        const MonitorUpdate *u = out.lastValues();
        testOk(!u->hasGate && u->gateSeverity == SEVERITY_INVALID, "no gate item: INVALID");
    }
}

}

MAIN(testMonitor)
{
    testPlan(0);
    testConfig();
    testEffective();
    testBackToBack();
    testSlidingAndCoverage();
    testOneUpdatePerCall();
    testPrecision();
    testHealth();
    testReplacementAndRuns();
    testCalibrationAndLayout();
    testR3Numbers();
    testRefusedConfigure();
    testGate();
    return testDone();
}
