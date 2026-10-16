/* esMonitor.cpp - the continuous monitor. See esMonitor.h and the ADEncstream guide */
#include "esMonitor.h"

#include <string.h>

#include <cmath>
#include <limits>

namespace encstream {

namespace {

double nan() { return std::numeric_limits<double>::quiet_NaN(); }


}

const char *healthName(MonitorHealth h)
{
    switch (h) {
    case HEALTH_STOPPED: return "stopped";
    case HEALTH_NO_STREAM: return "no stream";
    case HEALTH_WAITING: return "waiting for the first update";
    case HEALTH_OK: return "ok";
    case HEALTH_STALLED: return "DATA stalled";
    case HEALTH_SOURCE_STOPPED: return "data source stopped";
    case HEALTH_FAULTED: return "stream faulted";
    case HEALTH_REPLACED: return "stream replaced";
    case HEALTH_UNSUPPORTED: return "averaging time not representable at this stream's row rate";
    }
    return "unknown";
}

std::string checkMonitorConfig(const MonitorConfig &c)
{
    double t = c.averagingTimeS, u = c.updatePeriodS == 0 ? c.averagingTimeS : c.updatePeriodS;
    if (!std::isfinite(t) || t <= 0)
        return "averaging time must be a positive number";
    if (!std::isfinite(u) || u <= 0)
        return "update period must be a positive number";
    if (u < MONITOR_MIN_UPDATE_S * (1 - 1e-9))
        return "update period must be at least 10 ms";
    if (u > t * (1 + 1e-9))
        return "update period must not exceed the averaging time";
    double k = std::floor(t / u + 0.5);
    if (k < 1 || k > MONITOR_MAX_SUBBLOCKS)
        return "averaging time / update period must be 1..1000";
    if (!std::isfinite(c.minCoverage) || c.minCoverage < 0 || c.minCoverage > 1)
        return "minimum coverage must be 0..1";
    return "";
}

Monitor::Monitor(MonitorOutput &out, const CalibrationLookup &cal)
    : out_(out), cal_(cal), running_(false), run_(0), health_(HEALTH_STOPPED), hasLayout_(false),
      stream_(0), havePending_(false), lastCompleted_(0), completedThisCall_(0), updatesSkipped_(0),
      updatesPublished_(0), ignoredMotors_(0)
{
    memset(&layout_, 0, sizeof layout_);
    memset(&eff_, 0, sizeof eff_);
    memset(&last_, 0, sizeof last_);
    for (unsigned s = 0; s < MONITOR_SLOTS; ++s) {
        axisColumn_[s] = statusColumn_[s] = -1;
        last_.slot[s].mean = last_.slot[s].stddev = last_.slot[s].min = last_.slot[s].max = nan();
        last_.slot[s].severity = SEVERITY_INVALID;
    }
    last_.coverage = nan();
    last_.gateActiveFraction = nan();
    last_.gateSeverity = SEVERITY_INVALID;
    ring_.resize(MONITOR_MAX_SUBBLOCKS + 1);
}

/* ---- configuration and control (5.1) --------------------------------------------------------- */

static bool effectiveFor(const MonitorConfig &c, const LayoutSummary &l, MonitorEffective *out);

std::string Monitor::configure(const MonitorConfig &c, const StreamView &now)
{
    std::string why = checkMonitorConfig(c);
    if (!why.empty())
        return why;
    /* a refused configuration changes nothing: checked before anything is installed */
    MonitorEffective candidate;
    if (now.hasStream && !effectiveFor(c, now.layout, &candidate))
        return healthName(HEALTH_UNSUPPORTED);
    cfg_ = c;
    if (running_) {
        restart(&now);                        /* T or U changed while running: Stop/Start */
    } else {
        hasLayout_ = false;
        if (now.hasStream)
            setLayout(now.layout, now.stream);
        computeEffective();                   /* read-backs: unknown without a stream */
    }
    return "";
}

void Monitor::start(const StreamView &now)
{
    running_ = true;
    restart(&now);
}

void Monitor::stop()
{
    running_ = false;
    ++run_;
    health_ = HEALTH_STOPPED;
    win_.stop();
    publishState();
}

MonitorEffective Monitor::effective() const { return eff_; }

void Monitor::setLayout(const LayoutSummary &l, uint32_t stream)
{
    hasLayout_ = true;
    layout_ = l;
    stream_ = stream;
    for (unsigned s = 0; s < MONITOR_SLOTS; ++s)
        axisColumn_[s] = statusColumn_[s] = -1;
    ignoredMotors_ = 0;                       /* of this layout */
    for (unsigned i = 0; i < l.nAxes; ++i) {
        unsigned m = l.axisMotor[i];
        if (m >= 1 && m <= MONITOR_SLOTS)
            axisColumn_[m - 1] = (int)i;
        else
            ++ignoredMotors_;                 /* slots are Motor[1..8] */
    }
    for (unsigned i = 0; i < l.nStatus; ++i) {
        unsigned m = l.statusMotor[i];
        if (m >= 1 && m <= MONITOR_SLOTS)
            statusColumn_[m - 1] = (int)i;
    }
}

/* Effective U in rows and T/U for configuration `c` at the row interval of layout `l`, without
 * side effects. Every value is checked as a double before any cast: rows per update at most
 * 2^40 (far beyond any real T), so ticks per sub-block and the expected count fit in 64 bits.
 * false (and *out unknown): not representable. */
static bool effectiveFor(const MonitorConfig &c, const LayoutSummary &l, MonitorEffective *out)
{
    const double maxRows = 1099511627776.0;  /* 2^40 */
    memset(out, 0, sizeof *out);
    if (l.gatherPeriod == 0)
        return false;
    double rowT = l.servoPeriodS * l.gatherPeriod;
    if (!std::isfinite(rowT) || !(rowT > 0))
        return false;
    double t = c.averagingTimeS, u = c.updatePeriodS == 0 ? t : c.updatePeriodS;
    double r = std::floor(u / rowT + 0.5);    /* half-way rounds up */
    if (!std::isfinite(r) || r > maxRows)
        return false;
    if (r < 1)
        r = 1;
    if (r * rowT < MONITOR_MIN_UPDATE_S * (1 - 1e-9)) {
        r = std::ceil(MONITOR_MIN_UPDATE_S / rowT - 1e-9);   /* still at least 10 ms */
        if (!std::isfinite(r) || r > maxRows)
            return false;
    }
    uint64_t rowsU = (uint64_t)r;
    double k = std::floor(t / u + 0.5);
    unsigned subBlocks = k < 1 ? 1u : k > MONITOR_MAX_SUBBLOCKS ? (unsigned)MONITOR_MAX_SUBBLOCKS : (unsigned)k;
    out->known = true;
    out->rowsPerUpdate = rowsU;
    out->subBlocks = subBlocks;
    out->updatePeriodS = (double)rowsU * rowT;
    out->averagingTimeS = out->updatePeriodS * subBlocks;
    return true;
}

/* the installed configuration at the current layout; eff_ unknown without a layout */
bool Monitor::computeEffective()
{
    if (!hasLayout_) {
        memset(&eff_, 0, sizeof eff_);
        return false;
    }
    return effectiveFor(cfg_, layout_, &eff_);
}

/* clear all history; health from the stream's current facts */
void Monitor::restart(const StreamView *now)
{
    ++run_;
    for (size_t i = 0; i < ring_.size(); ++i)
        ring_[i].used = false;
    havePending_ = false;
    win_.stop();
    hasLayout_ = false;
    if (!now || !now->hasStream) {
        health_ = HEALTH_NO_STREAM;
    } else {
        setLayout(now->layout, now->stream);
        health_ = now->fault != FAULT_NONE ? HEALTH_FAULTED
                : now->hasFinal ? HEALTH_SOURCE_STOPPED : HEALTH_WAITING;
    }
    if (!computeEffective() && hasLayout_ && health_ == HEALTH_WAITING)
        health_ = HEALTH_UNSUPPORTED;         /* no windows until T/U change */
    if (hasLayout_ && eff_.known && health_ != HEALTH_FAULTED)
        win_.start(eff_.rowsPerUpdate * layout_.gatherPeriod);
    /* no values yet in this run */
    for (unsigned s = 0; s < MONITOR_SLOTS; ++s) {
        MonitorSlot &sl = last_.slot[s];
        sl.present = axisColumn_[s] >= 0;
        sl.hasStatus = statusColumn_[s] >= 0;
        sl.mean = sl.stddev = sl.min = sl.max = nan();
        sl.statusOr = 0;
        sl.severity = SEVERITY_INVALID;
    }
    last_.samples = 0;
    last_.expected = eff_.known ? eff_.rowsPerUpdate * eff_.subBlocks : 0;
    last_.coverage = nan();
    last_.calibrationChanged = false;
    last_.hasGate = hasLayout_ && layout_.hasGate;
    last_.gateLevel = 0;
    last_.gateEdges = 0;
    last_.gateActiveFraction = nan();
    last_.gateSeverity = SEVERITY_INVALID;
    publishState();
}

/* ---- events (5.4 table) ---------------------------------------------------------------------- */

void Monitor::event(const StreamEvent &e)
{
    if (!running_)
        return;
    switch (e.type) {
    case EVENT_STREAM_REPLACED:
        ++run_;
        for (size_t i = 0; i < ring_.size(); ++i)
            ring_[i].used = false;
        win_.stop();
        hasLayout_ = false;
        health_ = HEALTH_REPLACED;
        publishState();
        break;
    case EVENT_STREAM_STARTED: {
        StreamView v;
        memset(&v, 0, sizeof v);
        v.hasStream = true;
        v.stream = e.stream;
        v.layout = e.layout;
        restart(&v);                           /* a new stream: everything starts afresh */
        break;
    }
    case EVENT_STREAM_FAULTED:
        if (e.stream != stream_)
            break;
        health_ = HEALTH_FAULTED;
        for (size_t i = 0; i < ring_.size(); ++i)
            ring_[i].used = false;
        win_.stop();
        publishState();
        break;
    case EVENT_STREAM_ENDED:
        if (e.stream != stream_ || health_ == HEALTH_FAULTED)
            break;
        health_ = HEALTH_SOURCE_STOPPED;       /* latched; tail rows still accumulate */
        publishState();
        break;
    case EVENT_DATA_STALLED:
        if (health_ == HEALTH_OK || health_ == HEALTH_WAITING) {
            health_ = HEALTH_STALLED;
            publishState();
        }
        break;
    default:
        break;                                 /* stall ended: the next update restores it */
    }
}

void Monitor::gap(const GapReport &)
{
    /* nothing: the first row after the gap advances the windows (2.2) */
}

/* ---- rows (5.2, 5.3) ------------------------------------------------------------------------- */

void Monitor::resetSub(Sub &s, uint64_t index)
{
    s.used = true;
    s.index = index;
    s.rows = 0;
    s.gen = 0;
    s.mixedGen = false;
    for (unsigned a = 0; a < MONITOR_SLOTS; ++a) {
        Acc &c = s.acc[a];
        c.n = 0;
        c.mean = c.m2 = 0;
        c.min = c.max = 0;
        c.statusOr = 0;
    }
    s.haveGate = false;
    s.firstGate = s.lastGate = 0;
    s.high = 0;
    s.internalEdges = 0;
}

/* The ring has T/U + 1 entries (index mod T/U + 1): the sub-block being filled never shares an
 * entry with the T/U completed ones an update covers. */
void Monitor::advanceTo(const WindowStep &st)
{
    const unsigned k = eff_.subBlocks, ringN = k + 1;
    if (st.first) {
        resetSub(ring_[0], 0);
        return;
    }
    if (st.closed) {
        ++completedThisCall_;
        lastCompleted_ = st.closedIndex;
        havePending_ = true;
    }
    if (st.emptyCount > 0) {
        completedThisCall_ += st.emptyCount;
        lastCompleted_ = st.emptyFirst + st.emptyCount - 1;
        /* only the newest k empty sub-blocks can matter: at most k resets */
        uint64_t from = st.emptyCount > k ? st.emptyFirst + st.emptyCount - k : st.emptyFirst;
        for (uint64_t i = from; i < st.emptyFirst + st.emptyCount; ++i)
            resetSub(ring_[i % ringN], i);
    }
    if (st.closed)
        resetSub(ring_[st.index % ringN], st.index);
}

void Monitor::rows(const RowBlock &b)
{
    if (!running_ || !hasLayout_ || !win_.started() || b.stream != stream_ ||
        health_ == HEALTH_FAULTED || health_ == HEALTH_REPLACED)
        return;
    /* each block is corrected with its own generation's HomePos (live calibration) */
    double home[MONITOR_SLOTS];
    for (unsigned s = 0; s < MONITOR_SLOTS; ++s)
        home[s] = 0;
    Calibration c;
    if (cal_.calibration(b.calGeneration, &c))
        for (unsigned i = 0; i < c.nAxes; ++i)
            if (c.axes[i].motor >= 1 && c.axes[i].motor <= MONITOR_SLOTS)
                home[c.axes[i].motor - 1] = c.axes[i].home_pos;

    const unsigned ringN = eff_.subBlocks + 1;
    completedThisCall_ = 0;
    for (uint32_t r = 0; r < b.nRows; ++r) {
        WindowStep st = win_.row(b.tick[r]);
        advanceTo(st);
        Sub &s = ring_[st.index % ringN];
        if (s.rows == 0)
            s.gen = b.calGeneration;
        else if (s.gen != b.calGeneration)
            s.mixedGen = true;
        ++s.rows;
        for (unsigned a = 0; a < MONITOR_SLOTS; ++a) {
            if (axisColumn_[a] < 0)
                continue;
            double x = b.position[axisColumn_[a]][r] - home[a];
            Acc &q = s.acc[a];
            ++q.n;                             /* Welford */
            double d = x - q.mean;
            q.mean += d / (double)q.n;
            q.m2 += d * (x - q.mean);
            if (q.n == 1 || x < q.min)
                q.min = x;
            if (q.n == 1 || x > q.max)
                q.max = x;
            if (statusColumn_[a] >= 0)
                q.statusOr |= b.status[statusColumn_[a]][r];
        }
        if (b.gate) {
            uint8_t gv = b.gate[r];
            if (!s.haveGate) {
                s.firstGate = s.lastGate = gv;
                s.haveGate = true;
            } else if (gv != s.lastGate) {
                ++s.internalEdges;
            }
            s.lastGate = gv;
            if (gv)
                ++s.high;
        }
    }
    if (completedThisCall_ > 0) {
        updatesSkipped_ += completedThisCall_ - 1;   /* at most one update per call */
        computeAndPublish();
    }
}

Severity Monitor::healthSeverity() const
{
    return health_ == HEALTH_OK ? SEVERITY_NONE : SEVERITY_INVALID;
}

/* the window ending at the newest completed sub-block, combined afresh with Chan's merge */
void Monitor::computeAndPublish()
{
    const unsigned k = eff_.subBlocks, ringN = k + 1;
    const uint64_t c = lastCompleted_;
    const uint64_t first = c + 1 >= k ? c + 1 - k : 0;
    Acc tot[MONITOR_SLOTS];
    memset(tot, 0, sizeof tot);
    uint64_t rows = 0;
    bool mixed = false, haveGen = false;
    uint32_t gen = 0;
    uint64_t gateHigh = 0, gateRows = 0;
    uint32_t gateEdges = 0;
    uint8_t windowLastGate = 0, prevLastGate = 0;
    bool haveGate = false, havePrevGate = false;
    for (uint64_t i = first; i <= c; ++i) {
        const Sub &s = ring_[i % ringN];
        if (!s.used || s.index != i || s.rows == 0)
            continue;                          /* empty, or a stale entry */
        rows += s.rows;
        if (s.mixedGen || (haveGen && s.gen != gen))
            mixed = true;
        haveGen = true;
        gen = s.gen;
        for (unsigned a = 0; a < MONITOR_SLOTS; ++a) {
            const Acc &b = s.acc[a];
            if (b.n == 0)
                continue;
            Acc &t = tot[a];
            if (t.n == 0) {
                t = b;
                continue;
            }
            double n = (double)(t.n + b.n), d = b.mean - t.mean;
            t.mean += d * (double)b.n / n;
            t.m2 += b.m2 + d * d * (double)t.n * (double)b.n / n;
            t.n += b.n;
            if (b.min < t.min)
                t.min = b.min;
            if (b.max > t.max)
                t.max = b.max;
            t.statusOr |= b.statusOr;
        }
        if (s.haveGate) {
            gateHigh += s.high;
            gateRows += s.rows;
            gateEdges += s.internalEdges;
            if (havePrevGate && prevLastGate != s.firstGate)
                ++gateEdges;
            prevLastGate = s.lastGate;
            havePrevGate = true;
            windowLastGate = s.lastGate;
            haveGate = true;
        }
    }
    havePending_ = false;
    if (health_ == HEALTH_WAITING || health_ == HEALTH_STALLED)
        health_ = HEALTH_OK;                   /* source stop and fault stay latched */

    MonitorUpdate u;
    memset(&u, 0, sizeof u);
    u.run = run_;
    u.stream = stream_;
    u.health = health_;
    u.values = true;
    u.samples = rows;
    u.expected = eff_.rowsPerUpdate * k;
    u.coverage = u.expected ? (double)rows / (double)u.expected : 0;
    u.calibrationChanged = mixed;
    u.updatesSkipped = updatesSkipped_;
    Severity hs = healthSeverity();
    for (unsigned a = 0; a < MONITOR_SLOTS; ++a) {
        MonitorSlot &sl = u.slot[a];
        sl.present = axisColumn_[a] >= 0;
        sl.hasStatus = statusColumn_[a] >= 0;
        const Acc &t = tot[a];
        if (!sl.present || t.n == 0) {
            sl.mean = sl.stddev = sl.min = sl.max = nan();
            sl.severity = SEVERITY_INVALID;
            continue;
        }
        sl.mean = t.mean;
        sl.stddev = std::sqrt(t.m2 / (double)t.n);  /* population; 0 for one sample */
        sl.min = t.min;
        sl.max = t.max;
        sl.statusOr = t.statusOr;
        Severity s = hs;
        if (s < SEVERITY_MINOR && (u.coverage < cfg_.minCoverage || mixed))
            s = SEVERITY_MINOR;
        sl.severity = s;
    }
    u.hasGate = layout_.hasGate != 0;
    bool gateData = haveGate && gateRows > 0;
    u.gateLevel = windowLastGate;
    u.gateEdges = gateEdges;
    u.gateActiveFraction = gateData ? (double)gateHigh / (double)gateRows : nan();
    if (!u.hasGate || !gateData) {
        u.gateSeverity = SEVERITY_INVALID;
    } else {
        Severity gs = hs;
        if (gs < SEVERITY_MINOR && (u.coverage < cfg_.minCoverage || mixed))
            gs = SEVERITY_MINOR;
        u.gateSeverity = gs;
    }
    last_ = u;
    ++updatesPublished_;
    out_.publish(u);
}

/* a state change without a completed update: the last values with the current health */
void Monitor::publishState()
{
    MonitorUpdate u = last_;
    u.run = run_;
    u.stream = stream_;
    u.health = health_;
    u.values = false;
    u.updatesSkipped = updatesSkipped_;
    if (health_ != HEALTH_OK) {
        for (unsigned a = 0; a < MONITOR_SLOTS; ++a)
            u.slot[a].severity = SEVERITY_INVALID;
        u.gateSeverity = SEVERITY_INVALID;
    }
    out_.publish(u);
}

}
