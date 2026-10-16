/* esFramer.cpp - the acquisition framer. See esFramer.h and the ADEncstream guide */
#include "esFramer.h"

#include <stdio.h>
#include <string.h>

#include <cmath>
#include <limits>

namespace encstream {

FramerConfig::FramerConfig()
    : trigger(TRIGGER_FREE_RUNNING), mode(ACQUIRE_CONTINUOUS), numAcquire(1), framePoints(0),
      frameTimeS(0.1), maxFrameSamples(100000), minCoverage(0.9), withSampleIndex(false),
      withStatus(false)
{
}

const char *invalidReasonText(InvalidReason r)
{
    switch (r) {
    case INVALID_NONE: return "";
    case INVALID_STREAM_REPLACED: return "Acquisition invalid: the stream was replaced (encstream restarted)";
    case INVALID_STREAM_FAULTED: return "Acquisition invalid: the stream faulted";
    case INVALID_NO_FREE_FRAME:
    case INVALID_QUEUE_FULL: return "Acquisition invalid: frame queue full - the IOC could not keep up";
    case INVALID_PUBLICATION_FAILED: return "Acquisition invalid: a frame could not be published";
    case INVALID_FRAME_STORAGE: return "Acquisition invalid: the frame pool's storage is smaller than the acquisition needs";
    case INVALID_CLOCK_RANGE: return "Acquisition invalid: a window bound does not fit the 64-bit sample clock";
    }
    return "Acquisition invalid";
}

Framer::Framer(FrameOutput &out, const CalibrationLookup &cal)
    : out_(out), cal_(cal), epochCounter_(0), stream_(0), nextNumber_(1),
      havePrev_(false), prevGate_(0), prevRow_(0),
      framePoints_(0), cur_(0)
{
    memset(&st_, 0, sizeof st_);
    st_.minCoverage = std::numeric_limits<double>::quiet_NaN();
    memset(&layout_, 0, sizeof layout_);
    memset(&frozen_, 0, sizeof frozen_);
}

/* ---- control (4.1, 4.2) ---------------------------------------------------------------------- */

std::string Framer::acquire(const FramerConfig &c, const StreamView &now)
{
    if (st_.state != ACQ_IDLE)
        return "an acquisition is running; stop it first";
    if (c.mode == ACQUIRE_MULTIPLE && c.numAcquire < 1)
        return "NumAcquire must be at least 1";
    if (c.maxFrameSamples < 1)
        return "MaxFrameSamples must be at least 1";
    if (!std::isfinite(c.minCoverage) || c.minCoverage < 0 || c.minCoverage > 1)
        return "MinCoverage must be 0..1";
    /* the stream's current facts */
    if (!now.hasStream)
        return "no stream";
    if (now.fault != FAULT_NONE)
        return std::string("the stream is faulted: ") + faultName(now.fault);
    if (!now.ready)
        return "the stream has no calibration yet";
    const LayoutSummary &l = now.layout;
    if (l.nAxes == 0 || l.nAxes > FRAMER_MAX_AXES)
        return "the layout has no position axes, or more than 8";
    for (unsigned a = 0; a < l.nAxes; ++a)   /* rows and addresses are Motor[1..8] */
        if (l.axisMotor[a] < 1 || l.axisMotor[a] > FRAMER_MAX_AXES) {
            char why[96];
            sprintf(why, "Motor[%u] is a position item; acquisition supports Motor[1]-Motor[8]",
                    (unsigned)l.axisMotor[a]);
            return why;
        }
    if (l.nStatus > FRAMER_MAX_AXES)          /* status columns have their own motors */
        return "the layout has more than 8 encoder status words";
    if (c.trigger != TRIGGER_FREE_RUNNING && !l.hasGate)
        return "the layout has no gate item: only Free Running is possible";
    uint64_t fp = 0;
    if (c.trigger == TRIGGER_FREE_RUNNING) {
        if (c.framePoints > 0) {
            fp = c.framePoints;
        } else {
            /* every value checked as a double before any cast */
            double rowT = l.servoPeriodS * l.gatherPeriod;
            if (!std::isfinite(rowT) || !(rowT > 0))
                return "the stream's row interval is not a positive number";
            if (!std::isfinite(c.frameTimeS) || c.frameTimeS <= 0)
                return "FrameTime must be a positive number";
            double q = std::floor(c.frameTimeS / rowT + 0.5);       /* half-way rounds up */
            if (!std::isfinite(q) || q > (double)c.maxFrameSamples)
                return "FrameTime exceeds MaxFrameSamples";
            fp = q < 1 ? 1 : (uint64_t)q;
        }
        if (fp > c.maxFrameSamples)
            return "FramePoints exceeds MaxFrameSamples";
    }
    Calibration cal;
    if (!cal_.calibration(now.latestGeneration, &cal))
        return "the stream's calibration is not available";

    cfg_ = c;
    if (cfg_.mode == ACQUIRE_SINGLE)
        cfg_.numAcquire = 1;
    stream_ = now.stream;
    layout_ = l;
    frozen_ = cal;
    framePoints_ = fp;
    memset(&schema_, 0, sizeof schema_);      /* the outputs' schema, fixed now */
    schema_.trigger = cfg_.trigger;
    schema_.nAxes = l.nAxes;
    memcpy(schema_.motor, l.axisMotor, sizeof schema_.motor);
    schema_.hasGate = l.hasGate != 0;
    schema_.gateBit = l.gateBit;
    schema_.withSampleIndex = cfg_.withSampleIndex;
    schema_.withStatus = cfg_.withStatus;
    schema_.nStatus = cfg_.withStatus ? (l.hasGate ? 1u : 0u) + l.nStatus : 0u;
    {
        unsigned k = 0;
        if (cfg_.withStatus && l.hasGate)
            schema_.statusMotor[k++] = 0;     /* the gate word */
        for (unsigned j = 0; cfg_.withStatus && j < l.nStatus; ++j)
            schema_.statusMotor[k++] = l.statusMotor[j];
    }
    schema_.servoPeriodS = l.servoPeriodS;
    schema_.gatherPeriod = l.gatherPeriod;
    schema_.framePoints = fp;
    schema_.maxFrameSamples = cfg_.maxFrameSamples;
    schema_.frozen = cal;
    memset(&st_, 0, sizeof st_);
    st_.state = ACQ_ACQUIRING;
    st_.epoch = ++epochCounter_;
    st_.minCoverage = std::numeric_limits<double>::quiet_NaN();
    st_.sourceStopped = now.hasFinal;          /* shown at once */
    st_.sourceStopRow = now.hasFinal ? now.final.next_row : 0;
    st_.tailComplete = now.tailComplete;
    st_.framePoints = fp;
    nextNumber_ = 1;
    havePrev_ = false;
    cur_ = 0;
    win_.stop();
    if (cfg_.trigger == TRIGGER_FREE_RUNNING)
        win_.start(fp * l.gatherPeriod);
    return "";
}

void Framer::stop()
{
    dropAssembling();                         /* a frame being assembled is dropped (4.5) */
    if (st_.state != ACQ_IDLE)
        out_.stopped(st_.epoch);              /* truncate its empty runs (4.9.1) */
    st_.state = ACQ_IDLE;
    win_.stop();
}

void Framer::published(uint64_t epoch, uint64_t frames, bool ok)
{
    if (epoch != st_.epoch)
        return;                               /* a late report of an older acquisition */
    if (!ok) {
        st_.notDelivered += frames;
        invalidate(INVALID_PUBLICATION_FAILED);
        return;
    }
    st_.numAcquired += frames;
    if (st_.state == ACQ_ASSEMBLED && st_.numAcquired >= cfg_.numAcquire)
        st_.state = ACQ_COMPLETE;
}

void Framer::dropAssembling()
{
    if (cur_) {
        out_.release(cur_);
        cur_ = 0;
    }
}

void Framer::invalidate(InvalidReason r)
{
    if (st_.state != ACQ_ACQUIRING && st_.state != ACQ_ASSEMBLED)
        return;
    dropAssembling();
    st_.state = ACQ_INVALID;
    st_.invalid = r;
    win_.stop();
}

/* ---- events (4.7) ---------------------------------------------------------------------------- */

void Framer::event(const StreamEvent &e)
{
    if (st_.state == ACQ_IDLE)
        return;
    switch (e.type) {
    case EVENT_STREAM_REPLACED:
        if (e.oldStream == stream_)
            invalidate(INVALID_STREAM_REPLACED);
        break;
    case EVENT_STREAM_FAULTED:
        if (e.stream == stream_)
            invalidate(INVALID_STREAM_FAULTED);
        break;
    case EVENT_STREAM_ENDED:
        if (e.stream == stream_ && !st_.sourceStopped) {
            st_.sourceStopped = true;          /* reported, not invalidated */
            st_.sourceStopRow = e.finalNextRow;
        }
        break;
    case EVENT_TAIL_COMPLETE:
        if (e.stream == stream_)
            st_.tailComplete = true;
        break;
    default:
        break;
    }
}

void Framer::gap(const GapReport &)
{
    /* nothing: row numbers show gaps between observed rows (4.3) */
}

/* ---- frames ---------------------------------------------------------------------------------- */

bool Framer::wantMore() const
{
    return st_.state == ACQ_ACQUIRING &&
           (cfg_.mode == ACQUIRE_CONTINUOUS || nextNumber_ <= cfg_.numAcquire);
}

bool Framer::isActive(uint8_t gate) const
{
    return cfg_.trigger == TRIGGER_GATE_LOW ? gate == 0 : gate == 1;
}

bool Framer::qualifyingEdge(uint8_t from, uint8_t to) const
{
    switch (cfg_.trigger) {
    case TRIGGER_RISING: return from == 0 && to == 1;
    case TRIGGER_FALLING: return from == 1 && to == 0;
    case TRIGGER_BOTH: return from != to;
    default: return false;
    }
}

bool Framer::openFrame(uint64_t tick, uint32_t servo, uint8_t gate, bool edgeInGap, uint64_t gapRows)
{
    Frame *f = out_.acquire();
    if (!f) {                                 /* no free object when a frame must start (4.9) */
        ++st_.notDelivered;
        invalidate(INVALID_NO_FREE_FRAME);
        return false;
    }
    /* the object must hold what this acquisition writes, or nothing is written */
    const size_t cap = f->capacity;
    if (cap < cfg_.maxFrameSamples || f->position.size() < (size_t)schema_.nAxes * cap ||
        (schema_.withSampleIndex && f->tick.size() < cap) ||
        (schema_.withStatus && f->status.size() < (size_t)schema_.nStatus * cap)) {
        out_.release(f);
        ++st_.notDelivered;
        invalidate(INVALID_FRAME_STORAGE);
        return false;
    }
    /* the storage (capacity, arrays) stays; everything else is reset */
    f->epoch = st_.epoch;
    f->stream = stream_;
    f->number = nextNumber_++;
    f->schema = schema_;
    f->samples = 0;
    f->firstTick = f->lastTick = tick;
    f->firstServo = f->lastServo = servo;
    f->windowStart = f->windowEnd = 0;
    f->expected = f->missing = 0;
    f->coverage = 0;
    f->startEdgeInGap = edgeInGap;
    f->startGapRows = gapRows;
    f->endEdgeInGap = false;
    f->endGapRows = 0;
    f->overlong = f->calibrationChanged = f->empty = false;
    f->statusOr = 0;
    f->gateAtStart = f->gateAtEnd = gate;
    f->gateEdges = f->gateActive = 0;
    cur_ = f;
    return true;
}

void Framer::addRow(const RowBlock &b, uint32_t r)
{
    Frame &f = *cur_;
    const uint32_t s = f.samples;
    const uint8_t gate = b.gate ? b.gate[r] : 0;
    for (unsigned a = 0; a < schema_.nAxes; ++a) {
        double home = 0;
        for (unsigned i = 0; i < frozen_.nAxes; ++i)
            if (frozen_.axes[i].motor == schema_.motor[a])
                home = frozen_.axes[i].home_pos;
        f.position[(size_t)a * f.capacity + s] = b.position[a][r] - home;   /* frozen HomePos */
    }
    if (cfg_.withSampleIndex)
        f.tick[s] = b.tick[r];
    if (cfg_.withStatus) {
        unsigned k = 0;
        if (layout_.hasGate)
            f.status[(size_t)(k++) * f.capacity + s] = b.gateWord[r];
        for (unsigned j = 0; j < b.nStatus; ++j)
            f.status[(size_t)(k++) * f.capacity + s] = b.status[j][r];
    }
    for (unsigned j = 0; j < b.nStatus; ++j)
        f.statusOr |= b.status[j][r];
    if (s == 0) {
        f.firstTick = b.tick[r];
        f.firstServo = b.servoCount[r];
        f.gateAtStart = gate;
    } else if (gate != f.gateAtEnd) {
        ++f.gateEdges;
    }
    f.lastTick = b.tick[r];
    f.lastServo = b.servoCount[r];
    f.gateAtEnd = gate;
    if (gate)
        ++f.gateActive;
    if (b.calGeneration != frozen_.generation)
        f.calibrationChanged = true;
    f.samples = s + 1;
}

void Framer::finishCounts(Frame &f)
{
    if (cfg_.trigger == TRIGGER_FREE_RUNNING) {
        f.expected = framePoints_;
    } else {
        f.expected = (f.lastTick - f.firstTick) / layout_.gatherPeriod + 1;
    }
    f.missing = f.expected > f.samples ? f.expected - f.samples : 0;
    f.coverage = f.expected ? (double)f.samples / (double)f.expected : 0;
    if (f.missing > 0 || f.startEdgeInGap || f.endEdgeInGap)
        ++st_.withGaps;
    if (f.overlong)
        ++st_.overlong;
    if (f.calibrationChanged)
        ++st_.calibrationChanged;
    if (std::isnan(st_.minCoverage) || f.coverage < st_.minCoverage)
        st_.minCoverage = f.coverage;
    if (f.coverage < cfg_.minCoverage)
        st_.degraded = true;
}

bool Framer::emitFrame(Frame *f)
{
    if (!out_.push(f)) {                      /* queue insertion failed */
        out_.release(f);
        ++st_.notDelivered;
        invalidate(INVALID_QUEUE_FULL);
        return false;
    }
    ++st_.assembled;
    if (cfg_.mode != ACQUIRE_CONTINUOUS && nextNumber_ > cfg_.numAcquire && !cur_) {
        st_.state = st_.numAcquired >= cfg_.numAcquire ? ACQ_COMPLETE : ACQ_ASSEMBLED;
        win_.stop();
    }
    return true;
}

void Framer::closeFrame(bool endEdgeInGap, uint64_t gapRows)
{
    Frame *f = cur_;
    cur_ = 0;
    f->endEdgeInGap = endEdgeInGap;
    f->endGapRows = gapRows;
    finishCounts(*f);
    emitFrame(f);
}

/* empty Free Running windows firstIndex.. as one run (4.4, 4.9.1), cut at N */
void Framer::emitEmptyRun(uint64_t firstIndex, uint64_t count)
{
    uint64_t first = firstIndex + 1;
    if (cfg_.mode != ACQUIRE_CONTINUOUS) {
        if (first > cfg_.numAcquire)
            return;
        if (count > cfg_.numAcquire - first + 1)
            count = cfg_.numAcquire - first + 1;
    }
    if (count == 0)
        return;
    EmptyRun r;
    memset(&r, 0, sizeof r);
    r.epoch = st_.epoch;
    r.stream = stream_;
    r.firstNumber = first;
    r.count = count;
    if (!win_.windowStart(firstIndex, &r.firstWindowStart)) {   /* checked, not assumed */
        invalidate(INVALID_CLOCK_RANGE);
        return;
    }
    r.windowLength = win_.length();
    r.schema = schema_;
    nextNumber_ = first + count;              /* the run's numbers are reserved */
    if (!out_.pushEmptyRun(r)) {
        st_.notDelivered += count;
        invalidate(INVALID_QUEUE_FULL);
        return;
    }
    st_.assembled += count;
    st_.empty += count;
    st_.minCoverage = 0;
    st_.degraded = true;                      /* coverage 0 */
    if (cfg_.mode != ACQUIRE_CONTINUOUS && nextNumber_ > cfg_.numAcquire && !cur_) {
        st_.state = st_.numAcquired >= cfg_.numAcquire ? ACQ_COMPLETE : ACQ_ASSEMBLED;
        win_.stop();
    }
}

/* a gate-mode frame that has reached MaxFrameSamples is emitted at once, flagged (4.5, R4) */
void Framer::capOverlong()
{
    if (cur_ && cur_->samples >= cfg_.maxFrameSamples) {
        cur_->overlong = true;
        closeFrame(false, 0);
    }
}

/* the current Free Running frame's window bounds, checked */
bool Framer::setWindow(uint64_t index)
{
    uint64_t start;
    if (!win_.windowStart(index, &start) || start > ~(uint64_t)0 - win_.length()) {
        invalidate(INVALID_CLOCK_RANGE);
        return false;
    }
    cur_->windowStart = start;
    cur_->windowEnd = start + win_.length();
    return true;
}

/* ---- rows (4.2 - 4.5) ------------------------------------------------------------------------ */

void Framer::rows(const RowBlock &b)
{
    if (st_.state != ACQ_ACQUIRING || b.stream != stream_)
        return;
    /* the first block processed after Acquire supplies the arming row (4.2): blocks before it
     * arrived while the framer was idle */
    const bool free = cfg_.trigger == TRIGGER_FREE_RUNNING;
    const bool level = cfg_.trigger == TRIGGER_GATE_HIGH || cfg_.trigger == TRIGGER_GATE_LOW;
    for (uint32_t r = 0; r < b.nRows && st_.state == ACQ_ACQUIRING; ++r) {
        const uint64_t row = b.firstRow + r;
        const uint8_t gate = b.gate ? b.gate[r] : 0;
        const uint64_t gapRows = havePrev_ && row > prevRow_ + 1 ? row - prevRow_ - 1 : 0;

        if (free) {
            WindowStep w = win_.row(b.tick[r]);
            if (w.first) {
                if (!openFrame(b.tick[r], b.servoCount[r], gate, false, 0) || !setWindow(0))
                    break;
            } else if (w.closed) {
                if (cur_)
                    closeFrame(false, 0);
                if (w.emptyCount > 0 && st_.state == ACQ_ACQUIRING)
                    emitEmptyRun(w.emptyFirst, w.emptyCount);
                nextNumber_ = w.index + 1;
                if (wantMore()) {
                    if (!openFrame(b.tick[r], b.servoCount[r], gate, false, 0) || !setWindow(w.index))
                        break;
                }
            }
            if (cur_)
                addRow(b, r);
        } else if (!havePrev_) {
            /* the first observed row: no transition yet, so a gate already active at the arming
             * row can never open a frame (4.2): frames open only on an observed transition */
        } else if (level) {
            if (cur_) {
                if (isActive(gate)) {
                    addRow(b, r);
                    capOverlong();             /* the rest of the gate: no transition, no frame */
                } else {
                    closeFrame(gapRows > 0, gapRows);
                }
            } else if (isActive(gate) && !isActive(prevGate_) && wantMore()) {
                if (!openFrame(b.tick[r], b.servoCount[r], gate, gapRows > 0, gapRows))
                    break;
                addRow(b, r);
                capOverlong();
            }
        } else {                               /* Rising, Falling, Both */
            if (qualifyingEdge(prevGate_, gate)) {
                if (cur_)
                    closeFrame(gapRows > 0, gapRows);
                if (wantMore()) {
                    if (!openFrame(b.tick[r], b.servoCount[r], gate, gapRows > 0, gapRows))
                        break;
                }
            }
            if (cur_) {
                addRow(b, r);
                capOverlong();                 /* the next qualifying edge opens the next frame */
            }
        }
        prevGate_ = gate;
        prevRow_ = row;
        havePrev_ = true;
    }
}

}
