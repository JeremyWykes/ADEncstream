/* esEngine.cpp - the receiver thread's work. See esEngine.h and the ADEncstream guide. */
#include "esEngine.h"

#include <stdio.h>
#include <string.h>

namespace encstream {

const char *displayName(AcqDisplay d)
{
    switch (d) {
    case DISPLAY_IDLE: return "idle";
    case DISPLAY_ACQUIRING: return "acquiring";
    case DISPLAY_ASSEMBLED: return "assembled";
    case DISPLAY_STOPPING: return "stopping";
    case DISPLAY_COMPLETE: return "complete";
    case DISPLAY_INVALID: return "invalid";
    }
    return "?";
}

/* ---- Mailbox --------------------------------------------------------------------------------- */

Mailbox::Mailbox(epicsEvent &wake)
    : wake_(wake), hasMon_(false), hasSnap_(false), run_(0), monOver_(0), snapOver_(0), monSeq_(0),
      snapSeq_(0)
{
    memset(&mon_, 0, sizeof mon_);
    snap_ = Snapshot();                       /* value-initialised: zeros */
}

void Mailbox::postMonitor(const MonitorUpdate &u)
{
    {
        epicsGuard<epicsMutex> g(lock_);
        if (hasMon_)
            ++monOver_;
        mon_ = u;
        hasMon_ = true;
        ++monSeq_;
        run_ = u.run;                         /* every run change is published */
    }
    wake_.trigger();
}

void Mailbox::postSnapshot(const Snapshot &s)
{
    {
        epicsGuard<epicsMutex> g(lock_);
        if (hasSnap_)
            ++snapOver_;
        snap_ = s;
        hasSnap_ = true;
        ++snapSeq_;
    }
    wake_.trigger();
}

bool Mailbox::takeMonitor(MonitorUpdate *u, uint64_t *seq)
{
    epicsGuard<epicsMutex> g(lock_);
    if (!hasMon_)
        return false;
    *u = mon_;
    if (seq)
        *seq = monSeq_;
    hasMon_ = false;
    return true;
}

bool Mailbox::takeSnapshot(Snapshot *s, uint64_t *seq)
{
    epicsGuard<epicsMutex> g(lock_);
    if (!hasSnap_)
        return false;
    *s = snap_;
    if (seq)
        *seq = snapSeq_;
    hasSnap_ = false;
    return true;
}

uint64_t Mailbox::monitorSeq() const
{
    epicsGuard<epicsMutex> g(lock_);
    return monSeq_;
}

uint64_t Mailbox::snapshotSeq() const
{
    epicsGuard<epicsMutex> g(lock_);
    return snapSeq_;
}

uint64_t Mailbox::currentRun() const
{
    epicsGuard<epicsMutex> g(lock_);
    return run_;
}

uint64_t Mailbox::monitorOverwritten() const
{
    epicsGuard<epicsMutex> g(lock_);
    return monOver_;
}

uint64_t Mailbox::snapshotsOverwritten() const
{
    epicsGuard<epicsMutex> g(lock_);
    return snapOver_;
}

/* ---- Engine ---------------------------------------------------------------------------------- */

Engine::Engine(const EngineConfig &cfg, FrameQueue &queue, CommandSlots &commands, Mailbox &mailbox)
    : cfg_(cfg), q_(queue), cmds_(commands), mb_(mailbox), framer_(q_, *this), monitor_(*this, *this),
      fan_(framer_, monitor_), rx_(cfg.receiver, *this), display_(DISPLAY_IDLE), hold_(false),
      holdEpoch_(0), contactLost_(false), dataStalled_(false), dirty_(true), revision_(0),
      lastPostNs_(0), lastCountsNs_(0), lastNumAcquired_(0), lastAssembled_(0), late_(0)
{
    scratch_ = Snapshot();
}

Outcome Engine::datagram(const uint8_t *buf, size_t capturedLen, bool truncated, const Endpoint &src,
                         uint64_t nowNs)
{
    return rx_.onDatagram(buf, capturedLen, truncated, src, nowNs);
}

void Engine::timer(uint64_t nowNs)
{
    rx_.onTimer(nowNs);
}

void Engine::event(const StreamEvent &e)
{
    fan_.event(e);
    switch (e.type) {
    case EVENT_CONTACT_LOST: contactLost_ = true; break;
    case EVENT_CONTACT_RESTORED: contactLost_ = false; break;
    case EVENT_DATA_STALLED: dataStalled_ = true; break;
    case EVENT_DATA_STALL_ENDED: dataStalled_ = false; break;
    case EVENT_STREAM_REPLACED: dataStalled_ = false; break;
    default: break;
    }
    dirty_ = true;                            /* every receiver event is a lifecycle change */
}

bool Engine::active() const
{
    AcqState s = framer_.status().state;
    return s == ACQ_ACQUIRING || s == ACQ_ASSEMBLED || s == ACQ_INVALID || hold_;
}

std::string Engine::acquire(const FramerConfig &c)
{
    if (c.maxFrameSamples > q_.capacity()) {
        char why[128];
        sprintf(why, "MaxFrameSamples %u exceeds the configured pool capacity %u",
                (unsigned)c.maxFrameSamples, (unsigned)q_.capacity());
        return why;
    }
    StreamView now = rx_.current();
    if (c.withStatus && now.hasStream && !now.layout.hasGate && now.layout.nStatus == 0)
        return "EnableStatusArray: the layout has no gate word or encoder status words";
    AcqState s = framer_.status().state;
    if (s == ACQ_INVALID || s == ACQ_COMPLETE)
        framer_.stop();                       /* a new Acquire ends the previous one (4.4) */
    std::string why = framer_.acquire(c, now);
    if (!why.empty())
        return why;
    display_ = DISPLAY_ACQUIRING;
    hold_ = false;
    acqMessage_.clear();
    lastNumAcquired_ = lastAssembled_ = 0;
    return "";
}

void Engine::stopAcquisition()
{
    const AcqStatus &st = framer_.status();
    if (st.state == ACQ_IDLE)
        return;                               /* nothing running: a Stop is a no-op */
    bool wasInvalid = st.state == ACQ_INVALID;
    uint64_t epoch = st.epoch;
    framer_.stop();                           /* drops the frame being assembled, cuts empty runs */
    hold_ = q_.outstanding(epoch);
    holdEpoch_ = epoch;
    if (wasInvalid)
        display_ = DISPLAY_INVALID;
    else if (display_ != DISPLAY_COMPLETE)
        display_ = hold_ ? DISPLAY_STOPPING : DISPLAY_IDLE;
}

CommandResult Engine::apply(const Command &c)
{
    CommandResult r;
    r.applied = true;
    r.revision = 0;
    std::string why;
    switch (c.type) {
    case CMD_ACQUIRE:
        why = acquire(c.framer);
        break;
    case CMD_STOP:
        stopAcquisition();
        break;
    case CMD_MONITOR_START:
        monitor_.start(rx_.current());
        break;
    case CMD_MONITOR_STOP:
        monitor_.stop();
        break;
    case CMD_MONITOR_CONFIGURE: {
        MonitorConfig m = c.monitor;
        if (c.monitorFields) {                /* only the fields written, on the accepted config */
            m = monitor_.config();
            if (c.monitorFields & MONITOR_FIELD_T)
                m.averagingTimeS = c.monitor.averagingTimeS;
            if (c.monitorFields & MONITOR_FIELD_U)
                m.updatePeriodS = c.monitor.updatePeriodS;
            if (c.monitorFields & MONITOR_FIELD_MIN_COVERAGE)
                m.minCoverage = c.monitor.minCoverage;
        }
        why = monitor_.configure(m, rx_.current());
        break;
    }
    }
    if (!why.empty()) {
        r.applied = false;
        r.message = why;
    }
    return r;
}

void Engine::updateState()
{
    const AcqStatus &st = framer_.status();
    AcqDisplay before = display_;
    bool activeBefore = active();
    switch (st.state) {
    case ACQ_ACQUIRING:
        display_ = DISPLAY_ACQUIRING;
        break;
    case ACQ_ASSEMBLED:
        display_ = DISPLAY_ASSEMBLED;
        break;
    case ACQ_COMPLETE:                        /* every frame published: end it */
        framer_.stop();
        display_ = DISPLAY_COMPLETE;
        hold_ = false;
        break;
    case ACQ_INVALID:
        display_ = DISPLAY_INVALID;
        if (!q_.outstanding(st.epoch)) {      /* frames already queued are published first */
            framer_.stop();
            hold_ = false;
        }
        break;
    case ACQ_IDLE:
        break;
    }
    if (hold_ && !q_.outstanding(holdEpoch_)) {
        hold_ = false;
        if (display_ == DISPLAY_STOPPING)
            display_ = DISPLAY_IDLE;
    }
    if (framer_.status().invalid != INVALID_NONE && display_ == DISPLAY_INVALID) {
        char b[256];
        uint64_t nd = framer_.status().notDelivered;
        snprintf(b, sizeof b, "%s; %llu frame%s not delivered", invalidReasonText(framer_.status().invalid),
                 (unsigned long long)nd, nd == 1 ? "" : "s");
        acqMessage_ = b;
    }
    if (display_ != before || active() != activeBefore)
        dirty_ = true;
}

void Engine::post(uint64_t nowNs)
{
    Snapshot &s = scratch_;
    s.revision = ++revision_;
    s.tNs = nowNs;
    s.counters = rx_.counters();
    s.view = rx_.current();
    s.contactLost = contactLost_;
    s.dataStalled = dataStalled_;
    s.hasCalibration = s.view.ready && rx_.calibration(s.view.latestGeneration, &s.calibration);
    s.acq = framer_.status();
    s.display = display_;
    s.active = active();
    s.epoch = s.acq.epoch;
    s.cutByStop = q_.cutByStop(s.acq.epoch);
    q_.backlog(&s.queueEntries, &s.outstandingFrames);
    s.framePointsS = (double)s.acq.framePoints * s.view.layout.servoPeriodS * s.view.layout.gatherPeriod;
    s.monitorRunning = monitor_.running();
    s.monitorConfig = monitor_.config();
    s.monitor = monitor_.effective();
    s.monitorIgnoredMotors = monitor_.ignoredMotors();
    snprintf(s.acqMessage, sizeof s.acqMessage, "%s", acqMessage_.c_str());
    s.nEvents = (unsigned)rx_.recentEvents(s.events, SNAPSHOT_EVENTS);
    mb_.postSnapshot(s);
    lastPostNs_ = nowNs;
    lastNumAcquired_ = s.acq.numAcquired;
    lastAssembled_ = s.acq.assembled;
    dirty_ = false;
}

void Engine::service(uint64_t nowNs)
{
    /* publication counts first: they can complete or invalidate the acquisition */
    const uint64_t epoch = framer_.status().epoch;
    LedgerDelta d = q_.read(epoch);
    if (d.published)
        framer_.published(epoch, d.published, true);
    if (d.failed)
        framer_.published(epoch, d.failed, false);
    updateState();

    /* commands: apply, post the state they produced, then report */
    int id;
    Command c;
    while (cmds_.next(&id, &c)) {
        CommandResult r = apply(c);
        if (!r.applied && c.type == CMD_ACQUIRE)
            acqMessage_ = "Acquire refused: " + r.message;
        updateState();
        post(nowNs);
        r.revision = revision_;
        if (cmds_.complete(id, r)) {          /* its write already timed out: publish the result */
            ++late_;
            acqMessage_ = std::string("a control write timed out; its result: ") +
                          (r.applied ? "applied" : "refused: " + r.message);
            post(nowNs);
        }
    }

    const AcqStatus &st = framer_.status();
    const bool countsChanged = st.numAcquired != lastNumAcquired_ || st.assembled != lastAssembled_;
    const double sinceNs = (double)(nowNs - lastPostNs_);
    if (dirty_ || sinceNs >= cfg_.snapshotPeriodS * 1e9 ||
        (countsChanged && sinceNs >= cfg_.countsPeriodS * 1e9))
        post(nowNs);
}

}
