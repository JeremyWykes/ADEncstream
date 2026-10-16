/* esReceiver.cpp - the ADEncstream receiver core. See esReceiver.h and the ADEncstream guide; the
 * section numbers in comments refer to the ADEncstream guide. */
#include "esReceiver.h"

#include <string.h>

namespace encstream {

namespace {

const uint64_t MAX_SUBSCRIBE_PERIOD_NS = 2000000000ull;   /* 10.1: one loss within a 5 s lease */

bool bitsEqual(double a, double b)                         /* 6.6: doubles compare bitwise */
{
    uint64_t x, y;
    memcpy(&x, &a, sizeof x);
    memcpy(&y, &b, sizeof y);
    return x == y;
}

/* 6.6: field by field, never memcmp of structs */
bool contextEqual(const es_context &a, const es_context &b)
{
    if (a.byte_order != b.byte_order || a.n_items != b.n_items || a.row_bytes != b.row_bytes ||
        !bitsEqual(a.servo_period_s, b.servo_period_s) || a.gather_period != b.gather_period ||
        a.max_lines != b.max_lines || a.id_len != b.id_len || memcmp(a.id, b.id, a.id_len) != 0)
        return false;
    for (unsigned i = 0; i < a.n_items; ++i) {
        const es_item &x = a.items[i], &y = b.items[i];
        if (x.role != y.role || x.axis != y.axis || x.gate_bit != y.gate_bit || x.type != y.type ||
            x.words != y.words || x.addr != y.addr || strcmp(x.name, y.name) != 0)
            return false;
    }
    return true;
}

bool calibrationEqual(const Calibration &a, const es_calibration &b)
{
    if (a.generation != b.generation || a.nAxes != b.n_axes)
        return false;
    for (unsigned i = 0; i < a.nAxes; ++i)
        if (a.axes[i].motor != b.axes[i].motor || !bitsEqual(a.axes[i].home_pos, b.axes[i].home_pos) ||
            !bitsEqual(a.axes[i].pos_sf, b.axes[i].pos_sf))
            return false;
    return true;
}

bool statusEqual(const es_status &a, const es_status &b)
{
    return a.state == b.state && a.enable == b.enable && a.fault == b.fault &&
           a.overruns == b.overruns && a.rows_skipped == b.rows_skipped &&
           a.rows_sent == b.rows_sent && a.packets_sent == b.packets_sent &&
           a.rows_rejected == b.rows_rejected && a.clock_violations == b.clock_violations &&
           a.send_failures == b.send_failures && a.report_failures == b.report_failures &&
           a.worst_loop_us == b.worst_loop_us && a.backlog == b.backlog &&
           a.vmrss_kb == b.vmrss_kb && a.open_fds == b.open_fds && a.next_row == b.next_row &&
           a.flags == b.flags;
}

Calibration toCalibration(const es_calibration &c)
{
    Calibration out;
    memset(&out, 0, sizeof out);
    out.generation = c.generation;
    out.nAxes = c.n_axes;
    for (unsigned i = 0; i < c.n_axes && i < ES_MAX_AXES; ++i)
        out.axes[i] = c.axes[i];
    return out;
}

}

const char *faultName(FaultReason r)
{
    switch (r) {
    case FAULT_NONE: return "none";
    case FAULT_BRICK_STATUS: return "Brick STATUS fault";
    case FAULT_CONTEXT_CHANGED: return "CONTEXT changed";
    case FAULT_CALIBRATION_MOTORS: return "CALIBRATION motors wrong";
    case FAULT_CALIBRATION_REWRITTEN: return "CALIBRATION generation rewritten";
    case FAULT_FINAL_BEHIND_CURSOR: return "final STATUS behind the cursor";
    case FAULT_FINAL_REWRITTEN: return "final STATUS rewritten";
    case FAULT_DATA_BEYOND_FINAL: return "DATA beyond the final STATUS";
    case FAULT_CLOCK_DISCONTINUITY: return "ServoCount discontinuity";
    case FAULT_CLOCK_OUT_OF_RANGE: return "sample clock out of range";
    }
    return "unknown";
}

ReceiverConfig::ReceiverConfig()
    : subscribePeriodNs(1000000000ull), contactTimeoutNs(3000000000ull),
      dataTimeoutNs(3000000000ull), eventLogCapacity(64)
{
    memset(&brick, 0, sizeof brick);
}

std::string checkConfig(const ReceiverConfig &cfg)
{
    if (cfg.brick.port == 0)
        return "Brick port is 0";
    if (cfg.identification.size() > ES_MAX_SUB_ID)
        return "identification longer than 64 bytes";
    for (size_t i = 0; i < cfg.identification.size(); ++i) {
        unsigned char ch = (unsigned char)cfg.identification[i];
        if (ch < 0x20 || ch > 0x7E)
            return "identification is not printable ASCII";
    }
    if (cfg.subscribePeriodNs == 0 || cfg.subscribePeriodNs > MAX_SUBSCRIBE_PERIOD_NS)
        return "SUBSCRIBE period must be in (0, 2 s] (one loss within the Brick's 5 s lease)";
    if (cfg.contactTimeoutNs == 0 || cfg.dataTimeoutNs == 0)
        return "timeouts must be positive";
    if (cfg.eventLogCapacity == 0 || cfg.eventLogCapacity > 4096)
        return "event log capacity must be 1..4096";
    return "";
}

/* ---- construction ---------------------------------------------------------------------------- */

Receiver::Receiver(const ReceiverConfig &cfg, RowSink &sink)
    : cfg_(cfg), sink_(sink), now_(0), started_(false), nextSub_(0), sendLen_(0), pending_(false),
      subSeq_(0), unknownGen_(false), contactArmed_(false), contactLost_(false), lastContact_(0),
      dataArmed_(false), stalled_(false), lastData_(0), has_(false), id_(0), gateItem_(-1),
      ready_(false), fault_(FAULT_NONE), hasFinal_(false), tailComplete_(false),
      hasStatus_(false), hasReport_(false), latest_(0), seqValid_(false), lastSeq_(0),
      anchored_(false), q0_(0), cursor_(0), firstRow_(0), s0_(0), gapFree_(true), nRetired_(0),
      retiredNext_(0), logStart_(0), logCount_(0)
{
    memset(&c_, 0, sizeof c_);
    memset(&ctx_, 0, sizeof ctx_);
    memset(&layout_, 0, sizeof layout_);
    memset(&final_, 0, sizeof final_);
    memset(&lastStatus_, 0, sizeof lastStatus_);
    memset(&lastReport_, 0, sizeof lastReport_);
    memset(slots_, 0, sizeof slots_);
    memset(retired_, 0, sizeof retired_);
    sendBuf_.resize(ES_SUBSCRIBE_HDR_LEN + ES_MAX_SUB_ID);
    items_.reserve(ES_MAX_ITEMS);
    axisItem_.reserve(ES_MAX_ITEMS);
    statusItem_.reserve(ES_MAX_ITEMS);
    otherItem_.reserve(ES_MAX_ITEMS);
    tick_.resize(MAX_ROWS);
    servo_.resize(MAX_ROWS);
    gateWord_.resize(MAX_ROWS);
    gate_.resize(MAX_ROWS);
    pos_.resize((size_t)ES_MAX_AXES * MAX_ROWS);
    status_.resize((size_t)ES_MAX_ITEMS * MAX_ROWS);
    other_.resize((size_t)ES_MAX_ITEMS * MAX_ROWS);
    posPtr_.resize(ES_MAX_AXES);
    statusPtr_.resize(ES_MAX_ITEMS);
    otherPtr_.resize(ES_MAX_ITEMS);
    for (unsigned a = 0; a < ES_MAX_AXES; ++a)
        posPtr_[a] = &pos_[(size_t)a * MAX_ROWS];
    for (unsigned k = 0; k < ES_MAX_ITEMS; ++k) {
        statusPtr_[k] = &status_[(size_t)k * MAX_ROWS];
        otherPtr_[k] = &other_[(size_t)k * MAX_ROWS];
    }
    log_.resize(cfg_.eventLogCapacity ? cfg_.eventLogCapacity : 1);
}

/* ---- events and the log (8.3, 9.2) --------------------------------------------------------- */

StreamEvent Receiver::blankEvent(EventType t, uint32_t stream)
{
    StreamEvent e;
    memset(&e, 0, sizeof e);
    e.type = t;
    e.tNs = now_;
    e.stream = stream;
    return e;
}

void Receiver::emit(StreamEvent &e)
{
    e.tNs = now_;
    if (logCount_ == log_.size()) {           /* full: drop the oldest, count it only */
        logStart_ = (logStart_ + 1) % log_.size();
        --logCount_;
        ++c_.eventsLost;
    }
    log_[(logStart_ + logCount_) % log_.size()] = e;
    ++logCount_;
    ++c_.eventsLogged;
    sink_.event(e);                           /* never suppressed by the log */
}

size_t Receiver::recentEvents(StreamEvent *out, size_t max) const
{
    size_t n = logCount_ < max ? logCount_ : max;
    size_t first = logCount_ - n;             /* the most recent n, oldest first */
    for (size_t i = 0; i < n; ++i)
        out[i] = log_[(logStart_ + first + i) % log_.size()];
    return n;
}

/* ---- timers (10) ------------------------------------------------------------------------------ */

void Receiver::service(uint64_t now)
{
    if (now < now_) {                         /* 3.1: time must not go backwards */
        ++c_.timeRegressions;
        now = now_;
    }
    now_ = now;
    /* 10.1 heartbeat: due at the first call, then every period; missed periods coalesced */
    if (!started_ || now >= nextSub_) {
        if (started_) {
            c_.subscribesCoalesced += (now - nextSub_) / cfg_.subscribePeriodNs;
            if (pending_)
                ++c_.subscribesCoalesced;     /* the previous one was never taken: replaced */
        }
        prepareSubscribe();
        started_ = true;
        nextSub_ = now + cfg_.subscribePeriodNs;
    }
    /* 10.2 contact watchdog */
    if (contactArmed_ && !contactLost_ && now - lastContact_ > cfg_.contactTimeoutNs) {
        contactLost_ = true;
        ++c_.contactLosses;
        StreamEvent e = blankEvent(EVENT_CONTACT_LOST, has_ ? id_ : 0);
        emit(e);
    }
    /* 10.3 DATA watchdog */
    if (dataArmed_ && !stalled_ && now - lastData_ > cfg_.dataTimeoutNs) {
        stalled_ = true;
        ++c_.dataStalls;
        StreamEvent e = blankEvent(EVENT_DATA_STALLED, id_);
        emit(e);
    }
}

void Receiver::prepareSubscribe()
{
    es_common h;
    h.type = ES_SUBSCRIBE;
    h.stream_id = 0;
    h.seq = subSeq_++;
    es_subscribe sub;
    memset(&sub, 0, sizeof sub);
    /* 10.1: metadata while no stream, no calibration, or after DATA of an unknown generation */
    sub.flags = (!has_ || !ready_ || unknownGen_) ? ES_SUB_SEND_META : 0;
    sub.id_len = (uint16_t)cfg_.identification.size();
    memcpy(sub.id, cfg_.identification.data(), sub.id_len);
    unknownGen_ = false;
    size_t w = 0;
    pending_ = es_encode_subscribe(&sendBuf_[0], sendBuf_.size(), &h, &sub, &w) == ES_OK;
    sendLen_ = pending_ ? w : 0;
}

void Receiver::onTimer(uint64_t nowNs)
{
    service(nowNs);
}

uint64_t Receiver::nextDeadline() const
{
    if (!started_)
        return now_;
    uint64_t d = nextSub_;
    if (contactArmed_ && !contactLost_ && lastContact_ + cfg_.contactTimeoutNs + 1 < d)
        d = lastContact_ + cfg_.contactTimeoutNs + 1;
    if (dataArmed_ && !stalled_ && lastData_ + cfg_.dataTimeoutNs + 1 < d)
        d = lastData_ + cfg_.dataTimeoutNs + 1;
    return d;
}

bool Receiver::pendingSend(const uint8_t **buf, size_t *len)
{
    if (!pending_)
        return false;
    pending_ = false;
    *buf = &sendBuf_[0];
    *len = sendLen_;
    if (!contactArmed_) {                     /* 10.2: contact starts at the first SUBSCRIBE */
        contactArmed_ = true;
        lastContact_ = now_;
    }
    return true;
}

void Receiver::sent(bool ok, uint64_t nowNs)
{
    (void)nowNs;
    if (ok)
        ++c_.subscribesSent;
    else
        ++c_.subscribeSendFailures;           /* no retry: the next heartbeat (10.1) */
}

void Receiver::refreshContact()
{
    lastContact_ = now_;
    if (contactLost_) {
        contactLost_ = false;
        StreamEvent e = blankEvent(EVENT_CONTACT_RESTORED, has_ ? id_ : 0);
        emit(e);
    }
}

void Receiver::disarmData(StallEnd why)
{
    if (dataArmed_ && stalled_) {
        StreamEvent e = blankEvent(EVENT_DATA_STALL_ENDED, id_);
        e.stallEnd = why;
        emit(e);
    }
    dataArmed_ = false;
    stalled_ = false;
}

void Receiver::armDataIfDue()                 /* 10.3: armed exactly while ready, no fault, no final */
{
    if (!dataArmed_ && ready_ && fault_ == FAULT_NONE && !hasFinal_) {
        dataArmed_ = true;
        stalled_ = false;
        lastData_ = now_;
    }
}

/* ---- faults (6.4) ----------------------------------------------------------------------------- */

void Receiver::fault(FaultReason r, uint8_t brickState, uint16_t brickFault)
{
    ++c_.faults;
    if (fault_ != FAULT_NONE)
        return;                               /* the first reason is kept; no further event */
    fault_ = r;
    StreamEvent e = blankEvent(EVENT_STREAM_FAULTED, id_);
    e.fault = r;
    e.brickState = brickState;
    e.brickFault = brickFault;
    emit(e);
    disarmData(STALL_END_FAULT);
}

/* ---- input ------------------------------------------------------------------------------------ */

Outcome Receiver::onDatagram(const uint8_t *buf, size_t capturedLen, bool truncated,
                             const Endpoint &src, uint64_t nowNs)
{
    service(nowNs);                           /* 3.1: due timers first */
    ++c_.datagrams;

    /* stage 1 (4) */
    if (!(src == cfg_.brick)) {
        ++c_.foreignSource;
        return OUTCOME_DROPPED;
    }
    if (truncated) {
        ++c_.truncated;
        return OUTCOME_DROPPED;
    }
    if (capturedLen > MAX_DATAGRAM) {
        ++c_.oversize;
        return OUTCOME_DROPPED;
    }

    /* stage 2 (5) */
    es_packet p;
    int r = es_decode(buf, capturedLen, &p);
    if (r == ES_E_UNKNOWN_TYPE) {
        ++c_.unknownType;
        return OUTCOME_REJECTED;
    }
    if (r != ES_OK) {
        unsigned code = (unsigned)-r;
        if (code < sizeof c_.rejected / sizeof c_.rejected[0])
            ++c_.rejected[code];
        return OUTCOME_REJECTED;
    }
    if (p.h.type == ES_SUBSCRIBE) {
        ++c_.unexpectedType;
        return OUTCOME_REJECTED;
    }
    ++c_.decoded[p.h.type];
    refreshContact();                         /* 10.2: any valid packet from the Brick */

    /* stage 3 (6.1) */
    for (unsigned i = 0; i < nRetired_; ++i)
        if (retired_[i] == p.h.stream_id) {
            ++c_.retiredStream;
            return OUTCOME_IGNORED;
        }
    if (!has_ || p.h.stream_id != id_) {
        if (p.h.type != ES_CONTEXT) {
            ++c_.beforeContext;
            return OUTCOME_IGNORED;
        }
        return newStream(p);
    }
    if (seqValid_ && p.h.seq != (uint32_t)(lastSeq_ + 1))
        ++c_.sequenceGaps;                    /* diagnostic only */
    seqValid_ = true;
    lastSeq_ = p.h.seq;

    switch (p.h.type) {
    case ES_CONTEXT:
        return onContext(p);
    case ES_CALIBRATION:
        return onCalibration(p.u.calibration);
    case ES_SENDER_REPORT:
        hasReport_ = true;
        lastReport_ = p.u.report;
        return OUTCOME_ACCEPTED;
    case ES_STATUS:
        return onStatus(p.u.status);
    case ES_DATA:
        return onData(p.u.data);
    default:
        return OUTCOME_ACCEPTED;              /* not reached: the codec knows no other types */
    }
}

Outcome Receiver::newStream(const es_packet &p)
{
    uint32_t old = id_;
    bool replaced = has_;
    if (replaced) {
        retired_[retiredNext_] = id_;
        retiredNext_ = (retiredNext_ + 1) % MAX_RETIRED;
        if (nRetired_ < MAX_RETIRED)
            ++nRetired_;
        ++c_.streamChanges;
        disarmData(STALL_END_REPLACED);
    }
    /* the new stream: every per-stream fact starts afresh */
    has_ = true;
    id_ = p.h.stream_id;
    ctx_ = p.u.context;
    ready_ = false;
    fault_ = FAULT_NONE;
    hasFinal_ = false;
    tailComplete_ = false;
    hasStatus_ = false;
    hasReport_ = false;
    memset(slots_, 0, sizeof slots_);
    latest_ = 0;
    seqValid_ = true;
    lastSeq_ = p.h.seq;
    anchored_ = false;
    q0_ = cursor_ = firstRow_ = 0;
    s0_ = 0;
    gapFree_ = true;
    dataArmed_ = stalled_ = false;

    /* the layout (8.1) */
    memset(&layout_, 0, sizeof layout_);
    layout_.byteOrder = ctx_.byte_order;
    layout_.rowBytes = ctx_.row_bytes;
    layout_.gatherPeriod = ctx_.gather_period;
    layout_.servoPeriodS = ctx_.servo_period_s;
    memcpy(layout_.identification, ctx_.id, ctx_.id_len);
    items_.clear();
    axisItem_.clear();
    statusItem_.clear();
    otherItem_.clear();
    gateItem_ = -1;
    unsigned word = 0;
    for (unsigned i = 0; i < ctx_.n_items; ++i) {
        const es_item &it = ctx_.items[i];
        Item x;
        x.word = word;
        x.type = it.type;
        x.role = it.role;
        x.axis = it.axis;
        x.gateBit = it.gate_bit;
        items_.push_back(x);
        word += it.words;
        switch (it.role) {
        case ES_ROLE_GATE:
            gateItem_ = (int)i;
            layout_.hasGate = 1;
            layout_.gateBit = it.gate_bit;
            break;
        case ES_ROLE_POSITION:
            layout_.axisMotor[axisItem_.size()] = it.axis;
            axisItem_.push_back(i);
            break;
        case ES_ROLE_ENCODER_STATUS:
            layout_.statusMotor[statusItem_.size()] = it.axis;
            statusItem_.push_back(i);
            break;
        case ES_ROLE_OTHER:
            layout_.otherItem[otherItem_.size()] = (uint8_t)i;
            otherItem_.push_back(i);
            break;
        default:
            break;
        }
    }
    layout_.nAxes = (uint8_t)axisItem_.size();
    layout_.nStatus = (uint8_t)statusItem_.size();
    layout_.nOther = (uint8_t)otherItem_.size();

    /* events in order (6.1): the old stream is gone first */
    if (replaced) {
        StreamEvent e = blankEvent(EVENT_STREAM_REPLACED, id_);
        e.oldStream = old;
        emit(e);
    }
    StreamEvent e = blankEvent(EVENT_STREAM_STARTED, id_);
    e.layout = layout_;
    emit(e);
    return OUTCOME_ACCEPTED;
}

Outcome Receiver::onContext(const es_packet &p)
{
    if (contextEqual(p.u.context, ctx_))
        return OUTCOME_ACCEPTED;
    ++c_.contextChanged;                      /* never adopted */
    fault(FAULT_CONTEXT_CHANGED);
    return OUTCOME_FAULTED;
}

bool Receiver::keptGeneration(uint32_t g) const
{
    for (unsigned i = 0; i < MAX_GENERATIONS; ++i)
        if (slots_[i].used && slots_[i].cal.generation == g)
            return true;
    return false;
}

/* 6.5: the latest (highest) generation is always kept; when full, the lowest other one goes */
void Receiver::storeGeneration(const es_calibration &cal)
{
    int freeSlot = -1, lowest = -1;
    for (unsigned i = 0; i < MAX_GENERATIONS; ++i) {
        if (!slots_[i].used) {
            if (freeSlot < 0)
                freeSlot = (int)i;
        } else if (lowest < 0 || slots_[i].cal.generation < slots_[lowest].cal.generation) {
            lowest = (int)i;
        }
    }
    int target = freeSlot;
    if (target < 0) {
        /* full: never evict the latest; a delayed older generation goes in only if it is newer
         * than the lowest kept one */
        if (cal.generation < slots_[lowest].cal.generation)
            return;
        target = lowest;
    }
    slots_[target].used = true;
    slots_[target].cal = toCalibration(cal);
    if (!ready_ || cal.generation > latest_)
        latest_ = cal.generation;
    StreamEvent e = blankEvent(EVENT_CALIBRATION_GENERATION, id_);
    e.calibration = slots_[target].cal;
    emit(e);
}

Outcome Receiver::onCalibration(const es_calibration &cal)
{
    /* the motors of CONTEXT's positions, in CONTEXT's order */
    bool motorsOk = cal.n_axes == axisItem_.size();
    for (unsigned a = 0; motorsOk && a < cal.n_axes; ++a)
        motorsOk = cal.axes[a].motor == layout_.axisMotor[a];
    if (!motorsOk) {
        ++c_.calibrationMotors;
        fault(FAULT_CALIBRATION_MOTORS);
        return OUTCOME_FAULTED;
    }
    for (unsigned i = 0; i < MAX_GENERATIONS; ++i)
        if (slots_[i].used && slots_[i].cal.generation == cal.generation) {
            if (calibrationEqual(slots_[i].cal, cal))
                return OUTCOME_ACCEPTED;
            ++c_.calibrationRewritten;
            fault(FAULT_CALIBRATION_REWRITTEN);
            return OUTCOME_FAULTED;
        }
    storeGeneration(cal);
    if (!ready_ && keptGeneration(cal.generation)) {
        ready_ = true;
        armDataIfDue();                       /* not after a fault or a final (10.3) */
    }
    return OUTCOME_ACCEPTED;
}

Outcome Receiver::onStatus(const es_status &st)
{
    Outcome out = OUTCOME_ACCEPTED;
    hasStatus_ = true;
    lastStatus_ = st;
    if (st.state != ES_STATE_STREAMING) {     /* (a) */
        ++c_.faultStatus;
        fault(FAULT_BRICK_STATUS, st.state, st.fault);
        out = OUTCOME_FAULTED;
    }
    if (st.flags & ES_STATUS_FINAL) {         /* (b) */
        if (!hasFinal_) {
            hasFinal_ = true;
            final_ = st;
            if (anchored_ && st.next_row < cursor_) {
                ++c_.finalBehindCursor;
                fault(FAULT_FINAL_BEHIND_CURSOR);
                out = OUTCOME_FAULTED;
            }
            StreamEvent e = blankEvent(EVENT_STREAM_ENDED, id_);
            e.finalNextRow = st.next_row;
            e.finalRowsSent = st.rows_sent;
            emit(e);
            disarmData(STALL_END_FINAL);
            if (anchored_ && cursor_ == final_.next_row && !tailComplete_) {
                tailComplete_ = true;
                StreamEvent t = blankEvent(EVENT_TAIL_COMPLETE, id_);
                emit(t);
            }
        } else if (!statusEqual(st, final_)) {
            ++c_.finalRewritten;
            fault(FAULT_FINAL_REWRITTEN);
            out = OUTCOME_FAULTED;
        }
    }
    return out;
}

/* ---- DATA (7) --------------------------------------------------------------------------------- */

Outcome Receiver::onData(const es_data &d)
{
    /* 7.1: packet checks, nothing changes on failure */
    if (fault_ != FAULT_NONE) {
        ++c_.dataAfterFault;
        return OUTCOME_DATA_DROPPED;
    }
    if (!ready_) {
        ++c_.dataBeforeCalibration;
        return OUTCOME_DATA_DROPPED;
    }
    if (d.row_bytes != ctx_.row_bytes) {
        ++c_.rejectedRowSize;
        return OUTCOME_DATA_DROPPED;
    }
    if (!keptGeneration(d.cal_generation)) {
        ++c_.unknownGeneration;
        unknownGen_ = true;                   /* metadata with the next SUBSCRIBE (10.1) */
        return OUTCOME_DATA_DROPPED;
    }
    if (anchored_ && d.first_row < cursor_) {
        ++c_.lateOrDuplicate;
        return OUTCOME_DATA_DROPPED;
    }
    if (hasFinal_ && d.first_row + d.n_rows > final_.next_row) {
        ++c_.dataBeyondFinal;
        fault(FAULT_DATA_BEYOND_FINAL);
        return OUTCOME_FAULTED;
    }

    /* 7.2: decode and check every row into scratch; k = the good prefix */
    FaultReason why = FAULT_NONE;
    unsigned k = decodeRows(d, &why);
    if (k > 0)
        commit(d, k);
    if (k == d.n_rows)
        return OUTCOME_DATA_ADMITTED;
    if (why == FAULT_CLOCK_OUT_OF_RANGE)
        ++c_.clockOutOfRange;
    else
        ++c_.clockDiscontinuities;
    if (k > 0)
        ++c_.partialPackets;
    fault(why);
    return k > 0 ? OUTCOME_DATA_PARTIAL : OUTCOME_FAULTED;
}

/* Decodes rows into scratch (8.1) and checks them (7.3). Returns the good-prefix length. */
unsigned Receiver::decodeRows(const es_data &d, FaultReason *why)
{
    const uint8_t order = ctx_.byte_order;
    const uint64_t p = ctx_.gather_period;
    /* the anchor this packet is checked against: the stream's, or its own first row */
    const uint64_t q0 = anchored_ ? q0_ : d.first_row;
    const uint32_t s0 = anchored_ ? s0_ : es_row_word(d.rows, 0, order);
    unsigned k;
    for (k = 0; k < d.n_rows; ++k) {
        const uint8_t *row = d.rows + (size_t)k * ctx_.row_bytes;
        uint32_t s = es_row_word(row, 0, order);
        /* sample clock with overflow checks: S0 + p * (q - q0) */
        uint64_t dq = d.first_row + k - q0;
        if (dq > ~(uint64_t)0 / p) {
            *why = FAULT_CLOCK_OUT_OF_RANGE;
            break;
        }
        uint64_t prod = dq * p;
        if (prod > ~(uint64_t)0 - (uint64_t)s0) {
            *why = FAULT_CLOCK_OUT_OF_RANGE;
            break;
        }
        if ((uint32_t)(s - s0) != (uint32_t)prod) {
            *why = FAULT_CLOCK_DISCONTINUITY;
            break;
        }
        tick_[k] = (uint64_t)s0 + prod;
        servo_[k] = s;
        if (gateItem_ >= 0) {
            const Item &g = items_[(size_t)gateItem_];
            uint32_t w = es_row_word(row, g.word, order);
            gateWord_[k] = w;
            gate_[k] = (uint8_t)((w >> g.gateBit) & 1u);
        }
        for (size_t a = 0; a < axisItem_.size(); ++a) {
            const Item &it = items_[axisItem_[a]];
            uint32_t w[2] = { es_row_word(row, it.word, order), es_row_word(row, it.word + 1, order) };
            double v = 0;
            es_decode_value(it.type, w, order, &v);
            pos_[a * MAX_ROWS + k] = v;
        }
        for (size_t j = 0; j < statusItem_.size(); ++j)
            status_[j * MAX_ROWS + k] = es_row_word(row, items_[statusItem_[j]].word, order);
        for (size_t j = 0; j < otherItem_.size(); ++j) {
            const Item &it = items_[otherItem_[j]];
            uint32_t w[2] = { es_row_word(row, it.word, order), 0 };
            if (es_type_words(it.type) == 2)
                w[1] = es_row_word(row, it.word + 1, order);
            double v = 0;
            es_decode_value(it.type, w, order, &v);
            other_[j * MAX_ROWS + k] = v;
        }
    }
    return k;
}

/* 7.4: commit the first k rows */
void Receiver::commit(const es_data &d, unsigned k)
{
    if (!anchored_) {
        anchored_ = true;
        q0_ = d.first_row;
        s0_ = servo_[0];
        firstRow_ = d.first_row;
    } else if (d.first_row > cursor_) {
        uint64_t gap = d.first_row - cursor_;
        uint64_t skipped = 0;
        if (d.flags & ES_DATA_SKIPPED)
            skipped = gap < d.rows_skipped ? gap : d.rows_skipped;
        GapReport g;
        g.stream = id_;
        g.fromRow = cursor_;
        g.toRow = d.first_row;
        g.fromTick = (uint64_t)s0_ + (cursor_ - q0_) * ctx_.gather_period;   /* <= tick_[0] */
        g.toTick = tick_[0];
        g.skippedByBrick = skipped;
        g.lostInTransport = gap - skipped;
        c_.rowsSkippedByBrick += skipped;
        c_.rowsLost += gap - skipped;
        gapFree_ = false;
        sink_.gap(g);
    }
    cursor_ = d.first_row + k;
    c_.rowsAdmitted += k;
    ++c_.dataPackets;
    lastData_ = now_;
    if (stalled_) {
        stalled_ = false;
        StreamEvent e = blankEvent(EVENT_DATA_STALL_ENDED, id_);
        e.stallEnd = STALL_END_DATA;
        emit(e);
    }

    RowBlock b;
    b.stream = id_;
    b.firstRow = d.first_row;
    b.nRows = k;
    b.calGeneration = d.cal_generation;
    b.tick = &tick_[0];
    b.servoCount = &servo_[0];
    b.gate = gateItem_ >= 0 ? &gate_[0] : 0;
    b.gateWord = gateItem_ >= 0 ? &gateWord_[0] : 0;
    b.nAxes = layout_.nAxes;
    b.axisMotor = layout_.axisMotor;
    b.position = &posPtr_[0];
    b.nStatus = layout_.nStatus;
    b.statusMotor = layout_.statusMotor;
    b.status = &statusPtr_[0];
    b.nOther = layout_.nOther;
    b.otherItem = layout_.otherItem;
    b.other = &otherPtr_[0];
    sink_.rows(b);

    if (hasFinal_ && cursor_ == final_.next_row && !tailComplete_) {
        tailComplete_ = true;
        StreamEvent e = blankEvent(EVENT_TAIL_COMPLETE, id_);
        emit(e);
    }
}

/* ---- views ------------------------------------------------------------------------------------ */

StreamView Receiver::current() const
{
    StreamView v;
    memset(&v, 0, sizeof v);
    v.hasStream = has_;
    if (!has_)
        return v;
    v.stream = id_;
    v.layout = layout_;
    v.ready = ready_;
    v.fault = fault_;
    v.hasFinal = hasFinal_;
    v.final = final_;
    v.tailComplete = tailComplete_;
    v.hasRows = anchored_;
    v.firstRow = firstRow_;
    v.cursor = cursor_;
    v.gapFree = gapFree_;
    v.latestGeneration = latest_;
    v.hasStatus = hasStatus_;
    v.lastStatus = lastStatus_;
    v.hasReport = hasReport_;
    v.lastReport = lastReport_;
    return v;
}

bool Receiver::calibration(uint32_t generation, Calibration *out) const
{
    for (unsigned i = 0; i < MAX_GENERATIONS; ++i)
        if (slots_[i].used && slots_[i].cal.generation == generation) {
            *out = slots_[i].cal;
            return true;
        }
    return false;
}

}
