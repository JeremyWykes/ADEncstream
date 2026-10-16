/* esReceiver.h - the ADEncstream receiver core: encstream datagrams in, decoded rows out.
 *
 * The design, and the rules every part of this implements, are in the ADEncstream guide (section
 * numbers below refer to it). Sans-I/O: bytes and time in; sink calls, counters and the
 * SUBSCRIBE to send out. No sockets, clocks, threads, logging or EPICS. Portable C++11; uses the
 * vendored encstream codec (wire.h). Not thread-safe: one thread owns a Receiver (3.2).
 */
#ifndef ENCSTREAM_ES_RECEIVER_H
#define ENCSTREAM_ES_RECEIVER_H

#include <stddef.h>
#include <stdint.h>

#include <string>
#include <vector>

#include "esTrace.h"     /* Endpoint */
#include "esWire.h"

namespace encstream {

enum {
    MAX_DATAGRAM = 1472,                                  /* UDP payload at MTU 1500 */
    MAX_ROWS = (MAX_DATAGRAM - ES_DATA_HDR_LEN) / 4,      /* 358: rows in one DATA, 4-byte rows */
    MAX_GENERATIONS = 8,                                  /* calibration generations kept (6.5) */
    MAX_RETIRED = 8                                       /* retired stream IDs kept (6.1) */
};

struct ReceiverConfig {
    Endpoint brick;                           /* the only accepted source */
    std::string identification;               /* SUBSCRIBE identification, printable, <= 64 */
    uint64_t subscribePeriodNs;               /* heartbeat, <= 2 s (10.1) */
    uint64_t contactTimeoutNs;                /* 10.2 */
    uint64_t dataTimeoutNs;                   /* 10.3 */
    size_t eventLogCapacity;                  /* 9.2 */
    ReceiverConfig();                         /* the defaults of 3: 1 s, 3 s, 3 s, 64 */
};

/* empty if usable, else why not */
std::string checkConfig(const ReceiverConfig &cfg);

enum Outcome {                                /* 12.1 */
    OUTCOME_DROPPED,                          /* stage 1: source, truncation, size */
    OUTCOME_REJECTED,                         /* stage 2: codec, unknown or unexpected type */
    OUTCOME_IGNORED,                          /* stage 3: retired stream, before CONTEXT */
    OUTCOME_ACCEPTED,                         /* metadata handled */
    OUTCOME_DATA_DROPPED,                     /* 7.1 checks 1-4 */
    OUTCOME_DATA_ADMITTED,
    OUTCOME_DATA_PARTIAL,                     /* 7.2: good prefix, then a fault */
    OUTCOME_FAULTED                           /* this datagram caused a fault */
};

enum FaultReason {                            /* 6.4 */
    FAULT_NONE = 0,
    FAULT_BRICK_STATUS,
    FAULT_CONTEXT_CHANGED,
    FAULT_CALIBRATION_MOTORS,
    FAULT_CALIBRATION_REWRITTEN,
    FAULT_FINAL_BEHIND_CURSOR,
    FAULT_FINAL_REWRITTEN,
    FAULT_DATA_BEYOND_FINAL,
    FAULT_CLOCK_DISCONTINUITY,
    FAULT_CLOCK_OUT_OF_RANGE
};
const char *faultName(FaultReason r);

struct Calibration {
    uint32_t generation;
    uint16_t nAxes;
    es_cal_axis axes[ES_MAX_AXES];            /* CONTEXT's position order */
};

struct LayoutSummary {
    uint8_t byteOrder;                        /* enum es_byte_order */
    uint16_t rowBytes;
    uint16_t gatherPeriod;
    double servoPeriodS;
    uint8_t hasGate, gateBit;
    uint8_t nAxes, axisMotor[ES_MAX_AXES];
    uint8_t nStatus, statusMotor[ES_MAX_ITEMS];
    uint8_t nOther, otherItem[ES_MAX_ITEMS];
    char identification[ES_MAX_ID + 1];
};

struct GapReport {                            /* 8.2 */
    uint32_t stream;
    uint64_t fromRow, toRow;                  /* [fromRow, toRow) missing */
    uint64_t fromTick, toTick;
    uint64_t lostInTransport, skippedByBrick;
};

struct RowBlock {                             /* 8.1; every pointer valid during rows() only */
    uint32_t stream;
    uint64_t firstRow;
    uint32_t nRows;
    uint32_t calGeneration;
    const uint64_t *tick;
    const uint32_t *servoCount;
    const uint8_t *gate;                      /* nullptr without a gate item */
    const uint32_t *gateWord;                 /* nullptr without a gate item */
    unsigned nAxes;
    const uint8_t *axisMotor;
    const double *const *position;            /* position[axis][row], raw ActPos */
    unsigned nStatus;
    const uint8_t *statusMotor;
    const uint32_t *const *status;            /* status[k][row], raw words */
    unsigned nOther;
    const uint8_t *otherItem;
    const double *const *other;               /* other[k][row] */
};

enum EventType {                              /* 8.3 */
    EVENT_STREAM_REPLACED,
    EVENT_STREAM_STARTED,
    EVENT_CALIBRATION_GENERATION,
    EVENT_STREAM_FAULTED,
    EVENT_STREAM_ENDED,
    EVENT_TAIL_COMPLETE,
    EVENT_CONTACT_LOST,
    EVENT_CONTACT_RESTORED,
    EVENT_DATA_STALLED,
    EVENT_DATA_STALL_ENDED
};

enum StallEnd { STALL_END_DATA, STALL_END_FAULT, STALL_END_FINAL, STALL_END_REPLACED };

struct StreamEvent {                          /* a self-contained value: no pointers */
    EventType type;
    uint64_t tNs;
    uint32_t stream;                          /* the stream concerned (new one for REPLACED) */
    uint32_t oldStream;                       /* REPLACED */
    LayoutSummary layout;                     /* STARTED */
    Calibration calibration;                  /* CALIBRATION_GENERATION */
    FaultReason fault;                        /* FAULTED */
    uint8_t brickState;                       /* FAULTED by a Brick STATUS */
    uint16_t brickFault;
    uint64_t finalNextRow, finalRowsSent;     /* ENDED */
    StallEnd stallEnd;                        /* DATA_STALL_ENDED */
};

/* implemented by the framer; see the ADEncstream guide Each call must return promptly and never throw.
 * During a call the sink may use the Receiver's read-only getters calibration() and current(),
 * which copy state that does not change while a callback runs; no other Receiver method. */
class RowSink {
public:
    virtual ~RowSink() {}
    virtual void gap(const GapReport &g) = 0;
    virtual void rows(const RowBlock &b) = 0;
    virtual void event(const StreamEvent &e) = 0;
};

struct Counters {                             /* 9.1: cumulative, never reset */
    uint64_t subscribesSent, subscribeSendFailures, subscribesCoalesced;
    uint64_t contactLosses, dataStalls, timeRegressions;
    uint64_t datagrams, foreignSource, truncated, oversize;
    uint64_t decoded[17];                     /* by packet type */
    uint64_t rejected[12];                    /* by -es_result */
    uint64_t unknownType, unexpectedType;
    uint64_t retiredStream, beforeContext, streamChanges, sequenceGaps;
    uint64_t contextChanged, calibrationMotors, calibrationRewritten, faultStatus;
    uint64_t finalBehindCursor, finalRewritten, faults;
    uint64_t dataPackets, rowsAdmitted, rowsLost, rowsSkippedByBrick, lateOrDuplicate;
    uint64_t dataBeforeCalibration, dataAfterFault, rejectedRowSize, unknownGeneration;
    uint64_t dataBeyondFinal, partialPackets, clockDiscontinuities, clockOutOfRange;
    uint64_t eventsLogged, eventsLost;
};

struct StreamView {                           /* 9.3: a copy */
    bool hasStream;
    uint32_t stream;
    LayoutSummary layout;
    bool ready;
    FaultReason fault;
    bool hasFinal;
    es_status final;
    bool tailComplete;
    bool hasRows;                             /* firstRow and cursor are valid */
    uint64_t firstRow, cursor;
    bool gapFree;
    uint32_t latestGeneration;                /* valid when ready */
    bool hasStatus;
    es_status lastStatus;
    bool hasReport;
    es_sender_report lastReport;
};

class Receiver {
public:
    /* cfg must pass checkConfig(); all allocation happens here */
    Receiver(const ReceiverConfig &cfg, RowSink &sink);

    Outcome onDatagram(const uint8_t *buf, size_t capturedLen, bool truncated,
                       const Endpoint &src, uint64_t nowNs);
    void onTimer(uint64_t nowNs);
    uint64_t nextDeadline() const;
    bool pendingSend(const uint8_t **buf, size_t *len);
    void sent(bool ok, uint64_t nowNs);

    const Counters &counters() const { return c_; }
    StreamView current() const;
    bool calibration(uint32_t generation, Calibration *out) const;
    size_t recentEvents(StreamEvent *out, size_t max) const;

private:
    Receiver(const Receiver &);
    Receiver &operator=(const Receiver &);

    struct Item { unsigned word; uint16_t type; uint8_t role, axis, gateBit; };
    struct Slot { bool used; Calibration cal; };

    void service(uint64_t now);
    void prepareSubscribe();
    void emit(StreamEvent &e);
    StreamEvent blankEvent(EventType t, uint32_t stream);
    void fault(FaultReason r, uint8_t brickState = 0, uint16_t brickFault = 0);
    void disarmData(StallEnd why);
    void armDataIfDue();
    void refreshContact();

    Outcome newStream(const es_packet &p);
    Outcome onContext(const es_packet &p);
    Outcome onCalibration(const es_calibration &cal);
    Outcome onStatus(const es_status &st);
    Outcome onData(const es_data &d);
    unsigned decodeRows(const es_data &d, FaultReason *why);
    void commit(const es_data &d, unsigned k);
    bool keptGeneration(uint32_t g) const;
    void storeGeneration(const es_calibration &cal);

    ReceiverConfig cfg_;
    RowSink &sink_;
    Counters c_;

    /* timers (10) */
    uint64_t now_;
    bool started_;
    uint64_t nextSub_;
    std::vector<uint8_t> sendBuf_;
    size_t sendLen_;
    bool pending_;
    uint32_t subSeq_;
    bool unknownGen_;
    bool contactArmed_, contactLost_;
    uint64_t lastContact_;
    bool dataArmed_, stalled_;
    uint64_t lastData_;

    /* the current stream (6) */
    bool has_;
    uint32_t id_;
    es_context ctx_;
    LayoutSummary layout_;
    std::vector<Item> items_;
    int gateItem_;
    std::vector<unsigned> axisItem_, statusItem_, otherItem_;
    bool ready_;
    FaultReason fault_;
    bool hasFinal_;
    es_status final_;
    bool tailComplete_;
    bool hasStatus_;
    es_status lastStatus_;
    bool hasReport_;
    es_sender_report lastReport_;
    Slot slots_[MAX_GENERATIONS];
    uint32_t latest_;
    bool seqValid_;
    uint32_t lastSeq_;
    bool anchored_;
    uint64_t q0_, cursor_, firstRow_;
    uint32_t s0_;
    bool gapFree_;
    uint32_t retired_[MAX_RETIRED];
    unsigned nRetired_, retiredNext_;

    /* scratch for one DATA (8.1), allocated once */
    std::vector<uint64_t> tick_;
    std::vector<uint32_t> servo_, gateWord_;
    std::vector<uint8_t> gate_;
    std::vector<double> pos_, other_;
    std::vector<uint32_t> status_;
    std::vector<const double *> posPtr_, otherPtr_;
    std::vector<const uint32_t *> statusPtr_;

    /* event log (9.2) */
    std::vector<StreamEvent> log_;
    size_t logStart_, logCount_;
};

}

#endif
