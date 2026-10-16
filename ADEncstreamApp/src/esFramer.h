/* esFramer.h - the acquisition framer.
 *
 * Turns the receiver's row stream into numbered frames per trigger mode, from Acquire to the end
 * of the acquisition, and hands them to a FrameOutput (the driver's pool, queue and callback
 * thread, 4.9). A RowSink: the fan-out forwards the receiver's calls to it. Sans-I/O, portable
 * C++11, no allocation after construction except what the FrameOutput does; not thread-safe
 * (receiver thread only).
 */
#ifndef ENCSTREAM_ES_FRAMER_H
#define ENCSTREAM_ES_FRAMER_H

#include <stdint.h>

#include <string>
#include <vector>

#include "esMonitor.h"      /* CalibrationLookup */
#include "esReceiver.h"
#include "esWindow.h"

namespace encstream {

enum { FRAMER_MAX_AXES = 8, FRAMER_MAX_STATUS = 1 + FRAMER_MAX_AXES };   /* gate word + BiSS words */

enum TriggerMode {
    TRIGGER_FREE_RUNNING, TRIGGER_GATE_HIGH, TRIGGER_GATE_LOW, TRIGGER_RISING, TRIGGER_FALLING, TRIGGER_BOTH
};
enum AcquireMode { ACQUIRE_CONTINUOUS, ACQUIRE_MULTIPLE, ACQUIRE_SINGLE };

struct FramerConfig {                         /* latched at Acquire (4.1) */
    TriggerMode trigger;
    AcquireMode mode;
    uint64_t numAcquire;                      /* N (Multiple) */
    uint64_t framePoints;                     /* Free Running, in row slots; 0: use frameTimeS */
    double frameTimeS;                        /* Free Running, rounded to slots (half-way up) */
    uint32_t maxFrameSamples;
    double minCoverage;
    bool withSampleIndex, withStatus;         /* optional arrays */
    FramerConfig();
};

/* Everything needed to interpret and publish an acquisition's outputs, fixed at Acquire and
 * copied into every Frame and EmptyRun, so a queued output keeps its meaning across Stop, re-arm
 * and stream replacement. Empty frames (Free Running): positions are one NaN per
 * axis, the sample index one 0, each status column one 0; the Empty flag marks them. */
struct OutputSchema {
    TriggerMode trigger;
    unsigned nAxes;
    uint8_t motor[FRAMER_MAX_AXES];           /* position columns: Motor[motor[a]] */
    bool hasGate;
    uint8_t gateBit;
    bool withSampleIndex;                     /* address 9: the tick, u64 */
    bool withStatus;                          /* address 10: status columns, u32 */
    unsigned nStatus;                         /* status columns when withStatus */
    uint8_t statusMotor[FRAMER_MAX_STATUS];   /* each column's motor; 0 for the gate word (first) */
    double servoPeriodS;
    uint16_t gatherPeriod;
    uint64_t framePoints;                     /* Free Running */
    uint32_t maxFrameSamples;
    Calibration frozen;                       /* HomePos, PosSf at Acquire */
};

/* One frame: storage owned by the FrameOutput's pool, filled by the framer (4.8). The pool must
 * size every object for the configuration: capacity >= MaxFrameSamples, position >= 8 x capacity,
 * tick >= capacity (sample index), status >= 9 x capacity (status columns). openFrame() checks
 * the object against the acquisition and invalidates rather than write past it. */
struct Frame {
    /* storage, sized by the pool */
    uint32_t capacity;                        /* samples */
    std::vector<double> position;             /* [axis * capacity + sample] */
    std::vector<uint64_t> tick;               /* if withSampleIndex */
    std::vector<uint32_t> status;             /* [k * capacity + sample] if withStatus */

    /* identity */
    uint64_t epoch;
    uint32_t stream;
    uint64_t number;                          /* FrameNumber, 1-based since Acquire */
    OutputSchema schema;
    /* contents */
    uint32_t samples;
    uint64_t firstTick, lastTick;
    uint32_t firstServo, lastServo;
    uint64_t windowStart, windowEnd;          /* Free Running: [start, end) ticks */
    uint64_t expected, missing;
    double coverage;
    bool startEdgeInGap, endEdgeInGap;
    uint64_t startGapRows, endGapRows;
    bool overlong, calibrationChanged, empty;
    uint32_t statusOr;                        /* BiSS words */
    uint8_t gateAtStart, gateAtEnd;
    uint32_t gateEdges, gateActive;           /* edges inside the frame; samples with the gate 1 */
};

/* A run of empty Free Running frames (4.9.1): owned contents, no frame object. */
struct EmptyRun {
    uint64_t epoch;
    uint32_t stream;
    uint64_t firstNumber, count;
    uint64_t firstWindowStart, windowLength;  /* ticks */
    OutputSchema schema;
};

/* The driver's pool, queue and callback thread (4.9). All calls return promptly, never throw. */
class FrameOutput {
public:
    virtual ~FrameOutput() {}
    /* a free frame object sized for the configuration, or nullptr (exhaustion) */
    virtual Frame *acquire() = 0;
    /* hand a completed frame to the queue; false if the queue is full (the frame is then the
     * framer's again, to release) */
    virtual bool push(Frame *f) = 0;
    virtual bool pushEmptyRun(const EmptyRun &r) = 0;
    /* give back a frame that will not be pushed */
    virtual void release(Frame *f) = 0;
    /* the acquisition with this epoch stopped: truncate its empty runs (4.9.1) */
    virtual void stopped(uint64_t epoch) = 0;
};

enum AcqState {
    ACQ_IDLE,                                 /* not acquiring */
    ACQ_ACQUIRING,                            /* armed: framing */
    ACQ_ASSEMBLED,                            /* Multiple: all N frames assembled, publishing */
    ACQ_COMPLETE,                             /* Multiple: all N frames published */
    ACQ_INVALID                               /* invalidated (reason) until stopped */
};
enum InvalidReason {
    INVALID_NONE, INVALID_STREAM_REPLACED, INVALID_STREAM_FAULTED, INVALID_NO_FREE_FRAME,
    INVALID_QUEUE_FULL, INVALID_PUBLICATION_FAILED, INVALID_FRAME_STORAGE, INVALID_CLOCK_RANGE
};
const char *invalidReasonText(InvalidReason r);

struct AcqStatus {                            /* 4.10, per acquisition */
    AcqState state;
    InvalidReason invalid;
    uint64_t epoch;
    uint64_t numAcquired;                     /* published */
    uint64_t assembled;                       /* frames handed to the output (incl. empty) */
    uint64_t notDelivered;
    uint64_t withGaps, empty, overlong, calibrationChanged;
    double minCoverage;                       /* lowest coverage seen; NaN if none */
    bool degraded;                            /* a frame below the latched MinCoverage */
    bool sourceStopped;                       /* the stream's final STATUS was seen */
    uint64_t sourceStopRow;                   /* its next row */
    bool tailComplete;
    uint64_t framePoints;                     /* effective, Free Running */
};

class Framer : public RowSink {
public:
    Framer(FrameOutput &out, const CalibrationLookup &cal);

    /* Acquire (4.2): checks the stream's current facts; "" if armed, else why not */
    std::string acquire(const FramerConfig &c, const StreamView &now);
    void stop();
    const AcqStatus &status() const { return st_; }
    /* the driver's report: `frames` of epoch `epoch` published (ok) or failed (4.9) */
    void published(uint64_t epoch, uint64_t frames, bool ok);

    /* RowSink */
    void gap(const GapReport &g);
    void rows(const RowBlock &b);
    void event(const StreamEvent &e);

private:
    void invalidate(InvalidReason r);
    void dropAssembling();
    bool openFrame(uint64_t tick, uint32_t servo, uint8_t gate, bool edgeInGap, uint64_t gapRows);
    void addRow(const RowBlock &b, uint32_t r);
    void closeFrame(bool endEdgeInGap, uint64_t gapRows);
    void capOverlong();
    bool setWindow(uint64_t index);
    void finishCounts(Frame &f);
    bool emitFrame(Frame *f);
    void emitEmptyRun(uint64_t firstIndex, uint64_t count);
    bool wantMore() const;
    bool isActive(uint8_t gate) const;
    bool qualifyingEdge(uint8_t from, uint8_t to) const;

    FrameOutput &out_;
    const CalibrationLookup &cal_;
    FramerConfig cfg_;
    AcqStatus st_;
    uint64_t epochCounter_;

    /* latched at Acquire */
    uint32_t stream_;
    LayoutSummary layout_;
    Calibration frozen_;
    OutputSchema schema_;                     /* fixed at Acquire */
    uint64_t nextNumber_;                     /* the next frame number to assign */

    /* gate tracking */
    bool havePrev_;
    uint8_t prevGate_;
    uint64_t prevRow_;

    /* Free Running */
    Windows win_;
    uint64_t framePoints_;

    Frame *cur_;                              /* the frame being assembled, or nullptr */
};

}

#endif
