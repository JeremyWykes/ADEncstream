/* esMonitor.h - the continuous monitor.
 *
 * Averages every axis over an averaging time T, updated every U (T a whole multiple of U; U = T
 * back-to-back by default, U < T sliding), on the sample clock, and publishes per-axis mean,
 * standard deviation, min, max and status OR through a MonitorOutput, plus the gate over the window
 * (level, edges and active fraction). Independent of acquisition framing. A RowSink: the fan-out
 * forwards the receiver's calls to it. Sans-I/O, portable
 * C++11, all storage allocated in the constructor; not thread-safe (receiver thread only).
 */
#ifndef ENCSTREAM_ES_MONITOR_H
#define ENCSTREAM_ES_MONITOR_H

#include <stdint.h>

#include <string>
#include <vector>

#include "esReceiver.h"
#include "esWindow.h"

namespace encstream {

enum {
    MONITOR_SLOTS = 8,             /* slot n-1 is Motor[n] */
    MONITOR_MAX_SUBBLOCKS = 1000   /* T/U at most */
};

const double MONITOR_MIN_UPDATE_S = 0.010;    /* U >= 10 ms */

/* calibration of a generation, as Receiver::calibration() gives it (read-only) */
class CalibrationLookup {
public:
    virtual ~CalibrationLookup() {}
    virtual bool calibration(uint32_t generation, Calibration *out) const = 0;
};

enum Severity { SEVERITY_NONE = 0, SEVERITY_MINOR = 1, SEVERITY_MAJOR = 2, SEVERITY_INVALID = 3 };

enum MonitorHealth {                        /* the ADEncstream guide */
    HEALTH_STOPPED,                         /* the monitor is not running */
    HEALTH_NO_STREAM,
    HEALTH_WAITING,                         /* running, no update completed yet */
    HEALTH_OK,
    HEALTH_STALLED,                         /* DATA watchdog; clears with the next update */
    HEALTH_SOURCE_STOPPED,                  /* final STATUS: latched for the stream */
    HEALTH_FAULTED,                         /* stream fault: latched for the stream */
    HEALTH_REPLACED,                        /* the stream was replaced; waiting for the next one */
    HEALTH_UNSUPPORTED                      /* T/U not representable at this stream's row rate */
};
const char *healthName(MonitorHealth h);

struct MonitorSlot {
    bool present;                           /* Motor[n] is in the layout */
    bool hasStatus;                         /* a status word for Motor[n] is in the layout */
    double mean, stddev, min, max;          /* motor units relative to HomePos */
    uint32_t statusOr;
    Severity severity;
};

struct MonitorUpdate {                      /* an owned value, handed to the output */
    uint64_t run;                           /* incremented by every Start, Stop, restart, replacement */
    uint32_t stream;
    MonitorHealth health;
    bool values;                            /* false: a state change without a completed update */
    uint64_t samples, expected;             /* rows in the window; T in rows */
    double coverage;
    bool calibrationChanged;
    uint64_t updatesSkipped;                /* cumulative (coalesced updates) */
    bool hasGate;                           /* the layout has a gate item */
    uint8_t gateLevel;                      /* raw gate at the last sample in the window (0/1) */
    uint32_t gateEdges;                     /* gate transitions across the window */
    double gateActiveFraction;              /* fraction of window samples with the gate high */
    Severity gateSeverity;                  /* INVALID without a gate item or rows */
    MonitorSlot slot[MONITOR_SLOTS];
};

class MonitorOutput {
public:
    virtual ~MonitorOutput() {}
    virtual void publish(const MonitorUpdate &u) = 0;   /* returns promptly, never throws */
};

struct MonitorConfig {
    double averagingTimeS;                  /* T */
    double updatePeriodS;                   /* U; 0 means U = T */
    double minCoverage;                     /* 0..1 */
    MonitorConfig() : averagingTimeS(1.0), updatePeriodS(0.0), minCoverage(0.9) {}
};

/* empty if usable, else why not (validates T, U, T/U <= 1000, minCoverage) */
std::string checkMonitorConfig(const MonitorConfig &c);

struct MonitorEffective {                   /* read back after rounding to rows (5.1) */
    bool known;                             /* a stream layout is known */
    uint64_t rowsPerUpdate;                 /* U in rows */
    unsigned subBlocks;                     /* T/U */
    double updatePeriodS, averagingTimeS;
};

class Monitor : public RowSink {
public:
    Monitor(MonitorOutput &out, const CalibrationLookup &cal);

    /* control (receiver thread; `now` is the receiver's current stream view) */
    std::string configure(const MonitorConfig &c, const StreamView &now);
    void start(const StreamView &now);
    void stop();
    bool running() const { return running_; }
    MonitorEffective effective() const;
    const MonitorConfig &config() const { return cfg_; }   /* the last accepted configuration */
    uint64_t ignoredMotors() const { return ignoredMotors_; }   /* positions of the current layout outside Motor[1..8] */
    uint64_t updatesPublished() const { return updatesPublished_; }

    /* RowSink */
    void gap(const GapReport &g);
    void rows(const RowBlock &b);
    void event(const StreamEvent &e);

private:
    struct Acc {                            /* one axis in one sub-block */
        uint64_t n;
        double mean, m2, min, max;
        uint32_t statusOr;
    };
    struct Sub {
        bool used;
        uint64_t index;                     /* sub-block index since the window origin */
        uint64_t rows;
        uint32_t gen;
        bool mixedGen;
        Acc acc[MONITOR_SLOTS];
        bool haveGate;                      /* a gate sample seen in this sub-block */
        uint8_t firstGate, lastGate;
        uint64_t high;                      /* rows with the gate high */
        uint32_t internalEdges;             /* gate transitions within this sub-block */
    };

    void restart(const StreamView *now);    /* clear history; health from facts */
    void setLayout(const LayoutSummary &l, uint32_t stream);
    bool computeEffective();
    void resetSub(Sub &s, uint64_t index);
    void advanceTo(const WindowStep &st);
    void computeAndPublish();
    void publishState();
    Severity healthSeverity() const;

    MonitorOutput &out_;
    const CalibrationLookup &cal_;
    MonitorConfig cfg_;
    bool running_;
    uint64_t run_;
    MonitorHealth health_;

    /* the stream */
    bool hasLayout_;
    uint32_t stream_;
    LayoutSummary layout_;
    int axisColumn_[MONITOR_SLOTS];         /* RowBlock position column of Motor[n], or -1 */
    int statusColumn_[MONITOR_SLOTS];       /* RowBlock status column of Motor[n], or -1 */
    MonitorEffective eff_;

    /* windows */
    Windows win_;
    std::vector<Sub> ring_;                 /* MONITOR_MAX_SUBBLOCKS entries, allocated once */
    bool havePending_;
    uint64_t lastCompleted_;                /* the newest completed sub-block */
    uint64_t completedThisCall_;
    uint64_t updatesSkipped_, updatesPublished_, ignoredMotors_;
    MonitorUpdate last_;                    /* the last published values */
};

}

#endif
