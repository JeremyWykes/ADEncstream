/* esEngine.h - the receiver thread's work, without sockets or asyn.
 *
 * Owns the core (Receiver, FanOut, Framer, Monitor) and services the command slots, the
 * publication ledger and the acquisition's driver-level state, and posts monitor
 * updates and status snapshots to the Mailbox for the callback thread. The driver's receiver
 * thread only moves datagrams between the socket and the Engine and calls service() on every
 * pass. Uses EPICS libCom (mutexes, events) only; testable without a socket or an IOC.
 */
#ifndef ENCSTREAM_ES_ENGINE_H
#define ENCSTREAM_ES_ENGINE_H

#include <stdint.h>

#include <string>

#include <epicsEvent.h>
#include <epicsMutex.h>

#include "esCommands.h"
#include "esFanOut.h"
#include "esFrameQueue.h"
#include "esFramer.h"
#include "esMonitor.h"
#include "esReceiver.h"

namespace encstream {

enum { SNAPSHOT_EVENTS = 8 };                 /* recent receiver events in a snapshot */

enum AcqDisplay {                             /* AcqState_RBV */
    DISPLAY_IDLE, DISPLAY_ACQUIRING, DISPLAY_ASSEMBLED, DISPLAY_STOPPING, DISPLAY_COMPLETE,
    DISPLAY_INVALID
};
const char *displayName(AcqDisplay d);

/* everything the callback thread publishes from the receiver side, as one owned value */
struct Snapshot {
    uint64_t revision;                        /* increasing; shared sequence with nothing else */
    uint64_t tNs;
    Counters counters;
    StreamView view;
    bool contactLost, dataStalled;
    bool hasCalibration;
    Calibration calibration;                  /* the latest generation */
    AcqStatus acq;
    AcqDisplay display;
    bool active;                              /* the Acquire parameter is held at 1 (4.4) */
    uint64_t epoch;                           /* the acquisition the counts belong to */
    uint64_t cutByStop;
    unsigned queueEntries;
    uint64_t outstandingFrames;
    double framePointsS;                      /* effective Free Running frame time */
    bool monitorRunning;
    MonitorConfig monitorConfig;              /* the accepted configuration (control readbacks) */
    MonitorEffective monitor;
    uint64_t monitorIgnoredMotors;
    char acqMessage[256];
    unsigned nEvents;
    StreamEvent events[SNAPSHOT_EVENTS];      /* oldest first */
};

/* the receiver-to-callback-thread mailbox: the latest monitor update and the latest snapshot.
 * Every post gets a sequence number; the monitor's current run is kept too. The receiver posts
 * every monitor update, health-only changes included, before it reports a control result; the
 * callback thread publishes a taken entry only if, under gate(), it is still the latest posted
 * (and, for an update, of the current run), so nothing superseded is ever published. */
class Mailbox {
public:
    explicit Mailbox(epicsEvent &wake);

    void postMonitor(const MonitorUpdate &u);
    void postSnapshot(const Snapshot &s);

    /* the latest entry and its sequence number; false if none since the last take */
    bool takeMonitor(MonitorUpdate *u, uint64_t *seq = 0);
    bool takeSnapshot(Snapshot *s, uint64_t *seq = 0);

    /* hold while checking a taken entry and setting its parameters */
    epicsMutex &gate() { return lock_; }
    uint64_t currentRun() const;              /* call with gate() held */
    uint64_t monitorSeq() const;              /* the latest posted, call with gate() held */
    uint64_t snapshotSeq() const;
    uint64_t monitorOverwritten() const;
    uint64_t snapshotsOverwritten() const;

private:
    epicsEvent &wake_;
    mutable epicsMutex lock_;
    bool hasMon_, hasSnap_;
    MonitorUpdate mon_;
    Snapshot snap_;
    uint64_t run_, monOver_, snapOver_, monSeq_, snapSeq_;
};

struct EngineConfig {
    ReceiverConfig receiver;
    double snapshotPeriodS;                   /* cumulative counters (1 s) */
    double countsPeriodS;                     /* acquisition counts while they change (0.1 s) */
    EngineConfig() : snapshotPeriodS(1.0), countsPeriodS(0.1) {}
};

class Engine : private RowSink, private MonitorOutput, private CalibrationLookup {
public:
    /* cfg.receiver must pass checkConfig() */
    Engine(const EngineConfig &cfg, FrameQueue &queue, CommandSlots &commands, Mailbox &mailbox);

    /* socket side */
    Outcome datagram(const uint8_t *buf, size_t capturedLen, bool truncated, const Endpoint &src,
                     uint64_t nowNs);
    void timer(uint64_t nowNs);
    uint64_t nextDeadline() const { return rx_.nextDeadline(); }
    bool pendingSend(const uint8_t **buf, size_t *len) { return rx_.pendingSend(buf, len); }
    void sent(bool ok, uint64_t nowNs) { rx_.sent(ok, nowNs); }

    /* commands, publication counts, acquisition state, snapshots; call on every pass */
    void service(uint64_t nowNs);

    const Receiver &receiver() const { return rx_; }
    const Framer &framer() const { return framer_; }
    const Monitor &monitor() const { return monitor_; }
    AcqDisplay display() const { return display_; }
    bool active() const;
    uint64_t revision() const { return revision_; }
    uint64_t lateResults() const { return late_; }

private:
    Engine(const Engine &);
    Engine &operator=(const Engine &);

    /* RowSink: the receiver's output, to the fan-out */
    void gap(const GapReport &g) { fan_.gap(g); }
    void rows(const RowBlock &b) { fan_.rows(b); }
    void event(const StreamEvent &e);
    /* MonitorOutput */
    void publish(const MonitorUpdate &u) { mb_.postMonitor(u); }
    /* CalibrationLookup */
    bool calibration(uint32_t generation, Calibration *out) const { return rx_.calibration(generation, out); }

    CommandResult apply(const Command &c);
    std::string acquire(const FramerConfig &c);
    void stopAcquisition();
    void updateState();
    void post(uint64_t nowNs);

    EngineConfig cfg_;
    FrameQueue &q_;
    CommandSlots &cmds_;
    Mailbox &mb_;
    Framer framer_;
    Monitor monitor_;
    FanOut fan_;
    Receiver rx_;

    AcqDisplay display_;
    bool hold_;                               /* outputs of holdEpoch_ keep Acquire at 1 */
    uint64_t holdEpoch_;
    std::string acqMessage_;
    bool contactLost_, dataStalled_;

    bool dirty_;                              /* a lifecycle change: post now */
    uint64_t revision_;
    uint64_t lastPostNs_, lastCountsNs_;
    uint64_t lastNumAcquired_, lastAssembled_;
    uint64_t late_;
    Snapshot scratch_;
};

}

#endif
