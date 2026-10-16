/* ADEncstream.h - the areaDetector driver for encstream.
 *
 * The EPICS layer around the EPICS-free core: an asynNDArrayDriver with a receiver thread (the
 * socket and the Engine, never the driver lock) and a callback thread (NDArrays, parameters).
 * Configured with ADEncstreamConfig().
 */
#ifndef ADENCSTREAM_H
#define ADENCSTREAM_H

#include <set>
#include <string>

#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsThread.h>
#include <epicsTimer.h>
#include <osiSock.h>

#include "asynNDArrayDriver.h"

#include "esCommands.h"
#include "esEngine.h"
#include "esFrameQueue.h"

/* asyn parameter names */
#define ES_BrickAddressString      "ES_BRICK_ADDRESS"
#define ES_LocalPortString         "ES_LOCAL_PORT"
#define ES_SocketRcvBufString      "ES_SOCKET_RCVBUF"
#define ES_StreamIDString          "ES_STREAM_ID"
#define ES_StreamStateString       "ES_STREAM_STATE"
#define ES_StreamFaultString       "ES_STREAM_FAULT"
#define ES_ContactString           "ES_CONTACT"
#define ES_DataStalledString       "ES_DATA_STALLED"
#define ES_IdentificationString    "ES_IDENTIFICATION"
#define ES_ServoPeriodString       "ES_SERVO_PERIOD"
#define ES_GatherPeriodString      "ES_GATHER_PERIOD"
#define ES_NumAxesString           "ES_NUM_AXES"
#define ES_MotorsString            "ES_MOTORS"
#define ES_CalGenerationString     "ES_CAL_GENERATION"
#define ES_RowsAdmittedString      "ES_ROWS_ADMITTED"
#define ES_RowsLostString          "ES_ROWS_LOST"
#define ES_RowsSkippedString       "ES_ROWS_SKIPPED"
#define ES_LateOrDuplicateString   "ES_LATE_OR_DUPLICATE"
#define ES_SequenceGapsString      "ES_SEQUENCE_GAPS"
#define ES_RejectedString          "ES_REJECTED"
#define ES_StreamChangesString     "ES_STREAM_CHANGES"
#define ES_FaultsString            "ES_FAULTS"
#define ES_SubscribesSentString    "ES_SUBSCRIBES_SENT"
#define ES_FinalNextRowString      "ES_FINAL_NEXT_ROW"
#define ES_CursorString            "ES_CURSOR"
#define ES_GapFreeString           "ES_GAP_FREE"
#define ES_TailCompleteString      "ES_TAIL_COMPLETE"
#define ES_RecentEventsString      "ES_RECENT_EVENTS"
#define ES_DriverFaultString       "ES_DRIVER_FAULT"
#define ES_DriverFaultMsgString    "ES_DRIVER_FAULT_MSG"
/* resource use reported by encstream on the Brick, from its STATUS packet */
#define ES_BrickWorstLoopString    "ES_BRICK_WORST_LOOP"
#define ES_BrickBacklogString      "ES_BRICK_BACKLOG"
#define ES_BrickOverrunsString     "ES_BRICK_OVERRUNS"
#define ES_BrickSendFailuresString "ES_BRICK_SEND_FAILURES"
#define ES_BrickVmRSSString        "ES_BRICK_VMRSS"
#define ES_BrickOpenFdsString      "ES_BRICK_OPEN_FDS"

#define ES_AcquireModeString       "ES_ACQUIRE_MODE"
#define ES_NumAcquireString        "ES_NUM_ACQUIRE"
#define ES_NumAcquiredString       "ES_NUM_ACQUIRED"
#define ES_TriggerModeString       "ES_TRIGGER_MODE"
#define ES_FramePointsString       "ES_FRAME_POINTS"
#define ES_FrameTimeString         "ES_FRAME_TIME"
#define ES_FramePointsRbvString    "ES_FRAME_POINTS_RBV"
#define ES_FrameTimeRbvString      "ES_FRAME_TIME_RBV"
#define ES_MaxFrameSamplesString   "ES_MAX_FRAME_SAMPLES"
#define ES_MinCoverageString       "ES_MIN_COVERAGE"
#define ES_EnableSampleIndexString "ES_ENABLE_SAMPLE_INDEX"
#define ES_EnableStatusString      "ES_ENABLE_STATUS"
#define ES_AcqStateString          "ES_ACQ_STATE"
#define ES_AcqMessageString        "ES_ACQ_MESSAGE"
#define ES_NotDeliveredString      "ES_NOT_DELIVERED"
#define ES_CutByStopString         "ES_CUT_BY_STOP"
#define ES_SourceStoppedString     "ES_SOURCE_STOPPED"
#define ES_SourceStopRowString     "ES_SOURCE_STOP_ROW"
#define ES_FramesAssembledString   "ES_FRAMES_ASSEMBLED"
#define ES_FramesWithGapsString    "ES_FRAMES_WITH_GAPS"
#define ES_EmptyFramesString       "ES_EMPTY_FRAMES"
#define ES_OverlongFramesString    "ES_OVERLONG_FRAMES"
#define ES_CalChangedFramesString  "ES_CAL_CHANGED_FRAMES"
#define ES_MinCoverageSeenString   "ES_MIN_COVERAGE_SEEN"
#define ES_DegradedString          "ES_DEGRADED"
#define ES_QueueEntriesString      "ES_QUEUE_ENTRIES"
#define ES_OutstandingFramesString "ES_OUTSTANDING_FRAMES"

#define ES_MonRunString            "ES_MON_RUN"
#define ES_MonAveragingTimeString  "ES_MON_AVERAGING_TIME"
#define ES_MonUpdatePeriodString   "ES_MON_UPDATE_PERIOD"
#define ES_MonAveragingTimeRbvString "ES_MON_AVERAGING_TIME_RBV"
#define ES_MonUpdatePeriodRbvString  "ES_MON_UPDATE_PERIOD_RBV"
#define ES_MonMinCoverageString    "ES_MON_MIN_COVERAGE"
#define ES_MonHealthString         "ES_MON_HEALTH"
#define ES_MonSamplesString        "ES_MON_SAMPLES"
#define ES_MonExpectedString       "ES_MON_EXPECTED"
#define ES_MonCoverageString       "ES_MON_COVERAGE"
#define ES_MonCalChangedString     "ES_MON_CAL_CHANGED"
#define ES_MonUpdatesSkippedString "ES_MON_UPDATES_SKIPPED"
#define ES_MonGateString           "ES_MON_GATE"
#define ES_MonGateEdgesString      "ES_MON_GATE_EDGES"
#define ES_MonGateActiveString     "ES_MON_GATE_ACTIVE"

/* per motor, address n-1 */
#define ES_MeanString              "ES_MEAN"
#define ES_StdDevString            "ES_STDDEV"
#define ES_MinString               "ES_MIN"
#define ES_MaxString               "ES_MAX"
#define ES_StatusOrString          "ES_STATUS_OR"
#define ES_HomePosString           "ES_HOME_POS"
#define ES_PosSfString             "ES_POS_SF"
#define ES_PresentString           "ES_PRESENT"

/* NDArray addresses */
enum {
    ES_ADDR_MAIN = 8, ES_ADDR_INDEX = 9, ES_ADDR_STATUS = 10, ES_NUM_ADDR = 11
};

/* an NDArrayPool that knows the bytes held by arrays in use */
class esNDArrayPool : public NDArrayPool {
public:
    esNDArrayPool(asynNDArrayDriver *driver, size_t maxMemory);
    size_t inUse() const;
protected:
    virtual void onAllocateArray(NDArray *pArray);
    virtual void onReleaseArray(NDArray *pArray);
private:
    mutable epicsMutex lock_;
    size_t inUse_;
};

struct ADEncstreamArgs {
    std::string portName, brick, bindAddress, identification;
    int localPort;
    unsigned maxFrameSamples, queueDepth;
    size_t maxMemory;
    int priority, stackSize;
};

class ADEncstream : public asynNDArrayDriver {
public:
    /* staged start-up; returns 0 or -1 with the reason printed */
    static int configure(const ADEncstreamArgs &args);
    /* how the driver creates its two threads (a test seam: tests make one creation fail) */
    typedef epicsThreadId (*ThreadCreate)(const char *, unsigned, unsigned, EPICSTHREADFUNC, void *);
    static ThreadCreate createThread;
    /* a test seam: init() fails at this stage (1 events, 2 pool, 3 parameters); 0 never */
    static int failInitStage;

    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeInt64(asynUser *pasynUser, epicsInt64 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual void report(FILE *fp, int details);

    void receiverTask();
    void callbackTask();
    void receiverLoop();
    void callbackLoop();
    void shutdown();

private:
    friend class ADEncstreamTestAccess;       /* the driver-level tests (test/testDriver.cpp) */
    struct Parts;
    ADEncstream(const ADEncstreamArgs &args, Parts *parts);
    bool init(const ADEncstreamArgs &args, const std::string &brickText, std::string *why);
    void releaseSocket();
    void createEsParams();
    bool start(std::string *why);             /* the two threads, callback thread first */
    void failStartup(const std::string &why);
    bool stopping() const;
    bool faulted() const;

    asynStatus command(encstream::Command &c, std::string *why, uint64_t *revision);
    asynStatus startAcquisition();
    asynStatus refuse(asynUser *pasynUser, const std::string &why);
    void setAcqMessage(const std::string &m);
    void revertReadback(int function);
    void driverFault(const std::string &why);
    void applyFault(const std::string &why);  /* the terminal fault transition (callback thread) */

    /* callback thread; seq: the mailbox sequence number of the taken entry */
    void publishSnapshot(const encstream::Snapshot &s, uint64_t seq);
    void publishMonitor(const encstream::MonitorUpdate &u, uint64_t seq);
    void publishWork(const encstream::Work &w);
    bool publishFrame(const encstream::Frame *f, const encstream::EmptyRun *run, uint64_t index);
    void serviceBusy();
    /* a refused Acquire = 1: the busy-record handshake, on a timer so that it runs whatever the
     * driver's threads are doing (fault, start-up failure, dead worker) */
    void armRefusal();
    void refusalStep();
    static void refusalTimerC(void *arg);

    Parts *p_;
    esNDArrayPool *pool_;
    size_t maxMemory_;
    unsigned capacity_;
    int exiting_;                             /* epicsAtomic */
    int shutdownDone_;                        /* epicsAtomic */
    epicsEvent *rxDone_, *cbDone_;            /* created in init() */
    bool rxStarted_, cbStarted_;
    bool paramsReady_;                        /* init() created every parameter */
    int priority_;
    unsigned stackSize_;
    int uniqueId_;
    std::set<std::string> warned_;
    NDAttributeList *scratch_;

    /* Acquire/Busy bookkeeping, callback thread and port thread, driver lock */
    uint64_t acquireRevision_;                /* snapshots older than this cannot release Acquire */
    uint64_t controlRevision_;                /* the newest Acquire/Stop result or release seen */
    uint64_t monitorWriteRevision_;           /* the newest successful monitor control write */
    bool faultApplied_;
    double acquireSetAt_;                     /* monotonic seconds when Acquire was set to 1 */
    int refusedBusy_;                         /* a refused Acquire: toggle the busy readback (0, 1, 2) */
    epicsTimerQueueId refusalQueue_;          /* EPICS's shared timer queue, taken at the first refusal */
    epicsTimerId refusalTimer_;
    bool snapActive_;                         /* the latest snapshot's view of the acquisition */
    uint64_t snapRevision_;
    std::string lastEngineMessage_;
    uint64_t staleMonitor_, publishedFrames_, failedFrames_;

    int ES_BrickAddress;
#define FIRST_ES_PARAM ES_BrickAddress
    int ES_LocalPort, ES_SocketRcvBuf, ES_StreamID, ES_StreamState, ES_StreamFault, ES_Contact,
        ES_DataStalled, ES_Identification, ES_ServoPeriod, ES_GatherPeriod, ES_NumAxes, ES_Motors,
        ES_CalGeneration, ES_RowsAdmitted, ES_RowsLost, ES_RowsSkipped, ES_LateOrDuplicate,
        ES_SequenceGaps, ES_Rejected, ES_StreamChanges, ES_Faults, ES_SubscribesSent,
        ES_FinalNextRow, ES_Cursor, ES_GapFree, ES_TailComplete, ES_RecentEvents, ES_DriverFault,
        ES_DriverFaultMsg;
    int ES_BrickWorstLoop, ES_BrickBacklog, ES_BrickOverruns, ES_BrickSendFailures,
        ES_BrickVmRSS, ES_BrickOpenFds;
    int ES_AcquireMode, ES_NumAcquire, ES_NumAcquired, ES_TriggerMode, ES_FramePoints, ES_FrameTime,
        ES_FramePointsRbv, ES_FrameTimeRbv, ES_MaxFrameSamples, ES_MinCoverage, ES_EnableSampleIndex,
        ES_EnableStatus, ES_AcqState, ES_AcqMessage, ES_NotDelivered, ES_CutByStop, ES_SourceStopped,
        ES_SourceStopRow, ES_FramesAssembled, ES_FramesWithGaps, ES_EmptyFrames, ES_OverlongFrames,
        ES_CalChangedFrames, ES_MinCoverageSeen, ES_Degraded, ES_QueueEntries, ES_OutstandingFrames;
    int ES_MonRun, ES_MonAveragingTime, ES_MonUpdatePeriod, ES_MonAveragingTimeRbv,
        ES_MonUpdatePeriodRbv, ES_MonMinCoverage, ES_MonHealth, ES_MonSamples, ES_MonExpected,
        ES_MonCoverage, ES_MonCalChanged, ES_MonUpdatesSkipped,
        ES_MonGate, ES_MonGateEdges, ES_MonGateActive;
    int ES_Mean, ES_StdDev, ES_Min, ES_Max, ES_StatusOr, ES_HomePos, ES_PosSf, ES_Present;
};

#endif
