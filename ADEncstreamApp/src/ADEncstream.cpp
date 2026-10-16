/* ADEncstream.cpp - the areaDetector driver for encstream. See ADEncstream.h and the ADEncstream guide. */
#include <ctype.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cmath>
#include <limits>
#include <new>
#include <string>

#include <alarm.h>
#include <epicsAtomic.h>
#include <epicsExit.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <iocsh.h>

#include <epicsExport.h>

#include "ADEncstream.h"
#include "ADEncstreamParts.h"

using namespace encstream;

static const char *driverName = "ADEncstream";

namespace {

const double COMMAND_WAIT_S = 0.5;            /* 10 x the service interval */
const uint64_t SERVICE_CAP_NS = 50000000ull;  /* 50 ms */
const double BUSY_GUARD_S = 0.1;              /* see serviceBusy() */
const size_t SMALL_RESERVE = 1u << 20;        /* plugins' small time-series vectors */
const size_t FULL_FRAME_BYTES = 172;          /* per sample slot: main 64 + axes 64 + index 8 + status 36 */

/* the plugin reserve: the small vectors, plus one NDStdArrays conversion per axis
 * (it converts every array into a new one from our pool, freed after its waveform callback),
 * each possibly served by a free buffer up to 1.5 times its size (ADCore NDArrayPool's
 * THRESHOLD_SIZE_RATIO): 8 axes x 8 bytes x 1.5 = 96 bytes a sample */
size_t pluginReserve(unsigned capacity)
{
    return SMALL_RESERVE + (size_t)8 * sizeof(double) * 3 / 2 * capacity;
}
const int RCVBUF_REQUEST = 4 << 20;
const unsigned MAX_CAPACITY = 10000000u;

/* the driver lock for a scope: an exception never leaves it held */
struct DriverLock {
    asynPortDriver *d;
    explicit DriverLock(asynPortDriver *x) : d(x) { d->lock(); }
    ~DriverLock() { d->unlock(); }
};

/* the driver's own references to one frame's NDArrays: released on every exit, an exception
 * included */
struct FrameArrays {
    NDArray *a[ES_NUM_ADDR];
    FrameArrays() { memset(a, 0, sizeof a); }
    ~FrameArrays()
    {
        for (int i = 0; i < ES_NUM_ADDR; ++i)
            if (a[i])
                a[i]->release();
    }
};

/* a taken unit of work, settled with the queue exactly once: by settle(), or on an exception
 * by the destructor, which counts what was not published as failed */
struct WorkGuard {
    FrameQueue &q;
    const Work &w;
    uint64_t total, published;
    bool settled;
    WorkGuard(FrameQueue &queue, const Work &work)
        : q(queue), w(work), total(work.kind == Work::EMPTY ? work.run.count : 1), published(0), settled(false) {}
    void settle()
    {
        settled = true;
        q.done(w, published, total - published);
    }
    ~WorkGuard()
    {
        if (!settled)
            q.done(w, published, total - published);
    }
};

double monotonicS()
{
    return (double)epicsMonotonicGet() * 1e-9;
}

std::string motorList(const uint8_t *m, unsigned n)
{
    std::string s;
    char b[8];
    for (unsigned i = 0; i < n; ++i) {
        sprintf(b, "%s%u", i ? "," : "", (unsigned)m[i]);
        s += b;
    }
    return s;
}

const char *eventName(EventType t)
{
    switch (t) {
    case EVENT_STREAM_REPLACED: return "stream replaced";
    case EVENT_STREAM_STARTED: return "stream started";
    case EVENT_CALIBRATION_GENERATION: return "calibration";
    case EVENT_STREAM_FAULTED: return "stream faulted";
    case EVENT_STREAM_ENDED: return "stream ended";
    case EVENT_TAIL_COMPLETE: return "tail complete";
    case EVENT_CONTACT_LOST: return "contact lost";
    case EVENT_CONTACT_RESTORED: return "contact restored";
    case EVENT_DATA_STALLED: return "data stalled";
    case EVENT_DATA_STALL_ENDED: return "data stall ended";
    }
    return "?";
}

/* the frame attribute names: removed from the site's list before adding */
const char *const reservedNames[] = {
    "TriggerMode", "AcqEpoch", "FrameNumber", "StreamID", "Motors", "StatusMotors", "ServoPeriod",
    "GatherPeriod", "NumSamples", "ExpectedSamples", "MissingRows", "Coverage", "FirstTick",
    "LastTick", "FirstServoCount", "LastServoCount", "WindowStart", "WindowEnd", "StartEdgeInGap",
    "EndEdgeInGap", "StartGapRows", "EndGapRows", "Overlong", "CalibrationChanged", "Empty",
    "StatusOr", "HasGate", "GateAtStart", "GateAtEnd", "GateEdges", "GateActiveFraction",
    "HomePos1", "HomePos2", "HomePos3", "HomePos4", "HomePos5", "HomePos6", "HomePos7", "HomePos8",
    "PosSf1", "PosSf2", "PosSf3", "PosSf4", "PosSf5", "PosSf6", "PosSf7", "PosSf8"
};

void addInt32(NDAttributeList *l, const char *name, const char *desc, epicsInt32 v)
{
    l->add(name, desc, NDAttrInt32, &v);
}
void addUInt32(NDAttributeList *l, const char *name, const char *desc, epicsUInt32 v)
{
    l->add(name, desc, NDAttrUInt32, &v);
}
void addUInt64(NDAttributeList *l, const char *name, const char *desc, epicsUInt64 v)
{
    l->add(name, desc, NDAttrUInt64, &v);
}
void addFloat64(NDAttributeList *l, const char *name, const char *desc, epicsFloat64 v)
{
    l->add(name, desc, NDAttrFloat64, &v);
}
void addString(NDAttributeList *l, const char *name, const char *desc, const std::string &v)
{
    l->add(name, desc, NDAttrString, (void *)v.c_str());
}

/* the values of one frame's attributes, from a frame or one frame of an empty run */
struct FrameValues {
    const OutputSchema *schema;
    uint64_t epoch, number;
    uint32_t stream;
    uint32_t samples;
    uint64_t expected, missing;
    double coverage;
    uint64_t firstTick, lastTick, windowStart, windowEnd;
    uint32_t firstServo, lastServo;
    bool startEdgeInGap, endEdgeInGap;
    uint64_t startGapRows, endGapRows;
    bool overlong, calibrationChanged, empty;
    uint32_t statusOr;
    uint8_t gateAtStart, gateAtEnd;
    uint32_t gateEdges, gateActive;
};

FrameValues valuesOf(const Frame &f)
{
    FrameValues v;
    memset(&v, 0, sizeof v);
    v.schema = &f.schema;
    v.epoch = f.epoch;
    v.number = f.number;
    v.stream = f.stream;
    v.samples = f.samples;
    v.expected = f.expected;
    v.missing = f.missing;
    v.coverage = f.coverage;
    v.firstTick = f.firstTick;
    v.lastTick = f.lastTick;
    if (f.schema.trigger == TRIGGER_FREE_RUNNING) {
        v.windowStart = f.windowStart;
        v.windowEnd = f.windowEnd;
    } else {
        v.windowStart = f.firstTick;
        v.windowEnd = f.lastTick + f.schema.gatherPeriod;
    }
    v.firstServo = f.firstServo;
    v.lastServo = f.lastServo;
    v.startEdgeInGap = f.startEdgeInGap;
    v.endEdgeInGap = f.endEdgeInGap;
    v.startGapRows = f.startGapRows;
    v.endGapRows = f.endGapRows;
    v.overlong = f.overlong;
    v.calibrationChanged = f.calibrationChanged;
    v.empty = false;
    v.statusOr = f.statusOr;
    v.gateAtStart = f.gateAtStart;
    v.gateAtEnd = f.gateAtEnd;
    v.gateEdges = f.gateEdges;
    v.gateActive = f.gateActive;
    return v;
}

FrameValues valuesOf(const EmptyRun &r, uint64_t i)
{
    FrameValues v;
    memset(&v, 0, sizeof v);
    v.schema = &r.schema;
    v.epoch = r.epoch;
    v.number = r.firstNumber + i;
    v.stream = r.stream;
    v.expected = r.schema.framePoints;
    v.missing = r.schema.framePoints;
    v.coverage = 0;
    v.windowStart = r.firstWindowStart + i * r.windowLength;
    v.windowEnd = v.windowStart + r.windowLength;
    v.empty = true;
    return v;
}

void addFrameAttributes(NDAttributeList *l, const FrameValues &v)
{
    const OutputSchema &s = *v.schema;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    addInt32(l, "TriggerMode", "trigger mode", (epicsInt32)s.trigger);
    addUInt64(l, "AcqEpoch", "acquisition epoch", v.epoch);
    addUInt64(l, "FrameNumber", "frame number, 1-based since Acquire", v.number);
    addUInt32(l, "StreamID", "encstream stream ID", v.stream);
    addString(l, "Motors", "position motors present", motorList(s.motor, s.nAxes));
    addString(l, "StatusMotors", "status columns' motors, gate word as 0", motorList(s.statusMotor, s.nStatus));
    addFloat64(l, "ServoPeriod", "servo period, s", s.servoPeriodS);
    addUInt32(l, "GatherPeriod", "servo cycles per row", s.gatherPeriod);
    addUInt32(l, "NumSamples", "samples in the frame", v.samples);
    addUInt64(l, "ExpectedSamples", "expected samples", v.expected);
    addUInt64(l, "MissingRows", "rows missing inside the frame", v.missing);
    addFloat64(l, "Coverage", "NumSamples / ExpectedSamples", v.coverage);
    addUInt64(l, "FirstTick", "first observed sample's tick; 0 if empty", v.firstTick);
    addUInt64(l, "LastTick", "last observed sample's tick; 0 if empty", v.lastTick);
    addUInt32(l, "FirstServoCount", "first observed sample's ServoCount; 0 if empty", v.firstServo);
    addUInt32(l, "LastServoCount", "last observed sample's ServoCount; 0 if empty", v.lastServo);
    addUInt64(l, "WindowStart", "window start tick", v.windowStart);
    addUInt64(l, "WindowEnd", "window end tick (exclusive)", v.windowEnd);
    addInt32(l, "StartEdgeInGap", "start edge in a gap", v.startEdgeInGap);
    addInt32(l, "EndEdgeInGap", "end edge in a gap", v.endEdgeInGap);
    addUInt64(l, "StartGapRows", "gap rows at the start edge", v.startGapRows);
    addUInt64(l, "EndGapRows", "gap rows at the end edge", v.endGapRows);
    addInt32(l, "Overlong", "frame cut at MaxFrameSamples", v.overlong);
    addInt32(l, "CalibrationChanged", "a newer calibration during the frame", v.calibrationChanged);
    addInt32(l, "Empty", "no sample in the window", v.empty);
    addUInt32(l, "StatusOr", "OR of the BiSS status words", v.statusOr);
    addInt32(l, "HasGate", "the layout has a gate item", s.hasGate);
    addInt32(l, "GateAtStart", "gate at the first sample", v.gateAtStart);
    addInt32(l, "GateAtEnd", "gate at the last sample", v.gateAtEnd);
    addUInt32(l, "GateEdges", "gate edges inside the frame", v.gateEdges);
    addFloat64(l, "GateActiveFraction", "fraction of samples with the gate 1",
               s.hasGate && v.samples ? (double)v.gateActive / v.samples : nan);
    for (unsigned m = 1; m <= 8; ++m) {
        double home = nan, sf = nan;
        for (unsigned i = 0; i < s.frozen.nAxes; ++i)
            if (s.frozen.axes[i].motor == m) {
                home = s.frozen.axes[i].home_pos;
                sf = s.frozen.axes[i].pos_sf;
            }
        bool present = false;
        for (unsigned a = 0; a < s.nAxes; ++a)
            present = present || s.motor[a] == m;
        char name[16];
        sprintf(name, "HomePos%u", m);
        addFloat64(l, name, "HomePos frozen at Acquire", present ? home : nan);
        sprintf(name, "PosSf%u", m);
        addFloat64(l, name, "PosSf frozen at Acquire", present ? sf : nan);
    }
}

int severityToStatus(Severity s)
{
    switch (s) {
    case SEVERITY_NONE: return epicsAlarmNone;
    case SEVERITY_MINOR:
    case SEVERITY_MAJOR: return epicsAlarmState;
    case SEVERITY_INVALID: return epicsAlarmUDF;
    }
    return epicsAlarmUDF;
}

int severityToEpics(Severity s)
{
    switch (s) {
    case SEVERITY_NONE: return epicsSevNone;
    case SEVERITY_MINOR: return epicsSevMinor;
    case SEVERITY_MAJOR: return epicsSevMajor;
    case SEVERITY_INVALID: return epicsSevInvalid;
    }
    return epicsSevInvalid;
}

epicsInt64 sat(uint64_t v)
{
    return v > (uint64_t)LLONG_MAX ? LLONG_MAX : (epicsInt64)v;
}

}

/* ---- the pool with in-use accounting --------------------------------------------------------- */

esNDArrayPool::esNDArrayPool(asynNDArrayDriver *driver, size_t maxMemory)
    : NDArrayPool(driver, maxMemory), inUse_(0)
{
}

size_t esNDArrayPool::inUse() const
{
    epicsGuard<epicsMutex> g(lock_);
    return inUse_;
}

void esNDArrayPool::onAllocateArray(NDArray *pArray)
{
    if (!pArray)
        return;
    epicsGuard<epicsMutex> g(lock_);
    inUse_ += pArray->dataSize;
}

void esNDArrayPool::onReleaseArray(NDArray *pArray)
{
    if (pArray->getReferenceCount() != 0)
        return;
    epicsGuard<epicsMutex> g(lock_);
    inUse_ = inUse_ >= pArray->dataSize ? inUse_ - pArray->dataSize : 0;
}

static void receiverThreadC(void *p) { ((ADEncstream *)p)->receiverTask(); }
static void callbackThreadC(void *p) { ((ADEncstream *)p)->callbackTask(); }
static void exitHandlerC(void *p) { ((ADEncstream *)p)->shutdown(); }

int ADEncstream::configure(const ADEncstreamArgs &a)
{
    static const char *fn = "ADEncstreamConfig";
    std::string why;
    if (a.portName.empty())
        why = "portName is empty";
    else if (a.brick.empty())
        why = "brick is empty: give the Brick's address or hostname, host[:port]";
    else if (a.localPort < 1 || a.localPort > 65535)
        why = "localPort must be 1-65535";
    else if (a.maxFrameSamples < 1 || a.maxFrameSamples > MAX_CAPACITY)
        why = "maxFrameSamples must be 1-10000000";
    else if (a.queueDepth < 1 || a.queueDepth > 1024)
        why = "queueDepth must be 1-1024";
    else if (a.maxMemory == 0)
        why = "maxMemory must be set (0, unlimited, is not allowed)";
    else if (a.maxMemory < pluginReserve(a.maxFrameSamples) + FULL_FRAME_BYTES * (size_t)a.maxFrameSamples) {
        char b[160];
        sprintf(b, "maxMemory must hold the plugin reserve and one full frame: at least %.1f MB for %u samples",
                (pluginReserve(a.maxFrameSamples) + FULL_FRAME_BYTES * (size_t)a.maxFrameSamples) / 1e6,
                a.maxFrameSamples);
        why = b;
    }
    if (!why.empty()) {
        printf("%s: %s\n", fn, why.c_str());
        return -1;
    }

    /* the Brick: host[:port], resolved once */
    std::string host = a.brick;
    unsigned port = 5199;
    size_t colon = host.rfind(':');
    if (colon != std::string::npos) {
        char *end;
        unsigned long p = strtoul(host.c_str() + colon + 1, &end, 10);
        if (*end || p < 1 || p > 65535) {
            printf("%s: brick \"%s\": bad port\n", fn, a.brick.c_str());
            return -1;
        }
        port = (unsigned)p;
        host.erase(colon);
    }

    /* every allocation from here on may throw: a failure undoes what was built (5.4) */
    Parts *parts = 0;
    ADEncstream *d = 0;
    std::string brickText;
    try {
        parts = new Parts(a.maxFrameSamples, a.queueDepth);
        if (!osiSockAttach()) {
            printf("%s: osiSockAttach failed\n", fn);
            delete parts;
            return -1;
        }
        parts->sockAttached = true;

        struct in_addr ina;
        if (hostToIPAddr(host.c_str(), &ina) != 0) {
            printf("%s: brick \"%s\": the name does not resolve\n", fn, host.c_str());
            delete parts;
            return -1;
        }
        parts->brick.ia.sin_family = AF_INET;
        parts->brick.ia.sin_addr = ina;
        parts->brick.ia.sin_port = htons((unsigned short)port);
        char ip[32];
        ipAddrToDottedIP(&parts->brick.ia, ip, sizeof ip);
        brickText = host + " (" + ip + ")";

        /* the receiver core's configuration */
        ReceiverConfig &rc = parts->cfg.receiver;
        memcpy(rc.brick.addr, &ina.s_addr, 4);
        rc.brick.port = (uint16_t)port;
        if (a.identification.empty()) {
            char hn[128] = "";
            gethostname(hn, sizeof hn - 1);
            rc.identification = std::string("host=") + hn + ";ioc=" + a.portName;
            if (rc.identification.size() > 64)
                rc.identification.resize(64);
            for (size_t i = 0; i < rc.identification.size(); ++i)
                if (!isprint((unsigned char)rc.identification[i]))
                    rc.identification[i] = '?';
        } else {
            rc.identification = a.identification;
        }
        why = checkConfig(rc);
        if (!why.empty()) {
            printf("%s: %s\n", fn, why.c_str());
            delete parts;
            return -1;
        }
        parts->engine = new Engine(parts->cfg, parts->queue, parts->commands, parts->mailbox);

        /* the socket: non-blocking, so no receive or send can hold the receiver thread */
        parts->sock = epicsSocketCreate(AF_INET, SOCK_DGRAM, 0);
        if (parts->sock == INVALID_SOCKET) {
            printf("%s: cannot create a UDP socket\n", fn);
            delete parts;
            return -1;
        }
        int rb = RCVBUF_REQUEST;
        setsockopt(parts->sock, SOL_SOCKET, SO_RCVBUF, (char *)&rb, sizeof rb);
        osiSocklen_t rl = sizeof rb;
        if (getsockopt(parts->sock, SOL_SOCKET, SO_RCVBUF, (char *)&rb, &rl) == 0)
            parts->rcvBuf = rb;
        osiSockIoctl_t yes = 1;
        if (socket_ioctl(parts->sock, FIONBIO, &yes) != 0) {
            printf("%s: cannot make the UDP socket non-blocking\n", fn);
            delete parts;
            return -1;
        }
        osiSockAddr local;
        memset(&local, 0, sizeof local);
        local.ia.sin_family = AF_INET;
        local.ia.sin_port = htons((unsigned short)a.localPort);
        local.ia.sin_addr.s_addr = htonl(INADDR_ANY);
        if (!a.bindAddress.empty() && hostToIPAddr(a.bindAddress.c_str(), &local.ia.sin_addr) != 0) {
            printf("%s: bindAddress \"%s\" does not resolve\n", fn, a.bindAddress.c_str());
            delete parts;
            return -1;
        }
        if (bind(parts->sock, &local.sa, sizeof local.ia) != 0) {
            char err[128];
            epicsSocketConvertErrnoToString(err, sizeof err);
            printf("%s: cannot bind UDP port %d: %s\n", fn, a.localPort, err);
            delete parts;
            return -1;
        }

        /* the port: asyn cannot unregister it, so the constructor does nothing that can fail
         * (a constructor that throws would leave asyn a freed object);
         * everything fallible is in init(), on the live object */
        d = new ADEncstream(a, parts);
    } catch (std::exception &e) {
        printf("%s: %s while configuring (frame pool %.1f MB)\n", fn, e.what(),
               (double)(a.queueDepth + 2) * a.maxFrameSamples * FRAME_BYTES_PER_SAMPLE / 1e6);
        delete parts;                         /* d is 0: new threw before or in the base class */
        return -1;
    }
    epicsAtExit(exitHandlerC, d);
    if (!d->init(a, brickText, &why) || !d->start(&why)) {
        d->failStartup(why);
        printf("%s: %s; the port %s is marked faulted\n", fn, why.c_str(), a.portName.c_str());
        return -1;
    }
    return 0;
}

ADEncstream::ADEncstream(const ADEncstreamArgs &a, Parts *parts)
    : asynNDArrayDriver(a.portName.c_str(), ES_NUM_ADDR, 0, a.maxMemory, asynInt64Mask, asynInt64Mask,
                        ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, a.priority, a.stackSize),
      p_(parts), pool_(0), maxMemory_(a.maxMemory), capacity_(a.maxFrameSamples), exiting_(0), shutdownDone_(0),
      rxDone_(0), cbDone_(0), rxStarted_(false), cbStarted_(false), paramsReady_(false), uniqueId_(0), scratch_(0),
      acquireRevision_(0), controlRevision_(0), monitorWriteRevision_(0), faultApplied_(false), acquireSetAt_(0),
      refusedBusy_(0), refusalQueue_(0), refusalTimer_(0), snapActive_(false),
      snapRevision_(0), staleMonitor_(0),
      publishedFrames_(0), failedFrames_(0)
{
    priority_ = a.priority ? a.priority : (int)epicsThreadPriorityHigh;
    stackSize_ = a.stackSize ? (unsigned)a.stackSize : epicsThreadGetStackSize(epicsThreadStackMedium);
    /* the base class's Acquire and Busy exist already and must be defined even if init() fails:
     * asyn sends no callback for an undefined parameter, so a refused Acquire's handshake could
     * not return the Acquire record. Cannot fail: no allocation. */
    asynPortDriver::setIntegerParam(ADAcquire, 0);
    asynPortDriver::setIntegerParam(ADAcquireBusy, 0);
}

ADEncstream::ThreadCreate ADEncstream::createThread = epicsThreadCreate;
int ADEncstream::failInitStage = 0;

/* everything fallible after the port exists: a failure leaves a live,
 * faulted object; parameters are set only once all exist */
bool ADEncstream::init(const ADEncstreamArgs &a, const std::string &brickText, std::string *why)
{
    try {
        if (failInitStage == 1)
            throw std::bad_alloc();
        rxDone_ = new epicsEvent;
        cbDone_ = new epicsEvent;
        scratch_ = new NDAttributeList;
        if (failInitStage == 2)
            throw std::bad_alloc();
        /* our pool replaces the base class's, which stays empty */
        pool_ = new esNDArrayPool(this, a.maxMemory);
        this->pNDArrayPool = pool_;
        if (failInitStage == 3)
            throw std::bad_alloc();
        createEsParams();
        setStringParam(ADManufacturer, "encstream");
        setStringParam(ADModel, "Power Brick encoder stream");
        setStringParam(ES_BrickAddress, brickText.c_str());
        setIntegerParam(ES_LocalPort, a.localPort);
        setIntegerParam(ES_SocketRcvBuf, p_->rcvBuf);
        setIntegerParam(ES_MaxFrameSamples, (int)capacity_);
        setDoubleParam(NDPoolMaxMemory, a.maxMemory / 1048576.0);
        paramsReady_ = true;
        callParamCallbacks();
    } catch (std::exception &e) {
        *why = std::string("initialisation failed: ") + e.what();
        return false;
    }
    return true;
}

bool ADEncstream::start(std::string *why)
{
    /* the callback thread first: it publishes everything, a fault included */
    cbStarted_ = createThread("ESCallback", epicsThreadPriorityMedium, stackSize_, callbackThreadC, this) != 0;
    if (!cbStarted_) {
        *why = "the callback thread could not be created";
        return false;
    }
    rxStarted_ = createThread("ESReceiver", (unsigned)priority_, stackSize_, receiverThreadC, this) != 0;
    if (!rxStarted_) {
        *why = "the receiver thread could not be created";
        epicsAtomicSetIntT(&exiting_, 1);     /* stop the callback thread again */
        p_->wakeCb.trigger();
        if (cbDone_->wait(2.0))
            cbStarted_ = false;
        return false;
    }
    return true;
}

/* a start-up failure: published here, synchronously, never by a thread that may not exist */
void ADEncstream::failStartup(const std::string &why)
{
    {
        epicsGuard<epicsMutex> g(p_->faultLock);
        p_->fault = true;
        p_->faultMsg = why;
    }
    p_->commands.close("driver fault: " + why);
    if (paramsReady_) {
        DriverLock lk(this);
        applyFault(why);
    }
    /* the receiver thread never ran: release the socket now, so the UDP endpoint can be bound
     * again; only the faulted port stays */
    if (!rxStarted_)
        releaseSocket();
}

void ADEncstream::releaseSocket()
{
    if (p_->sock != INVALID_SOCKET) {
        epicsSocketDestroy(p_->sock);
        p_->sock = INVALID_SOCKET;
    }
    if (p_->sockAttached) {
        osiSockRelease();
        p_->sockAttached = false;
    }
}

bool ADEncstream::stopping() const
{
    return epicsAtomicGetIntT(&exiting_) != 0;
}

bool ADEncstream::faulted() const
{
    epicsGuard<epicsMutex> g(p_->faultLock);
    return p_->fault;
}

void ADEncstream::createEsParams()
{
    createParam(ES_BrickAddressString, asynParamOctet, &ES_BrickAddress);
    createParam(ES_LocalPortString, asynParamInt32, &ES_LocalPort);
    createParam(ES_SocketRcvBufString, asynParamInt32, &ES_SocketRcvBuf);
    createParam(ES_StreamIDString, asynParamOctet, &ES_StreamID);
    createParam(ES_StreamStateString, asynParamInt32, &ES_StreamState);
    createParam(ES_StreamFaultString, asynParamOctet, &ES_StreamFault);
    createParam(ES_ContactString, asynParamInt32, &ES_Contact);
    createParam(ES_DataStalledString, asynParamInt32, &ES_DataStalled);
    createParam(ES_IdentificationString, asynParamOctet, &ES_Identification);
    createParam(ES_ServoPeriodString, asynParamFloat64, &ES_ServoPeriod);
    createParam(ES_GatherPeriodString, asynParamInt32, &ES_GatherPeriod);
    createParam(ES_NumAxesString, asynParamInt32, &ES_NumAxes);
    createParam(ES_MotorsString, asynParamOctet, &ES_Motors);
    createParam(ES_CalGenerationString, asynParamInt32, &ES_CalGeneration);
    createParam(ES_RowsAdmittedString, asynParamInt64, &ES_RowsAdmitted);
    createParam(ES_RowsLostString, asynParamInt64, &ES_RowsLost);
    createParam(ES_RowsSkippedString, asynParamInt64, &ES_RowsSkipped);
    createParam(ES_LateOrDuplicateString, asynParamInt64, &ES_LateOrDuplicate);
    createParam(ES_SequenceGapsString, asynParamInt64, &ES_SequenceGaps);
    createParam(ES_RejectedString, asynParamInt64, &ES_Rejected);
    createParam(ES_StreamChangesString, asynParamInt64, &ES_StreamChanges);
    createParam(ES_FaultsString, asynParamInt64, &ES_Faults);
    createParam(ES_SubscribesSentString, asynParamInt64, &ES_SubscribesSent);
    createParam(ES_FinalNextRowString, asynParamInt64, &ES_FinalNextRow);
    createParam(ES_CursorString, asynParamInt64, &ES_Cursor);
    createParam(ES_GapFreeString, asynParamInt32, &ES_GapFree);
    createParam(ES_TailCompleteString, asynParamInt32, &ES_TailComplete);
    createParam(ES_RecentEventsString, asynParamOctet, &ES_RecentEvents);
    createParam(ES_DriverFaultString, asynParamInt32, &ES_DriverFault);
    createParam(ES_DriverFaultMsgString, asynParamOctet, &ES_DriverFaultMsg);
    createParam(ES_BrickWorstLoopString, asynParamInt32, &ES_BrickWorstLoop);
    createParam(ES_BrickBacklogString, asynParamInt32, &ES_BrickBacklog);
    createParam(ES_BrickOverrunsString, asynParamInt64, &ES_BrickOverruns);
    createParam(ES_BrickSendFailuresString, asynParamInt64, &ES_BrickSendFailures);
    createParam(ES_BrickVmRSSString, asynParamInt32, &ES_BrickVmRSS);
    createParam(ES_BrickOpenFdsString, asynParamInt32, &ES_BrickOpenFds);

    createParam(ES_AcquireModeString, asynParamInt32, &ES_AcquireMode);
    createParam(ES_NumAcquireString, asynParamInt64, &ES_NumAcquire);
    createParam(ES_NumAcquiredString, asynParamInt64, &ES_NumAcquired);
    createParam(ES_TriggerModeString, asynParamInt32, &ES_TriggerMode);
    createParam(ES_FramePointsString, asynParamInt32, &ES_FramePoints);
    createParam(ES_FrameTimeString, asynParamFloat64, &ES_FrameTime);
    createParam(ES_FramePointsRbvString, asynParamInt32, &ES_FramePointsRbv);
    createParam(ES_FrameTimeRbvString, asynParamFloat64, &ES_FrameTimeRbv);
    createParam(ES_MaxFrameSamplesString, asynParamInt32, &ES_MaxFrameSamples);
    createParam(ES_MinCoverageString, asynParamFloat64, &ES_MinCoverage);
    createParam(ES_EnableSampleIndexString, asynParamInt32, &ES_EnableSampleIndex);
    createParam(ES_EnableStatusString, asynParamInt32, &ES_EnableStatus);
    createParam(ES_AcqStateString, asynParamInt32, &ES_AcqState);
    createParam(ES_AcqMessageString, asynParamOctet, &ES_AcqMessage);
    createParam(ES_NotDeliveredString, asynParamInt64, &ES_NotDelivered);
    createParam(ES_CutByStopString, asynParamInt64, &ES_CutByStop);
    createParam(ES_SourceStoppedString, asynParamInt32, &ES_SourceStopped);
    createParam(ES_SourceStopRowString, asynParamInt64, &ES_SourceStopRow);
    createParam(ES_FramesAssembledString, asynParamInt64, &ES_FramesAssembled);
    createParam(ES_FramesWithGapsString, asynParamInt64, &ES_FramesWithGaps);
    createParam(ES_EmptyFramesString, asynParamInt64, &ES_EmptyFrames);
    createParam(ES_OverlongFramesString, asynParamInt64, &ES_OverlongFrames);
    createParam(ES_CalChangedFramesString, asynParamInt64, &ES_CalChangedFrames);
    createParam(ES_MinCoverageSeenString, asynParamFloat64, &ES_MinCoverageSeen);
    createParam(ES_DegradedString, asynParamInt32, &ES_Degraded);
    createParam(ES_QueueEntriesString, asynParamInt32, &ES_QueueEntries);
    createParam(ES_OutstandingFramesString, asynParamInt64, &ES_OutstandingFrames);

    createParam(ES_MonRunString, asynParamInt32, &ES_MonRun);
    createParam(ES_MonAveragingTimeString, asynParamFloat64, &ES_MonAveragingTime);
    createParam(ES_MonUpdatePeriodString, asynParamFloat64, &ES_MonUpdatePeriod);
    createParam(ES_MonAveragingTimeRbvString, asynParamFloat64, &ES_MonAveragingTimeRbv);
    createParam(ES_MonUpdatePeriodRbvString, asynParamFloat64, &ES_MonUpdatePeriodRbv);
    createParam(ES_MonMinCoverageString, asynParamFloat64, &ES_MonMinCoverage);
    createParam(ES_MonHealthString, asynParamInt32, &ES_MonHealth);
    createParam(ES_MonSamplesString, asynParamInt64, &ES_MonSamples);
    createParam(ES_MonExpectedString, asynParamInt64, &ES_MonExpected);
    createParam(ES_MonCoverageString, asynParamFloat64, &ES_MonCoverage);
    createParam(ES_MonCalChangedString, asynParamInt32, &ES_MonCalChanged);
    createParam(ES_MonUpdatesSkippedString, asynParamInt64, &ES_MonUpdatesSkipped);
    createParam(ES_MonGateString, asynParamInt32, &ES_MonGate);
    createParam(ES_MonGateEdgesString, asynParamInt32, &ES_MonGateEdges);
    createParam(ES_MonGateActiveString, asynParamFloat64, &ES_MonGateActive);

    createParam(ES_MeanString, asynParamFloat64, &ES_Mean);
    createParam(ES_StdDevString, asynParamFloat64, &ES_StdDev);
    createParam(ES_MinString, asynParamFloat64, &ES_Min);
    createParam(ES_MaxString, asynParamFloat64, &ES_Max);
    createParam(ES_StatusOrString, asynParamInt32, &ES_StatusOr);
    createParam(ES_HomePosString, asynParamFloat64, &ES_HomePos);
    createParam(ES_PosSfString, asynParamFloat64, &ES_PosSf);
    createParam(ES_PresentString, asynParamInt32, &ES_Present);

    /* defaults: the core's (FramerConfig, MonitorConfig) */
    const double nan = std::numeric_limits<double>::quiet_NaN();
    FramerConfig fc;
    MonitorConfig mc;
    setIntegerParam(ADAcquire, 0);            /* defined, so readbacks and getIntegerParam work */
    setIntegerParam(ADAcquireBusy, 0);
    setIntegerParam(ES_AcquireMode, ACQUIRE_CONTINUOUS);
    setInteger64Param(ES_NumAcquire, 1);
    setInteger64Param(ES_NumAcquired, 0);
    setIntegerParam(ES_TriggerMode, TRIGGER_FREE_RUNNING);
    setIntegerParam(ES_FramePoints, 0);
    setDoubleParam(ES_FrameTime, fc.frameTimeS);
    setIntegerParam(ES_FramePointsRbv, 0);
    setDoubleParam(ES_FrameTimeRbv, nan);
    setDoubleParam(ES_MinCoverage, fc.minCoverage);
    setIntegerParam(ES_EnableSampleIndex, 0);
    setIntegerParam(ES_EnableStatus, 0);
    setIntegerParam(ES_AcqState, DISPLAY_IDLE);
    setStringParam(ES_AcqMessage, "");
    setIntegerParam(ES_MonRun, 0);
    setDoubleParam(ES_MonAveragingTime, mc.averagingTimeS);
    setDoubleParam(ES_MonUpdatePeriod, mc.updatePeriodS);
    setDoubleParam(ES_MonAveragingTimeRbv, nan);
    setDoubleParam(ES_MonUpdatePeriodRbv, nan);
    setDoubleParam(ES_MonMinCoverage, mc.minCoverage);
    setIntegerParam(ES_MonHealth, HEALTH_STOPPED);
    setIntegerParam(ES_DriverFault, 0);
    setStringParam(ES_DriverFaultMsg, "");
    setStringParam(ES_StreamID, "");
    setStringParam(ES_StreamFault, "");
    setStringParam(ES_Identification, "");
    setStringParam(ES_Motors, "");
    setStringParam(ES_RecentEvents, "");
    for (int a = 0; a < 8; ++a) {
        setDoubleParam(a, ES_Mean, nan);
        setDoubleParam(a, ES_StdDev, nan);
        setDoubleParam(a, ES_Min, nan);
        setDoubleParam(a, ES_Max, nan);
        setIntegerParam(a, ES_StatusOr, 0);
        setDoubleParam(a, ES_HomePos, nan);
        setDoubleParam(a, ES_PosSf, nan);
        setIntegerParam(a, ES_Present, 0);
        int params[] = { ES_Mean, ES_StdDev, ES_Min, ES_Max, ES_StatusOr };
        for (unsigned k = 0; k < 5; ++k) {
            setParamAlarmStatus(a, params[k], epicsAlarmUDF);
            setParamAlarmSeverity(a, params[k], epicsSevInvalid);
        }
    }
    for (int a = 0; a < ES_NUM_ADDR; ++a) {   /* array diagnostics defined at every address */
        setIntegerParam(a, NDArrayCounter, 0);
        setIntegerParam(a, NDArraySizeX, 0);
        setIntegerParam(a, NDArraySizeY, 0);
        setIntegerParam(a, NDArraySize, 0);
        setIntegerParam(a, NDNDimensions, 0);
        setIntegerParam(a, NDDataType, a == ES_ADDR_INDEX ? NDUInt64 : a == ES_ADDR_STATUS ? NDUInt32 : NDFloat64);
        setIntegerParam(a, NDUniqueId, 0);
        setDoubleParam(a, NDTimeStamp, 0);
        setIntegerParam(a, NDEpicsTSSec, 0);
        setIntegerParam(a, NDEpicsTSNsec, 0);
    }
    for (int a = 0; a < ES_NUM_ADDR; ++a)
        callParamCallbacks(a);
}

/* ---- control (port thread, driver lock held) ------------------------------------------------- */

asynStatus ADEncstream::refuse(asynUser *pasynUser, const std::string &why)
{
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, "%s", why.c_str());
    asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: %s\n", driverName, why.c_str());
    return asynError;
}

/* return an output record with readback to the parameter's value after a refused write: asyn
 * re-sends an unchanged parameter only when its status changes, and an output record takes the
 * value only from a success (as for the Acquire record, serviceBusy()) */
void ADEncstream::revertReadback(int function)
{
    setParamStatus(function, asynError);
    callParamCallbacks();
    setParamStatus(function, asynSuccess);
    callParamCallbacks();
}

void ADEncstream::setAcqMessage(const std::string &m)
{
    setStringParam(ES_AcqMessage, m.c_str());
}

asynStatus ADEncstream::command(Command &c, std::string *why, uint64_t *revision)
{
    int id = p_->commands.submit(c);
    if (id < 0) {
        epicsGuard<epicsMutex> g(p_->faultLock);
        *why = p_->fault ? p_->faultMsg : (p_->commands.closed() ? std::string("the driver is shutting down")
                                                                 : std::string("no free command slot"));
        return asynError;
    }
    CommandResult r;
    switch (p_->commands.wait(id, COMMAND_WAIT_S, &r)) {
    case WAIT_DONE:
        if (revision)
            *revision = r.revision;
        if (r.applied)
            return asynSuccess;
        *why = r.message;
        return asynError;
    case WAIT_CANCELLED:
        *why = "the receiver thread did not respond within 0.5 s; not applied";
        return asynError;
    case WAIT_TIMED_OUT:
        *why = "timed out; the result is shown when it arrives";
        return asynError;
    case WAIT_SHUTDOWN:
        *why = r.message.empty() ? std::string("the driver is shutting down") : r.message;
        return asynError;
    }
    return asynError;
}

asynStatus ADEncstream::startAcquisition()
{
    Command c;
    c.type = CMD_ACQUIRE;
    FramerConfig &f = c.framer;
    int i;
    epicsInt64 n;
    double d;
    getIntegerParam(ES_TriggerMode, &i);
    f.trigger = (TriggerMode)i;
    getIntegerParam(ES_AcquireMode, &i);
    f.mode = (AcquireMode)i;
    getInteger64Param(ES_NumAcquire, &n);
    f.numAcquire = n < 1 ? 1 : (uint64_t)n;
    getIntegerParam(ES_FramePoints, &i);
    f.framePoints = i < 0 ? 0 : (uint64_t)i;
    getDoubleParam(ES_FrameTime, &d);
    f.frameTimeS = d;
    getIntegerParam(ES_MaxFrameSamples, &i);
    f.maxFrameSamples = i < 1 ? 1 : (uint32_t)i;
    getDoubleParam(ES_MinCoverage, &d);
    f.minCoverage = d;
    getIntegerParam(ES_EnableSampleIndex, &i);
    f.withSampleIndex = i != 0;
    getIntegerParam(ES_EnableStatus, &i);
    f.withStatus = i != 0;

    std::string why;
    uint64_t rev = 0;
    asynStatus st = command(c, &why, &rev);
    if (rev > controlRevision_)
        controlRevision_ = rev;
    if (st == asynSuccess) {
        acquireRevision_ = rev;
        acquireSetAt_ = monotonicS();
        setIntegerParam(ADAcquire, 1);        /* sets the busy readback (base class) */
        setAcqMessage("");
        return asynSuccess;
    }
    setAcqMessage("Acquire refused: " + why);
    armRefusal();                             /* the Acquire record's link sets the busy record */
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: Acquire refused: %s\n", driverName, why.c_str());
    return asynError;
}

asynStatus ADEncstream::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    const int function = pasynUser->reason;
    if (!paramsReady_) {
        if (function == ADAcquire && value)
            armRefusal();                     /* the base class's Acquire records exist */
        return refuse(pasynUser, "the driver failed to initialise");
    }
    int addr = 0;
    getAddress(pasynUser, &addr);

    /* one acquisition, one pool: the global controls live at address 0 only */
    if (addr != 0 && (function == ADAcquire || function == ADWaitForPlugins || function == NDArrayCallbacks ||
                      function == NDPoolEmptyFreeList || function == NDPoolPollStats ||
                      function >= FIRST_ES_PARAM))
        return refuse(pasynUser, "this control exists at address 0 only");

    if (function == ADAcquire) {
        int cur = 0;
        getIntegerParam(ADAcquire, &cur);
        asynStatus st = asynSuccess;
        if (value && !cur) {
            st = startAcquisition();
        } else if (!value) {
            Command c;
            c.type = CMD_STOP;
            std::string why;
            uint64_t rev = 0;
            st = command(c, &why, &rev);
            if (rev > controlRevision_)
                controlRevision_ = rev;
            if (st != asynSuccess)
                setAcqMessage("Stop failed: " + why);
            /* Acquire goes to 0 when the stopped acquisition's output is settled (4.4) */
        }
        callParamCallbacks();
        return st;
    }
    if (function == ES_MonRun) {
        Command c;
        c.type = value ? CMD_MONITOR_START : CMD_MONITOR_STOP;
        std::string why;
        uint64_t rev = 0;
        if (command(c, &why, &rev) != asynSuccess) {
            revertReadback(ES_MonRun);
            return refuse(pasynUser, "MonitorRun: " + why);
        }
        monitorWriteRevision_ = rev;
        setIntegerParam(ES_MonRun, value ? 1 : 0);
        callParamCallbacks();
        return asynSuccess;
    }
    if (function == ES_TriggerMode && (value < TRIGGER_FREE_RUNNING || value > TRIGGER_BOTH))
        return refuse(pasynUser, "TriggerMode must be 0-5");
    if (function == ES_AcquireMode && (value < ACQUIRE_CONTINUOUS || value > ACQUIRE_SINGLE))
        return refuse(pasynUser, "AcquireMode must be 0-2");
    if (function == ES_FramePoints && value < 0)
        return refuse(pasynUser, "FramePoints must not be negative");
    if (function == ES_MaxFrameSamples && (value < 1 || (unsigned)value > capacity_)) {
        char b[96];
        sprintf(b, "MaxFrameSamples must be 1-%u (the configured pool capacity)", capacity_);
        return refuse(pasynUser, b);
    }
    if (function < FIRST_ES_PARAM)
        return asynNDArrayDriver::writeInt32(pasynUser, value);
    if (function == ES_EnableSampleIndex || function == ES_EnableStatus)
        value = value ? 1 : 0;
    setIntegerParam(function, value);
    callParamCallbacks();
    return asynSuccess;
}

asynStatus ADEncstream::writeInt64(asynUser *pasynUser, epicsInt64 value)
{
    if (!paramsReady_)
        return refuse(pasynUser, "the driver failed to initialise");
    const int function = pasynUser->reason;
    int addr = 0;
    getAddress(pasynUser, &addr);
    if (function == ES_NumAcquire) {
        if (addr != 0)
            return refuse(pasynUser, "this control exists at address 0 only");
        if (value < 1)
            return refuse(pasynUser, "NumAcquire must be at least 1");
        setInteger64Param(ES_NumAcquire, value);
        callParamCallbacks();
        return asynSuccess;
    }
    return asynNDArrayDriver::writeInt64(pasynUser, value);
}

asynStatus ADEncstream::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    if (!paramsReady_)
        return refuse(pasynUser, "the driver failed to initialise");
    const int function = pasynUser->reason;
    int addr = 0;
    getAddress(pasynUser, &addr);
    if (function >= FIRST_ES_PARAM && addr != 0)
        return refuse(pasynUser, "this control exists at address 0 only");
    if (function == ES_FrameTime && !(std::isfinite(value) && value > 0))
        return refuse(pasynUser, "FrameTime must be a positive number");
    if (function == ES_MinCoverage && !(value >= 0 && value <= 1))
        return refuse(pasynUser, "MinCoverage must be 0-1");
    if (function == ES_MonAveragingTime || function == ES_MonUpdatePeriod || function == ES_MonMinCoverage) {
        /* only the field written: the engine applies it to the accepted configuration, so a
         * stale cached field can never revert a setting applied late */
        Command c;
        c.type = CMD_MONITOR_CONFIGURE;
        if (function == ES_MonAveragingTime) {
            c.monitor.averagingTimeS = value;
            c.monitorFields = MONITOR_FIELD_T;
        } else if (function == ES_MonUpdatePeriod) {
            c.monitor.updatePeriodS = value;
            c.monitorFields = MONITOR_FIELD_U;
        } else {
            c.monitor.minCoverage = value;
            c.monitorFields = MONITOR_FIELD_MIN_COVERAGE;
        }
        std::string why;
        uint64_t rev = 0;
        if (command(c, &why, &rev) != asynSuccess) {
            revertReadback(function);
            return refuse(pasynUser, "monitor configuration refused: " + why);
        }
        monitorWriteRevision_ = rev;
        setDoubleParam(function, value);
        callParamCallbacks();
        return asynSuccess;
    }
    if (function < FIRST_ES_PARAM)
        return asynNDArrayDriver::writeFloat64(pasynUser, value);
    setDoubleParam(function, value);
    callParamCallbacks();
    return asynSuccess;
}

/* ---- the receiver thread: socket and Engine, never the driver lock ----------- */

void ADEncstream::driverFault(const std::string &why)
{
    {
        epicsGuard<epicsMutex> g(p_->faultLock);
        if (p_->fault)
            return;
        p_->fault = true;
        p_->faultMsg = why;
    }
    p_->commands.close("driver fault: " + why);
    p_->wakeCb.trigger();
}

void ADEncstream::receiverTask()
{
    try {
        receiverLoop();
    } catch (std::exception &e) {
        driverFault(std::string("receiver thread: ") + e.what());
    } catch (...) {
        driverFault("receiver thread: unknown exception");
    }
    rxDone_->trigger();
}

void ADEncstream::receiverLoop()
{
    Engine &eng = *p_->engine;
    const SOCKET s = p_->sock;
    uint8_t buf[2048];
    while (!stopping() && !faulted()) {
        uint64_t now = epicsMonotonicGet();
        uint64_t deadline = eng.nextDeadline();
        uint64_t waitNs = deadline > now ? deadline - now : 0;   /* signed-safe, clamped */
        if (waitNs > SERVICE_CAP_NS)
            waitNs = SERVICE_CAP_NS;
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(s, &rd);
        struct timeval tv;
        tv.tv_sec = (long)(waitNs / 1000000000ull);
        tv.tv_usec = (long)((waitNs % 1000000000ull) / 1000);
        int n = select((int)s + 1, &rd, 0, 0, &tv);
        if (n < 0) {
            int e = SOCKERRNO;
            if (e == SOCK_EINTR)
                continue;
            char err[128];
            epicsSocketConvertErrnoToString(err, sizeof err);
            driverFault(std::string("select failed: ") + err);
            return;
        }
        now = epicsMonotonicGet();
        if (n > 0) {
            /* the socket is non-blocking: read what is queued, at most 64 datagrams a pass */
            for (unsigned k = 0; k < 64 && !stopping(); ++k) {
                osiSockAddr from;
                memset(&from, 0, sizeof from);
                osiSocklen_t fl = sizeof from.ia;
                int flags = 0;
#if defined(MSG_TRUNC) && !defined(_WIN32)
                flags = MSG_TRUNC;            /* Linux: the real length of a truncated datagram.
                                                 Windows defines MSG_TRUNC but rejects it as a
                                                 recvfrom input flag (WSAEOPNOTSUPP); there
                                                 truncation is reported by WSAEMSGSIZE below */
#endif
                int r = recvfrom(s, (char *)buf, sizeof buf, flags, &from.sa, &fl);
                size_t len;
                bool truncated = false;
                if (r < 0) {
                    int e = SOCKERRNO;
                    if (e == SOCK_EMSGSIZE) {             /* Windows: WSAEMSGSIZE */
                        truncated = true;
                        len = sizeof buf;
                    } else if (e == SOCK_EWOULDBLOCK || e == SOCK_EINTR || e == SOCK_ECONNRESET) {
                        break;                            /* none left, or transient (Windows ICMP) */
                    } else {
                        char err[128];
                        epicsSocketConvertErrnoToString(err, sizeof err);
                        driverFault(std::string("recvfrom failed: ") + err);
                        return;
                    }
                } else if ((size_t)r > sizeof buf) {
                    truncated = true;
                    len = sizeof buf;
                } else {
                    len = (size_t)r;
                }
                Endpoint src;
                memcpy(src.addr, &from.ia.sin_addr.s_addr, 4);
                src.port = ntohs(from.ia.sin_port);
                eng.datagram(buf, len, truncated, src, now);
            }
        } else {
            eng.timer(now);
        }
        const uint8_t *out;
        size_t outLen;
        if (eng.pendingSend(&out, &outLen)) {
            /* non-blocking: a full send buffer is a failed send, the receiver's retry policy applies */
            int r = sendto(s, (const char *)out, (int)outLen, 0, &p_->brick.sa, sizeof p_->brick.ia);
            eng.sent(r == (int)outLen, epicsMonotonicGet());
        }
        eng.service(epicsMonotonicGet());
    }
}

/* ---- the callback thread ----------------------------------------------------- */

void ADEncstream::callbackTask()
{
    try {
        callbackLoop();
    } catch (std::exception &e) {
        driverFault(std::string("callback thread: ") + e.what());
    } catch (...) {
        driverFault("callback thread: unknown exception");
    }
    try {
        std::string msg;
        {
            epicsGuard<epicsMutex> g(p_->faultLock);
            msg = p_->faultMsg;
        }
        DriverLock g(this);
        if (faulted() && !faultApplied_)
            applyFault(msg);
    } catch (...) {
    }
    cbDone_->trigger();
}

void ADEncstream::callbackLoop()
{
    std::vector<Snapshot> snapBuf(1);
    Snapshot &snap = snapBuf[0];
    bool more = false;
    while (!stopping()) {
        if (!more)
            p_->wakeCb.wait(0.05);
        more = false;
        uint64_t seq;
        MonitorUpdate u;
        Work w;
        if (faulted()) {
            /* terminal: applied once; queued output and status are discarded */
            if (!faultApplied_) {
                std::string msg;
                {
                    epicsGuard<epicsMutex> g(p_->faultLock);
                    msg = p_->faultMsg;
                }
                DriverLock g(this);
                applyFault(msg);
            }
            p_->mailbox.takeSnapshot(&snap);
            p_->mailbox.takeMonitor(&u);
            while (p_->queue.take(&w))
                p_->queue.done(w, 0, w.kind == Work::EMPTY ? w.run.count : 1);
            continue;
        }
        if (p_->mailbox.takeSnapshot(&snap, &seq))
            publishSnapshot(snap, seq);
        if (p_->mailbox.takeMonitor(&u, &seq))
            publishMonitor(u, seq);
        unsigned n = 0;
        while (!stopping() && !faulted() && n < 16 && p_->queue.take(&w)) {
            publishWork(w);
            ++n;
        }
        if (n == 16)
            more = true;                      /* interleave status with a long backlog */
        serviceBusy();
    }
}

/* The terminal driver fault, driver lock held: commands are already closed and
 * the receiver thread has stopped or stops on its next pass. Shows the acquisition invalid and
 * the monitor faulted, and releases Acquire and Busy directly: the base class would keep Busy
 * while a plugin holds arrays (WaitForPlugins), and a fault must never leave it waiting. Plugin
 * arrays are not touched. */
void ADEncstream::applyFault(const std::string &why)
{
    faultApplied_ = true;
    setIntegerParam(ES_DriverFault, 1);
    setParamAlarmStatus(ES_DriverFault, epicsAlarmComm);
    setParamAlarmSeverity(ES_DriverFault, epicsSevMajor);
    setStringParam(ES_DriverFaultMsg, why.c_str());
    setIntegerParam(ES_AcqState, DISPLAY_INVALID);
    setParamAlarmStatus(ES_AcqState, epicsAlarmComm);
    setParamAlarmSeverity(ES_AcqState, epicsSevMajor);
    setAcqMessage("driver fault: " + why);
    setIntegerParam(ES_MonHealth, HEALTH_FAULTED);
    setParamAlarmStatus(ES_MonHealth, epicsAlarmComm);
    setParamAlarmSeverity(ES_MonHealth, epicsSevMajor);
    setIntegerParam(ES_MonRun, 0);
    const int params[] = { ES_Mean, ES_StdDev, ES_Min, ES_Max, ES_StatusOr };
    for (int a = 0; a < MONITOR_SLOTS; ++a)
        for (unsigned k = 0; k < 5; ++k) {
            setParamAlarmStatus(a, params[k], epicsAlarmComm);
            setParamAlarmSeverity(a, params[k], epicsSevInvalid);
        }
    asynPortDriver::setIntegerParam(ADAcquire, 0);
    asynPortDriver::setIntegerParam(ADAcquireBusy, 0);
    for (int a = 0; a < ES_NUM_ADDR; ++a)
        callParamCallbacks(a);
}

/* A refused Acquire = 1 (port thread, driver lock held). The Acquire output record keeps the
 * refused 1 and its forward link (SetAcquireBusy) sets the busy record; only a 1 -> 0 transition
 * of the busy readback (AcquireBusyCB -> ClearAcquireBusy, a CP link) clears it. So the busy
 * readback is toggled once the link has run, one step per BUSY_GUARD_S (the CP link coalesces
 * updates that arrive together: 1 and 0 at once would show no transition), and the Acquire record
 * is returned to 0. This runs on a timer of EPICS's shared timer queue, not on the callback thread,
 * so it also works after a terminal fault, a start-up failure, a dead worker or a failed init()
 *. It touches only the busy readback and the Acquire record's status:
 * it cannot start an acquisition or restore any other state. */
void ADEncstream::armRefusal()
{
    refusedBusy_ = 1;
    if (!refusalTimer_) {
        refusalQueue_ = epicsTimerQueueAllocate(1, epicsThreadPriorityScanLow);
        if (refusalQueue_)
            refusalTimer_ = epicsTimerQueueCreateTimer(refusalQueue_, refusalTimerC, this);
    }
    if (refusalTimer_)
        epicsTimerStartDelay(refusalTimer_, BUSY_GUARD_S);   /* restarts a pending one */
    else
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: no timer for the refused Acquire's busy handshake; AcquireBusy stays set\n", driverName);
}

void ADEncstream::refusalTimerC(void *arg)
{
    static_cast<ADEncstream *>(arg)->refusalStep();
}

void ADEncstream::refusalStep()
{
    DriverLock lk(this);
    int acquire = 0;
    getIntegerParam(ADAcquire, &acquire);
    if (refusedBusy_ == 1) {
        /* an Acquire accepted since then owns Busy: nothing to undo */
        refusedBusy_ = acquire ? 0 : 2;
        if (!acquire) {
            asynPortDriver::setIntegerParam(ADAcquireBusy, 1);
            callParamCallbacks();
            epicsTimerStartDelay(refusalTimer_, BUSY_GUARD_S);
        }
    } else if (refusedBusy_ == 2) {
        refusedBusy_ = 0;
        if (!acquire) {
            asynPortDriver::setIntegerParam(ADAcquireBusy, 0);
            callParamCallbacks();
            /* and the Acquire record back to 0: its value is still the refused 1, so the next
             * Acquire = 1 would be no transition for its link and would never set the busy
             * record. asyn re-sends an unchanged parameter only when its status changes, and an
             * output record takes the value only from a success: error, then success (the
             * record's own callback ring keeps both, in order). */
            setParamStatus(ADAcquire, asynError);
            callParamCallbacks();
            setParamStatus(ADAcquire, asynSuccess);
            callParamCallbacks();
        }
    }
}

void ADEncstream::serviceBusy()
{
    DriverLock lk(this);
    const double now = monotonicS();
    int acquire = 0;
    getIntegerParam(ADAcquire, &acquire);
    /* (a refused Acquire's busy handshake runs on its own timer: armRefusal()) */
    /* Acquire released when the acquisition's output is settled (4.4): only by a snapshot at
     * least as new as the Acquire command's own, and not before the Acquire record's forward
     * link has set the busy record (a release before it would leave Busy set for good) */
    if (acquire && !snapActive_ && snapRevision_ >= acquireRevision_ && now - acquireSetAt_ >= BUSY_GUARD_S) {
        setIntegerParam(ADAcquire, 0);        /* the base class clears Busy (or after the plugins) */
        callParamCallbacks();
        if (snapRevision_ > controlRevision_)
            controlRevision_ = snapRevision_;
    }
}

void ADEncstream::publishSnapshot(const Snapshot &s, uint64_t seq)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const StreamView &v = s.view;
    const Counters &c = s.counters;
    DriverLock lk(this);
    {
        epicsGuard<epicsMutex> g(p_->mailbox.gate());
        if (seq != p_->mailbox.snapshotSeq())
            return;                           /* superseded: the newer one is published next */
    }
    char b[64];
    sprintf(b, "%08x", (unsigned)v.stream);
    setStringParam(ES_StreamID, v.hasStream ? b : "");
    int state = !v.hasStream ? 0 : v.fault != FAULT_NONE ? 4 : v.hasFinal ? 3 : !v.ready ? 1 : 2;
    setIntegerParam(ES_StreamState, state);
    if (v.fault != FAULT_NONE) {
        char f[160];
        epicsSnprintf(f, sizeof f, "%s (Brick state %u, fault %u)", faultName(v.fault),
                      v.hasStatus ? (unsigned)v.lastStatus.state : 0u, v.hasStatus ? (unsigned)v.lastStatus.fault : 0u);
        setStringParam(ES_StreamFault, f);
    } else {
        setStringParam(ES_StreamFault, "");
    }
    setIntegerParam(ES_Contact, s.contactLost ? 0 : 1);
    setIntegerParam(ES_DataStalled, s.dataStalled ? 1 : 0);
    setStringParam(ES_Identification, v.hasStream ? v.layout.identification : "");
    setDoubleParam(ES_ServoPeriod, v.hasStream ? v.layout.servoPeriodS : nan);
    setIntegerParam(ES_GatherPeriod, v.hasStream ? v.layout.gatherPeriod : 0);
    setIntegerParam(ES_NumAxes, v.hasStream ? v.layout.nAxes : 0);
    setStringParam(ES_Motors, v.hasStream ? motorList(v.layout.axisMotor, v.layout.nAxes).c_str() : "");
    setIntegerParam(ES_CalGeneration, v.ready ? (int)v.latestGeneration : 0);
    uint64_t rejected = c.unknownType + c.unexpectedType + c.oversize + c.truncated + c.rejectedRowSize;
    for (size_t i = 0; i < sizeof c.rejected / sizeof c.rejected[0]; ++i)
        rejected += c.rejected[i];
    setInteger64Param(ES_RowsAdmitted, sat(c.rowsAdmitted));
    setInteger64Param(ES_RowsLost, sat(c.rowsLost));
    setInteger64Param(ES_RowsSkipped, sat(c.rowsSkippedByBrick));
    setInteger64Param(ES_LateOrDuplicate, sat(c.lateOrDuplicate));
    setInteger64Param(ES_SequenceGaps, sat(c.sequenceGaps));
    setInteger64Param(ES_Rejected, sat(rejected));
    setInteger64Param(ES_StreamChanges, sat(c.streamChanges));
    setInteger64Param(ES_Faults, sat(c.faults));
    setInteger64Param(ES_SubscribesSent, sat(c.subscribesSent));
    setInteger64Param(ES_FinalNextRow, v.hasFinal ? sat(v.final.next_row) : 0);
    setInteger64Param(ES_Cursor, v.hasRows ? sat(v.cursor) : 0);
    setIntegerParam(ES_GapFree, v.gapFree ? 1 : 0);
    setIntegerParam(ES_TailComplete, v.tailComplete ? 1 : 0);
    /* resource use encstream reports from the Brick (its STATUS packet); 0 until the first STATUS */
    setIntegerParam(ES_BrickWorstLoop, v.hasStatus ? (int)v.lastStatus.worst_loop_us : 0);
    setIntegerParam(ES_BrickBacklog, v.hasStatus ? (int)v.lastStatus.backlog : 0);
    setInteger64Param(ES_BrickOverruns, v.hasStatus ? sat(v.lastStatus.overruns) : 0);
    setInteger64Param(ES_BrickSendFailures, v.hasStatus ? sat(v.lastStatus.send_failures) : 0);
    setIntegerParam(ES_BrickVmRSS, v.hasStatus ? (int)v.lastStatus.vmrss_kb : 0);
    setIntegerParam(ES_BrickOpenFds, v.hasStatus ? (int)v.lastStatus.open_fds : 0);
    std::string ev;
    for (unsigned i = 0; i < s.nEvents; ++i) {
        char line[96];
        epicsSnprintf(line, sizeof line, "%.3f %s %08x\n", s.events[i].tNs * 1e-9,
                      eventName(s.events[i].type), (unsigned)s.events[i].stream);
        ev += line;
    }
    setStringParam(ES_RecentEvents, ev.c_str());

    /* acquisition */
    const AcqStatus &a = s.acq;
    setIntegerParam(ES_AcqState, s.display);
    setParamAlarmStatus(ES_AcqState, s.display == DISPLAY_INVALID ? epicsAlarmState : epicsAlarmNone);
    setParamAlarmSeverity(ES_AcqState, s.display == DISPLAY_INVALID ? epicsSevMajor : epicsSevNone);
    setInteger64Param(ES_NumAcquired, sat(a.numAcquired));
    setInteger64Param(ES_NotDelivered, sat(a.notDelivered));
    setParamAlarmStatus(ES_NotDelivered, a.notDelivered ? epicsAlarmState : epicsAlarmNone);
    setParamAlarmSeverity(ES_NotDelivered, a.notDelivered ? epicsSevMajor : epicsSevNone);
    setInteger64Param(ES_CutByStop, sat(s.cutByStop));
    setIntegerParam(ES_SourceStopped, a.sourceStopped ? 1 : 0);
    setInteger64Param(ES_SourceStopRow, sat(a.sourceStopRow));
    setInteger64Param(ES_FramesAssembled, sat(a.assembled));
    setInteger64Param(ES_FramesWithGaps, sat(a.withGaps));
    setInteger64Param(ES_EmptyFrames, sat(a.empty));
    setInteger64Param(ES_OverlongFrames, sat(a.overlong));
    setInteger64Param(ES_CalChangedFrames, sat(a.calibrationChanged));
    setDoubleParam(ES_MinCoverageSeen, a.minCoverage);
    setIntegerParam(ES_Degraded, a.degraded ? 1 : 0);
    setParamAlarmStatus(ES_Degraded, a.degraded ? epicsAlarmState : epicsAlarmNone);
    setParamAlarmSeverity(ES_Degraded, a.degraded ? epicsSevMinor : epicsSevNone);
    setIntegerParam(ES_QueueEntries, (int)s.queueEntries);
    setInteger64Param(ES_OutstandingFrames, sat(s.outstandingFrames));
    setIntegerParam(ES_FramePointsRbv, a.framePoints > (uint64_t)INT_MAX ? INT_MAX : (int)a.framePoints);
    setDoubleParam(ES_FrameTimeRbv, a.framePoints ? s.framePointsS : nan);
    /* the engine's message only when it changes, so a port-thread refusal stays until then */
    if (lastEngineMessage_ != s.acqMessage) {
        lastEngineMessage_ = s.acqMessage;
        setAcqMessage(lastEngineMessage_);
    }
    snapActive_ = s.active;                   /* Acquire is released in serviceBusy() */
    snapRevision_ = s.revision;
    /* the engine's actual control state: a command applied after its write
     * timed out reaches the readbacks, unless a newer write has succeeded since */
    if (s.revision >= monitorWriteRevision_) {
        setIntegerParam(ES_MonRun, s.monitorRunning ? 1 : 0);
        setDoubleParam(ES_MonAveragingTime, s.monitorConfig.averagingTimeS);
        setDoubleParam(ES_MonUpdatePeriod, s.monitorConfig.updatePeriodS);
        setDoubleParam(ES_MonMinCoverage, s.monitorConfig.minCoverage);
    }
    int acquireNow = 0;
    getIntegerParam(ADAcquire, &acquireNow);
    if (s.active && !acquireNow && s.revision > controlRevision_) {
        /* an Acquire applied after its write timed out: show it running. The busy record stays
         * clear: that write was reported as failed. */
        setIntegerParam(ADAcquire, 1);
        acquireRevision_ = s.revision;
        acquireSetAt_ = monotonicS();
        controlRevision_ = s.revision;
    }

    /* monitor read-backs */
    setDoubleParam(ES_MonAveragingTimeRbv, s.monitor.known ? s.monitor.averagingTimeS : nan);
    setDoubleParam(ES_MonUpdatePeriodRbv, s.monitor.known ? s.monitor.updatePeriodS : nan);

    /* per motor: presence and the latest calibration */
    for (unsigned m = 1; m <= 8; ++m) {
        bool present = false;
        for (unsigned i = 0; v.hasStream && i < v.layout.nAxes; ++i)
            present = present || v.layout.axisMotor[i] == m;
        double home = nan, sf = nan;
        for (unsigned i = 0; s.hasCalibration && i < s.calibration.nAxes; ++i)
            if (s.calibration.axes[i].motor == m) {
                home = s.calibration.axes[i].home_pos;
                sf = s.calibration.axes[i].pos_sf;
            }
        setIntegerParam(m - 1, ES_Present, present ? 1 : 0);
        setDoubleParam(m - 1, ES_HomePos, home);
        setDoubleParam(m - 1, ES_PosSf, sf);
    }
    setDoubleParam(NDPoolUsedMemory, pool_->getMemorySize() / 1048576.0);
    setIntegerParam(NDPoolAllocBuffers, pool_->getNumBuffers());
    setIntegerParam(NDPoolFreeBuffers, pool_->getNumFree());
    for (int i = 0; i < 8; ++i)
        callParamCallbacks(i);
}

void ADEncstream::publishMonitor(const MonitorUpdate &u, uint64_t seq)
{
    DriverLock lk(this);
    {
        /* the publication gate: only the latest posted update,
         * of the current run; health-only changes advance the sequence too */
        epicsGuard<epicsMutex> g(p_->mailbox.gate());
        if (seq != p_->mailbox.monitorSeq() || u.run != p_->mailbox.currentRun()) {
            ++staleMonitor_;
            return;
        }
        setIntegerParam(ES_MonHealth, u.health);
        int hs = u.health == HEALTH_OK ? epicsSevNone :
                 (u.health == HEALTH_STOPPED || u.health == HEALTH_WAITING) ? epicsSevNone : epicsSevMajor;
        setParamAlarmStatus(ES_MonHealth, hs ? epicsAlarmState : epicsAlarmNone);
        setParamAlarmSeverity(ES_MonHealth, hs);
        setInteger64Param(ES_MonSamples, sat(u.samples));
        setInteger64Param(ES_MonExpected, sat(u.expected));
        setDoubleParam(ES_MonCoverage, u.coverage);
        setIntegerParam(ES_MonCalChanged, u.calibrationChanged ? 1 : 0);
        setInteger64Param(ES_MonUpdatesSkipped, sat(u.updatesSkipped));
        const int params[] = { ES_Mean, ES_StdDev, ES_Min, ES_Max, ES_StatusOr };
        for (int a = 0; a < MONITOR_SLOTS; ++a) {
            const MonitorSlot &sl = u.slot[a];
            if (u.values) {
                setDoubleParam(a, ES_Mean, sl.mean);
                setDoubleParam(a, ES_StdDev, sl.stddev);
                setDoubleParam(a, ES_Min, sl.min);
                setDoubleParam(a, ES_Max, sl.max);
                setIntegerParam(a, ES_StatusOr, (int)sl.statusOr);
            }
            for (unsigned k = 0; k < 5; ++k) {
                Severity sev = sl.severity;
                if (params[k] == ES_StatusOr && !sl.hasStatus)
                    sev = SEVERITY_INVALID;
                setParamAlarmStatus(a, params[k], severityToStatus(sev));
                setParamAlarmSeverity(a, params[k], severityToEpics(sev));
            }
        }
        if (u.values) {
            setIntegerParam(ES_MonGate, u.gateLevel);
            setIntegerParam(ES_MonGateEdges, (int)u.gateEdges);
            setDoubleParam(ES_MonGateActive, u.gateActiveFraction);
        }
        {
            int gst = severityToStatus(u.gateSeverity), gse = severityToEpics(u.gateSeverity);
            const int gp[] = { ES_MonGate, ES_MonGateEdges, ES_MonGateActive };
            for (unsigned k = 0; k < 3; ++k) {
                setParamAlarmStatus(gp[k], gst);
                setParamAlarmSeverity(gp[k], gse);
            }
        }
    }
    for (int a = 0; a < 8; ++a)
        callParamCallbacks(a);
}

void ADEncstream::publishWork(const Work &w)
{
    WorkGuard g(p_->queue, w);
    if (w.kind == Work::FRAME) {
        if (publishFrame(w.frame, 0, 0))
            g.published = 1;
    } else {
        while (g.published < w.run.count && !stopping() && publishFrame(0, &w.run, g.published))
            ++g.published;
    }
    g.settle();
}

bool ADEncstream::publishFrame(const Frame *f, const EmptyRun *run, uint64_t index)
{
    const OutputSchema &sc = f ? f->schema : run->schema;
    const size_t S = f ? f->samples : 1;
    const size_t cap = f ? f->capacity : 0;
    const double nan = std::numeric_limits<double>::quiet_NaN();

    int callbacks = 1;
    {
        DriverLock lk(this);
        getIntegerParam(NDArrayCallbacks, &callbacks);
    }
    if (!callbacks) {                         /* counted as published, arrays not built (4.4) */
        ++publishedFrames_;
        return true;
    }

    /* every array of the frame allocated before the first callback */
    int column[8];                            /* layout column of Motor[m+1], or -1 */
    for (int m = 0; m < 8; ++m) {
        column[m] = -1;
        for (unsigned a = 0; a < sc.nAxes; ++a)
            if (sc.motor[a] == m + 1)
                column[m] = (int)a;
    }
    const unsigned nStatus = sc.withStatus ? sc.nStatus : 0;
    size_t need = 8 * 8 * S + (size_t)sc.nAxes * 8 * S + (sc.withSampleIndex ? 8 * S : 0) + 4 * (size_t)nStatus * S;
    FrameArrays owned;
    NDArray **arr = owned.a;
    bool ok = pool_->inUse() + need + pluginReserve(capacity_) <= maxMemory_;
    if (ok) {
        size_t d2[2] = { 8, S };
        arr[ES_ADDR_MAIN] = pool_->alloc(2, d2, NDFloat64, 0, 0);
        ok = arr[ES_ADDR_MAIN] != 0;
        for (int m = 0; ok && m < 8; ++m)
            if (column[m] >= 0) {
                size_t d1 = S;
                arr[m] = pool_->alloc(1, &d1, NDFloat64, 0, 0);
                ok = arr[m] != 0;
            }
        if (ok && sc.withSampleIndex) {
            size_t d1 = S;
            arr[ES_ADDR_INDEX] = pool_->alloc(1, &d1, NDUInt64, 0, 0);
            ok = arr[ES_ADDR_INDEX] != 0;
        }
        if (ok && nStatus) {
            size_t ds[2] = { nStatus, S };
            arr[ES_ADDR_STATUS] = pool_->alloc(2, ds, NDUInt32, 0, 0);
            ok = arr[ES_ADDR_STATUS] != 0;
        }
    }
    /* the reserve, again, on what was actually allocated: the pool reuses free
     * buffers up to 1.5 times the size asked for, and charges their whole size */
    if (ok && pool_->inUse() + pluginReserve(capacity_) > maxMemory_)
        ok = false;
    if (ok) {                                 /* the frame's attribute list, built once (3.1) */
        DriverLock lk(this);
        scratch_->clear();
        this->getAttributes(scratch_);
    }
    if (ok) {
        for (size_t i = 0; i < sizeof reservedNames / sizeof reservedNames[0]; ++i)
            if (scratch_->find(reservedNames[i])) {
                scratch_->remove(reservedNames[i]);
                if (warned_.insert(reservedNames[i]).second)
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,      /* once per name */
                              "%s: site attribute \"%s\" removed: the name is a frame attribute\n",
                              driverName, reservedNames[i]);
            }
        addFrameAttributes(scratch_, f ? valuesOf(*f) : valuesOf(*run, index));
    }
    if (!ok) {
        ++failedFrames_;
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: frame %llu: arrays could not be allocated (in use %.1f MB, need %.1f MB, budget %.1f MB)\n",
                  driverName, (unsigned long long)(f ? f->number : run->firstNumber + index),
                  pool_->inUse() / 1e6, need / 1e6, maxMemory_ / 1e6);
        return false;
    }

    /* data: main rows by motor number, per-axis arrays, optional arrays */
    double *mainData = (double *)arr[ES_ADDR_MAIN]->pData;
    for (size_t s = 0; s < S; ++s)
        for (int m = 0; m < 8; ++m)
            mainData[s * 8 + m] = (f && column[m] >= 0) ? f->position[(size_t)column[m] * cap + s] : nan;
    for (int m = 0; m < 8; ++m)
        if (arr[m]) {
            double *d = (double *)arr[m]->pData;
            if (f)
                memcpy(d, &f->position[(size_t)column[m] * cap], S * sizeof(double));
            else
                d[0] = nan;
        }
    if (arr[ES_ADDR_INDEX]) {
        epicsUInt64 *d = (epicsUInt64 *)arr[ES_ADDR_INDEX]->pData;
        if (f)
            memcpy(d, &f->tick[0], S * sizeof(epicsUInt64));
        else
            d[0] = 0;
    }
    if (arr[ES_ADDR_STATUS]) {
        epicsUInt32 *d = (epicsUInt32 *)arr[ES_ADDR_STATUS]->pData;
        for (size_t s = 0; s < S; ++s)
            for (unsigned k = 0; k < nStatus; ++k)
                d[s * nStatus + k] = f ? f->status[(size_t)k * cap + s] : 0;
    }

    /* identity and time, shared by every array of the frame (3, R10) */
    uniqueId_ = uniqueId_ >= INT_MAX ? 1 : uniqueId_ + 1;
    NDArray *first = arr[ES_ADDR_MAIN];
    updateTimeStamps(first);
    for (int i = 0; i < ES_NUM_ADDR; ++i)
        if (arr[i]) {
            arr[i]->uniqueId = uniqueId_;
            arr[i]->epicsTS = first->epicsTS;
            arr[i]->timeStamp = first->timeStamp;
            arr[i]->pAttributeList->clear();
            scratch_->copy(arr[i]->pAttributeList);
        }

    /* Plugin callbacks must not throw (as ADCore's do not): an exception inside asyn's callback
     * loop would leave its interrupt list locked. If one escapes anyway, the frame is counted
     * as failed (WorkGuard) and logged as partly published: some addresses may have had it. */
    unsigned called = 0;
    struct Partial {
        const unsigned &n;
        const ADEncstream *d;
        uint64_t number;
        bool done;
        ~Partial()
        {
            if (!done && n)
                printf("%s %s: frame %llu partly published (%u addresses) when an exception occurred\n",
                       driverName, d->portName, (unsigned long long)number, n);
        }
    } partial = { called, this, f ? f->number : run->firstNumber + index, false };
    {
    DriverLock lk(this);
    for (int i = 0; i < ES_NUM_ADDR; ++i) {
        if (!arr[i])
            continue;
        NDArrayInfo_t info;
        arr[i]->getInfo(&info);
        int counter;
        getIntegerParam(i, NDArrayCounter, &counter);
        setIntegerParam(i, NDArrayCounter, counter >= INT_MAX ? 0 : counter + 1);   /* wraps */
        setIntegerParam(i, NDArraySizeX, (int)arr[i]->dims[0].size);
        setIntegerParam(i, NDArraySizeY, arr[i]->ndims > 1 ? (int)arr[i]->dims[1].size : 0);
        setIntegerParam(i, NDArraySize, (int)info.totalBytes);
        setIntegerParam(i, NDNDimensions, arr[i]->ndims);
        setIntegerParam(i, NDDataType, arr[i]->dataType);
        setIntegerParam(i, NDUniqueId, uniqueId_);
        setDoubleParam(i, NDTimeStamp, arr[i]->timeStamp);
        setIntegerParam(i, NDEpicsTSSec, arr[i]->epicsTS.secPastEpoch);
        setIntegerParam(i, NDEpicsTSNsec, arr[i]->epicsTS.nsec);
        ++called;
        doCallbacksGenericPointer(arr[i], NDArrayData, i);
        callParamCallbacks(i);
    }
    }
    partial.done = true;
    ++publishedFrames_;
    return true;
}

/* ---- shutdown and report --------------------------------------------------------------------- */

/* epicsAtExit, once: stop both threads, and only when both have finished close
 * the socket and release the socket subsystem. The port, its parameters and the NDArray pool
 * stay to process exit, as for every areaDetector driver: plugins may still release arrays into
 * the pool while the IOC exits. */
void ADEncstream::shutdown()
{
    if (epicsAtomicCmpAndSwapIntT(&shutdownDone_, 0, 1) != 0)
        return;
    p_->commands.close("the driver is shutting down");
    if (refusalTimer_)
        epicsTimerCancel(refusalTimer_);      /* waits for a running step; never under the driver lock */
    epicsAtomicSetIntT(&exiting_, 1);
    p_->wakeCb.trigger();
    bool rxOk = !rxStarted_ || rxDone_->wait(2.0);
    if (rxOk)
        rxStarted_ = false;
    bool cbOk = !cbStarted_ || cbDone_->wait(5.0);
    if (cbOk)
        cbStarted_ = false;
    if (!rxOk || !cbOk) {
        printf("%s %s: shutdown: the %s thread did not finish; the socket is left open\n", driverName, portName,
               !rxOk ? "receiver" : "callback");
        epicsAtomicSetIntT(&shutdownDone_, 0);   /* a later call may try again */
        return;
    }
    p_->queue.clear();
    releaseSocket();
}

void ADEncstream::report(FILE *fp, int details)
{
    fprintf(fp, "ADEncstream %s: pool %u samples x %u + 2 frames, NDArrays %.1f/%.1f MB in use%s\n",
            portName, capacity_, p_->queue.depth(), pool_ ? pool_->inUse() / 1e6 : 0.0, maxMemory_ / 1e6,
            paramsReady_ ? "" : " (failed to initialise)");
    if (details > 0) {
        fprintf(fp, "  frames published %llu, failed %llu; stale monitor updates %llu\n",
                (unsigned long long)publishedFrames_, (unsigned long long)failedFrames_,
                (unsigned long long)staleMonitor_);
        fprintf(fp, "  commands cancelled %llu, late results %llu; stale ledger reports %llu\n",
                (unsigned long long)p_->commands.cancelled(), (unsigned long long)p_->commands.late(),
                (unsigned long long)p_->queue.staleReports());
    }
    asynNDArrayDriver::report(fp, details);
}

/* ---- iocsh ----------------------------------------------------------------------------------- */

extern "C" int ADEncstreamConfig(const char *portName, const char *brick, int localPort,
                                 const char *bindAddress, const char *identification,
                                 int maxFrameSamples, int queueDepth, double maxMemory, int priority,
                                 int stackSize)
{
    ADEncstreamArgs a;
    a.portName = portName ? portName : "";
    a.brick = brick ? brick : "";
    a.localPort = localPort;
    a.bindAddress = bindAddress ? bindAddress : "";
    a.identification = identification ? identification : "";
    a.maxFrameSamples = maxFrameSamples > 0 ? (unsigned)maxFrameSamples : 0;
    a.queueDepth = queueDepth > 0 ? (unsigned)queueDepth : 0;
    a.maxMemory = maxMemory > 0 && maxMemory < 1e15 ? (size_t)maxMemory : 0;
    a.priority = priority;
    a.stackSize = stackSize;
    return ADEncstream::configure(a);
}

static const iocshArg configArg0 = { "portName", iocshArgString };
static const iocshArg configArg1 = { "brick (host[:port])", iocshArgString };
static const iocshArg configArg2 = { "localPort", iocshArgInt };
static const iocshArg configArg3 = { "bindAddress", iocshArgString };
static const iocshArg configArg4 = { "identification", iocshArgString };
static const iocshArg configArg5 = { "maxFrameSamples", iocshArgInt };
static const iocshArg configArg6 = { "queueDepth", iocshArgInt };
static const iocshArg configArg7 = { "maxMemory", iocshArgDouble };
static const iocshArg configArg8 = { "priority", iocshArgInt };
static const iocshArg configArg9 = { "stackSize", iocshArgInt };
static const iocshArg *const configArgs[] = { &configArg0, &configArg1, &configArg2, &configArg3, &configArg4,
                                              &configArg5, &configArg6, &configArg7, &configArg8, &configArg9 };
static const iocshFuncDef configFuncDef = { "ADEncstreamConfig", 10, configArgs };

static void configCallFunc(const iocshArgBuf *args)
{
    ADEncstreamConfig(args[0].sval, args[1].sval, args[2].ival, args[3].sval, args[4].sval, args[5].ival,
                      args[6].ival, args[7].dval, args[8].ival, args[9].ival);
}

static void ADEncstreamRegister(void)
{
    iocshRegister(&configFuncDef, configCallFunc);
}

extern "C" {
epicsExportRegistrar(ADEncstreamRegister);
}
