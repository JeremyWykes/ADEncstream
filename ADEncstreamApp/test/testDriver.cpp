/* testDriver.cpp - the areaDetector driver itself (ADEncstream.cpp). Drivers are configured on loopback ports; the publication functions are called
 * directly with the workers stopped, or the workers run where the case needs them. Needs asyn
 * and ADCore; binds UDP ports 45301-45312 on 127.0.0.1. */
#include <limits.h>
#include <stdio.h>
#include <string.h>

#include <stdexcept>
#include <string>
#include <vector>

#include <alarm.h>
#include <asynPortDriver.h>
#include <epicsThread.h>
#include <epicsUnitTest.h>
#include <testMain.h>

#include "ADEncstream.h"
#include "ADEncstreamParts.h"

using namespace encstream;

/* the driver's private parts, for the tests only (declared a friend in ADEncstream.h) */
class ADEncstreamTestAccess {
public:
    explicit ADEncstreamTestAccess(ADEncstream *d) : d(d) {}
    ADEncstream *d;
    esNDArrayPool *pool() { return d->pool_; }
    size_t maxMemory() { return d->maxMemory_; }
    Mailbox &mailbox() { return d->p_->mailbox; }
    CommandSlots &commands() { return d->p_->commands; }
    EngineConfig engineConfig() { return d->p_->cfg; }
    bool socketOpen() { return d->p_->sock != INVALID_SOCKET; }
    bool rxStarted() { return d->rxStarted_; }
    bool cbStarted() { return d->cbStarted_; }
    bool faulted() { return d->faulted(); }
    FrameQueue &queue() { return d->p_->queue; }
    void publishWork(const Work &w) { d->publishWork(w); }
    void stopWorkers() { d->shutdown(); }
    bool publishFrame(const Frame *f) { return d->publishFrame(f, 0, 0); }
    void publishMonitor(const MonitorUpdate &u, uint64_t seq) { d->publishMonitor(u, seq); }
    void publishSnapshot(const Snapshot &s, uint64_t seq) { d->publishSnapshot(s, seq); }
    void driverFault(const char *why) { d->driverFault(why); }
    void incQueued() { d->incrementQueuedArrayCount(); }
    void decQueued() { d->decrementQueuedArrayCount(); }
    uint64_t &monitorWriteRevision() { return d->monitorWriteRevision_; }
    uint64_t &controlRevision() { return d->controlRevision_; }
    int esParam(const char *name)
    {
        int i = -1;
        d->findParam(name, &i);
        return i;
    }
    int getInt(int addr, int param)
    {
        int v = -12345;
        d->lock();
        d->getIntegerParam(addr, param, &v);
        d->unlock();
        return v;
    }
    double getDouble(int param)
    {
        double v = -1;
        d->lock();
        d->getDoubleParam(param, &v);
        d->unlock();
        return v;
    }
    void setInt(int addr, int param, int v)
    {
        d->lock();
        d->setIntegerParam(addr, param, v);
        d->unlock();
    }
    int severity(int addr, int param)
    {
        int s = -1;
        d->lock();
        d->getParamAlarmSeverity(addr, param, &s);
        d->unlock();
        return s;
    }
};

namespace {

ADEncstreamArgs args(const char *port, int localPort, unsigned capacity, size_t maxMemory)
{
    ADEncstreamArgs a;
    a.portName = port;
    a.brick = "127.0.0.1:45399";
    a.bindAddress = "127.0.0.1";
    a.identification = "testDriver";
    a.localPort = localPort;
    a.maxFrameSamples = capacity;
    a.queueDepth = 2;
    a.maxMemory = maxMemory;
    a.priority = 0;
    a.stackSize = 0;
    return a;
}

/* the reserve and minimum budget as the driver computes them */
size_t reserveFor(unsigned capacity) { return (1u << 20) + (size_t)96 * capacity; }
size_t minimumFor(unsigned capacity) { return reserveFor(capacity) + (size_t)172 * capacity; }

ADEncstream *configured(const ADEncstreamArgs &a, int *rc = 0)
{
    int r = ADEncstream::configure(a);
    if (rc)
        *rc = r;
    return dynamic_cast<ADEncstream *>(static_cast<asynPortDriver *>(findAsynPortDriver(a.portName.c_str())));
}

/* a plugin that, at its callback, records the pool's bytes in use and asks for the eight
 * std-arrays conversions the reserve covers */
struct Consumer {
    ADEncstreamTestAccess *t;
    size_t samples;
    size_t observed;
    unsigned conversions;
    std::vector<NDArray *> held;
};

void consume(void *arg, asynUser *, void *)
{
    Consumer &c = *(Consumer *)arg;
    c.observed = c.t->pool()->inUse();
    for (unsigned i = 0; i < 8; ++i) {
        NDArray *a = c.t->pool()->alloc(1, &c.samples, NDFloat64, 0, 0);
        if (!a)
            break;
        c.held.push_back(a);
        ++c.conversions;
    }
}

Frame fullFrame(unsigned capacity, uint32_t samples)
{
    Frame f = Frame();
    f.capacity = capacity;
    f.samples = samples;
    f.number = 1;
    f.epoch = 1;
    f.position.assign((size_t)8 * capacity, 1.0);
    f.tick.assign(capacity, 7);
    f.status.assign((size_t)9 * capacity, 0);
    f.schema.trigger = TRIGGER_FREE_RUNNING;
    f.schema.nAxes = 8;
    for (unsigned i = 0; i < 8; ++i)
        f.schema.motor[i] = (uint8_t)(i + 1);
    f.schema.withSampleIndex = true;
    f.schema.withStatus = true;
    f.schema.nStatus = 9;
    f.schema.servoPeriodS = 1e-4;
    f.schema.gatherPeriod = 1;
    return f;
}

/* R1, R6, R7: one driver, workers stopped */
void testReserveCounterShutdown()
{
    const unsigned cap = 100000;
    ADEncstreamArgs a = args("TD_RESERVE", 45301, cap, minimumFor(cap));
    ADEncstream *d = configured(a);
    testOk(d != 0, "configured at the minimum budget (%.1f MB)", a.maxMemory / 1e6);
    if (!d)
        return;
    ADEncstreamTestAccess t(d);
    t.stopWorkers();
    testOk(!t.socketOpen(), "R6: shutdown closes the socket");
    t.stopWorkers();
    testOk(!t.socketOpen(), "R6: a second shutdown is harmless");

    /* a previous full frame in the free list, and arrays a plugin still holds */
    size_t heldBytes = (size_t)172 * (cap - 70000);
    NDArray *held = t.pool()->alloc(1, &heldBytes, NDUInt8, 0, 0);
    std::vector<NDArray *> previous;
    size_t mainDims[] = { 8, cap }, statusDims[] = { 9, cap }, axis = cap;
    previous.push_back(t.pool()->alloc(2, mainDims, NDFloat64, 0, 0));
    for (int i = 0; i < 9; ++i)
        previous.push_back(t.pool()->alloc(1, &axis, NDFloat64, 0, 0));
    previous.push_back(t.pool()->alloc(2, statusDims, NDUInt32, 0, 0));
    bool primed = held != 0;
    for (size_t i = 0; i < previous.size(); ++i) {
        primed = primed && previous[i] != 0;
        if (previous[i])
            previous[i]->release();
    }
    testOk(primed, "free list primed with a full frame, %zu bytes held", heldBytes);

    Consumer c;
    c.t = &t;
    c.samples = 70000;
    c.observed = 0;
    c.conversions = 0;
    asynUser *u = pasynManager->createAsynUser(0, 0);
    pasynManager->connectDevice(u, a.portName.c_str(), ES_ADDR_MAIN);
    u->reason = t.esParam(NDArrayDataString);
    asynInterface *iface = pasynManager->findInterface(u, asynGenericPointerType, 1);
    void *registration = 0;
    asynGenericPointer *gp = iface ? (asynGenericPointer *)iface->pinterface : 0;
    if (gp)
        gp->registerInterruptUser(iface->drvPvt, u, consume, &c, &registration);

    Frame f = fullFrame(cap, 70000);
    t.setInt(0, t.esParam(NDArrayCallbacksString), 1);
    bool published = t.publishFrame(&f);
    testOk(!published || c.observed + reserveFor(cap) <= t.maxMemory(),
           "R1: a frame is published only with the reserve free (published %d, in use %.2f MB, reserve %.2f MB, "
           "budget %.2f MB)", published, c.observed / 1e6, reserveFor(cap) / 1e6, t.maxMemory() / 1e6);
    testOk(!published || c.conversions == 8, "R1: when published, the 8 conversions fit (%u)", c.conversions);
    for (size_t i = 0; i < c.held.size(); ++i)
        c.held[i]->release();
    c.held.clear();
    if (held)
        held->release();

    /* after the plugin let go, the same frame fits */
    c.conversions = 0;
    published = t.publishFrame(&f);
    testOk(published && c.conversions == 8 && c.observed + reserveFor(cap) <= t.maxMemory(),
           "R1: with the held arrays released the frame is published, conversions fit");
    for (size_t i = 0; i < c.held.size(); ++i)
        c.held[i]->release();
    c.held.clear();
    if (gp)
        gp->cancelInterruptUser(iface->drvPvt, u, registration);
    pasynManager->freeAsynUser(u);

    /* R7: the array counters wrap, at every address */
    const int counter = t.esParam(NDArrayCounterString);
    bool defined = true;
    for (int addr = 0; addr < ES_NUM_ADDR; ++addr)
        defined = defined && t.getInt(addr, counter) != -12345;
    testOk(defined, "R7: array counters defined at every address");
    for (int addr = 0; addr < ES_NUM_ADDR; ++addr)
        t.setInt(addr, counter, INT_MAX);
    f.samples = 1;
    t.publishFrame(&f);
    bool wrapped = true;
    for (int addr = 0; addr < ES_NUM_ADDR; ++addr)
        wrapped = wrapped && t.getInt(addr, counter) == 0;
    testOk(wrapped, "R7: INT_MAX + 1 wraps to 0 on every published address");
}

/* R2 and R4: publication against the mailbox, workers stopped */
void testMonitorOrderAndLateControls()
{
    ADEncstreamArgs a = args("TD_ORDER", 45302, 1000, minimumFor(1000) + (8u << 20));
    ADEncstream *d = configured(a);
    if (!d) {
        testFail("configure");
        return;
    }
    ADEncstreamTestAccess t(d);
    t.stopWorkers();
    const int health = t.esParam(ES_MonHealthString), mean = t.esParam(ES_MeanString);

    MonitorUpdate good = MonitorUpdate();
    good.run = 10;
    good.health = HEALTH_OK;
    good.values = true;
    good.slot[0].mean = 42;
    t.mailbox().postMonitor(good);
    MonitorUpdate taken;
    uint64_t seq = 0;
    t.mailbox().takeMonitor(&taken, &seq);
    MonitorUpdate bad = good;
    bad.health = HEALTH_FAULTED;
    bad.values = false;
    bad.slot[0].severity = SEVERITY_INVALID;
    t.mailbox().postMonitor(bad);             /* the same run: a health-only change */
    t.publishMonitor(taken, seq);
    testOk(t.getInt(0, health) != HEALTH_OK, "R2: the OK update taken before a same-run fault is not published");
    MonitorUpdate latest;
    t.mailbox().takeMonitor(&latest, &seq);
    t.publishMonitor(latest, seq);
    testOk(t.getInt(0, health) == HEALTH_FAULTED && t.severity(0, mean) == epicsSevInvalid,
           "R2: the fault is published, the values INVALID");

    /* R4: a late Monitor Start and a late configuration, through a separate engine */
    epicsEvent wake;
    FrameQueue q(1, 2, wake);
    CommandSlots slots(2);
    Mailbox mb(wake);
    Engine eng(t.engineConfig(), q, slots, mb);
    Command start;
    start.type = CMD_MONITOR_START;
    int id = slots.submit(start);
    CommandResult r;
    eng.service(1000);
    slots.wait(id, 0, &r);
    Command conf;
    conf.type = CMD_MONITOR_CONFIGURE;
    conf.monitor.averagingTimeS = 0.3;
    conf.monitor.updatePeriodS = 0.1;
    id = slots.submit(conf);
    eng.service(2000);
    slots.wait(id, 0, &r);
    Snapshot snap;
    mb.takeSnapshot(&snap);
    t.mailbox().postSnapshot(snap);
    Snapshot mine;
    t.mailbox().takeSnapshot(&mine, &seq);
    const int run = t.esParam(ES_MonRunString), avg = t.esParam(ES_MonAveragingTimeString);
    t.setInt(0, run, 0);
    t.monitorWriteRevision() = 0;
    t.publishSnapshot(mine, seq);
    testOk(t.getInt(0, run) == 1 && t.getDouble(avg) == 0.3,
           "R4: the applied Start and T = 0.3 s reach the control readbacks");
    t.setInt(0, run, 0);
    t.monitorWriteRevision() = mine.revision + 1;   /* a newer write succeeded meanwhile */
    t.mailbox().postSnapshot(mine);
    t.mailbox().takeSnapshot(&mine, &seq);
    t.publishSnapshot(mine, seq);
    testOk(t.getInt(0, run) == 0, "R4: an older snapshot never overwrites a newer write's state");

    /* R4: a late Acquire shows the acquisition running */
    const int acquire = t.esParam(ADAcquireString);
    Snapshot act = Snapshot();
    act.revision = 1000;
    act.active = true;
    act.display = DISPLAY_ACQUIRING;
    t.controlRevision() = 999;
    t.mailbox().postSnapshot(act);
    t.mailbox().takeSnapshot(&act, &seq);
    t.publishSnapshot(act, seq);
    testOk(t.getInt(0, acquire) == 1, "R4: an Acquire applied after its write timed out shows Acquire = 1");
    /* R2: a snapshot superseded before publication is skipped */
    Snapshot older = act;
    older.active = false;
    older.revision = 1001;
    t.mailbox().postSnapshot(older);
    Snapshot o;
    uint64_t oseq;
    t.mailbox().takeSnapshot(&o, &oseq);
    t.mailbox().postSnapshot(act);
    const int state = t.esParam(ES_AcqStateString);
    t.setInt(0, state, 99);
    t.publishSnapshot(o, oseq);
    testOk(t.getInt(0, state) == 99, "R2: a superseded snapshot is not published");
}

/* R3: a fault with WaitForPlugins and a plugin holding an array, workers running */
void testFaultReleasesBusy()
{
    ADEncstreamArgs a = args("TD_FAULT", 45303, 1000, minimumFor(1000) + (8u << 20));
    ADEncstream *d = configured(a);
    if (!d) {
        testFail("configure");
        return;
    }
    ADEncstreamTestAccess t(d);
    const int acquire = t.esParam(ADAcquireString), busy = t.esParam(ADAcquireBusyString);
    const int wait = t.esParam(ADWaitForPluginsString), fault = t.esParam(ES_DriverFaultString);
    t.setInt(0, wait, 1);
    t.incQueued();                            /* a plugin holding an array */
    t.setInt(0, acquire, 1);
    t.driverFault("injected permanent receiver failure");
    epicsThreadSleep(0.3);
    testOk(t.getInt(0, fault) == 1 && t.getInt(0, acquire) == 0 && t.getInt(0, busy) == 0,
           "R3: fault with WaitForPlugins and a held array: Acquire 0, Busy 0 (busy %d)", t.getInt(0, busy));
    testOk(t.getInt(0, t.esParam(ES_AcqStateString)) == DISPLAY_INVALID &&
           t.getInt(0, t.esParam(ES_MonHealthString)) == HEALTH_FAULTED &&
           t.severity(0, t.esParam(ES_MeanString)) == epicsSevInvalid,
           "R3: acquisition invalid, monitor faulted, monitor values INVALID");
    Command c;
    c.type = CMD_STOP;
    testOk(t.commands().submit(c) < 0, "R3: commands are refused after the fault");
    MonitorUpdate ok = MonitorUpdate();
    ok.health = HEALTH_OK;
    t.mailbox().postMonitor(ok);
    epicsThreadSleep(0.2);
    testOk(t.getInt(0, t.esParam(ES_MonHealthString)) == HEALTH_FAULTED, "R3: later updates never undo the fault");
    t.decQueued();
    t.stopWorkers();
}

/* R5: a thread that cannot be created */
const char *failName = 0;
epicsThreadId failingCreate(const char *name, unsigned prio, unsigned stack, EPICSTHREADFUNC fn, void *arg)
{
    if (failName && strcmp(name, failName) == 0)
        return 0;
    return epicsThreadCreate(name, prio, stack, fn, arg);
}

void testStartupFailure()
{
    ADEncstream::ThreadCreate original = ADEncstream::createThread;
    ADEncstream::createThread = failingCreate;
    const char *names[] = { "ESCallback", "ESReceiver" };
    const char *ports[] = { "TD_NOCB", "TD_NORX" };
    for (int k = 0; k < 2; ++k) {
        failName = names[k];
        int rc = 0;
        ADEncstream *d = configured(args(ports[k], 45304 + k, 1000, minimumFor(1000) + (8u << 20)), &rc);
        ADEncstreamTestAccess t(d);
        testOk(rc == -1 && d && t.getInt(0, t.esParam(ES_DriverFaultString)) == 1 && !t.cbStarted() &&
               !t.rxStarted(), "R5: %s not created: configure fails, the port shows the fault, no thread runs",
               names[k]);
        Command c;
        c.type = CMD_STOP;
        testOk(d && t.commands().submit(c) < 0, "R5: and refuses commands");
        ADEncstream::createThread = original;
        char again[32];
        sprintf(again, "%s_AGAIN", ports[k]);
        ADEncstream *r = configured(args(again, 45304 + k, 1000, minimumFor(1000) + (8u << 20)), &rc);
        testOk(rc == 0 && r, "F5: its UDP port is released at once: a new port binds it");
        if (r)
            ADEncstreamTestAccess(r).stopWorkers();
        ADEncstream::createThread = failingCreate;
        if (d)
            t.stopWorkers();
    }
    ADEncstream::createThread = original;
    failName = 0;
}

/* F2: plugin conversions served by oversized free buffers still fit in the reserve */
void testReserveWithReusedConversions()
{
    const unsigned cap = 100000;
    ADEncstreamArgs a = args("TD_REUSE", 45306, cap, minimumFor(cap));
    ADEncstream *d = configured(a);
    if (!d) {
        testFail("configure");
        return;
    }
    ADEncstreamTestAccess t(d);
    t.stopWorkers();
    /* a plugin holds 2 MB: with a reserve sized on the conversions' logical size (7.4 MB) the
     * frame would pass, and its conversions, served by the 1.2 MB buffers, would not fit */
    size_t heldBytes = 2000000;
    NDArray *held = t.pool()->alloc(1, &heldBytes, NDUInt8, 0, 0);
    /* the free list: one frame's exact source buffers, and six old main arrays of 1.2 MB, which
     * the 0.8 MB conversions may reuse (within ADCore's 1.5 ratio) */
    std::vector<NDArray *> prime;
    size_t mainDims[] = { 8, cap }, statusDims[] = { 9, cap }, axis = cap, oldMain[] = { 8, 18750 };
    prime.push_back(t.pool()->alloc(2, mainDims, NDFloat64, 0, 0));
    for (int i = 0; i < 9; ++i)
        prime.push_back(t.pool()->alloc(1, &axis, NDFloat64, 0, 0));
    prime.push_back(t.pool()->alloc(2, statusDims, NDUInt32, 0, 0));
    for (int i = 0; i < 6; ++i)
        prime.push_back(t.pool()->alloc(2, oldMain, NDFloat64, 0, 0));
    bool primed = true;
    for (size_t i = 0; i < prime.size(); ++i) {
        primed = primed && prime[i] != 0;
        if (prime[i])
            prime[i]->release();
    }
    testOk(primed && held, "F2: 2 MB held, free list primed with source buffers and six 1.2 MB buffers");

    Consumer c;
    c.t = &t;
    c.samples = cap;                          /* 0.8 MB conversions */
    c.observed = 0;
    c.conversions = 0;
    asynUser *u = pasynManager->createAsynUser(0, 0);
    pasynManager->connectDevice(u, a.portName.c_str(), ES_ADDR_MAIN);
    u->reason = t.esParam(NDArrayDataString);
    asynInterface *iface = pasynManager->findInterface(u, asynGenericPointerType, 1);
    void *registration = 0;
    asynGenericPointer *gp = (asynGenericPointer *)iface->pinterface;
    gp->registerInterruptUser(iface->drvPvt, u, consume, &c, &registration);
    Frame f = fullFrame(cap, cap);
    t.setInt(0, t.esParam(NDArrayCallbacksString), 1);
    bool published = t.publishFrame(&f);
    testOk(!published || c.conversions == 8,
           "F2: published %d; with reused 1.2 MB buffers all 8 conversions fit (%u)", published, c.conversions);
    for (size_t i = 0; i < c.held.size(); ++i)
        c.held[i]->release();
    if (held)
        held->release();
    gp->cancelInterruptUser(iface->drvPvt, u, registration);
    pasynManager->freeAsynUser(u);
}

/* F4: an exception during publication settles the work and releases the driver's arrays */
void throwingConsume(void *, asynUser *, void *)
{
    throw std::runtime_error("injected callback exception");
}

void testExceptionSettlesWork()
{
    ADEncstreamArgs a = args("TD_THROW", 45307, 1000, minimumFor(1000) + (8u << 20));
    ADEncstream *d = configured(a);
    if (!d) {
        testFail("configure");
        return;
    }
    ADEncstreamTestAccess t(d);
    t.stopWorkers();
    asynUser *u = pasynManager->createAsynUser(0, 0);
    pasynManager->connectDevice(u, a.portName.c_str(), 0);       /* the first axis */
    u->reason = t.esParam(NDArrayDataString);
    asynInterface *iface = pasynManager->findInterface(u, asynGenericPointerType, 1);
    void *registration = 0;
    asynGenericPointer *gp = (asynGenericPointer *)iface->pinterface;
    gp->registerInterruptUser(iface->drvPvt, u, throwingConsume, 0, &registration);
    /* (the registration is left: the exception left asyn's interrupt list locked, which is why
     * plugin callbacks must not throw; this port is not used again) */

    t.setInt(0, t.esParam(NDArrayCallbacksString), 1);
    Frame *f = t.queue().acquire();
    Frame tmpl = fullFrame(1000, 500);
    f->epoch = 7;
    f->number = 1;
    f->samples = 500;
    f->schema = tmpl.schema;
    t.queue().push(f);
    Work w;
    t.queue().take(&w);
    bool threw = false;
    try {
        t.publishWork(w);
    } catch (std::exception &) {
        threw = true;
    }
    unsigned entries;
    uint64_t frames;
    t.queue().backlog(&entries, &frames);
    LedgerDelta ld = t.queue().read(7);
    testOk(threw && t.pool()->inUse() == 0, "F4: after the exception the driver holds no NDArray bytes (%zu)",
           t.pool()->inUse());
    testOk(entries == 0 && frames == 0 && !t.queue().outstanding(7) && ld.failed == 1 && ld.published == 0,
           "F4: the work was settled once, as 1 failed frame, nothing in flight");
    Frame *again[4];
    int n = 0;
    while (n < 4 && (again[n] = t.queue().acquire()) != 0)
        ++n;
    testOk(n == 4, "F4: the frame object is back in the pool (all Q + 2 = 4 free)");
}

/* F1 and F5: a failed initialisation leaves a live faulted port and releases the socket */
void testInitFailure()
{
    const char *stage[] = { "events", "pool", "parameters" };
    for (int k = 1; k <= 3; ++k) {
        char port[32];
        sprintf(port, "TD_INIT%d", k);
        ADEncstream::failInitStage = k;
        int rc = 0;
        ADEncstream *d = configured(args(port, 45308, 1000, minimumFor(1000) + (8u << 20)), &rc);
        ADEncstream::failInitStage = 0;
        ADEncstreamTestAccess t(d);
        /* the registered object must be alive: these calls go through it (ASan would object) */
#ifdef _WIN32
        FILE *null = fopen("NUL", "w");
#else
        FILE *null = fopen("/dev/null", "w");
#endif
        if (d && null)
            d->report(null, 1);
        if (null)
            fclose(null);
        asynUser *u = pasynManager->createAsynUser(0, 0);
        asynStatus st = asynError;
        if (pasynManager->connectDevice(u, port, 0) == asynSuccess) {
            asynInterface *ifc = pasynManager->findInterface(u, asynInt32Type, 1);
            if (ifc) {
                u->reason = t.esParam(ADAcquireString);
                st = ((asynInt32 *)ifc->pinterface)->write(ifc->drvPvt, u, 1);
            }
        }
        pasynManager->freeAsynUser(u);
        testOk(rc == -1 && d && t.faulted() && st != asynSuccess,
               "F1: init failing at the %s: configure fails, the port is alive, faulted, refuses writes", stage[k - 1]);
        /* F5: the same UDP endpoint can be bound again at once */
        sprintf(port, "TD_REBIND%d", k);
        ADEncstream *r = configured(args(port, 45308, 1000, minimumFor(1000) + (8u << 20)), &rc);
        testOk(rc == 0 && r, "F5: the UDP port is released: a new port binds it again");
        if (r)
            ADEncstreamTestAccess(r).stopWorkers();
    }
}

}

MAIN(testDriver)
{
    testPlan(0);
    testReserveCounterShutdown();
    testReserveWithReusedConversions();
    testMonitorOrderAndLateControls();
    testFaultReleasesBusy();
    testStartupFailure();
    testInitFailure();
    testExceptionSettlesWork();
    return testDone();
}
