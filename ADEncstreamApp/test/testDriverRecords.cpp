/* testDriverRecords.cpp - the driver with the real areaDetector records, in an in-process IOC
 * (dbUnitTest): a refused Acquire must finish its put callback and leave the Acquire and busy
 * records at Done, whatever state the driver's threads are in. A parameter-level test
 * cannot see this: the busy record is set by the Acquire record's forward link (SetAcquireBusy)
 * and cleared only by a 1 -> 0 transition of AcquireBusyCB (ClearAcquireBusy, a CP link).
 * Needs asyn, busy and ADCore's NDArrayBase.template; binds UDP ports 45421-45425 on 127.0.0.1. */
#define USE_TYPED_RSET                        /* dbAccess.h without the deprecated rset */
#include <stdio.h>
#include <string.h>

#include <string>

#include <asynPortDriver.h>
#include <dbAccess.h>
#include <dbChannel.h>
#include <dbNotify.h>
#include <dbUnitTest.h>
#include <epicsEvent.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsUnitTest.h>
#include <testMain.h>

#include "ADEncstream.h"
#include "ADEncstreamParts.h"

extern "C" int testDriverRecords_registerRecordDeviceDriver(struct dbBase *pdbbase);

/* the driver's private parts, for the tests only (declared a friend in ADEncstream.h) */
class ADEncstreamTestAccess {
public:
    static bool faulted(ADEncstream *d) { return d->faulted(); }
    static bool paramsReady(ADEncstream *d) { return d->paramsReady_; }
    static void driverFault(ADEncstream *d, const char *why) { d->driverFault(why); }
    static int acquireParam(ADEncstream *d)
    {
        int v = -1;
        d->lock();
        d->getIntegerParam(d->ADAcquire, &v);
        d->unlock();
        return v;
    }
    /* what callbackTask() leaves behind when its loop throws: the fault applied, the thread gone */
    static void dieLikeTheCallbackThread(ADEncstream *d)
    {
        d->driverFault("callback thread: injected exception");
        d->lock();
        if (!d->faultApplied_)
            d->applyFault("callback thread: injected exception");
        d->unlock();
        d->cbDone_->trigger();
    }
};

namespace {

ADEncstreamArgs args(const char *port, int localPort)
{
    ADEncstreamArgs a;
    a.portName = port;
    a.brick = "127.0.0.1:45429";              /* nothing answers: no stream, Acquire is refused */
    a.bindAddress = "127.0.0.1";
    a.identification = "testDriverRecords";
    a.localPort = localPort;
    a.maxFrameSamples = 1000;
    a.queueDepth = 2;
    a.maxMemory = (8u << 20) + (1u << 20) + 268 * 1000;
    a.priority = 0;
    a.stackSize = 0;
    return a;
}

ADEncstream *configured(const ADEncstreamArgs &a)
{
    ADEncstream::configure(a);
    return dynamic_cast<ADEncstream *>(static_cast<asynPortDriver *>(findAsynPortDriver(a.portName.c_str())));
}

/* the dead callback worker: its thread starts, leaves the state of an escaped exception, ends */
void dyingCallback(void *arg)
{
    ADEncstreamTestAccess::dieLikeTheCallbackThread((ADEncstream *)arg);
}

const char *failName;
bool dyingWorker;
ADEncstream *dyingDriver;

epicsThreadId substituteCreate(const char *name, unsigned prio, unsigned stack, EPICSTHREADFUNC fn, void *arg)
{
    if (failName && strcmp(name, failName) == 0)
        return 0;
    if (dyingWorker && strcmp(name, "ESCallback") == 0) {
        dyingDriver = (ADEncstream *)arg;
        return epicsThreadCreate(name, prio, stack, dyingCallback, arg);
    }
    return epicsThreadCreate(name, prio, stack, fn, arg);
}

/* ---- a put with completion (as caput -c): dbProcessNotify on the record ---------------------- */

struct PutRequest {
    processNotify pn;
    epicsEvent done;
    epicsInt32 value;
    notifyStatus status;
};

int putCallback(processNotify *pn, notifyPutType type)
{
    PutRequest *r = (PutRequest *)pn->usrPvt;
    if (pn->status == notifyCanceled)
        return 0;
    long st = 0;
    switch (type) {
    case putDisabledType:
        pn->status = notifyError;
        return 0;
    case putFieldType:
        st = dbChannelPutField(pn->chan, DBR_LONG, &r->value, 1);
        break;
    case putType:
        st = dbChannelPut(pn->chan, DBR_LONG, &r->value, 1);
        break;
    }
    if (st)
        pn->status = notifyError;
    return 1;
}

void doneCallback(processNotify *pn)
{
    PutRequest *r = (PutRequest *)pn->usrPvt;
    r->status = pn->status;
    r->done.signal();
}

/* returns true if the put's processing (forward links and the busy record included) completed */
bool putWithCompletion(const char *pv, epicsInt32 value, double timeout)
{
    PutRequest r;
    memset(&r.pn, 0, sizeof r.pn);
    r.value = value;
    r.status = notifyError;
    r.pn.requestType = putProcessRequest;
    r.pn.chan = dbChannelCreate(pv);
    if (!r.pn.chan || dbChannelOpen(r.pn.chan)) {
        testDiag("cannot open %s", pv);
        return false;
    }
    r.pn.putCallback = putCallback;
    r.pn.doneCallback = doneCallback;
    r.pn.usrPvt = &r;
    dbProcessNotify(&r.pn);
    bool done = r.done.wait(timeout);
    /* always: dbNotify still uses the request after doneCallback (notifyCleanup), and returns from
     * this only once it no longer does; without it a completed put races with this frame's end */
    dbNotifyCancel(&r.pn);
    dbChannelDelete(r.pn.chan);
    return done;
}

long getLong(const char *pv)
{
    DBADDR addr;
    epicsInt32 v = -1;
    long n = 1;
    if (dbNameToAddr(pv, &addr) || dbGetField(&addr, DBR_LONG, &v, 0, &n, 0))
        return -1;
    return v;
}

/* the four records of the Acquire/busy handshake, waited for until all read 0 (Done) */
bool recordsDone(const std::string &p, double timeout)
{
    const char *names[] = { "Acquire", "Acquire_RBV", "AcquireBusy", "AcquireBusyCB" };
    epicsTimeStamp t0;
    epicsTimeGetCurrent(&t0);
    for (;;) {
        bool all = true;
        for (unsigned k = 0; k < 4; ++k)
            all = all && getLong((p + names[k]).c_str()) == 0;
        epicsTimeStamp now;
        epicsTimeGetCurrent(&now);
        if (all || epicsTimeDiffInSeconds(&now, &t0) > timeout) {
            for (unsigned k = 0; k < 4; ++k)
                testDiag("  %s%s = %ld", p.c_str(), names[k], getLong((p + names[k]).c_str()));
            return all;
        }
        epicsThreadSleep(0.02);
    }
}

struct Case {
    const char *label;
    const char *prefix;
    const char *port;
    int localPort;
    bool expectFault;
    ADEncstream *d;
};

void refusedAcquire(Case &c, int waitForPlugins)
{
    const std::string p = c.prefix;
    if (waitForPlugins) {
        long st = 0;
        {
            DBADDR addr;
            epicsInt32 v = 1;
            st = dbNameToAddr((p + "WaitForPlugins").c_str(), &addr) ||
                 dbPutField(&addr, DBR_LONG, &v, 1);
        }
        /* refused when init() failed (every write is): then the run repeats the plain case */
        testDiag("%s: WaitForPlugins = 1 written (%s)", c.label, st ? "failed" : "ok");
        epicsThreadSleep(0.1);
    }
    bool completed = putWithCompletion((p + "Acquire").c_str(), 1, 2.0);
    testOk(completed, "%s, WaitForPlugins %d: the Acquire put callback completes", c.label, waitForPlugins);
    testOk(recordsDone(p, 1.0), "%s, WaitForPlugins %d: Acquire, Acquire_RBV, AcquireBusy, AcquireBusyCB all Done",
           c.label, waitForPlugins);
    testOk(c.d && ADEncstreamTestAccess::acquireParam(c.d) == 0 &&
               ADEncstreamTestAccess::faulted(c.d) == c.expectFault,
           "%s, WaitForPlugins %d: no acquisition started; driver %s", c.label, waitForPlugins,
           c.expectFault ? "still faulted" : "not faulted");
    /* a second attempt must go through the same cycle (the Acquire record is back at 0, so its
     * link sets the busy record again) */
    testOk(putWithCompletion((p + "Acquire").c_str(), 1, 2.0) && recordsDone(p, 1.0),
           "%s, WaitForPlugins %d: a second refused Acquire completes too", c.label, waitForPlugins);
}

}  // namespace

MAIN(testDriverRecords)
{
    testPlan(5 * 2 * 4 + 5);
    testdbPrepare();
    testdbReadDatabase("testDriverRecords.dbd", 0, 0);
    testDriverRecords_registerRecordDeviceDriver(pdbbase);

    Case cases[] = {
        { "healthy driver, Acquire refused (no stream)", "TDR1:", "TDR_OK", 45421, false, 0 },
        { "terminal receiver fault, callback worker alive", "TDR2:", "TDR_FAULT", 45422, true, 0 },
        { "callback thread never started", "TDR3:", "TDR_NOCB", 45423, true, 0 },
        { "callback worker died", "TDR4:", "TDR_DEAD", 45424, true, 0 },
        { "init() failed (parameters)", "TDR5:", "TDR_INIT", 45425, true, 0 },
    };
    ADEncstream::ThreadCreate original = ADEncstream::createThread;
    ADEncstream::createThread = substituteCreate;
    cases[0].d = configured(args(cases[0].port, cases[0].localPort));
    cases[1].d = configured(args(cases[1].port, cases[1].localPort));
    failName = "ESCallback";
    cases[2].d = configured(args(cases[2].port, cases[2].localPort));
    failName = 0;
    dyingWorker = true;
    cases[3].d = configured(args(cases[3].port, cases[3].localPort));
    dyingWorker = false;
    ADEncstream::failInitStage = 3;
    cases[4].d = configured(args(cases[4].port, cases[4].localPort));
    ADEncstream::failInitStage = 0;
    ADEncstream::createThread = original;

    for (unsigned k = 0; k < 5; ++k) {
        std::string macros = std::string("P=") + cases[k].prefix + ",R=,PORT=" + cases[k].port +
                             ",ADDR=0,TIMEOUT=1";
        testdbReadDatabase(ES_ADCORE_DB "/NDArrayBase.template", 0, macros.c_str());
    }
    testIocInitOk();

    /* the live worker's terminal fault: injected after iocInit, applied by the callback thread */
    if (cases[1].d)
        ADEncstreamTestAccess::driverFault(cases[1].d, "test: injected receiver failure");
    epicsThreadSleep(0.3);

    testOk(cases[0].d && !ADEncstreamTestAccess::faulted(cases[0].d), "setup: %s", cases[0].label);
    testOk(cases[1].d && ADEncstreamTestAccess::faulted(cases[1].d), "setup: %s", cases[1].label);
    testOk(cases[2].d && ADEncstreamTestAccess::faulted(cases[2].d), "setup: %s", cases[2].label);
    testOk(cases[3].d && dyingDriver == cases[3].d && ADEncstreamTestAccess::faulted(cases[3].d),
           "setup: %s", cases[3].label);
    testOk(cases[4].d && !ADEncstreamTestAccess::paramsReady(cases[4].d), "setup: %s", cases[4].label);

    for (unsigned k = 0; k < 5; ++k)
        for (int wfp = 0; wfp <= 1; ++wfp)
            refusedAcquire(cases[k], wfp);

    testIocShutdownOk();
    testdbCleanup();
    return testDone();
}
