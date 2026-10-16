/* testDriverParts.cpp - the driver's thread-crossing parts: the frame queue and publication
 * ledger, the command slots (5.1, R2) and the mailbox's run gate
 * (5.3, R5). They use EPICS libCom only, so they are tested here without asyn. */
#include <string.h>

#include <vector>

#include <epicsEvent.h>
#include <epicsThread.h>
#include <epicsUnitTest.h>
#include <testMain.h>

#include "esCommands.h"
#include "esEngine.h"
#include "esFrameQueue.h"

using namespace encstream;

namespace {

EmptyRun run(uint64_t epoch, uint64_t first, uint64_t count)
{
    EmptyRun r;
    memset(&r, 0, sizeof r);
    r.epoch = epoch;
    r.firstNumber = first;
    r.count = count;
    r.firstWindowStart = 1000;
    r.windowLength = 10;
    return r;
}

Frame *frame(FrameQueue &q, uint64_t epoch, uint64_t number)
{
    Frame *f = q.acquire();
    if (f) {
        f->epoch = epoch;
        f->number = number;
    }
    return f;
}

/* ---- frame queue and ledger ------------------------------------------------------------------ */

void testPoolAndQueue()
{
    epicsEvent wake;
    FrameQueue q(100, 2, wake);
    testOk(q.capacity() == 100 && q.depth() == 2, "capacity and depth");
    Frame *f[4];
    for (int i = 0; i < 4; ++i)
        f[i] = q.acquire();
    testOk(f[0] && f[1] && f[2] && f[3] && !q.acquire(), "Q + 2 = 4 objects, then none");
    testOk(f[0]->position.size() == 8 * 100 && f[0]->tick.size() == 100 && f[0]->status.size() == 9 * 100,
           "every object sized for 8 axes, the sample index and 9 status columns");
    f[0]->epoch = f[1]->epoch = f[2]->epoch = 1;
    testOk(q.push(f[0]) && q.push(f[1]) && !q.push(f[2]), "insertion decided by entries: Q = 2");
    testOk(wake.tryWait(), "a push wakes the callback thread");
    q.release(f[2]);
    q.release(f[3]);
    Work w;
    testOk(q.take(&w) && w.kind == Work::FRAME && w.frame == f[0] && q.outstanding(1), "FIFO: first frame taken");
    q.done(w, 1, 0);
    testOk(q.take(&w) && w.frame == f[1], "then the second");
    q.done(w, 1, 0);
    testOk(!q.take(&w) && !q.outstanding(1), "then nothing outstanding");
    LedgerDelta d = q.read(1);
    testOk(d.published == 2 && d.failed == 0, "ledger: 2 published");
    d = q.read(1);
    testOk(d.published == 0, "read once: counts are deltas, never applied twice");
}

void testEmptyRuns()
{
    epicsEvent wake;
    FrameQueue q(10, 4, wake);
    Frame *a = frame(q, 3, 1);
    q.push(a);
    q.pushEmptyRun(run(3, 2, 150));
    Frame *b = frame(q, 3, 152);
    q.push(b);
    unsigned entries;
    uint64_t frames;
    q.backlog(&entries, &frames);
    testOk(entries == 3 && frames == 152, "backlog: 3 entries, 152 logical frames");
    Work w;
    q.take(&w);
    q.done(w, 1, 0);
    q.take(&w);
    testOk(w.kind == Work::EMPTY && w.run.firstNumber == 2 && w.run.count == EMPTY_BATCH &&
           w.run.firstWindowStart == 1000, "an empty run is taken in batches of 64 from its start");
    q.done(w, EMPTY_BATCH, 0);
    q.take(&w);
    testOk(w.run.firstNumber == 66 && w.run.count == 64 && w.run.firstWindowStart == 1000 + 64 * 10,
           "the next batch continues the numbers and the windows");
    q.done(w, 64, 0);
    q.take(&w);
    testOk(w.run.firstNumber == 130 && w.run.count == 22, "the last batch holds the rest");
    q.done(w, 22, 0);
    q.take(&w);
    testOk(w.kind == Work::FRAME && w.frame == b, "a later frame never overtakes the run");
    q.done(w, 1, 0);
    testOk(q.read(3).published == 152, "ledger: 152 published");
}

void testStopCutsRuns()
{
    epicsEvent wake;
    FrameQueue q(10, 4, wake);
    q.pushEmptyRun(run(5, 1, 1000));
    Frame *f = frame(q, 5, 1001);
    q.push(f);
    Work w;
    q.take(&w);                               /* a batch in flight */
    q.stopped(5);
    testOk(q.cutByStop(5) == 1000 - 64, "Stop cuts the rest of the run (936), not the batch in flight");
    q.done(w, 64, 0);
    q.take(&w);
    testOk(w.kind == Work::FRAME && w.frame == f, "a real frame already queued is still published");
    q.done(w, 1, 0);
    testOk(!q.outstanding(5), "then nothing of the stopped acquisition is outstanding");
    LedgerDelta d = q.read(5);
    testOk(d.published == 65 && d.failed == 0, "ledger: 65 published, the cut counted apart");
}

void testLedgerEpochs()
{
    epicsEvent wake;
    FrameQueue q(10, 4, wake);
    Frame *old = frame(q, 1, 7);
    q.push(old);
    Frame *n = frame(q, 2, 1);
    q.push(n);
    Work w;
    q.take(&w);
    q.done(w, 0, 1);                          /* epoch 1: a failure */
    q.take(&w);
    q.done(w, 1, 0);                          /* epoch 2: published */
    testOk(q.read(1).failed == 0, "epoch 1's counts are superseded by epoch 2's");
    LedgerDelta d = q.read(2);
    testOk(d.published == 1 && d.failed == 0, "epoch 2: 1 published");
    /* a late report of an older epoch after the ledger moved on */
    Work late;
    memset(&late, 0, sizeof late);
    late.kind = Work::EMPTY;
    late.epoch = 1;
    q.done(late, 3, 0);
    testOk(q.staleReports() == 1 && q.read(2).published == 0, "a stale epoch's report is dropped, counted");
}

/* R1: publication results never travel through the command slots */
void testLedgerIndependentOfCommands()
{
    epicsEvent wake;
    FrameQueue q(10, 4, wake);
    CommandSlots cmds(2);
    Command c;
    c.type = CMD_STOP;
    testOk(cmds.submit(c) >= 0 && cmds.submit(c) >= 0 && cmds.submit(c) < 0, "every command slot full");
    Frame *f = frame(q, 9, 1);
    q.push(f);
    q.pushEmptyRun(run(9, 2, 10));
    Work w;
    q.take(&w);
    q.done(w, 1, 0);
    q.take(&w);
    q.done(w, 4, 6);                          /* an allocation failure part-way through a batch */
    LedgerDelta d = q.read(9);
    testOk(d.published == 5 && d.failed == 6, "R1: success and failure counted with the slots full");
    unsigned e;
    uint64_t fr;
    q.backlog(&e, &fr);
    Frame *all[6];
    int got = 0;
    while (got < 6 && (all[got] = q.acquire()) != 0)
        ++got;
    testOk(e == 0 && fr == 0 && got == 6, "R1: nothing leaked: all Q + 2 objects free again");
}

/* ---- command slots ---------------------------------------------------------------------- */

void testCommandBasics()
{
    epicsEvent wakeRx;
    CommandSlots s(4, &wakeRx);
    Command c;
    c.type = CMD_MONITOR_START;
    int id = s.submit(c);
    testOk(id >= 0 && wakeRx.tryWait(), "submit wakes the receiver");
    int got;
    Command g;
    testOk(s.next(&got, &g) && got == id && g.type == CMD_MONITOR_START, "the receiver takes it");
    CommandResult r;
    r.applied = false;
    r.message = "no";
    r.revision = 42;
    testOk(!s.complete(got, r), "its waiter is still there");
    CommandResult out;
    testOk(s.wait(id, 1.0, &out) == WAIT_DONE && !out.applied && out.message == "no" && out.revision == 42,
           "the waiter gets the result and its revision");
    /* FIFO */
    c.type = CMD_ACQUIRE;
    int a = s.submit(c);
    c.type = CMD_STOP;
    int b = s.submit(c);
    s.next(&got, &g);
    testOk(got == a && g.type == CMD_ACQUIRE, "oldest first");
    r.applied = true;
    s.complete(got, r);
    s.next(&got, &g);
    testOk(got == b && g.type == CMD_STOP, "then the next");
    s.complete(got, r);
    s.wait(a, 1, &out);
    s.wait(b, 1, &out);
}

void testCommandTimeouts()
{
    CommandSlots s(2);
    Command c;
    c.type = CMD_ACQUIRE;
    CommandResult out;
    int id = s.submit(c);
    testOk(s.wait(id, 0.01, &out) == WAIT_CANCELLED, "timeout while pending: cancelled");
    int got;
    Command g;
    testOk(!s.next(&got, &g) && s.cancelled() == 1, "R2: a cancelled command is never applied");

    id = s.submit(c);
    s.next(&got, &g);
    testOk(s.wait(id, 0.01, &out) == WAIT_TIMED_OUT, "timeout after it was taken: timed out, result later");
    CommandResult r;
    r.applied = true;
    r.revision = 1;
    testOk(s.complete(got, r) && s.late() == 1, "R2: the late result is reported to the caller to publish");
    int again = s.submit(c);
    testOk(again >= 0, "the slot is free again");
    s.next(&got, &g);
    s.complete(got, r);
    s.wait(again, 1, &out);
}

struct Racer {
    CommandSlots *s;
    volatile bool stop;
    unsigned applied;
};

void receiverLoop(void *arg)
{
    Racer *x = (Racer *)arg;
    while (!x->stop) {
        int id;
        Command c;
        if (x->s->next(&id, &c)) {
            if (x->applied & 1)                 /* every other command is slow to apply */
                epicsThreadSleep(0.005);
            CommandResult r;
            r.applied = true;
            r.revision = 0;
            ++x->applied;
            x->s->complete(id, r);
        } else {
            epicsThreadSleep(0.0005);
        }
    }
}

/* timeout racing application: every command is either applied (and its result returned now or
 * later) or cancelled (and never applied) */
void testCommandRace()
{
    CommandSlots s(4);
    Racer x;
    x.s = &s;
    x.stop = false;
    x.applied = 0;
    epicsThreadId t = epicsThreadCreate("racer", epicsThreadPriorityMedium,
                                        epicsThreadGetStackSize(epicsThreadStackSmall), receiverLoop, &x);
    unsigned done = 0, cancelled = 0, timedOut = 0, refusedSubmit = 0;
    for (int i = 0; i < 400; ++i) {
        Command c;
        c.type = CMD_STOP;
        int id = s.submit(c);
        if (id < 0) {
            ++refusedSubmit;
            epicsThreadSleep(0.003);
            continue;
        }
        CommandResult out;
        switch (s.wait(id, (i % 4) * 0.002, &out)) {
        case WAIT_DONE: ++done; break;
        case WAIT_CANCELLED: ++cancelled; break;
        case WAIT_TIMED_OUT: ++timedOut; break;
        case WAIT_SHUTDOWN: break;
        }
    }
    epicsThreadSleep(0.05);
    x.stop = true;
    epicsThreadSleep(0.01);
    (void)t;
    testOk(x.applied == done + timedOut && s.late() == timedOut && s.cancelled() == cancelled,
           "R2 race: applied %u = done %u + timed out %u; late %llu; cancelled %u never applied",
           x.applied, done, timedOut, (unsigned long long)s.late(), cancelled);
}

void closer(void *arg)
{
    epicsThreadSleep(0.02);
    ((CommandSlots *)arg)->close("shutting down");
}

void testCommandShutdown()
{
    CommandSlots s(2);
    Command c;
    c.type = CMD_ACQUIRE;
    int id = s.submit(c);
    epicsThreadCreate("closer", epicsThreadPriorityMedium, epicsThreadGetStackSize(epicsThreadStackSmall),
                      closer, &s);
    CommandResult out;
    WaitOutcome w = s.wait(id, 2.0, &out);
    testOk(w == WAIT_DONE && !out.applied && out.message == "shutting down",
           "shutdown wakes a pending waiter with a refusal");
    testOk(s.submit(c) < 0 && s.closed(), "after close no command is accepted");
}

/* ---- mailbox ---------------------------------------------------------------------------- */

void testMailbox()
{
    epicsEvent wake;
    Mailbox mb(wake);
    MonitorUpdate u;
    memset(&u, 0, sizeof u);
    u.run = 3;
    mb.postMonitor(u);
    testOk(wake.tryWait(), "posting wakes the callback thread");
    MonitorUpdate held;
    testOk(mb.takeMonitor(&held) && held.run == 3, "the callback thread takes run 3");
    u.run = 4;                                /* a Stop arrives before it publishes */
    mb.postMonitor(u);
    {
        epicsGuard<epicsMutex> g(mb.gate());
        testOk(mb.currentRun() == 4 && held.run != mb.currentRun(),
               "R5: the held update is stale against the current run: discarded");
    }
    MonitorUpdate next;
    testOk(mb.takeMonitor(&next) && next.run == 4 && !mb.takeMonitor(&next), "the newest is published once");
    /* a health-only change keeps the run: the sequence number catches it */
    u.health = HEALTH_OK;
    mb.postMonitor(u);
    uint64_t heldSeq = 0;
    mb.takeMonitor(&held, &heldSeq);
    u.health = HEALTH_FAULTED;
    mb.postMonitor(u);
    {
        epicsGuard<epicsMutex> g(mb.gate());
        testOk(held.run == mb.currentRun() && heldSeq != mb.monitorSeq(),
               "R2: an OK update taken before a same-run fault is stale by sequence");
    }
    uint64_t newSeq = 0;
    testOk(mb.takeMonitor(&next, &newSeq) && next.health == HEALTH_FAULTED && newSeq == mb.monitorSeq(),
           "the fault is the latest and publishable");
    mb.postMonitor(u);
    mb.postMonitor(u);
    testOk(mb.monitorOverwritten() == 1, "an unread update overwritten is counted");
    Snapshot s = Snapshot();
    s.revision = 1;
    mb.postSnapshot(s);
    s.revision = 2;
    mb.postSnapshot(s);
    Snapshot t;
    uint64_t sseq = 0;
    testOk(mb.takeSnapshot(&t, &sseq) && t.revision == 2 && mb.snapshotsOverwritten() == 1 &&
           sseq == mb.snapshotSeq(), "snapshots: the newest revision, older ones overwritten and counted");
    s.revision = 3;
    mb.postSnapshot(s);
    testOk(sseq != mb.snapshotSeq(), "a snapshot taken before a newer post is superseded");
}

}

MAIN(testDriverParts)
{
    testPlan(0);
    testPoolAndQueue();
    testEmptyRuns();
    testStopCutsRuns();
    testLedgerEpochs();
    testLedgerIndependentOfCommands();
    testCommandBasics();
    testCommandTimeouts();
    testCommandRace();
    testCommandShutdown();
    testMailbox();
    return testDone();
}
