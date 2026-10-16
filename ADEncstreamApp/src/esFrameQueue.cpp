/* esFrameQueue.cpp - the frame pool, queue and publication ledger. See esFrameQueue.h. */
#include "esFrameQueue.h"

#include <string.h>

namespace encstream {

FrameQueue::FrameQueue(uint32_t capacity, unsigned depth, epicsEvent &wake)
    : capacity_(capacity), depth_(depth ? depth : 1), wake_(wake), frames_(depth_ + 2),
      head_(0), count_(0), inFlight_(false), inFlightEpoch_(0), inFlightFrames_(0), stale_(0)
{
    memset(&ledger_, 0, sizeof ledger_);
    ring_.resize(depth_);
    for (size_t i = 0; i < frames_.size(); ++i) {
        Frame &f = frames_[i];
        f.capacity = capacity_;
        f.position.assign((size_t)FRAMER_MAX_AXES * capacity_, 0.0);
        f.tick.assign(capacity_, 0);
        f.status.assign((size_t)FRAMER_MAX_STATUS * capacity_, 0);
        free_.push_back(&f);
    }
}

FrameQueue::~FrameQueue()
{
}

Frame *FrameQueue::acquire()
{
    epicsGuard<epicsMutex> g(lock_);
    if (free_.empty())
        return 0;
    Frame *f = free_.back();
    free_.pop_back();
    return f;
}

bool FrameQueue::push(Frame *f)
{
    {
        epicsGuard<epicsMutex> g(lock_);
        if (count_ == depth_)
            return false;                     /* C1: insertion is decided by entries */
        Entry &e = at(count_);
        e.isRun = false;
        e.frame = f;
        e.epoch = f->epoch;
        ++count_;
    }
    wake_.trigger();
    return true;
}

bool FrameQueue::pushEmptyRun(const EmptyRun &r)
{
    {
        epicsGuard<epicsMutex> g(lock_);
        if (count_ == depth_)
            return false;
        Entry &e = at(count_);
        e.isRun = true;
        e.frame = 0;
        e.run = r;
        e.epoch = r.epoch;
        ++count_;
    }
    wake_.trigger();
    return true;
}

void FrameQueue::release(Frame *f)
{
    epicsGuard<epicsMutex> g(lock_);
    free_.push_back(f);
}

void FrameQueue::stopped(uint64_t epoch)
{
    epicsGuard<epicsMutex> g(lock_);
    /* drop the empty runs of the stopped acquisition, keeping every other entry in order */
    uint64_t cut = 0;
    unsigned kept = 0;
    for (unsigned i = 0; i < count_; ++i) {
        Entry e = at(i);
        if (e.isRun && e.epoch == epoch) {
            cut += e.run.count;
            continue;
        }
        at(kept++) = e;
    }
    count_ = kept;
    if (cut)
        ledgerAdd(epoch, 0, 0, cut);
}

bool FrameQueue::take(Work *w)
{
    epicsGuard<epicsMutex> g(lock_);
    w->kind = Work::NONE;
    if (count_ == 0)
        return false;
    Entry &e = at(0);
    w->epoch = e.epoch;
    if (!e.isRun) {
        w->kind = Work::FRAME;
        w->frame = e.frame;
        inFlightFrames_ = 1;
        head_ = (head_ + 1) % depth_;
        --count_;
    } else {
        w->kind = Work::EMPTY;
        w->frame = 0;
        w->run = e.run;
        uint64_t n = e.run.count < EMPTY_BATCH ? e.run.count : EMPTY_BATCH;
        w->run.count = n;
        inFlightFrames_ = n;
        e.run.firstNumber += n;
        e.run.firstWindowStart += n * e.run.windowLength;
        e.run.count -= n;
        if (e.run.count == 0) {
            head_ = (head_ + 1) % depth_;
            --count_;
        }
    }
    inFlight_ = true;
    inFlightEpoch_ = w->epoch;
    return true;
}

void FrameQueue::done(const Work &w, uint64_t published, uint64_t failed)
{
    epicsGuard<epicsMutex> g(lock_);
    if (w.kind == Work::FRAME && w.frame)
        free_.push_back(w.frame);
    if (w.kind != Work::NONE)
        ledgerAdd(w.epoch, published, failed, 0);
    inFlight_ = false;
    inFlightFrames_ = 0;
}

void FrameQueue::ledgerAdd(uint64_t epoch, uint64_t published, uint64_t failed, uint64_t cut)
{
    if (epoch < ledger_.epoch) {
        ++stale_;
        return;
    }
    if (epoch > ledger_.epoch) {
        memset(&ledger_, 0, sizeof ledger_);
        ledger_.epoch = epoch;
    }
    ledger_.published += published;
    ledger_.failed += failed;
    ledger_.cut += cut;
}

LedgerDelta FrameQueue::read(uint64_t epoch)
{
    epicsGuard<epicsMutex> g(lock_);
    LedgerDelta d = { 0, 0 };
    if (epoch != ledger_.epoch)
        return d;
    d.published = ledger_.published - ledger_.readPublished;
    d.failed = ledger_.failed - ledger_.readFailed;
    ledger_.readPublished = ledger_.published;
    ledger_.readFailed = ledger_.failed;
    return d;
}

uint64_t FrameQueue::cutByStop(uint64_t epoch) const
{
    epicsGuard<epicsMutex> g(lock_);
    return epoch == ledger_.epoch ? ledger_.cut : 0;
}

bool FrameQueue::outstanding(uint64_t epoch) const
{
    epicsGuard<epicsMutex> g(lock_);
    if (inFlight_ && inFlightEpoch_ == epoch)
        return true;
    for (unsigned i = 0; i < count_; ++i)
        if (at(i).epoch == epoch)
            return true;
    return false;
}

void FrameQueue::backlog(unsigned *entries, uint64_t *frames) const
{
    epicsGuard<epicsMutex> g(lock_);
    uint64_t n = inFlight_ ? inFlightFrames_ : 0;
    for (unsigned i = 0; i < count_; ++i)
        n += at(i).isRun ? at(i).run.count : 1;
    *entries = count_;
    *frames = n;
}

uint64_t FrameQueue::staleReports() const
{
    epicsGuard<epicsMutex> g(lock_);
    return stale_;
}

void FrameQueue::clear()
{
    epicsGuard<epicsMutex> g(lock_);
    for (unsigned i = 0; i < count_; ++i)
        if (!at(i).isRun)
            free_.push_back(at(i).frame);
    count_ = 0;
    head_ = 0;
}

}
