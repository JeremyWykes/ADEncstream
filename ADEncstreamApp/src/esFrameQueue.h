/* esFrameQueue.h - the frame pool, the queue to the callback thread and the publication ledger
 *.
 *
 * The framer's FrameOutput on the receiver thread, the work source of the driver's callback
 * thread. Q + 2 frame objects, sized once at construction for every acquisition this pool can
 * serve (8 position axes, the sample index, 9 status columns); a fixed ring of Q entries holding
 * frames and empty runs in frame-number order. The callback thread takes a frame, or a batch of at
 * most EMPTY_BATCH frames of the empty run at the front, and reports what it published; the
 * receiver thread reads those counts back. Uses EPICS libCom (epicsMutex, epicsEvent) only.
 */
#ifndef ENCSTREAM_ES_FRAME_QUEUE_H
#define ENCSTREAM_ES_FRAME_QUEUE_H

#include <stddef.h>
#include <stdint.h>

#include <vector>

#include <epicsEvent.h>
#include <epicsMutex.h>

#include "esFramer.h"

namespace encstream {

enum { EMPTY_BATCH = 64 };                    /* empty frames per batch */

/* one unit of callback-thread work */
struct Work {
    enum Kind { NONE, FRAME, EMPTY } kind;
    Frame *frame;                             /* FRAME */
    EmptyRun run;                             /* EMPTY: firstNumber, count, firstWindowStart of this batch */
    uint64_t epoch;
};

/* publication counts of one acquisition, read back by the receiver thread */
struct LedgerDelta {
    uint64_t published, failed;
};

/* bytes of frame-object storage per sample slot: 8 positions, the tick, 9 status words */
const size_t FRAME_BYTES_PER_SAMPLE = 8 * sizeof(double) + sizeof(uint64_t) + 9 * sizeof(uint32_t);

class FrameQueue : public FrameOutput {
public:
    /* allocates (depth + 2) frame objects of `capacity` samples; throws std::bad_alloc.
     * `wake` is signalled when work is queued. */
    FrameQueue(uint32_t capacity, unsigned depth, epicsEvent &wake);
    ~FrameQueue();

    uint32_t capacity() const { return capacity_; }
    unsigned depth() const { return depth_; }

    /* FrameOutput (receiver thread) */
    Frame *acquire();
    bool push(Frame *f);
    bool pushEmptyRun(const EmptyRun &r);
    void release(Frame *f);
    void stopped(uint64_t epoch);

    /* callback thread: the next work, or false if none. take() and done() alternate. */
    bool take(Work *w);
    /* the work's outcome: `published` frames published, `failed` not (frames of an empty batch
     * may be split); a frame object goes back to the free list */
    void done(const Work &w, uint64_t published, uint64_t failed);

    /* receiver thread: counts reported for `epoch` since the last read */
    LedgerDelta read(uint64_t epoch);
    /* empty frames of `epoch` cut by Stop (cumulative for the latest epoch reported) */
    uint64_t cutByStop(uint64_t epoch) const;
    /* outputs of `epoch` still queued or being published */
    bool outstanding(uint64_t epoch) const;
    /* queue entries, and logical frames queued or in flight */
    void backlog(unsigned *entries, uint64_t *frames) const;
    /* reports for an epoch older than the ledger's (dropped; the framer ignores them anyway) */
    uint64_t staleReports() const;

    /* shutdown: queued frames back to free, runs dropped */
    void clear();

private:
    FrameQueue(const FrameQueue &);
    FrameQueue &operator=(const FrameQueue &);

    struct Entry {
        bool isRun;
        Frame *frame;
        EmptyRun run;                         /* run.firstNumber/count/firstWindowStart: the unpublished rest */
        uint64_t epoch;
    };
    struct Ledger {
        uint64_t epoch;
        uint64_t published, failed, cut;
        uint64_t readPublished, readFailed;
    };

    void ledgerAdd(uint64_t epoch, uint64_t published, uint64_t failed, uint64_t cut);
    Entry &at(unsigned i) { return ring_[(head_ + i) % depth_]; }
    const Entry &at(unsigned i) const { return ring_[(head_ + i) % depth_]; }

    const uint32_t capacity_;
    const unsigned depth_;
    epicsEvent &wake_;
    mutable epicsMutex lock_;

    std::vector<Frame> frames_;
    std::vector<Frame *> free_;
    std::vector<Entry> ring_;
    unsigned head_, count_;

    bool inFlight_;
    uint64_t inFlightEpoch_, inFlightFrames_;
    Ledger ledger_;
    uint64_t stale_;
};

}

#endif
