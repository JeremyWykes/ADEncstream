/* esFanOut.h - forwards the receiver's row stream to the acquisition framer and the continuous
 * monitor: every gap, rows and event call to both, framer first, on
 * the receiver thread. Adds nothing else; each consumer copies what it needs and returns promptly.
 * Portable C++11, header-only. */
#ifndef ENCSTREAM_ES_FANOUT_H
#define ENCSTREAM_ES_FANOUT_H

#include "esReceiver.h"

namespace encstream {

class FanOut : public RowSink {
public:
    FanOut(RowSink &framer, RowSink &monitor) : a_(framer), b_(monitor) {}
    void gap(const GapReport &g) { a_.gap(g); b_.gap(g); }
    void rows(const RowBlock &r) { a_.rows(r); b_.rows(r); }
    void event(const StreamEvent &e) { a_.event(e); b_.event(e); }

private:
    RowSink &a_, &b_;
};

}

#endif
