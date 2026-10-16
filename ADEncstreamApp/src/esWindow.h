/* esWindow.h - windows on the sample clock, shared by Free Running and the
 * continuous monitor. Half-open windows [origin + k*L, origin + (k+1)*L) in ticks, origin the tick
 * of the first row the consumer accepts. A window closes only when a row at or beyond its end
 * arrives; the end of the data never closes one. A gap is reported as a count of empty windows,
 * never one by one. Portable C++11, header-only, no allocation. */
#ifndef ENCSTREAM_ES_WINDOW_H
#define ENCSTREAM_ES_WINDOW_H

#include <stdint.h>

namespace encstream {

struct WindowStep {
    bool first;               /* the first row since start(): window `index` opened */
    bool closed;              /* window `closedIndex` closed (it had rows) */
    uint64_t closedIndex;
    uint64_t emptyFirst;      /* windows emptyFirst .. emptyFirst + emptyCount - 1 had no rows */
    uint64_t emptyCount;
    uint64_t index;           /* the window this row belongs to */
};

class Windows {
public:
    Windows() : started_(false), hasRow_(false), origin_(0), length_(1), current_(0) {}

    /* length in ticks (>= 1); the origin is the tick of the next row given to row() */
    void start(uint64_t lengthTicks)
    {
        started_ = true;
        hasRow_ = false;
        length_ = lengthTicks ? lengthTicks : 1;
        current_ = 0;
    }
    void stop() { started_ = false; hasRow_ = false; }
    bool started() const { return started_; }
    bool hasRow() const { return hasRow_; }
    uint64_t length() const { return length_; }
    uint64_t origin() const { return origin_; }
    uint64_t current() const { return current_; }

    /* first tick of window k; false if it does not fit in 64 bits */
    bool windowStart(uint64_t k, uint64_t *tick) const
    {
        if (k != 0 && length_ > ~(uint64_t)0 / k)
            return false;
        uint64_t off = k * length_;
        if (off > ~(uint64_t)0 - origin_)
            return false;
        *tick = origin_ + off;
        return true;
    }

    /* A row at `tick` (ticks never decrease within a stream). */
    WindowStep row(uint64_t tick)
    {
        WindowStep s;
        s.first = false;
        s.closed = false;
        s.closedIndex = 0;
        s.emptyFirst = 0;
        s.emptyCount = 0;
        if (!hasRow_) {
            hasRow_ = true;
            origin_ = tick;
            current_ = 0;
            s.first = true;
            s.index = 0;
            return s;
        }
        uint64_t k = tick >= origin_ ? (tick - origin_) / length_ : current_;
        if (k > current_) {
            s.closed = true;
            s.closedIndex = current_;
            s.emptyFirst = current_ + 1;
            s.emptyCount = k - current_ - 1;
            current_ = k;
        }
        s.index = current_;
        return s;
    }

private:
    bool started_, hasRow_;
    uint64_t origin_, length_, current_;
};

}

#endif
