/* esCommands.cpp - control command slots. See esCommands.h. */
#include "esCommands.h"

namespace encstream {

CommandSlots::CommandSlots(unsigned nSlots, epicsEvent *wakeReceiver)
    : slots_(nSlots ? nSlots : 1), wakeReceiver_(wakeReceiver), seq_(0), closed_(false),
      cancelled_(0), late_(0)
{
    for (size_t i = 0; i < slots_.size(); ++i) {
        slots_[i].state = FREE;
        slots_[i].waiterGone = false;
        slots_[i].seq = 0;
        slots_[i].done = new epicsEvent(epicsEventEmpty);
    }
}

CommandSlots::~CommandSlots()
{
    for (size_t i = 0; i < slots_.size(); ++i)
        delete slots_[i].done;
}

int CommandSlots::submit(const Command &c)
{
    int id = -1;
    {
        epicsGuard<epicsMutex> g(lock_);
        if (closed_)
            return -1;
        for (size_t i = 0; i < slots_.size(); ++i)
            if (slots_[i].state == FREE) {
                id = (int)i;
                break;
            }
        if (id < 0)
            return -1;
        Slot &s = slots_[id];
        s.state = PENDING;
        s.waiterGone = false;
        s.seq = ++seq_;
        s.cmd = c;
        s.result = CommandResult();
        s.done->tryWait();                    /* no stale signal from an earlier use */
    }
    if (wakeReceiver_)
        wakeReceiver_->trigger();
    return id;
}

WaitOutcome CommandSlots::wait(int id, double timeoutS, CommandResult *result)
{
    if (id < 0 || (size_t)id >= slots_.size())
        return WAIT_SHUTDOWN;
    Slot &s = slots_[id];
    s.done->wait(timeoutS);
    epicsGuard<epicsMutex> g(lock_);
    switch (s.state) {
    case DONE:
        *result = s.result;
        s.state = FREE;
        return WAIT_DONE;
    case PENDING:                             /* never taken: cancel, it will not be applied */
        s.state = FREE;
        ++cancelled_;
        if (closed_) {
            result->applied = false;
            result->message = closedWhy_;
            return WAIT_SHUTDOWN;
        }
        return WAIT_CANCELLED;
    case TAKEN:                               /* being applied: its result is published later */
        s.waiterGone = true;
        return WAIT_TIMED_OUT;
    case FREE:
        break;
    }
    return WAIT_SHUTDOWN;
}

bool CommandSlots::next(int *id, Command *c)
{
    epicsGuard<epicsMutex> g(lock_);
    int best = -1;
    for (size_t i = 0; i < slots_.size(); ++i)
        if (slots_[i].state == PENDING && (best < 0 || slots_[i].seq < slots_[best].seq))
            best = (int)i;
    if (best < 0)
        return false;
    slots_[best].state = TAKEN;
    *id = best;
    *c = slots_[best].cmd;
    return true;
}

bool CommandSlots::complete(int id, const CommandResult &r)
{
    if (id < 0 || (size_t)id >= slots_.size())
        return false;
    epicsGuard<epicsMutex> g(lock_);
    Slot &s = slots_[id];
    if (s.state != TAKEN)
        return false;
    if (s.waiterGone) {
        s.state = FREE;
        ++late_;
        return true;
    }
    s.result = r;
    s.state = DONE;
    s.done->trigger();
    return false;
}

void CommandSlots::close(const std::string &why)
{
    epicsGuard<epicsMutex> g(lock_);
    closed_ = true;
    closedWhy_ = why;
    for (size_t i = 0; i < slots_.size(); ++i) {
        Slot &s = slots_[i];
        if (s.state == PENDING) {
            s.result.applied = false;
            s.result.message = why;
            s.result.revision = 0;
            s.state = DONE;
            s.done->trigger();
        }
    }
}

bool CommandSlots::closed() const
{
    epicsGuard<epicsMutex> g(lock_);
    return closed_;
}

uint64_t CommandSlots::cancelled() const
{
    epicsGuard<epicsMutex> g(lock_);
    return cancelled_;
}

uint64_t CommandSlots::late() const
{
    epicsGuard<epicsMutex> g(lock_);
    return late_;
}

}
