/* esCommands.h - control commands from the asyn port thread to the receiver thread
 *.
 *
 * A fixed set of slots owned by this object, never by a caller. A slot moves
 *     free -> pending -> taken -> done -> free          (applied or refused)
 *     free -> pending -> free                           (cancelled by the waiter's timeout)
 *     free -> pending -> taken -> (waiter gone) -> free (a late result, kept for publication)
 * under one mutex, so a timed-out command is either never applied or its real result is kept;
 * the waiter's storage is never touched after it returns. Uses EPICS libCom only.
 */
#ifndef ENCSTREAM_ES_COMMANDS_H
#define ENCSTREAM_ES_COMMANDS_H

#include <stdint.h>

#include <string>
#include <vector>

#include <epicsEvent.h>
#include <epicsMutex.h>

#include "esFramer.h"
#include "esMonitor.h"

namespace encstream {

enum CommandType {
    CMD_ACQUIRE, CMD_STOP, CMD_MONITOR_START, CMD_MONITOR_STOP, CMD_MONITOR_CONFIGURE
};

enum {                                        /* CMD_MONITOR_CONFIGURE: the fields written */
    MONITOR_FIELD_T = 1, MONITOR_FIELD_U = 2, MONITOR_FIELD_MIN_COVERAGE = 4
};

struct Command {
    CommandType type;
    FramerConfig framer;                      /* CMD_ACQUIRE: the latched parameters */
    MonitorConfig monitor;                    /* CMD_MONITOR_CONFIGURE */
    /* CMD_MONITOR_CONFIGURE: the fields of `monitor` to apply to the accepted configuration
     * (a write changes only its own field); 0 means all of `monitor` */
    unsigned monitorFields;
    Command() : type(CMD_STOP), monitorFields(0) {}
};

struct CommandResult {
    bool applied;                             /* false: refused */
    std::string message;                      /* why refused, or a note */
    uint64_t revision;                        /* the status revision posted after applying it */
};

enum WaitOutcome {
    WAIT_DONE,                                /* *result is the receiver's answer */
    WAIT_CANCELLED,                           /* timed out before the receiver took it: never applied */
    WAIT_TIMED_OUT,                           /* timed out after the receiver took it: the result comes later */
    WAIT_SHUTDOWN                             /* refused: the driver is shutting down or faulted */
};

class CommandSlots {
public:
    explicit CommandSlots(unsigned nSlots, epicsEvent *wakeReceiver = 0);
    ~CommandSlots();

    /* port thread: queue a command; -1 if no slot is free or the slots are closed */
    int submit(const Command &c);
    /* port thread: wait for the command's result, at most timeoutS; frees or abandons the slot */
    WaitOutcome wait(int id, double timeoutS, CommandResult *result);

    /* receiver thread: the oldest pending command, now taken; false if none */
    bool next(int *id, Command *c);
    /* receiver thread: the taken command's result. Returns true if its waiter had gone (a late
     * result: the caller publishes it, since no write will report it). */
    bool complete(int id, const CommandResult &r);

    /* shutdown or fault: refuse every pending command and future submissions */
    void close(const std::string &why);
    bool closed() const;

    /* diagnostics */
    uint64_t cancelled() const;
    uint64_t late() const;

private:
    CommandSlots(const CommandSlots &);
    CommandSlots &operator=(const CommandSlots &);

    enum State { FREE, PENDING, TAKEN, DONE };
    struct Slot {
        State state;
        bool waiterGone;
        uint64_t seq;
        Command cmd;
        CommandResult result;
        epicsEvent *done;
    };

    mutable epicsMutex lock_;
    std::vector<Slot> slots_;
    epicsEvent *wakeReceiver_;
    uint64_t seq_;
    bool closed_;
    std::string closedWhy_;
    uint64_t cancelled_, late_;
};

}

#endif
