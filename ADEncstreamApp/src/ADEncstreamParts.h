/* ADEncstreamParts.h - the driver's parts built before its port exists. Private
 * to the driver and its tests (test/testDriver.cpp); not installed. */
#ifndef ADENCSTREAM_PARTS_H
#define ADENCSTREAM_PARTS_H

#include <string.h>

#include <string>

#include <epicsEvent.h>
#include <epicsMutex.h>
#include <osiSock.h>

#include "ADEncstream.h"

struct ADEncstream::Parts {
    epicsEvent wakeCb;
    encstream::FrameQueue queue;
    encstream::CommandSlots commands;
    encstream::Mailbox mailbox;
    encstream::EngineConfig cfg;
    encstream::Engine *engine;
    SOCKET sock;
    osiSockAddr brick;
    int rcvBuf;
    bool sockAttached;
    /* a fault seen on the receiver thread, published by the callback thread */
    epicsMutex faultLock;
    bool fault;
    std::string faultMsg;

    Parts(unsigned capacity, unsigned depth)
        : queue(capacity, depth, wakeCb), commands(8), mailbox(wakeCb), engine(0), sock(INVALID_SOCKET),
          rcvBuf(0), sockAttached(false), fault(false)
    {
        memset(&brick, 0, sizeof brick);
    }
    ~Parts()
    {
        delete engine;
        if (sock != INVALID_SOCKET)
            epicsSocketDestroy(sock);
        if (sockAttached)
            osiSockRelease();
    }
};

#endif
