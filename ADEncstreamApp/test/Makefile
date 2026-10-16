# ADEncstreamApp/test - unit tests of the EPICS-free core, run with `make runtests` (or
# `make tapfiles`). They use EPICS's epicsUnitTest; the core itself does not depend on EPICS.
# Each test compiles the core sources it needs directly, so no library exports are needed on
# Windows.
TOP = ../..
include $(TOP)/configure/CONFIG

VENDOR = $(TOP)/vendor/encstream
SRC_DIRS += $(VENDOR)
USR_INCLUDES += -I$(VENDOR)
# golden packets, read at run time from the source tree
USR_CPPFLAGS += -DES_GOLDEN_DIR="\"$(abspath $(VENDOR)/golden)\""

TESTPROD_HOST += testWire
testWire_SRCS += testWire.c wire.c
testWire_LIBS += Com
TESTS += testWire

# the core, compiled into each test (ADEncstreamApp/src)
CORE = $(TOP)/ADEncstreamApp/src
SRC_DIRS += $(CORE)
USR_INCLUDES += -I$(CORE)
# trace fixtures, read at run time from the source tree
USR_CPPFLAGS += -DES_TRACE_DIR="\"$(abspath ../traces)\""

TESTPROD_HOST += testTrace
testTrace_SRCS += testTrace.cpp esTrace.cpp
testTrace_LIBS += Com
TESTS += testTrace

TESTPROD_HOST += testReceiver
testReceiver_SRCS += testReceiver.cpp esReceiver.cpp esTrace.cpp wire.c
testReceiver_LIBS += Com
TESTS += testReceiver

# replay of recorded traces; esReplay is a tool, not a test
TESTPROD_HOST += esReplay
esReplay_SRCS += esReplay.cpp esReceiver.cpp esTrace.cpp wire.c

TESTPROD_HOST += testReplay
testReplay_SRCS += testReplay.cpp esReceiver.cpp esTrace.cpp wire.c
testReplay_LIBS += Com
TESTS += testReplay

TESTPROD_HOST += testMonitor
testMonitor_SRCS += testMonitor.cpp esMonitor.cpp esReceiver.cpp esTrace.cpp wire.c
testMonitor_LIBS += Com
TESTS += testMonitor

TESTPROD_HOST += testFramer
testFramer_SRCS += testFramer.cpp esFramer.cpp esMonitor.cpp esReceiver.cpp esTrace.cpp wire.c
testFramer_LIBS += Com
TESTS += testFramer

TESTPROD_HOST += testFanOut
testFanOut_SRCS += testFanOut.cpp esFramer.cpp esMonitor.cpp esReceiver.cpp esTrace.cpp wire.c
testFanOut_LIBS += Com
TESTS += testFanOut

# the driver's thread-crossing parts (libCom only, no asyn)
TESTPROD_HOST += testDriverParts
testDriverParts_SRCS += testDriverParts.cpp esFrameQueue.cpp esCommands.cpp esEngine.cpp
testDriverParts_SRCS += esFramer.cpp esMonitor.cpp esReceiver.cpp esTrace.cpp wire.c
testDriverParts_LIBS += Com
TESTS += testDriverParts

TESTPROD_HOST += testEngine
testEngine_SRCS += testEngine.cpp esFrameQueue.cpp esCommands.cpp esEngine.cpp
testEngine_SRCS += esFramer.cpp esMonitor.cpp esReceiver.cpp esTrace.cpp wire.c
testEngine_LIBS += Com
TESTS += testEngine

# the driver itself (needs asyn and ADCore). The driver's sources are compiled in, not linked from
# the ADEncstream library: the tests reach into the driver class (test seams, a friend class), and
# a Windows DLL exports only the registrar
DRIVER_SRCS = ADEncstream.cpp esFrameQueue.cpp esCommands.cpp esEngine.cpp
DRIVER_SRCS += esFramer.cpp esMonitor.cpp esReceiver.cpp esTrace.cpp wire.c
TESTPROD_HOST += testDriver
testDriver_SRCS += testDriver.cpp $(DRIVER_SRCS)
testDriver_LIBS += ADBase asyn dbCore ca Com
# the driver's osiSock calls are raw Winsock here too (DRIVER_SRCS compiled in, not linked
# from the ADEncstream DLL), so this PROD links ws2_32 itself (as the library does)
testDriver_SYS_LIBS_WIN32 += ws2_32
TESTS += testDriver

# the driver with the real Acquire/busy records, in an in-process IOC (dbUnitTest): a refused
# Acquire must leave the records Done in every driver state
TARGETS += $(COMMON_DIR)/testDriverRecords.dbd
DBDDEPENDS_FILES += testDriverRecords.dbd$(DEP)
testDriverRecords_DBD += base.dbd asyn.dbd busySupport.dbd
TESTFILES += $(COMMON_DIR)/testDriverRecords.dbd
TESTPROD_HOST += testDriverRecords
testDriverRecords_SRCS += testDriverRecords.cpp testDriverRecords_registerRecordDeviceDriver.cpp $(DRIVER_SRCS)
testDriverRecords_LIBS += ADBase asyn busy $(EPICS_BASE_IOC_LIBS)
testDriverRecords_SYS_LIBS_WIN32 += ws2_32
USR_CPPFLAGS += -DES_ADCORE_DB="\"$(ADCORE)/db\""
TESTS += testDriverRecords

# A fully static link (e.g. windows-x64-static) must resolve libxml2 itself: asynNDArrayDriver (in
# ADBase, compiled into these two PRODs via DRIVER_SRCS) parses NDAttributes with it, and a shared
# build gets it from ADBase's DLL. ADSupport provides xml2 and its zlib dependency. A shared/DLL
# build needs neither (the symbols live inside ADBase.dll), so this is gated on a static arch.
ifneq ($(findstring static,$(T_A)),)
testDriver_LIBS += xml2 zlib
testDriverRecords_LIBS += xml2 zlib
endif

TESTSCRIPTS_HOST += $(TESTS:%=%.t)

include $(TOP)/configure/RULES
