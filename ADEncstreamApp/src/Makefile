# ADEncstreamApp/src - the ADEncstream library: the EPICS-free core (packet codec, receiver,
# framer, monitor), the driver's thread-crossing parts (libCom only) and the areaDetector driver.
# The tests in ../test compile the core sources directly.
TOP = ../..
include $(TOP)/configure/CONFIG

VENDOR = $(TOP)/vendor/encstream
SRC_DIRS += $(VENDOR)
USR_INCLUDES += -I$(VENDOR)

INC += ADEncstream.h

LIBRARY_IOC = ADEncstream
# the core
LIB_SRCS += wire.c esReceiver.cpp esTrace.cpp esFramer.cpp esMonitor.cpp
# the driver's parts and the driver
LIB_SRCS += esFrameQueue.cpp esCommands.cpp esEngine.cpp ADEncstream.cpp

# The driver's socket I/O goes through osiSock, whose Windows calls (sendto, recvfrom,
# select, bind, htons, ...) are raw Winsock. As a DLL consumer we must link ws2_32
# ourselves: base's Com DLL links it but does not re-export those symbols (asyn and
# ADCore's NDPlugin do the same).
ADEncstream_SYS_LIBS_WIN32 += ws2_32

DBD += ADEncstreamSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

include $(TOP)/configure/RULES
