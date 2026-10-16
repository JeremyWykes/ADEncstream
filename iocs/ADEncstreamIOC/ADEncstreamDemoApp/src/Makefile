# the example IOC's executable: ADEncstream plus ADCore's plugins and the usual support
TOP = ../..
include $(TOP)/configure/CONFIG

PROD_NAME = ADEncstreamDemoApp
PROD_IOC = $(PROD_NAME)
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += ADEncstreamSupport.dbd
PROD_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp
PROD_LIBS += ADEncstream

include $(ADCORE)/ADApp/commonDriverMakefile

include $(TOP)/configure/RULES
