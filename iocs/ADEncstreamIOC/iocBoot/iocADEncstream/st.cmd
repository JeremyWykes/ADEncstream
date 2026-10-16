# st.cmd - the ADEncstream example IOC.
#
# As shipped it talks to encstream's mock streamer on this host:
#     encstream/brick/build/host/encstream_mock -b 127.0.0.1 -p 25199
# For a Brick or the simulator, set the environment variable BRICK to its address or hostname
# (port 5199 by default) before starting the IOC, e.g. on Windows `set BRICK=192.0.2.10`, and
# LOCAL_PORT to the local UDP port to subscribe from; or edit the defaults below.

< envPaths

# the plugin templates by bare name
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

epicsEnvSet("PREFIX", "ES1:")
# R (the areaDetector record infix) is empty for this IOC; it is defaulted inline below as
# $(R=) rather than with epicsEnvSet("R",""), because on Windows setting an environment
# variable to "" deletes it, leaving $(R) undefined and iocsh skipping the dbLoadRecords.
epicsEnvSet("PORT",   "ENC1")
epicsEnvSet("BRICK",  "$(BRICK=127.0.0.1:25199)")
epicsEnvSet("LOCAL_PORT", "$(LOCAL_PORT=25198)")
# the frame pool: samples per frame, queue depth (pool = Q + 2 frames)
epicsEnvSet("MAX_SAMPLES", "200000")
epicsEnvSet("QUEUE_DEPTH", "8")
# NDArray memory: per frame 25.6 MB at 200000 samples and 8 axes, times the deepest plugin
# queue + 2, plus the plugin reserve (1 MB + 96 x MAX_SAMPLES: one std-arrays conversion per
# axis, allowing ADCore's 1.5 buffer-reuse ratio)
epicsEnvSet("MAX_MEMORY", "300000000")
# plugin queues
epicsEnvSet("QSIZE", "8")
# one CA waveform of one axis: 8 x MAX_SAMPLES bytes
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "2000000")
# time-series length of the TS plugins
epicsEnvSet("NCHANS", "2048")

# one letter per plugin
epicsEnvSet("ROISTAT_PREFIX", "R")
epicsEnvSet("HDF_PREFIX",     "H")
epicsEnvSet("ATTR_PREFIX",    "A")
epicsEnvSet("STATS_PREFIX",   "S")
epicsEnvSet("ARRAY_PREFIX",   "W")
epicsEnvSet("TS_PREFIX",      "T")

dbLoadDatabase("$(TOP)/dbd/ADEncstreamDemoApp.dbd")
ADEncstreamDemoApp_registerRecordDeviceDriver(pdbbase)

ADEncstreamConfig("$(PORT)", "$(BRICK)", $(LOCAL_PORT), "", "", $(MAX_SAMPLES), $(QUEUE_DEPTH), $(MAX_MEMORY), 0, 0)
dbLoadRecords("$(ADENCSTREAM)/db/ADEncstream.template", "P=$(PREFIX),R=$(R=),PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(ADCORE)/db/NDArrayBase.template",      "P=$(PREFIX),R=$(R=),PORT=$(PORT),ADDR=0,TIMEOUT=1")

iocshLoad("encstreamPlugins.cmd")
iocshLoad("encstreamAxisPlugins.cmd", "AXIS=1,ADDR=0,NAME=axis 1")
iocshLoad("encstreamAxisPlugins.cmd", "AXIS=2,ADDR=1,NAME=axis 2")
iocshLoad("encstreamAxisPlugins.cmd", "AXIS=3,ADDR=2,NAME=axis 3")
iocshLoad("encstreamAxisPlugins.cmd", "AXIS=4,ADDR=3,NAME=axis 4")
iocshLoad("encstreamAxisPlugins.cmd", "AXIS=5,ADDR=4,NAME=axis 5")
iocshLoad("encstreamAxisPlugins.cmd", "AXIS=6,ADDR=5,NAME=axis 6")
iocshLoad("encstreamAxisPlugins.cmd", "AXIS=7,ADDR=6,NAME=axis 7")
iocshLoad("encstreamAxisPlugins.cmd", "AXIS=8,ADDR=7,NAME=axis 8")

iocInit()

# settings a site would keep with autosave
iocshLoad("encstreamSettings.cmd")
iocshLoad("encstreamAxisSettings.cmd", "AXIS=1")
iocshLoad("encstreamAxisSettings.cmd", "AXIS=2")
iocshLoad("encstreamAxisSettings.cmd", "AXIS=3")
iocshLoad("encstreamAxisSettings.cmd", "AXIS=4")
iocshLoad("encstreamAxisSettings.cmd", "AXIS=5")
iocshLoad("encstreamAxisSettings.cmd", "AXIS=6")
iocshLoad("encstreamAxisSettings.cmd", "AXIS=7")
iocshLoad("encstreamAxisSettings.cmd", "AXIS=8")
