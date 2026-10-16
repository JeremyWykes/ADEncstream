# encstreamAxisPlugins.cmd - one motor's records and plugins, loaded once per motor:
#     iocshLoad("encstreamAxisPlugins.cmd", "AXIS=<n>,ADDR=<n-1>,NAME=<human name>")
# (iocsh has no arithmetic: give both the motor number and its address.)
# Everything about Motor[n] lives under $(PREFIX)<n>:.
# Needs: PREFIX, PORT, QSIZE, NCHANS, MAX_SAMPLES, STATS_PREFIX, ARRAY_PREFIX, TS_PREFIX.

dbLoadRecords("$(ADENCSTREAM)/db/ADEncstreamAxis.template",  "P=$(PREFIX),R=$(AXIS):,PORT=$(PORT),ADDR=$(ADDR),TIMEOUT=1,NAME=$(NAME)")
dbLoadRecords("$(ADENCSTREAM)/db/ADEncstreamArray.template", "P=$(PREFIX),R=$(AXIS):,PORT=$(PORT),ADDR=$(ADDR),TIMEOUT=1")

# the positions as a waveform
NDStdArraysConfigure("Image$(AXIS)", 3, 0, "$(PORT)", $(ADDR))
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=$(AXIS):$(ARRAY_PREFIX):,PORT=Image$(AXIS),ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=$(ADDR),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(MAX_SAMPLES),ENABLED=1")

# statistics (the frame means) and their time series (the stats plugin's address 1)
NDStatsConfigure("STATS$(AXIS)", $(QSIZE), 0, "$(PORT)", $(ADDR), 0, 0, 0, 0, 1)
dbLoadRecords("NDStats.template", "P=$(PREFIX),R=$(AXIS):$(STATS_PREFIX):,PORT=STATS$(AXIS),ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=16,YSIZE=16,NCHANS=$(NCHANS),NDARRAY_PORT=$(PORT),NDARRAY_ADDR=$(ADDR),ENABLED=1")
NDTimeSeriesConfigure("STATS$(AXIS)_TS", $(QSIZE), 0, "STATS$(AXIS)", 1, 23)
dbLoadRecords("$(ADCORE)/db/NDTimeSeries.template", "P=$(PREFIX),R=$(AXIS):$(STATS_PREFIX):$(TS_PREFIX):,PORT=STATS$(AXIS)_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATS$(AXIS),NDARRAY_ADDR=1,NCHANS=$(NCHANS),ENABLED=1")
