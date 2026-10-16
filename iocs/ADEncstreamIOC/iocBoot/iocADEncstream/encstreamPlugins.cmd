# encstreamPlugins.cmd - the top-level plugins, loaded once. Every plugin is fed
# by the driver's main array (address 8: 8 rows, row n-1 = Motor[n]).
# Needs: PREFIX, PORT, QSIZE, NCHANS, ROISTAT_PREFIX, ATTR_PREFIX, HDF_PREFIX, TS_PREFIX.

# the array records of addresses 8, 9 and 10
dbLoadRecords("$(ADENCSTREAM)/db/ADEncstreamArray.template", "P=$(PREFIX),R=$(R=)Main:,PORT=$(PORT),ADDR=8,TIMEOUT=1")
dbLoadRecords("$(ADENCSTREAM)/db/ADEncstreamArray.template", "P=$(PREFIX),R=$(R=)Index:,PORT=$(PORT),ADDR=9,TIMEOUT=1")
dbLoadRecords("$(ADENCSTREAM)/db/ADEncstreamArray.template", "P=$(PREFIX),R=$(R=)Status:,PORT=$(PORT),ADDR=10,TIMEOUT=1")

# ROIStat, 8 ROIs (ROI n = Motor[n]); its array output is turned off in encstreamSettings.cmd.
# Full-frame ROI means are wrong in this ADCore revision: use the per-axis
# Stats plugins for frame means.
NDROIStatConfigure("ROISTAT1", $(QSIZE), 0, "$(PORT)", 8, 8, 0, 0, 0, 0, 1)
dbLoadRecords("NDROIStat.template",  "P=$(PREFIX),R=$(ROISTAT_PREFIX):,PORT=ROISTAT1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=8,NCHANS=$(NCHANS),ENABLED=1")
dbLoadRecords("NDROIStatN.template", "P=$(PREFIX),R=$(ROISTAT_PREFIX):1:,PORT=ROISTAT1,ADDR=0,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template", "P=$(PREFIX),R=$(ROISTAT_PREFIX):2:,PORT=ROISTAT1,ADDR=1,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template", "P=$(PREFIX),R=$(ROISTAT_PREFIX):3:,PORT=ROISTAT1,ADDR=2,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template", "P=$(PREFIX),R=$(ROISTAT_PREFIX):4:,PORT=ROISTAT1,ADDR=3,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template", "P=$(PREFIX),R=$(ROISTAT_PREFIX):5:,PORT=ROISTAT1,ADDR=4,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template", "P=$(PREFIX),R=$(ROISTAT_PREFIX):6:,PORT=ROISTAT1,ADDR=5,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template", "P=$(PREFIX),R=$(ROISTAT_PREFIX):7:,PORT=ROISTAT1,ADDR=6,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template", "P=$(PREFIX),R=$(ROISTAT_PREFIX):8:,PORT=ROISTAT1,ADDR=7,TIMEOUT=1,NCHANS=$(NCHANS)")

# attribute plugin, 8 channels, and its time series (its address 1)
NDAttrConfigure("ATTR1", $(QSIZE), 0, "$(PORT)", 8, 8, 0, 0, 0)
dbLoadRecords("NDAttribute.template",  "P=$(PREFIX),R=$(ATTR_PREFIX):,PORT=ATTR1,ADDR=0,TIMEOUT=1,NCHANS=$(NCHANS),NDARRAY_PORT=$(PORT),NDARRAY_ADDR=8,ENABLED=1")
dbLoadRecords("NDAttributeN.template", "P=$(PREFIX),R=$(ATTR_PREFIX):1:,PORT=ATTR1,ADDR=0,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDAttributeN.template", "P=$(PREFIX),R=$(ATTR_PREFIX):2:,PORT=ATTR1,ADDR=1,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDAttributeN.template", "P=$(PREFIX),R=$(ATTR_PREFIX):3:,PORT=ATTR1,ADDR=2,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDAttributeN.template", "P=$(PREFIX),R=$(ATTR_PREFIX):4:,PORT=ATTR1,ADDR=3,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDAttributeN.template", "P=$(PREFIX),R=$(ATTR_PREFIX):5:,PORT=ATTR1,ADDR=4,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDAttributeN.template", "P=$(PREFIX),R=$(ATTR_PREFIX):6:,PORT=ATTR1,ADDR=5,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDAttributeN.template", "P=$(PREFIX),R=$(ATTR_PREFIX):7:,PORT=ATTR1,ADDR=6,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDAttributeN.template", "P=$(PREFIX),R=$(ATTR_PREFIX):8:,PORT=ATTR1,ADDR=7,TIMEOUT=1,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("ATTR1_TS", $(QSIZE), 0, "ATTR1", 1, 8)
dbLoadRecords("$(ADCORE)/db/NDTimeSeries.template", "P=$(PREFIX),R=$(ATTR_PREFIX):$(TS_PREFIX):,PORT=ATTR1_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=ATTR1,NDARRAY_ADDR=1,NCHANS=$(NCHANS),ENABLED=1")

# HDF5 file writer, fed by the driver
NDFileHDF5Configure("FileHDF1", $(QSIZE), 0, "$(PORT)", 8)
dbLoadRecords("NDFileHDF5.template", "P=$(PREFIX),R=$(HDF_PREFIX):,PORT=FileHDF1,ADDR=0,TIMEOUT=1,XMLSIZE=2048,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=8,ENABLED=1")
