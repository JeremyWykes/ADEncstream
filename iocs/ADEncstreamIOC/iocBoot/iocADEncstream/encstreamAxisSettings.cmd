# encstreamAxisSettings.cmd - one motor's plugin settings after iocInit, once per motor:
#     iocshLoad("encstreamAxisSettings.cmd", "AXIS=<n>")
# The std-arrays and stats plugins pass no arrays on: with ArrayCallbacks on, each would forward a
# full copy of every frame from the driver's NDArray pool. The stats time series (the
# stats plugin's address 1) does not depend on it.
dbpf("$(PREFIX)$(AXIS):$(ARRAY_PREFIX):ArrayCallbacks", "0")
dbpf("$(PREFIX)$(AXIS):$(STATS_PREFIX):ArrayCallbacks", "0")
dbpf("$(PREFIX)$(AXIS):$(STATS_PREFIX):ComputeStatistics", "1")
