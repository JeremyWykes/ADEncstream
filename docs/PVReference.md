# ADEncstream PV reference

A reference for the process variables the **ADEncstream** areaDetector IOC exposes, for people
building GUIs and client tools. It is written in the style of the areaDetector plugin docs and
`drvQuadEM`: an overview, the record-name layout, then grouped tables of records with their type,
access and meaning. The database templates define the record names and types; the driver and core C++ sources define
their behaviour. The [ADEncstream guide](ADENCSTREAM_GUIDE.md) explains the design. This reference describes the implemented behaviour.

Scope: ADEncstream's own records. The example IOC also loads standard areaDetector plugins
(NDPluginROIStat, NDPluginStats, NDPluginStdArrays, NDFileHDF5, NDPluginAttribute, NDTimeSeries);
their PVs are documented upstream in the
[ADCore documentation](https://areadetector.github.io/areaDetector/ADCore/ADCore.html) and are not
repeated here (see "Standard plugins" at the end for how the example wires them).

## Overview

ADEncstream is a C++ `asynNDArrayDriver` (like `drvQuadEM` and `ADCSimDetector`). Each configured driver instance serves one
Power Brick endpoint: a background streamer (**encstream**) on the Brick sends encoder positions and a gate
signal over UDP, and the IOC turns them into NDArrays and PVs. Two independent consumers run off the
same row stream:

- **Acquisition** — six trigger modes (Free Running / Gate High / Gate Low / Rising / Falling /
  Both); with `ArrayCallbacks` enabled, each successfully published frame produces arrays at
  the applicable addresses for standard AD plugins.
- **Continuous monitor** — per-axis mean/σ/min/max over a sliding or back-to-back window, published
  to plain PVs, independent of acquisition.

For a normal `Motor[x].ActPos` layout, positions are `ActPos − HomePos` in motor units,
relative to the motor zero. `ActPos` already includes the controller's position scaling; the IOC
does **not** multiply it by `PosSf` again. Acquisition freezes calibration at Acquire; the monitor
uses each received block's calibration generation. Custom position sources (including the test
layout below) are still reduced by the associated motor's `HomePos`. No dial/user-coordinate
conversion is applied.

Frame and monitor windows use the source sample clock, not UDP arrival spacing. Contact/stall
watchdogs use local elapsed time, and NDArray timestamps use the IOC clock. Local faults and
publication failures can also raise alarms.

## Record-name layout

Records live at three kinds of prefix. The example IOC (`iocs/ADEncstreamIOC`) sets `P` and the
plugin infixes in `st.cmd`; a site chooses its own.

| Group | Prefix | Loaded from | Example (example IOC) |
|---|---|---|---|
| Driver-level (stream, acquisition, monitor) | `$(P)$(R)` | `ADEncstream.template` + `NDArrayBase.template` | `ES1:` |
| Per motor *n* (calibration + monitor values) | `$(P)<n>:` | `ADEncstreamAxis.template` | `ES1:1:` … `ES1:8:` |
| NDArray-address diagnostics | per address (below) | `ADEncstreamArray.template` | `ES1:1:` (axis), `ES1:Main:`, `ES1:Index:`, `ES1:Status:` |

**NDArray addresses** (what each carries; the ADEncstream guide §3):

| Address | Array | Type | Dimensions | Array-record prefix (example) |
|---|---|---|---|---|
| 0–7 | position of `Motor[n]` (n = address+1), relative to `HomePos` | Float64 | `[nSamples]` | `ES1:<n>:` |
| 8 | all positions — **row n−1 = `Motor[n]`**, NaN rows for absent motors | Float64 | `[8, nSamples]` | `ES1:Main:` |
| 9 | sample index (the tick), *optional, off by default* | UInt64 | `[nSamples]` | `ES1:Index:` |
| 10 | status words — gate word if present, then encoder-status words, *optional, off by default* | UInt32 | `[nStatus, nSamples]` | `ES1:Status:` |

The index array contains reconstructed 64-bit servo-cycle ticks, **not** zero-based frame sample
numbers; consecutive received rows normally differ by GatherPeriod. FirstTick/LastTick use the
same clock. The status gate column contains the full gathered word, while gate decisions use the
configured bit. Dimension 0 varies fastest: main data is eight motor values for each sample,
then the next sample. A combined waveform consumer must account for this interleaving.

Per-axis records always follow the **motor number**, not the layout column, so a GUI panel for
`Motor[3]` is always `$(P)3:` whatever order the Brick gathers them in. A motor absent from the
current position layout has `Present_RBV = Absent` and no per-axis NDArray; its row in the main
array is NaN. Completed monitor updates give NaN/INVALID for an absent axis. Health-only updates
can retain older numbers with INVALID severity; always inspect health and alarms.

In the tables below, **Access** describes intended use: `R` for input/status records and `R/W` for
controls, usually paired with an `_RBV`. It does not configure Channel Access security. In paired
rows only the output record is a control. Input records generally use I/O Intr callbacks, which
can coalesce; this is not a promise of one CA event per source change. Receiver snapshots publish
approximately once per second and on relevant state changes. Slow Brick resource reads run
approximately once per minute. Values can remain at the last received value after contact is lost.

---

## 1. Stream / receiver

State of the link to the Brick and the receiver's accounting.
Receiver counters are cumulative for the driver instance's lifetime, across stream replacements;
Acquire and MonitorRun do not reset them. Recreating the instance/restarting the IOC resets them.
Unsigned 64-bit values exposed through `int64in` saturate at the signed 64-bit maximum.
Current-stream cursor, layout and final-checkpoint fields have a different lifetime.

| Record | Type | Access | Values / EGU | Description |
|---|---|---|---|---|
| `BrickAddress_RBV` | lsi | R | string | The configured Brick endpoint as given, with the resolved `a.b.c.d:port`. |
| `LocalPort_RBV` | longin | R | | The fixed local UDP port SUBSCRIBEs are sent from. |
| `SocketRcvBuf_RBV` | longin | R | bytes | Socket receive buffer the OS actually granted. |
| `StreamID_RBV` | stringin | R | hex | Current stream ID, empty before CONTEXT. A change identifies a replacement encstream stream, usually a process restart; it does not prove the controller rebooted. |
| `StreamState_RBV` | mbbi | R | None / Waiting for calibration (MINOR) / Streaming / Ended (MINOR) / Faulted (MAJOR) | The current stream's lifecycle state. |
| `StreamFault_RBV` | lsi | R | string | Receiver fault reason, plus the last received Brick STATUS state/code (0 if unavailable). A locally detected fault can therefore accompany a non-fault Brick code. Empty when not faulted. |
| `Contact_RBV` | bi | R | Lost (MAJOR) / OK | Contact watchdog: any valid packet from the configured source refreshes it. Armed at the first SUBSCRIBE attempt; default timeout 3 s. Initial OK is not proof of contact. Another subscriber monopolising the Brick is one possible cause of Lost, not a diagnosis. |
| `DataStalled_RBV` | bi | R | No / Stalled (MINOR) | DATA watchdog, independent of Acquire/MonitorRun. Armed while the stream is calibrated, not faulted and not final; default timeout 3 s since arming or the last admitted rows. Alarm only; final/fault disarms it. |
| `Identification_RBV` | lsi | R | string | Self-description from CONTEXT: version, source revision, layout/hash, host, firmware and CPU as space permits. The sender omits complete trailing fields to fit its bounded identification string; fields are not guaranteed present. |
| `ServoPeriod_RBV` | ai | R | s | The Brick's servo period (sets the sample-clock tick). |
| `GatherPeriod_RBV` | longin | R | servo cycles | `Gather.Period`: servo cycles per gathered row. |
| `NumAxes_RBV` | longin | R | | Number of position axes in the current layout. |
| `Motors_RBV` | lsi | R | e.g. `1,2,5` | The motor numbers present in the layout. |
| `CalGeneration_RBV` | longin | R |  | Latest accepted calibration generation, 0 before ready. The sender increments it when a stable poll detects changed HomePos/PosSf values; transient changes between polls may be missed. Published through a signed 32-bit field. |
| `RowsAdmitted_RBV` | int64in | R | rows | Total source rows admitted by this receiver instance, across all streams, including rows received while acquisition and monitor are stopped. |
| `RowsLost_RBV` | int64in | R | rows | Forward row gaps remaining after the received Brick skip report is applied. Signals missing admitted data, but does not distinguish network loss, local rejection or a lost skip report. Rows before the first admitted row are excluded. |
| `RowsSkipped_RBV` | int64in | R | rows | Portion of observed forward row gaps explained by the next admitted DATA packet's Brick catch-up report, capped at the gap length. A lost report can leave Brick skips counted as RowsLost instead. |
| `LateOrDuplicate_RBV` | int64in | R | datagrams | DATA datagrams dropped because their first row is below the current cursor; the entire datagram is dropped, even if it overlaps newer rows. |
| `SequenceGaps_RBV` | int64in | R |  | Count of sequence discontinuities, not the number of missing packets or rows. Metadata packets, loss, duplication, reordering and sender failures can affect it; use row accounting for data coverage. |
| `Rejected_RBV` | int64in | R |  | Aggregate codec errors plus unknown/unexpected packet types, oversized/truncated datagrams and rejected row sizes. Not a count of every ignored packet: late, foreign-source and other ignored traffic have separate accounting internally. |
| `StreamChanges_RBV` | int64in | R | | Times the current stream was replaced by a new stream ID. |
| `Faults_RBV` | int64in | R |  | Calls to the receiver's stream-fault handler, including repeated detections after the first reason has latched. Not a count of distinct faulted streams. |
| `SubscribesSent_RBV` | int64in | R |  | Successful local SUBSCRIBE sends, nominally one per second. Does not prove receipt or acceptance by the Brick. |
| `FinalNextRow_RBV` | int64in | R | row | The Brick's final-STATUS "next row"; the end-of-stream checkpoint. 0 until a final STATUS. |
| `Cursor_RBV` | int64in | R | row | Current stream's next expected row number, 0 before any rows. It advances across gaps. Subtracting lifetime RowsAdmitted does not yield a reliable late-join offset. |
| `GapFree_RBV` | bi | R | No / Yes | No observed row gap since this stream's first admitted row. Meaningful after rows arrive; says nothing about rows before joining or an unreceived tail. |
| `TailComplete_RBV` | bi | R | No / Yes | An anchored cursor equals the final next-row checkpoint. Can be Yes despite earlier gaps; combine with GapFree to assess the received interval. It does not certify rows before joining. |
| `RecentEvents_RBV` | lsi | R | text | Most recent receiver lifecycle/alarm events (up to 8 in a snapshot), with local monotonic seconds, event name and stream ID. Not a persistent log or wall-clock timestamp. |
| `DriverFault_RBV` | bi | R | OK / Fault (MAJOR) | Terminal local initialisation/thread/socket fault. Commands needing the engine fail; some cached parameter writes can still succeed without recovery. Restart/recreate the driver to recover. Acquire/Busy are released on this fault. |
| `DriverFaultMsg_RBV` | lsi | R | string | The driver-fault reason. |

**Brick resource use** — what encstream reports about itself from the Brick, in its STATUS packet.
Use trends to assess load: some backlog is normal; sustained growth or increasing overruns
indicates difficulty keeping up. Fields are 0 until a STATUS supplies them and retain the latest
received value if updates stop. The two Brick counters below are raw unsigned 32-bit sender values
carried by int64 records: they can wrap and reset when encstream restarts.

| Record | Type | Access | Values / EGU | Description | Updates |
|---|---|---|---|---|---|
| `BrickWorstLoop_RBV` | longin | R | µs | Maximum time between reader-loop wake-ups since the previous STATUS send attempt; includes scheduling/sleep time, not just CPU execution. | ≈1 s |
| `BrickBacklog_RBV` | longin | R | rows | Latest observed ring rows safe to read beyond the reader cursor, excluding the newest unsafe row; not a network queue size. | ≈1 s |
| `BrickOverruns_RBV` | int64in | R |  | Sender's current 32-bit count of ring catch-up events. | ≈1 s |
| `BrickSendFailures_RBV` | int64in | R |  | Sender's current 32-bit count of local send failures; not end-to-end UDP loss. | ≈1 s |
| `BrickVmRSS_RBV` | longin | R | kB | encstream resident memory (0 = unavailable). | ≈1 min |
| `BrickOpenFds_RBV` | longin | R | | encstream open file descriptors (0 = unavailable). | ≈1 min |

---

## 2. Acquisition

### 2.1 Base controls (from `NDArrayBase.template`, upstream)

The Acquire button and plugin-callback controls come from `asynNDArrayDriver`; ADEncstream governs
their behaviour. Full definitions are in the
[NDArray / asynNDArrayDriver docs](https://areadetector.github.io/areaDetector/ADCore/NDArray.html).

| Record | Type | Access | Description |
|---|---|---|---|
| `Acquire`, `Acquire_RBV`, `AcquireBusy` | bo, bi, busy | R/W, R, status | Write Acquire 1 to start or 0 to stop. Acquire_RBV remains active until assembly and pending driver output settle; AcquireBusy can additionally wait for plugins. Both active readbacks have MINOR severity. A refused start fails the write; see AcqMessage_RBV. Terminal driver faults release both even if plugins hold arrays. |
| `ArrayCallbacks`, `ArrayCallbacks_RBV` | bo, bi | R/W | Master enable for emitting NDArrays to plugins. Off ⇒ frames are assembled and counted but no arrays are built. |
| `WaitForPlugins` | bo | R/W | Include ADCore-tracked queued plugin arrays in AcquireBusy completion. This is not a guarantee that every downstream consumer received or saved every frame; inspect plugin drop/error counters too. |
| `NDAttributesFile` | waveform | R/W | CHAR waveform containing the XML attribute-file name, not XML content. Loaded site attributes are copied to emitted arrays; driver-reserved names are replaced. |
| `PoolUsedMem`, `PoolMaxMem` | ai, ai | R | Allocated NDArray buffer memory and configured limit, in MiB (template label MB). PoolUsedMem includes reusable free buffers. These names have **no _RBV suffix**. |
| `PoolAllocBuffers`, `PoolFreeBuffers`, `PoolUsedBuffers` | longin, longin, calc | R | Allocated, reusable free and allocated-minus-free buffer counts. The inherited template has no PoolFreeMem PV. |
| `PoolPollStats` | bo | R/W | Refresh pool statistics; inherited default scan is 1 second. ADEncstream also updates used memory/buffer counts in receiver snapshots. |

### 2.2 ADEncstream acquisition controls (latched at Acquire)

| Record | Type | Access | Values | Description |
|---|---|---|---|---|
| `AcquireMode`, `AcquireMode_RBV` | mbbo, mbbi | R/W | Continuous / Multiple / Single | 0 = Continuous until stopped; 1 = Multiple, NumAcquire frames; 2 = Single, one frame. Faults can invalidate any mode. |
| `NumAcquire`, `NumAcquire_RBV` | int64out, int64in | R/W | ≥ 1 | N frames for Multiple. |
| `TriggerMode`, `TriggerMode_RBV` | mbbo, mbbi | R/W | Free Running / Gate High / Gate Low / Rising / Falling / Both | Enum values 0–5 in the listed order; boundaries below. FramePoints/FrameTime apply only to Free Running. Other modes require a gate item. |
| `FramePoints`, `FramePoints_RBV` | longout, longin | R/W | 0 or positive sample slots | Free Running length selector: **0 uses `FrameTime`; a positive value sets the sample count and overrides `FrameTime`**. `_RBV` reports the effective sample count, including when time-based framing is selected. |
| `FrameTime`, `FrameTime_RBV` | ao, ai | R/W | positive seconds | Used only when **`FramePoints = 0`**. Rounded to the nearest sample slot (half-way rounds up), at least one. `_RBV` reports the effective duration, including when a positive `FramePoints` selects the length. |
| `MaxFrameSamples`, `MaxFrameSamples_RBV` | longout, longin | R/W | samples | Per-acquisition cap, 1 through the configured frame-pool capacity; does not resize the pool. Free Running lengths above it are refused; gate frames reaching it are emitted immediately as Overlong. |
| `MinCoverage`, `MinCoverage_RBV` | ao, ai | R/W | 0–1 | Below this fractional coverage a frame raises the degraded alarm. |
| `EnableSampleIndex`, `EnableSampleIndex_RBV` | bo, bi | R/W | Disable / Enable | Emit the optional sample-index array (address 9). |
| `EnableStatusArray`, `EnableStatusArray_RBV` | bo, bi | R/W | Disable / Enable | Emit address 10. Acquire is refused if enabled when the layout has neither a gate word nor encoder-status words. |

**Choosing a Free Running frame length:** set `FramePoints = 0` to use `FrameTime`.
If `FramePoints` is positive, changing `FrameTime` has no effect on frame length,
even after stopping and restarting acquisition. Both controls are ignored in gate-triggered modes.

Writes to acquisition settings are accepted while running, but become **pending settings for the
next Acquire**; they do not alter the active run. Most paired `_RBV` records mirror these pending
parameters, so they are not proof of the active run's configuration. FramePoints_RBV and
FrameTime_RBV are exceptions: they report the effective Free Running length.

The effective length is latched at the next accepted `Acquire = 1`. Stop acquisition and wait
for `Acquire_RBV` and `AcquireBusy` to show Done before changing the settings and restarting.
`FramePoints_RBV` retains the last accepted count (0 before any acquisition or in gate modes),
and `FrameTime_RBV` is NaN when that count is 0. They do not preview pending edits while stopped.
Implementation caveat: FrameTime_RBV multiplies that count by the **current stream** row interval;
after a stream replacement its value can change without a new Acquire. For historical frame
timing, use the frame's frozen ServoPeriod/GatherPeriod attributes. For example, at a 1 kHz row rate,
`FramePoints = 0` and `FrameTime = 0.25` produce 250-sample frames after restart.
With `FramePoints = 1000`, the same time setting produces 1000-sample frames instead.

This controls acquired NDArrays, not the continuous monitor, whose window and update
cadence are set by `MonitorAveragingTime` and `MonitorUpdatePeriod`.

Acquire starts the IOC framer, not encstream or the controller's gather. It requires an available,
calibrated, unfaulted stream, 1–8 position items assigned to Motor[1..8], and at most eight encoder
status words. Start refusals leave a reason in AcqMessage_RBV. The monitor can continue independently;
it maps Motor[1..8] and ignores position motors outside those slots.

**Trigger boundaries:** Free Running starts its first window at the first admitted row processed
after Acquire, then uses consecutive fixed windows. A later row closes a window; stopping or a
final STATUS does not flush a partial frame. Gate High/Low collect only the selected level,
opening on an observed transition into it and excluding the first row at the opposite level.
Rising/Falling collect from one qualifying edge to the next edge of the same polarity; **they are
not fixed-duration captures following an edge**. Both collects between successive edges of either
polarity. An edge's first observed row belongs to the new frame. No gate frame opens solely
because the gate is already active when acquisition is armed. Unsampled pulses cannot trigger.
At the cap, a gate frame is published and collection waits for the next qualifying opening edge.
Stop drops the frame still being assembled, keeps queued ordinary frames, and discards queued
empty-window runs; already in-flight output can finish.

### 2.3 ADEncstream acquisition status

Acquisition counters/flags reset on an accepted Acquire and remain available after completion.
Publication is the driver's handoff/counting stage, not confirmation of downstream plugin delivery.

| Record | Type | Access | Values | Description |
|---|---|---|---|---|
| `AcqState_RBV` | mbbi | R | Idle / Acquiring / Assembled / Stopping / Complete / Invalid (MAJOR) | Acquisition state machine. `Invalid` = the run was invalidated (e.g. the IOC could not keep up, or the stream faulted). |
| `AcqMessage_RBV` | lsi | R | string | Reason for a refused start or an invalidation; also a late control result. |
| `NumAcquired_RBV` | int64in | R | frames | Frames successfully handled by the publication stage this acquisition. Also increments when ArrayCallbacks is off, although no arrays are built. Does not confirm plugin completion. |
| `FramesAssembled_RBV` | int64in | R | frames | Logical frames successfully queued, including all windows represented by an empty-run entry. Can exceed NumAcquired by many queued/in-flight frames. |
| `FramesWithGaps_RBV` | int64in | R | frames | Completed nonempty frames with missing interior rows or an inferred start/end edge in a gap. Edge uncertainty can occur at Coverage = 1. Compressed empty-window runs are counted separately, not here. |
| `EmptyFrames_RBV` | int64in | R | frames | Completely empty Free Running windows successfully queued. Each published empty window uses one NaN position placeholder per present axis and main-array row, with NumSamples = 0; index/status placeholders are 0. |
| `OverlongFrames_RBV` | int64in | R | frames | Gate frames truncated at `MaxFrameSamples`. |
| `CalChangedFrames_RBV` | int64in | R | frames | Completed nonempty frames containing any row block whose calibration generation differs from that frozen at Acquire; the transition need not have occurred inside that frame. |
| `MinCoverageSeen_RBV` | ai | R | 0–1 | Lowest completed-frame coverage; NaN before any frame completes. Missing slots are not filled into ordinary position arrays. |
| `Degraded_RBV` | bi | R | No / Degraded (MINOR) | Latched for coverage below the acquisition's MinCoverage. Current implementation also latches it for a queued empty-window run even when MinCoverage = 0. Edge uncertainty alone need not set it. |
| `NotDelivered_RBV` | int64in | R | frames | Frames unable to obtain adequate frame storage, rejected at queue insertion or failed at publication; nonzero raises MAJOR and invalidates acquisition. Does not include downstream plugin drops or the in-progress partial frame discarded by Stop. |
| `CutByStop_RBV` | int64in | R | frames | Logical empty-window frames removed from queued empty-run entries by Stop; excludes ordinary queued frames, in-flight work and the partial assembling frame. |
| `SourceStopped_RBV` | bi | R | No / Stopped (MINOR) | Final STATUS seen for the acquisition's stream, including if it was already final at Acquire. Does not itself invalidate or flush a partial frame; late tail rows can still complete frames. |
| `SourceStopRow_RBV` | int64in | R | row | Final STATUS next-row checkpoint, 0 until final. It is the sender reader cursor, not necessarily the hardware gather stop position. |
| `QueueEntries_RBV` | longin | R |  | Queued entries only, excluding in-flight work; an entry can represent many empty-window frames. |
| `OutstandingFrames_RBV` | int64in | R | frames | Logical frames in queued entries **plus in-flight work** awaiting publication accounting; excludes the frame still being assembled. |

---

## 3. Continuous monitor

Per-axis statistics over an averaging window, independent of acquisition. Driver-level control/status here; the per-axis values are in §4.

| Record | Type | Access | Values | Description |
|---|---|---|---|---|
| `MonitorRun`, `MonitorRun_RBV` | bo, bi | R/W | Stop / Run | Start/stop the monitor (no automatic start). |
| `MonitorAveragingTime`, `MonitorAveragingTime_RBV` | ao, ai | R/W | s | Requested window T; _RBV is the effective window, an integer number of effective update periods. NaN when no layout is available. |
| `MonitorUpdatePeriod`, `MonitorUpdatePeriod_RBV` | ao, ai | R/W | s | Requested U; **0 means use T**. Effective U is rounded to whole source rows, with a minimum of 10 ms. _RBV reports that effective value, or NaN without a layout. |
| `MonitorMinCoverage` | ao | R/W | 0–1 | Fractional coverage threshold for MINOR on otherwise valid axis/gate values; mixed calibration generations also cause MINOR. INVALID health/absence takes precedence. No paired _RBV record. |
| `MonitorHealth_RBV` | mbbi | R | Stopped / No stream / Waiting / OK / Stalled / Source stopped / Faulted / Replaced / Unsupported | Enum 0–8 in the listed order. OK, Stopped and Waiting have no severity on this health record; other states have MAJOR. Numeric monitor data may retain older values; inspect their alarms too. |
| `MonitorSamples_RBV` | int64in | R | samples | Rows in the last update's window. |
| `MonitorExpected_RBV` | int64in | R | samples | Expected rows for a full window (T in rows). |
| `MonitorCoverage_RBV` | ai | R | 0–1 | `Samples / Expected`. |
| `MonitorCalChanged_RBV` | bi | R | No / Changed | The completed window contains more than one calibration generation. Unlike acquisition, the monitor subtracts each block's own HomePos; mixed windows are flagged MINOR. |
| `MonitorUpdatesSkipped_RBV` | int64in | R |  | Cumulative intermediate completed-window updates coalesced by the monitor: at most one numerical update is emitted per input-row-block call. Lifetime counter, not reset by restarting the monitor; does not include callback-mailbox or CA coalescing. |
| `MonitorGate_RBV` | bi | R | Low / High | Gate bit at the last received sample in the latest **completed** window, not a live input PV. May disagree with the window's high fraction. INVALID without gate data or on unhealthy/stopped monitor updates. |
| `MonitorGateEdges_RBV` | longin | R | count | Observed transitions between received gate samples in the completed window. Across a gap, differing endpoint levels count once; hidden transitions are unknowable. Does not count a transition from outside the window. Exposed through signed 32-bit longin. |
| `MonitorGateActiveFraction_RBV` | ai | R | 0–1 | Fraction of received gate samples that are high, regardless of TriggerMode. NaN/INVALID without gate samples; missing samples are not imputed. Approximates duty cycle only with representative sampling. |

---

A requested T must be finite and positive. After replacing U = 0 with T, require
10 ms ≤ U ≤ T and `round(T/U)` between 1 and 1000. Half-way rounds up. With row interval
`d = ServoPeriod × GatherPeriod`, U is rounded to whole rows (raised if needed to preserve
the 10 ms minimum), then `effective T = round(T/U) × effective U`. For example, T = 1 s and
U = 0.3 s at a 1 kHz row rate give effective U = 0.3 s and T = 0.9 s. Check both _RBVs.
Unrepresentable settings are refused; a later incompatible stream can show Unsupported.

Monitor configuration writes, including MinCoverage, can be made while running. An accepted write
restarts window history; a refused write leaves the previous configuration. MonitorRun does not
start acquisition, and acquisition need not run for monitoring. Windows advance on received source
rows, not on an IOC timer. Sliding startup windows can have less than full coverage until T of
source data is available. Stopping, stalling or other health-only changes generally retain numeric
axis/gate values and change their severity to INVALID. A new history starts without valid values;
the first completed update replaces retained data. No missing row is interpolated.

Implementation caveat: the terminal local DriverFault path invalidates axis statistics but currently
leaves the three gate PV alarms/values as they were. A GUI must also check DriverFault_RBV and
MonitorHealth_RBV before treating gate values as valid; gate PV severity alone is insufficient.

## 4. Per-axis records (`$(P)<n>:`, Motor[n])

Loaded once per motor from `ADEncstreamAxis.template`. The monitor values carry the slot's alarm
severity: MINOR for coverage below MonitorMinCoverage or mixed calibration generations;
INVALID when stopped/unhealthy, without rows, or without that position motor. Numeric records
do not declare EGU in the template; the units described here are semantic units.

| Record | Type | Access | Values / EGU | Description |
|---|---|---|---|---|
| `Mean_RBV` | ai | R | motor units | Mean of the layout's position source for Motor[n], reduced by the HomePos associated with each received block. For normal layouts this is ActPos − HomePos. |
| `StdDev_RBV` | ai | R | motor units | Population standard deviation over the window. |
| `Min_RBV` | ai | R | motor units | Minimum over the window. |
| `Max_RBV` | ai | R | motor units | Maximum over the window. |
| `StatusOr_RBV` | longin | R | bitmask | Bitwise OR of this motor's role-4 encoder-status words over the window (normally BiSS). 0/INVALID on a completed window without status. Treat as a 32-bit mask: high bit set can display as a negative longin. |
| `HomePos_RBV` | ai | R | motor units | Latest calibration HomePos received for this motor, in motor units; NaN when unavailable. Informational, independent of MonitorRun, and not the necessarily frozen acquisition zero. No freshness alarm is supplied here. |
| `PosSf_RBV` | ai | R |  | Latest received controller position scale, motor units per source unit; NaN when unavailable. Informational: the IOC does not apply it again to ActPos. Can retain an old value without a freshness alarm. |
| `Present_RBV` | bi | R | Absent / Present | Motor[n] has a position-role item in the current layout; does not indicate physical encoder connection. A status-only item does not make it Present. |
| `Name` | stringout | R/W | string | Local human-readable string initialised from NAME, for screens. Does not rename the controller motor or frame attributes. |

---

## 5. NDArray-address diagnostics (`ADEncstreamArray.template`)

Loaded per NDArray address (the per-axis prefix for 0–7; `Main:`/`Index:`/`Status:` for 8/9/10).
Standard read-only array diagnostics; the array data itself is consumed by plugins (std arrays,
ROIStat, Stats, HDF5), not published as a driver PV.

| Record | Type | Access | Values | Description |
|---|---|---|---|---|
| `ArrayCounter_RBV` | longin | R |  | Per-address emitted-array count; retained across Acquire, wraps from INT_MAX to 0. Does not increase while ArrayCallbacks is off; inherited address-0 ArrayCounter can reset that address's counter. |
| `ArraySizeX_RBV`, `ArraySizeY_RBV` | longin | R | elements | Size of dimensions 0 and 1 of the last array; Y is 0 for a 1-D array. Main X = 8, Y = sample count (or 1 for an empty placeholder). |
| `ArraySize_RBV` | longin | R | bytes | Physical size in bytes of the last emitted array, including the one-sample placeholder for an empty frame; not the NumSamples attribute. |
| `NDimensions_RBV` | longin | R | | Number of dimensions. |
| `DataType_RBV` | mbbi | R | Int8…Float64 | NDArray element type (Float64 for positions, UInt64 for the index, UInt32 for status words). |
| `UniqueId_RBV` | longin | R |  | Driver-wide emitted-frame ID shared by all arrays of that frame, retained across Acquire; runs 1..INT_MAX then wraps to 1. Not the per-acquisition FrameNumber. |
| `TimeStamp_RBV` | ai | R | s | IOC publication timestamp via ADCore updateTimeStamps, seconds since the EPICS epoch (1990-01-01), shared by a frame's arrays. Not a source sample time; custom asyn timestamp sources can override the default clock. |

---

These diagnostics retain the last emitted array's values while stopped, with callbacks off, or
when an optional/absent-axis address emits nothing. They do not attest to current source health.
Arrays contain **received samples only**; ordinary missing rows shorten the sample dimension,
while NumSamples and Coverage describe completeness. Array dimensions alone cannot prove coverage.

## 6. Per-frame NDAttributes

Not driver PVs, but every published NDArray carries this attribute list; a GUI
can extract numeric attributes through **NDPluginAttribute** or inspect stored attributes in HDF5.
String attributes require a consumer that supports strings. All reserved attributes below are
present on every published array; site attributes from NDAttributesFile are added, but reserved
names are replaced with driver values. Inapplicable values use documented NaN, 0 or empty strings
(0 is also a legitimate clock/count value, so inspect the accompanying flags). Types:
`Int32`, `UInt32`, `UInt64`, `Float64`, `String`.

| Attribute | Type | Meaning |
|---|---|---|
| `TriggerMode` | Int32 | 0–5, the trigger mode. |
| `AcqEpoch` | UInt64 | The acquisition the frame belongs to. |
| `FrameNumber` | UInt64 | 1-based logical frame number since Acquire. Counts assembled windows/observed qualifying frames, not every physical gate edge; active-at-arm and unsampled edges are excluded. |
| `StreamID` | UInt32 | The stream the frame came from. |
| `Motors` | String | Position motors present, e.g. "1,2,5". |
| `StatusMotors` | String | Emitted status columns' motor numbers in order, gate first as 0 if present. Empty string when EnableStatusArray was off at Acquire, even if the layout has status. |
| `ServoPeriod` | Float64 | Seconds per servo cycle. |
| `GatherPeriod` | UInt32 | Servo cycles per row. |
| `NumSamples` | UInt32 | Received samples in the frame (0 for an empty placeholder). Interpret with ExpectedSamples, Coverage and edge-gap flags. |
| `ExpectedSamples`, `MissingRows` | UInt64 | Free Running: configured slots and slots minus received samples. Gate modes: slots from FirstTick through LastTick inclusive and slots minus samples; uncertain boundary gaps lie outside this span. |
| `Coverage` | Float64 | `NumSamples / ExpectedSamples`, 0–1. |
| `FirstTick`, `LastTick` | UInt64 | First/last observed sample on the sample clock (0 for an empty frame). |
| `FirstServoCount`, `LastServoCount` | UInt32 | First/last observed `ServoCount` (0 for an empty frame). |
| `WindowStart`, `WindowEnd` | UInt64 | Free Running: scheduled [start, end) tick window. Gate modes: [FirstTick, LastTick + GatherPeriod), the observed sample span, not exact physical trigger times. Empty Free Running windows retain scheduled bounds. |
| `StartEdgeInGap`, `EndEdgeInGap` | Int32 | 0/1: an opening/closing transition was inferred between differing observed levels separated by missing rows; its exact position is unknown. |
| `StartGapRows`, `EndGapRows` | UInt64 | Number of unreceived row slots between the observed samples surrounding each inferred boundary transition; not added to gate-mode ExpectedSamples. |
| `Overlong`, `CalibrationChanged`, `Empty` | Int32 | 0/1 flags: capped at MaxFrameSamples; a block differs from the calibration generation frozen at Acquire; no received samples. CalibrationChanged does not require a transition within this frame. |
| `StatusOr` | UInt32 | OR of all role-4 encoder-status words (normally BiSS) over the frame; excludes the gate word. Computed even if the optional status array is disabled; 0 without status or samples. |
| `HasGate` | Int32 | 0/1: the layout has a gate item. |
| `GateAtStart`, `GateAtEnd` | Int32 | 0/1 gate bit at the first/last **received sample**, not an estimate of the physical boundary; 0 without a gate or samples. |
| `GateEdges` | UInt32 | Observed transitions between consecutive received samples inside the frame, including comparisons across gaps; hidden edges cannot be counted. Normally 0 in Gate High/Low and Both; a full Rising/Falling cycle has 1 return edge, but a capped frame may have 0. The closing edge belongs to the next frame. Use MonitorGateEdges_RBV for an acquisition-independent window count. |
| `GateActiveFraction` | Float64 | Fraction of received samples whose raw gate bit is **high**, even in Gate Low mode (NaN without gate/samples). Missing rows are not imputed. |
| `HomePos1`…`HomePos8`, `PosSf1`…`PosSf8` | Float64 | Calibration frozen at accepted Acquire, per position motor (NaN for absent motors). HomePos is subtracted from positions; PosSf is informational, not an extra conversion factor. |

---

## Standard plugins (example IOC)

The example IOC feeds its arrays to standard areaDetector plugins; their PVs are documented
upstream. In the example (`st.cmd`, one-letter prefixes, motor number first):

| Plugin | Example prefix | Fed from | Upstream docs |
|---|---|---|---|
| NDPluginStdArrays (per axis) | `$(P)<n>:W:` | driver addr n−1 | [NDPluginStdArrays](https://areadetector.github.io/areaDetector/ADCore/NDPluginStdArrays.html) |
| NDPluginStats (per axis) | `$(P)<n>:S:`, `$(P)<n>:S:T:` | driver addr n−1 | [NDPluginStats](https://areadetector.github.io/areaDetector/ADCore/NDPluginStats.html) |
| NDPluginROIStat (8 ROIs on the main array) | `$(P)R:`, `$(P)R:1:`… | driver addr 8 | [NDPluginROIStat](https://areadetector.github.io/areaDetector/ADCore/NDPluginROIStat.html) |
| NDPluginAttribute (+ time series) | `$(P)A:`, `$(P)A:T:` | driver addr 8 | [NDPluginAttribute](https://areadetector.github.io/areaDetector/ADCore/NDPluginAttribute.html) |
| NDFileHDF5 | `$(P)H:` | driver addr 8 | [NDFileHDF5](https://areadetector.github.io/areaDetector/ADCore/NDFileHDF5.html) |

The array waveform a GUI usually plots is the std-arrays plugin's `$(P)<n>:W:ArrayData` (one axis)
or, for all eight, an **additional** std-arrays plugin connected to driver address 8.
The shipped example has no main-array waveform plugin: `$(P)Main:` contains diagnostics only,
so `ES1:Main:ArrayData` does not exist. Configure/load a new NDStdArrays instance and use its
chosen prefix for a combined waveform. Per-frame scalars come from the ROIStat
ROIs, the per-axis Stats (`$(P)<n>:S:MeanValue_RBV` etc.) and the attribute plugin's time series.

> Note on ROIStat in this ADCore revision: ROIStat clamps and writes back
> its ROI size, so after a short frame a later longer frame can be averaged over the retained shorter ROI. Use the per-axis **Stats**
> plugins for reliable frame means until the upstream fix.

## Generating test data (simulator or unwired Brick)

A simulator or an unwired test Brick may have flat position data. This is not guaranteed:
ActPos depends on feedback configuration, and an unwired encoder can return nonzero data.
To exercise the pipeline with controlled values, use a custom layout gathering writable doubles
and drive them with a background PLC. This changes the stream's data source, not real motors.

Use only a test controller with the proposed PLC slots and shared-memory locations available.
Sys.Ddata and Sys.Udata are overlapping views of the same shared buffer. This example uses
Ddata[1..8] (bytes 8..71) and Udata[0] (bytes 0..3), which do not overlap each other, but they
must also be unused by the rest of the project. Deployment owns/reconfigures Gather; do not run
it alongside another gather user. The associated Motor[1..8].HomePos is still subtracted by the
IOC, so the plotted signals have that offset unless HomePos is zero.

1. Deploy encstream gathering `Sys.Ddata[1..8]` as the positions (from the encstream repo):

   ```
   python tools/brick_deploy.py root@<controller> deploy --positions "Sys.Ddata[i]"
   ```

2. Load and enable this test PLC at the IDE terminal. It writes a distinct nominal 50 ms sine to each
   `Sys.Ddata[1..8]`. **Use a PLC number not already in your project** (31 shown), and do **not**
   `save` — keep it transient so it doesn't alter the saved project:

   ```
   open plc 31
   Sys.Ddata[1]=5000*sin(Sys.Time*125.6637)
   Sys.Ddata[2]=4000*sin(Sys.Time*125.6637+0.8)
   Sys.Ddata[3]=3000*sin(Sys.Time*125.6637+1.6)
   Sys.Ddata[4]=2000*sin(Sys.Time*125.6637+2.4)
   Sys.Ddata[5]=5000*sin(Sys.Time*125.6637+3.1)
   Sys.Ddata[6]=4000*sin(Sys.Time*125.6637+3.9)
   Sys.Ddata[7]=3000*sin(Sys.Time*125.6637+4.7)
   Sys.Ddata[8]=2000*sin(Sys.Time*125.6637+5.5)
   close
   enable plc 31
   ```

   `125.6637 ≈ 2π / 0.05` gives a nominal 50 ms period (`sin` takes radians; Sys.Time is
   seconds since power-up). For another period T seconds, replace 125.6637 with 2π/T. The PLC
   updates asynchronously, so values are held between PLC executions. `disable plc 31` to stop.

   To also exercise the gate (the `MonitorGate*` PVs), drive the gate stand-in `Sys.Udata[0]` from
   a second transient PLC (unused slot 30; the generated layout above and sim_test use bit 0
   of Sys.Udata[0]):

   ```
   open plc 30
   Sys.Udata[0]=int((Sys.Time*5-int(Sys.Time*5))*2)
   close
   enable plc 30
   ```

   For positive Sys.Time, this requests a nominal 5 Hz, 50%-high 0/1 waveform. int() truncates
   toward zero; PLC execution and gather sampling determine the observed transitions.
   To observe it with a 1-second monitor window and 20 ms updates, use:

   ```
   caput ES1:MonitorRun 0
   caput ES1:MonitorUpdatePeriod 0   # Select U = T before changing T
   caput ES1:MonitorAveragingTime 1
   caput ES1:MonitorUpdatePeriod 0.02
   caput ES1:MonitorRun 1
   ```

   Once the window fills, expect roughly 10 observed edges per second and a high fraction near
   0.5 if sampling resolves each half-cycle. MonitorGate_RBV shows the last sample in each
   completed window. With a slow or commensurate update period it can appear constant by aliasing,
   even while the edge count changes. These monitor PVs do not require Acquire.
   `disable plc 30` to stop.

3. On the IOC, enable a frame path and acquire (example prefix `ES1:`):

   ```
   caput ES1:Acquire 0
   # Wait for Acquire_RBV and AcquireBusy to show Done before continuing.
   caput ES1:ArrayCallbacks 1
   caput ES1:1:W:EnableCallbacks 1
   caput ES1:TriggerMode 0        # Free Running
   caput ES1:AcquireMode 0        # Continuous
   caput ES1:FramePoints 0       # Use FrameTime; positive FramePoints overrides it
   caput ES1:FrameTime 0.1
   caput ES1:Acquire 1
   ```

   Then plot `ES1:1:W:ArrayData`; its .NORD is the published waveform length. For other axes,
   use ES1:2:W:ArrayData through ES1:8:W:ArrayData. A combined waveform needs the additional
   plugin described above. A CA client needs a sufficient EPICS_CA_MAX_ARRAY_BYTES setting
   (the example IOC allows 2,000,000 bytes for a 200,000-element Float64 waveform).
   Monitor PVs populate only with MonitorRun on; per-axis Stats also need their EnableCallbacks
   and ComputeStatistics controls enabled, as configured in encstreamAxisSettings.cmd.

The trace is coarse/jagged on a simulator: the background PLC rewrites `Sys.Ddata` more slowly and
less regularly than the servo-rate gather, and simulator scheduling can add jitter. A hardware
gather uses the configured source clock, but the background PLC still updates asynchronously;
repeated/jagged values are expected. UDP arrival jitter does not redefine the sample clock, and
missing received rows must still be detected through the coverage and gap diagnostics. When
finished, disable both test PLCs and redeploy the intended real-data layout. Avoid saving this
transient test setup; disabling the PLCs alone does not restore the original source layout.
