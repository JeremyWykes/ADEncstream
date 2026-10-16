# ADEncstream: a guided tour of the IOC-side module

> A readable orientation to **ADEncstream**, the EPICS areaDetector module that turns encstream's
> UDP stream into NDArrays and PVs. It covers the design, the data flow and a component map — what
> each piece does and which file to open — for a reader who knows EPICS and areaDetector in outline
> and wants to see how this module is put together. It defers the how-to: building and running an
> IOC are in [`../README.md`](../README.md); every PV is in [`PVReference.md`](PVReference.md). The
> Brick that produces the stream, and the wire protocol, are documented in the **encstream** repo.
> Source is under `ADEncstreamApp/src/`.

## What it is

ADEncstream is an areaDetector **driver module**, not an IOC: site IOCs are built on it (the repo
ships one example under `iocs/`). It subscribes to one encstream sender on a Brick, decodes the
`DATA`/`CONTEXT`/`CALIBRATION`/`STATUS` packets, and presents two things to EPICS:

- **NDArrays** — one *frame* per acquisition trigger, fanned out to the standard areaDetector plugins
  (std arrays, Stats, ROIStat, Attribute, HDF5) for plotting, statistics and files;
- **PVs** — the stream's health and counters, the acquisition state, a continuous per-axis monitor,
  and per-axis calibration, as ordinary records.

The design has two deliberate properties worth stating up front, because they explain almost every
file:

1. **An EPICS-free, sans-I/O core.** The decode/framing/monitor logic is portable C++11 with no
   sockets and no asyn — it is driven with constructed datagrams and unit-tested on a PC. Socket I/O
   and asyn live only in the thin driver layer on top.
2. **Two worker threads, with the main data and control flow crossing through three small, bounded,
   libCom-only objects.** The hot receive-and-decode path takes no asyn lock and doesn't allocate; it
   hands work to the areaDetector callback thread through a frame queue, a mailbox and a command slot.
   (Those three are the main channels, not the only shared state — see below.)

## The data path

```
   UDP datagrams
        |                         ----- receiver thread (driver) -------------------------
        v                        |                                                         |
   socket recv  --> Engine.datagram() --> Receiver  --decoded rows-->  FanOut              |
                                   (decode, admit,          (one RowSink)   |   \           |
                                    gaps, stream state)                     v    v          |
                                                                        Framer   Monitor    |
                                                                          |         |       |
                                                   frames / empty runs    |         | windowed
                                                          v               |         v  stats
                                                     FrameQueue           |      Mailbox <-- Snapshot
                                                          |               |         |   (counters,
        ----- areaDetector callback thread --------------|---------------|---------|    state, cal)
       |                                                  v                         v
       |   Work (frame) --> build NDArrays + attributes --> plugins       take under the gate -->
       |                    (addr 0-7 per axis, 8 main,                    set PVs (stream, monitor,
       |                     9 index, 10 status)                            acquisition, calibration)
        -------------------------------------------------------------------------------------------
   control: records (Acquire/Stop, MonitorRun/Config) --> CommandSlots --> Engine applies; the other
            acquisition settings (TriggerMode, AcquireMode, frame size) are latched at Acquire
```

- **In** at the top: the driver's **receiver thread** reads a datagram off the socket and feeds it to
  the `Engine`. The `Receiver` validates and decodes it, tracks the stream, accounts for gaps, and —
  for `DATA` — emits decoded rows to a `RowSink`.
- The `FanOut` hands every row (and every gap and lifecycle event) to **two** independent consumers:
  the **Framer** (groups rows into acquisition frames) and the **Monitor** (continuous per-axis
  statistics). Each copies what it needs and returns promptly.
- **Out** at the bottom: frames go through the **FrameQueue** to the callback thread, which builds
  NDArrays and publishes them to plugins; monitor updates and periodic **Snapshots** (counters,
  stream state, calibration) go through the **Mailbox** to the callback thread, which sets the PVs.
- **Control** flows the other way: a record write (in the asyn/record calling context, *not* the
  callback worker) drops a `Command` in the **CommandSlots** for the receiver thread to apply and
  report back. Only Acquire/Stop and the monitor run/config are commands; TriggerMode, AcquireMode,
  frame size and the rest are stored in driver parameters and **latched into the Acquire command**.

## The two threads and the three objects between them

This is the part a careful reader wonders about, so it's worth being explicit.

- **The receiver thread** (in the driver) owns the socket and the whole EPICS-free core. Each pass it
  moves one datagram in, lets the `Engine` decode/frame/monitor it, sends any due `SUBSCRIBE`
  heartbeat, and calls `Engine::service()` to apply commands, read back publication counts, update
  the acquisition state and post snapshots. It is single-threaded; the hot DATA path is
  allocation-free (command, fault and late-result handling, also on this thread, do build strings).
- **The areaDetector callback thread** does the EPICS-visible work: it takes frames from the queue,
  builds NDArrays and runs plugin callbacks, and takes the latest monitor update / snapshot from the
  mailbox and writes the parameters and records. asyn locking lives here, off the hot path.
- Between them, **three bounded objects using only EPICS libCom (mutex + event)**, no asyn. Their
  pool/queue/slot storage is sized once at construction; the normal DATA path doesn't `malloc`, though
  command and fault *results* carry `std::string`:

| Object | File | Carries | Note |
|---|---|---|---|
| `FrameQueue` | `esFrameQueue.h/.cpp` | frames and empty-runs, in frame-number order, plus a publication **ledger** | A pool of `depth + 2` frame objects sized **once** at construction; the callback thread takes a frame (or a batch of empty frames), publishes, and reports `published`/`failed`; the receiver thread reads those counts back. |
| `Mailbox` | `esEngine.h/.cpp` | the latest `MonitorUpdate` and the latest `Snapshot`, each sequenced | A **publication gate**: the callback thread publishes a taken entry only if, under the gate, it is still the newest posted (and of the current monitor run), so a superseded value is never pushed to a PV. |
| `CommandSlots` | `esCommands.h/.cpp` | control commands and their results | How a record write (Acquire, trigger mode, monitor config, …) reaches the receiver thread, and how the result comes back. |

Why this shape: the receive-decode-frame path must keep up with ~1000 packets/s without blocking on
an asyn lock or the network, and areaDetector's NDArray publication and record I/O must not stall it.
The queue/mailbox/slots are the main channels, each with one writer and one reader per direction, so
the locking is small. (There is a little more shared state — a fault lock and message, an `exiting_`
flag, wake/completion events, pool accounting — and the asyn record writers and the refusal timer are
further contexts that touch driver state; publication is not lock-free and can briefly wait on a
gate.)

## The EPICS-free core (portable, sans-I/O)

Driven by datagrams, tested on a PC with no socket or IOC. One `RowSink` interface (`gap`, `rows`,
`event`) threads through it.

| File | Role |
|---|---|
| `esReceiver.{h,cpp}` | The heart: datagrams → decoded rows. Five stages — datagram checks, structural decode, stream admission (identity, per-stream facts, calibration generations), DATA admission (packet + row checks, the `ServoCount` continuity check, an in-order cursor), then emit to the `RowSink`. Also owns the `SUBSCRIBE` heartbeat, the contact/data watchdogs, the counters and the event log. See `the ADEncstream guide`. |
| `esWire.h` | A thin C++ wrapper over the vendored C codec (`vendor/encstream/wire.{c,h}`, copied unchanged from the Brick) so both ends decode identically. |
| `esWindow.h` | Sample-clock windows (start/length in rows), shared by the framer and monitor. |
| `esFramer.{h,cpp}` | Groups rows into acquisition **frames** by trigger mode (Free Running by points/time; Gate High/Low/Rising/Falling/Both), tracks gate state and edges, and owns each frame's lifecycle. See `the ADEncstream guide` §4. |
| `esMonitor.{h,cpp}` | The **continuous monitor**: per-axis mean/stddev/min/max/status-OR over an averaging window, plus the gate (level, edges, active fraction), independent of acquisition. See `the ADEncstream guide` §5. |
| `esFanOut.h` | Forwards every `RowSink` call to the framer and the monitor (framer first). |
| `esTrace.{h,cpp}` | Records and replays traffic traces, for the differential/golden tests. |

The receiver finds items **by role and axis, never by name** (the layout travels in `CONTEXT`), so a
re-homed axis or a synthetic test layout (e.g. `Sys.Ddata`) decodes the same way.

## The thread-crossing layer and the driver

| File | Role |
|---|---|
| `esFrameQueue.{h,cpp}` | The frame pool, the queue, and the publication ledger (above). The framer's `FrameOutput`; the callback thread's work source. |
| `esCommands.{h,cpp}` | The control command slots and their results. |
| `esEngine.{h,cpp}` | The receiver thread's work (no socket, asyn or ADCore — but EPICS **libCom** for its mutexes/events): owns the `Receiver`/`FanOut`/`Framer`/`Monitor`, services the commands, drives the displayed acquisition state (`AcqDisplay`: Idle / Acquiring / Assembled / Stopping / Complete / Invalid — the framer's internal `AcqState` has fewer), and posts monitor updates and `Snapshot`s to the `Mailbox`. Testable without a socket. |
| `ADEncstream.{h,cpp}` | The driver proper, an **`asynNDArrayDriver`**: creates the socket, the two threads, the `FrameQueue`/`Mailbox`/`CommandSlots`; defines the asyn parameters and records; builds NDArrays from frames and publishes them; writes the stream, monitor, acquisition and calibration PVs; and maps control writes to commands. See `the ADEncstream guide`. |

The split means the whole pipeline up to and including the acquisition state machine (`esEngine` and
below) is exercised by `make runtests` on any host; only `ADEncstream` itself needs asyn and ADCore.

## What comes out: NDArrays, attributes and PVs

Each acquisition frame is published on a set of NDArray **addresses**, so plugins can take one axis
or all of them:

| Address | NDArray |
|---|---|
| 0–7 | one per **present** position axis (address = motor number − 1), Float64 |
| 8 | the **main** array, Float64 `[8, S]` in motor-number slots (NaN for absent motors — not a compact stack) |
| 9 | the sample **index** (optional, `EnableSampleIndex`), UInt64 ticks |
| 10 | the **status** words (optional, `EnableStatusArray`), UInt32 `[nStatus, S]`; the gate word is first **only if a gate is configured**, else the encoder-status words lead |

With `ArrayCallbacks` off, frames are still assembled and counted but no NDArrays are emitted.

Every published NDArray carries a list of per-frame **NDAttributes** — trigger mode, acquisition
epoch and frame number, sample count and window, coverage and missing-rows, the gate fields
(`HasGate`/`GateAtStart`/`GateAtEnd`/`GateEdges`/`GateActiveFraction`), the status OR, and the frozen
per-motor calibration. The example IOC feeds these to std arrays, Stats, ROIStat, the Attribute
plugin and HDF5. The PVs — stream health/counters, acquisition control and status, the continuous
monitor, and per-axis values — are the ordinary records; all of them, with meanings, are in
`PVReference.md`.

## Stream lifecycle and how correctness surfaces

Because loss is *detected, not repaired*, the interesting question is how that reaches EPICS:

- **A new stream ID** means the Brick restarted (new gather setup): the receiver adopts it once its
  `CONTEXT` passes the layout checks, decodes `DATA` only after the matching `CALIBRATION`, and
  **invalidates an armed acquisition**. Late packets from a retired stream are dropped and counted.
- **A STATUS fault** (gather changed/stopped, clock discontinuity, …) ends the stream's data and
  invalidates an armed acquisition; the fault reason shows in a PV.
- **Loss is counted several ways.** Sequence gaps (diagnostic — the per-stream sequence, over all
  packet types and send attempts, skipped a number; also duplicates/reorders) do **not** themselves
  reduce frame coverage. Row-number gaps split into the Brick's reported skips (the DATA flag + count,
  after an overrun) and a residual. A `ServoCount` that doesn't advance by `Gather.Period` is a
  latched **clock fault** (good prefix admitted first), not a third gap class. Per-frame **coverage**
  is observed ÷ expected samples over the frame's span, with a degraded alarm below a threshold.
- **Calibration generations**: each `DATA` batch carries the generation current when it was *packed*
  (the Brick polls `HomePos`/`PosSf` only ~1 s; `PosSf` is scale metadata, not an offset). How the IOC
  uses it differs by consumer: an **acquisition frame freezes the calibration at Acquire** and
  subtracts that frozen `HomePos` for the whole run, while the **monitor** applies each row block's own
  generation. A frame's `CalibrationChanged` attribute means a block's generation differed from the
  frozen one — not that a transition fell inside the frame — and the ~1 s polling means neither
  consumer can place the physical change precisely.
- **The final STATUS** is the end-of-stream checkpoint: the receiver can assert it got every row from
  its first admitted DATA to that checkpoint only if its cursor reached it with no gap or skip.

The two watchdogs — contact (any valid packet refreshes it) and data-stalled (admitted DATA) — are
the silence detection; they raise alarms without, by themselves, ending a run.

## Where to look next

- [`PVReference.md`](PVReference.md) — every PV, grouped, with values and meaning.
- `ADEncstreamApp/test/` — the host tests that exercise each layer (`make runtests`), and
  `vendor/encstream/` — the shared wire codec. The sender side is in the encstream repo
  (`docs/BRICK_ENCSTREAM_GUIDE.md`, `docs/WIRE_PROTOCOL_GUIDE.md`).
