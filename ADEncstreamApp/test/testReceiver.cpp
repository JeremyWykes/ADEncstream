/* testReceiver.cpp - the receiver core against the ADEncstream guide: the outcome contract (12.1),
 * encstream_rx's replay cases translated (12.2), the intended differences (12.3) and the further
 * cases of 12.4, including every review finding R1-R9. Packets come from the vendored encoder
 * (rxFixtures.h); testWire keeps the independent golden vectors. */
#include <string.h>

#include <epicsUnitTest.h>
#include <testMain.h>

#include "rxFixtures.h"

using namespace encstream;
using namespace fx;

namespace {

bool countersEqual(const Counters &a, const Counters &b) { return memcmp(&a, &b, sizeof a) == 0; }

/* ---- 12.2: encstream_rx replay cases, translated ------------------------------------------- */

void testCleanStream()
{
    Rig r; Brick b;
    r.start(b);
    for (unsigned k = 0; k < 5; ++k)
        testOk1(r.feed(b.data(2 * k)) == OUTCOME_DATA_ADMITTED);
    r.feed(b.status(0, 10, true, 10));
    StreamView v = r.rx.current();
    testOk(v.hasRows && v.firstRow == 0 && v.cursor == 10 && v.gapFree && v.hasFinal &&
           v.tailComplete && v.fault == FAULT_NONE, "clean stream: complete, gap-free, tail complete");
    testOk(r.rx.counters().rowsAdmitted == 10 && r.sink.gaps.empty(), "10 rows, no gap");
    testOk(r.sink.sequence() == "Started,Calibration,rows,rows,rows,rows,rows,Ended,TailComplete",
           "events and rows in order: %s", r.sink.sequence().c_str());
}

void testLateJoiner()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(500));
    r.feed(b.data(502));
    StreamView v = r.rx.current();
    testOk(v.firstRow == 500 && v.cursor == 504 && v.gapFree && r.sink.gaps.empty(),
           "late joiner: first row 500, no gap before it");
}

void testNoFinalAndTailLost()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    testOk(!r.rx.current().hasFinal, "no final STATUS: none in the facts");
    r.feed(b.status(0, 4, true, 4));          /* rows 2..3 never arrive */
    StreamView v = r.rx.current();
    testOk(v.hasFinal && v.cursor == 2 && !v.tailComplete && v.fault == FAULT_NONE,
           "tail lost: final next row 4, cursor 2, not complete, no fault");
}

void testBrickSkipAndTransportGap()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    r.feed(b.data(10, 2, 1, 5));              /* 8 missing, 5 of them skipped by the Brick */
    testOk(r.sink.gaps.size() == 1 && r.sink.gaps[0].fromRow == 2 && r.sink.gaps[0].toRow == 10 &&
           r.sink.gaps[0].skippedByBrick == 5 && r.sink.gaps[0].lostInTransport == 3,
           "gap [2,10): 5 skipped by the Brick, 3 lost");
    testOk(r.sink.gaps[0].fromTick == 1002 && r.sink.gaps[0].toTick == 1010, "gap ticks 1002..1010");
    testOk(r.rx.counters().rowsSkippedByBrick == 5 && r.rx.counters().rowsLost == 3 &&
           !r.rx.current().gapFree, "counted; no longer gap-free");
    testOk(r.sink.sequence() == "Started,Calibration,rows,gap,rows", "gap reported before its rows");
    r.feed(b.data(20, 2, 1, 50));             /* skipped count larger than the gap */
    testOk(r.sink.gaps[1].skippedByBrick == 8 && r.sink.gaps[1].lostInTransport == 0,
           "skipped capped at the gap size");
}

void testDataAfterFault()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    testOk1(r.feed(b.status(ES_FAULT_CONFIG_DIFFERS)) == OUTCOME_FAULTED);
    size_t rows = r.sink.rowBlocks.size();
    testOk(r.feed(b.data(2)) == OUTCOME_DATA_DROPPED && r.rx.counters().dataAfterFault == 1 &&
           r.sink.rowBlocks.size() == rows, "DATA after a fault: dropped, no rows");
    testOk(r.sink.events.back().type == EVENT_STREAM_FAULTED &&
           r.sink.events.back().fault == FAULT_BRICK_STATUS && r.sink.events.back().brickFault == 1,
           "fault event carries the Brick's code");
}

void testFaultThenRestart()
{
    Rig r; Brick a(0x11111111u), c(0x22222222u);
    r.start(a);
    r.feed(a.data(0));
    r.feed(a.status(ES_FAULT_CLOCK_DISCONTINUITY));
    r.sink.clear();
    r.feed(c.context());
    testOk(r.rx.counters().faults == 1 && r.rx.counters().streamChanges == 1,
           "fault and restart both counted");
    testOk(r.sink.sequence() == "Replaced,Started" && r.sink.events[0].oldStream == 0x11111111u &&
           r.sink.events[0].stream == 0x22222222u, "Replaced (old, new) before Started");
    StreamView v = r.rx.current();
    testOk(v.stream == 0x22222222u && v.fault == FAULT_NONE && !v.ready,
           "the new stream starts with fresh facts");
}

void testChangedContext()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    b.motors.push_back(2);                    /* a different layout, same stream ID */
    testOk1(r.feed(b.context()) == OUTCOME_FAULTED);
    b.motors.pop_back();
    testOk(r.rx.current().fault == FAULT_CONTEXT_CHANGED && r.rx.counters().contextChanged == 1,
           "CONTEXT changed: fault");
    testOk(r.rx.current().layout.nAxes == 1, "the changed CONTEXT is never adopted");
    testOk1(r.feed(b.data(2)) == OUTCOME_DATA_DROPPED);
    Rig r2; Brick b2;
    r2.start(b2);
    testOk(r2.feed(b2.context()) == OUTCOME_ACCEPTED && r2.rx.current().fault == FAULT_NONE,
           "an identical CONTEXT repeat is fine");
}

void testCalibrationMotors()
{
    {
        Rig r; Brick b;
        r.feed(b.context());
        std::vector<uint8_t> wrong(1, 2);
        testOk(r.feed(b.calibration(1, 0, &wrong)) == OUTCOME_FAULTED &&
               r.rx.current().fault == FAULT_CALIBRATION_MOTORS, "calibration for another motor");
    }
    {
        Rig r; Brick b;
        b.motors.push_back(2);
        r.feed(b.context());
        std::vector<uint8_t> extra; extra.push_back(1); extra.push_back(2); extra.push_back(3);
        std::vector<uint8_t> missing(1, 1);
        std::vector<uint8_t> reversed; reversed.push_back(2); reversed.push_back(1);
        testOk1(r.feed(b.calibration(1, 0, &extra)) == OUTCOME_FAULTED);
        Rig r2; Brick b2; b2.motors.push_back(2);
        r2.feed(b2.context());
        testOk1(r2.feed(b2.calibration(1, 0, &missing)) == OUTCOME_FAULTED);
        Rig r3; Brick b3; b3.motors.push_back(2);
        r3.feed(b3.context());
        testOk(r3.feed(b3.calibration(1, 0, &reversed)) == OUTCOME_FAULTED,
               "calibration motors out of CONTEXT order");
    }
}

void testUnknownGenerationRecovers()
{
    Rig r; Brick b;
    r.takeSends();
    r.start(b);
    r.feed(b.data(0));
    Counters before = r.rx.counters();
    testOk1(r.feed(b.data(2, 2, 2)) == OUTCOME_DATA_DROPPED);
    testOk(r.rx.counters().unknownGeneration == 1 && r.rx.current().cursor == 2 &&
           r.rx.counters().rowsAdmitted == before.rowsAdmitted, "unknown generation: dropped, nothing else");
    r.rx.onTimer(r.t + S);                    /* the next heartbeat */
    r.takeSends();
    testOk(r.subscribes.back() & ES_SUB_SEND_META, "the next SUBSCRIBE asks for metadata");
    r.rx.onTimer(r.t + 2 * S);
    r.takeSends();
    testOk(!(r.subscribes.back() & ES_SUB_SEND_META), "and only that one");
    r.t += 2 * S;
    r.feed(b.calibration(2, 5.0));
    testOk(r.feed(b.data(2, 2, 2)) == OUTCOME_DATA_ADMITTED && r.rx.current().cursor == 4,
           "recovers once the generation arrives");
}

void testDelayedOldGeneration()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.calibration(2));
    testOk(r.feed(b.data(0, 2, 1)) == OUTCOME_DATA_ADMITTED && r.sink.rowBlocks.back().gen == 1,
           "DATA of a kept older generation is admitted with it");
    testOk(r.rx.current().latestGeneration == 2, "latest stays 2");
}

void testGenerationRewritten()
{
    Rig r; Brick b;
    r.start(b);
    testOk1(r.feed(b.calibration(1, 0.0)) == OUTCOME_ACCEPTED);       /* identical repeat */
    testOk(r.feed(b.calibration(1, 9.0)) == OUTCOME_FAULTED &&
           r.rx.current().fault == FAULT_CALIBRATION_REWRITTEN, "generation 1 with other values");
}

void testWrongRowSize()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    Counters before = r.rx.counters();
    size_t calls = r.sink.calls.size();
    Bytes bad = b.data(2, 1);                 /* 12-byte layout; declare 16-byte rows and pad */
    bad[26] = 16;                             /* DATA header: row size (u16 LE at 26) */
    bad.insert(bad.end(), 4, 0);
    es_packet chk;
    testOk(es_decode(&bad[0], bad.size(), &chk) == ES_OK && chk.u.data.row_bytes == 16,
           "a structurally valid DATA with 16-byte rows");
    Outcome o = r.feed(bad);
    Counters after = r.rx.counters();
    testOk(o == OUTCOME_DATA_DROPPED && after.rejectedRowSize == 1, "wrong row size: DATA_DROPPED");
    after.rejectedRowSize = 0; after.datagrams--; after.decoded[ES_DATA]--;
    testOk(countersEqual(after, before) && r.sink.calls.size() == calls &&
           r.rx.current().cursor == 2, "nothing else changed");
}

void testLateDuplicateAndRetired()
{
    Rig r; Brick a(0x11111111u), c(0x22222222u);
    r.start(a);
    r.feed(a.data(0));
    Bytes dup = a.data(0);
    testOk(r.feed(dup) == OUTCOME_DATA_DROPPED && r.rx.counters().lateOrDuplicate == 1, "duplicate");
    r.start(c);
    testOk(r.feed(a.data(2)) == OUTCOME_IGNORED && r.rx.counters().retiredStream == 1,
           "a late packet of the retired stream is ignored");
    testOk(r.feed(a.context()) == OUTCOME_IGNORED && r.rx.current().stream == 0x22222222u,
           "even its CONTEXT cannot take over again");
    testOk(r.feed(a.status(0, 2, true)) == OUTCOME_IGNORED && !r.rx.current().hasFinal,
           "nor its final STATUS");
}

/* ---- watchdogs: encstream_rx's silence cases, now the DATA watchdog (10.3) ------------------ */

void testDataStall()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    r.rx.onTimer(r.t + 2 * S);
    testOk(r.sink.count("DataStalled") == 0, "2 s: no stall yet");
    r.rx.onTimer(r.t + 3 * S + 1);
    testOk(r.sink.count("DataStalled") == 1 && r.rx.counters().dataStalls == 1, "past 3 s: stalled");
    r.rx.onTimer(r.t + 9 * S);
    testOk(r.sink.count("DataStalled") == 1, "counted once per stall");
    r.t += 9 * S;
    r.feed(b.data(2));
    testOk(r.sink.events.back().type == EVENT_DATA_STALL_ENDED &&
           r.sink.events.back().stallEnd == STALL_END_DATA, "ended by DATA");
}

void testStallDetectedBeforeEndingPacket()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    r.feed(b.data(2), 5 * S);                 /* the next DATA 5 s later */
    testOk(r.sink.sequence() == "Started,Calibration,rows,DataStalled,DataStallEnded,rows",
           "stall seen before the packet that ends it: %s", r.sink.sequence().c_str());
    Rig r2; Brick b2;
    r2.start(b2);
    r2.feed(b2.data(0));
    r2.feed(b2.status(0, 2, true), 5 * S);
    testOk(r2.sink.sequence() == "Started,Calibration,rows,DataStalled,Ended,DataStallEnded,TailComplete",
           "ended by the final: %s", r2.sink.sequence().c_str());
    testOk(r2.sink.events[4].stallEnd == STALL_END_FINAL, "reason final");
}

void testNoStallAfterFinalOrFault()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    r.feed(b.status(0, 2, true));
    r.rx.onTimer(r.t + 10 * S);
    testOk(r.sink.count("DataStalled") == 0, "no stall after the final");
    Rig r2; Brick b2;
    r2.start(b2);
    r2.feed(b2.data(0));
    r2.feed(b2.status(ES_FAULT_NOT_ENABLED));
    r2.rx.onTimer(r2.t + 10 * S);
    testOk(r2.sink.count("DataStalled") == 0, "no stall after a fault");
}

void testOtherTrafficDoesNotHideStall()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    Bytes junk(20, 0x55);
    for (unsigned i = 0; i < 40; ++i)
        r.feed(junk, 100 * MS);               /* malformed traffic for 4 s */
    for (unsigned i = 0; i < 40; ++i)
        r.rx.onDatagram(&b.data(2)[0], b.data(2).size(), false, endpoint(10, 0, 0, 9, 5199),
                        r.t += 100 * MS);     /* foreign DATA for 4 s */
    testOk(r.sink.count("DataStalled") == 1 && r.sink.count("DataStallEnded") == 0,
           "malformed and foreign traffic neither ends nor hides a stall");
    for (unsigned i = 0; i < 40; ++i)
        r.feed(b.report(), 100 * MS);         /* valid non-DATA packets */
    testOk(r.sink.count("DataStallEnded") == 0, "SENDER_REPORTs do not end it either");
}

/* ---- final STATUS -------------------------------------------------------------------- */

void testFinalRules()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    r.feed(b.status(0, 4, true, 4));
    testOk1(r.feed(b.status(0, 4, true, 4)) == OUTCOME_ACCEPTED);
    testOk(r.sink.count("Ended") == 1 && r.rx.current().fault == FAULT_NONE,
           "identical final repeat: harmless, no second event");
    testOk(r.feed(b.data(2)) == OUTCOME_DATA_ADMITTED && r.rx.current().tailComplete &&
           r.sink.sequence().find("Ended,rows,TailComplete") != std::string::npos,
           "straggler below the final admitted after Ended, then TailComplete");
    testOk(r.feed(b.status(0, 6, true, 6)) == OUTCOME_FAULTED &&
           r.rx.current().fault == FAULT_FINAL_REWRITTEN && r.rx.current().final.next_row == 4,
           "a changed final: fault; the first stays the end point");

    Rig r2; Brick b2;
    r2.start(b2);
    r2.feed(b2.data(0));
    r2.feed(b2.status(0, 3, true));
    testOk(r2.feed(b2.data(2)) == OUTCOME_FAULTED && r2.rx.current().fault == FAULT_DATA_BEYOND_FINAL,
           "DATA partly beyond the final: fault");

    Rig r3; Brick b3;
    r3.start(b3);
    r3.feed(b3.data(0, 4));
    testOk(r3.feed(b3.status(0, 2, true)) == OUTCOME_FAULTED &&
           r3.rx.current().fault == FAULT_FINAL_BEHIND_CURSOR && r3.rx.current().hasFinal,
           "final behind the cursor: fault, still recorded");

    Rig r4; Brick b4;
    r4.feed(b4.context());
    r4.feed(b4.status(0, 0, true));
    r4.feed(b4.calibration());
    r4.rx.onTimer(r4.t + 10 * S);
    testOk(r4.rx.current().hasFinal && r4.rx.current().ready && r4.sink.count("DataStalled") == 0,
           "final before calibration: calibration does not arm the DATA watchdog");
}

void testFaultAndFinalTogether()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    r.sink.clear();
    testOk1(r.feed(b.status(ES_FAULT_CLOCK_DISCONTINUITY, 2, true)) == OUTCOME_FAULTED);
    testOk(r.sink.sequence() == "Faulted,Ended,TailComplete", "Faulted before Ended: %s",
           r.sink.sequence().c_str());
    testOk(r.feed(b.data(2)) == OUTCOME_DATA_DROPPED, "a final never revives admission after a fault");

    Rig r2; Brick b2;
    r2.start(b2);
    r2.feed(b2.data(0));
    r2.feed(b2.data(2, 2, 1, 0, 1));         /* receiver-detected fault (row 1 breaks) */
    r2.feed(b2.status(0, 10, true));          /* then a healthy final */
    testOk(r2.rx.current().fault == FAULT_CLOCK_DISCONTINUITY && r2.rx.current().hasFinal &&
           r2.feed(b2.data(4)) == OUTCOME_DATA_DROPPED, "a healthy final does not clear a fault");
    r2.feed(b2.status(0, 12, true));
    testOk(r2.rx.counters().finalRewritten == 1 && r2.rx.current().fault == FAULT_CLOCK_DISCONTINUITY,
           "conflicting final while faulted: counted, first fault kept");
    testOk1(r2.feed(b2.calibration(2)) == OUTCOME_ACCEPTED);
    Calibration cal;
    testOk(r2.rx.calibration(2, &cal) && cal.generation == 2,
           "CALIBRATION after a fault is recorded for diagnosis");
    testOk(r2.feed(b2.data(4, 2, 2)) == OUTCOME_DATA_DROPPED, "but no DATA is admitted");
}

/* ---- R2: the good prefix ---------------------------------------------------------------------- */

void testPrefix()
{
    for (int at = 0; at < 4; ++at) {
        Rig r; Brick b;
        r.start(b);
        r.feed(b.data(0));
        Counters before = r.rx.counters();
        r.sink.clear();
        Outcome o = r.feed(b.data(2, 4, 1, 0, at));
        StreamView v = r.rx.current();
        const Counters &c = r.rx.counters();
        if (at == 0) {
            testOk(o == OUTCOME_FAULTED && v.cursor == 2 && c.rowsAdmitted == before.rowsAdmitted &&
                   c.dataPackets == before.dataPackets && r.sink.sequence() == "Faulted" &&
                   c.partialPackets == 0, "break at row 0: fault only, nothing committed");
        } else {
            testOk(o == OUTCOME_DATA_PARTIAL && v.cursor == 2u + at &&
                   c.rowsAdmitted == before.rowsAdmitted + at && c.partialPackets == 1 &&
                   r.sink.sequence() == "rows,Faulted" && r.sink.rowBlocks[0].n == (uint32_t)at,
                   "break at row %d: %d rows, then the fault", at, at);
        }
        testOk(c.clockDiscontinuities == 1 && v.fault == FAULT_CLOCK_DISCONTINUITY, "counted as a discontinuity");
    }
    Rig r; Brick b;                           /* after a gap */
    r.start(b);
    r.feed(b.data(0));
    r.sink.clear();
    r.feed(b.data(10, 4, 1, 0, 2));
    testOk(r.sink.sequence() == "gap,rows,Faulted" && r.rx.current().cursor == 12,
           "after a gap: gap, prefix, fault");
    Rig r2; Brick b2;                         /* break inside the stream's first DATA */
    r2.start(b2);
    r2.sink.clear();
    r2.feed(b2.data(0, 4, 1, 0, 3));
    testOk(r2.sink.sequence() == "rows,Faulted" && r2.rx.current().firstRow == 0 &&
           r2.rx.current().cursor == 3, "first packet: 3 rows, anchor set, then the fault");
    Rig r3; Brick b3;                         /* break at row 0 after a gap: no gap report */
    r3.start(b3);
    r3.feed(b3.data(0));
    r3.sink.clear();
    r3.feed(b3.data(10, 4, 1, 0, 0));
    testOk(r3.sink.sequence() == "Faulted" && r3.rx.counters().rowsLost == 0,
           "k = 0 after a gap: no gap reported, no loss counted");
}

/* ---- clock arithmetic ------------------------------------------------------------------------- */

void testClockWrapAndRange()
{
    Rig r; Brick b;
    b.clock0 = 0xFFFFFFFEu;                   /* ServoCount wraps inside the first packet */
    r.start(b);
    testOk1(r.feed(b.data(0, 4)) == OUTCOME_DATA_ADMITTED);
    testOk(r.sink.rowBlocks[0].tick[3] == 0xFFFFFFFEull + 3 && r.sink.rowBlocks[0].servo[3] == 1,
           "tick keeps counting past the 32-bit wrap");
    testOk1(r.feed(b.data(100)) == OUTCOME_DATA_ADMITTED);
    testOk(r.sink.gaps[0].fromTick == 0xFFFFFFFEull + 4 && r.sink.gaps[0].toTick == 0xFFFFFFFEull + 100,
           "and across a gap");

    Rig r2; Brick b2;
    b2.period = 2;
    r2.start(b2);
    r2.feed(b2.data(0));
    Counters before = r2.rx.counters();
    Bytes far = b2.data(0x8000000000000000ull, 1);   /* row 2^63 x period 2: no 64-bit tick */
    testOk(r2.feed(far) == OUTCOME_FAULTED && r2.rx.current().fault == FAULT_CLOCK_OUT_OF_RANGE &&
           r2.rx.counters().clockOutOfRange == 1 && r2.rx.counters().rowsAdmitted == before.rowsAdmitted &&
           r2.rx.counters().rowsLost == 0, "sample clock out of range: fault, nothing committed");
}

void testSequenceGaps()
{
    Rig r; Brick b;
    b.seq = 0xFFFFFFFEu;                      /* wraps */
    r.start(b);
    r.feed(b.data(0));
    r.feed(b.data(2));
    testOk(r.rx.counters().sequenceGaps == 0, "sequence across 2^32: no gap");
    b.seq += 3;
    r.feed(b.data(4));
    testOk(r.rx.counters().sequenceGaps == 1 && r.rx.counters().rowsLost == 0,
           "a sequence gap is diagnostic only");
}

/* ---- R7: layout in the RowBlock --------------------------------------------------------------- */

void testDecoding()
{
    for (int big = 0; big < 2; ++big) {
        Rig r; Brick b;
        b.big = big != 0;
        b.gate = true;
        b.motors.clear(); b.motors.push_back(3); b.motors.push_back(1);
        b.statusMotors.push_back(1); b.statusMotors.push_back(3); b.statusMotors.push_back(7);
        b.otherTypes.push_back(5); b.otherTypes.push_back(3);
        r.start(b);
        testOk1(r.feed(b.data(4, 3)) == OUTCOME_DATA_ADMITTED);
        const RowsCopy &x = r.sink.rowBlocks.back();
        testOk(x.hasGate && x.gate[0] == 0 && x.gate[1] == 1 && x.gateWord[1] == (0xA5000000u | 0x800u),
               "%s: gate bit and word", big ? "big-endian" : "little-endian");
        testOk(x.axisMotor.size() == 2 && x.axisMotor[0] == 3 && x.axisMotor[1] == 1 &&
               x.pos[0][0] == 4.3 && x.pos[1][2] == 6.1, "positions in CONTEXT order");
        testOk(x.statusMotor.size() == 3 && x.statusMotor[1] == 3 && x.statusMotor[2] == 7 &&
               x.status[2][1] == 0xA0000000u + 7 * 256 + 5, "status words with their own motor order");
        testOk(x.otherItem.size() == 2 && x.other[0][2] == -3.0 && x.other[1][0] == -1.0,
               "role-5 items decoded: double -3.0, signed 24-bit -1 (got %g, %g)", x.other[0][2],
               x.other[1][0]);
    }
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    testOk(!r.sink.rowBlocks[0].hasGate && r.sink.rowBlocks[0].status.empty(), "no gate, no status");

    Rig r2; Brick b2;                         /* 16 axes, the most rows a datagram holds */
    b2.motors.clear();
    for (uint8_t m = 1; m <= 16; ++m)
        b2.motors.push_back(m);
    r2.start(b2);
    unsigned n = (MAX_DATAGRAM - ES_DATA_HDR_LEN) / b2.rowBytes();
    testOk(r2.feed(b2.data(0, n)) == OUTCOME_DATA_ADMITTED && r2.sink.rowBlocks[0].n == n &&
           r2.sink.rowBlocks[0].pos[15][n - 1] == (n - 1) + 1.6, "16 axes, %u rows", n);
    Rig r3; Brick b3;
    b3.motors.clear();
    r3.start(b3);
    testOk(r3.feed(b3.data(0, MAX_ROWS)) == OUTCOME_DATA_ADMITTED && r3.sink.rowBlocks[0].n == MAX_ROWS,
           "clock-only layout: %d rows", (int)MAX_ROWS);
}

/* ---- R8: calibration retention ---------------------------------------------------------------- */

void testCalibrationRetention()
{
    Rig r; Brick b;
    r.feed(b.context());
    r.feed(b.calibration(100));
    for (uint32_t g = 1; g <= 8; ++g)
        r.feed(b.calibration(g, (double)g));
    Calibration c;
    testOk(r.rx.current().latestGeneration == 100 && r.rx.calibration(100, &c) && c.generation == 100,
           "100 then delayed 1..8: 100 stays latest and kept");
    testOk(!r.rx.calibration(1, &c) && r.rx.calibration(8, &c) && r.rx.calibration(2, &c),
           "the lowest older one was evicted");
    testOk(r.feed(b.data(0, 2, 100)) == OUTCOME_DATA_ADMITTED, "DATA under 100 still admitted");
    testOk(r.feed(b.calibration(1, 77.0)) == OUTCOME_ACCEPTED && !r.rx.calibration(1, &c),
           "an evicted generation older than every kept one is not stored again");
    r.feed(b.calibration(101));
    testOk(r.rx.current().latestGeneration == 101 && r.rx.calibration(100, &c) && !r.rx.calibration(2, &c),
           "a newer generation evicts the lowest");
    Calibration copy;
    r.rx.calibration(101, &copy);
    for (uint32_t g = 102; g < 120; ++g)
        r.feed(b.calibration(g));
    testOk(copy.generation == 101 && copy.axes[0].home_pos == 1.0, "a copy taken at Acquire stays valid");
    testOk(r.rx.current().latestGeneration == 119, "latest follows");
}

/* ---- 12.1 outcome contract and stage 1-2 rejections ------------------------------------------- */

void testRejectionsChangeNothing()
{
    Rig r; Brick b;
    r.start(b);
    r.feed(b.data(0));
    std::vector<Bytes> bad;
    Bytes d = b.data(2);
    bad.push_back(Bytes(d.begin(), d.end() - 1));
    Bytes longer = d; longer.push_back(0); bad.push_back(longer);
    Bytes magic = d; magic[0] ^= 0xFF; bad.push_back(magic);
    Bytes swapped = d; std::swap(swapped[0], swapped[3]); std::swap(swapped[1], swapped[2]); bad.push_back(swapped);
    Bytes version = d; version[4] = 2; bad.push_back(version);
    Bytes ctx = b.context(); ctx.pop_back(); bad.push_back(ctx);
    for (size_t i = 0; i < bad.size(); ++i) {
        Counters before = r.rx.counters();
        StreamView v0 = r.rx.current();
        size_t calls = r.sink.calls.size();
        Outcome o = r.feed(bad[i], 1);
        Counters after = r.rx.counters();
        StreamView v1 = r.rx.current();
        bool onlyReject = after.datagrams == before.datagrams + 1;
        uint64_t rej = 0, rej0 = 0;
        for (unsigned k = 0; k < 12; ++k) { rej += after.rejected[k]; rej0 += before.rejected[k]; }
        onlyReject = onlyReject && rej == rej0 + 1;
        Counters a2 = after, b2 = before;
        memset(a2.rejected, 0, sizeof a2.rejected); memset(b2.rejected, 0, sizeof b2.rejected);
        a2.datagrams = b2.datagrams = 0;
        testOk(o == OUTCOME_REJECTED && onlyReject && countersEqual(a2, b2) &&
               r.sink.calls.size() == calls && v1.cursor == v0.cursor,
               "malformed datagram %u: REJECTED, only datagrams and one reject counter change", (unsigned)i);
    }
    Bytes sub(ES_SUBSCRIBE_HDR_LEN + 4);
    es_common h; h.type = ES_SUBSCRIBE; h.stream_id = 0; h.seq = 1;
    es_subscribe s; memset(&s, 0, sizeof s); s.id_len = 4; memcpy(s.id, "evil", 4);
    size_t w = 0;
    es_encode_subscribe(&sub[0], sub.size(), &h, &s, &w);
    Counters before = r.rx.counters();
    testOk(r.feed(sub, 1) == OUTCOME_REJECTED && r.rx.counters().unexpectedType == 1 &&
           r.rx.counters().decoded[ES_SUBSCRIBE] == 0 && r.rx.counters().decoded[ES_DATA] == before.decoded[ES_DATA],
           "a SUBSCRIBE from the Brick: rejected, not counted as decoded");
    Bytes unknown = b.report();
    unknown[5] = 9;                           /* type 9: valid header, unknown type */
    testOk(r.feed(unknown, 1) == OUTCOME_REJECTED && r.rx.counters().unknownType == 1, "unknown type skipped");
}

void testDatagramChecks()
{
    Rig r; Brick b;
    Bytes c = b.context();
    testOk(r.rx.onDatagram(&c[0], c.size(), false, endpoint(192, 0, 2, 2, 5199), 1) == OUTCOME_DROPPED &&
           r.rx.counters().foreignSource == 1, "foreign address dropped");
    testOk(r.rx.onDatagram(&c[0], c.size(), false, endpoint(192, 0, 2, 1, 5200), 2) == OUTCOME_DROPPED &&
           r.rx.counters().foreignSource == 2, "foreign port dropped");
    testOk(r.rx.onDatagram(&c[0], c.size(), true, brickEndpoint(), 3) == OUTCOME_DROPPED &&
           r.rx.counters().truncated == 1, "truncated dropped");
    Bytes big(MAX_DATAGRAM + 1, 0);
    testOk(r.rx.onDatagram(&big[0], big.size(), false, brickEndpoint(), 4) == OUTCOME_DROPPED &&
           r.rx.counters().oversize == 1, "1473 bytes dropped as oversize");
    testOk(!r.rx.current().hasStream && r.rx.counters().datagrams == 4, "nothing adopted");
    r.rx.onTimer(2);
    testOk(r.rx.counters().timeRegressions == 1, "time going backwards counted");
}

void testMetadataLifecycle()
{
    Rig r; Brick a(0x11111111u), c(0x22222222u);
    testOk(r.feed(a.data(0)) == OUTCOME_IGNORED && r.rx.counters().beforeContext == 1, "DATA before CONTEXT");
    r.feed(a.context());
    testOk(r.feed(a.data(0)) == OUTCOME_DATA_DROPPED && r.rx.counters().dataBeforeCalibration == 1,
           "DATA before CALIBRATION");
    r.feed(a.calibration());
    r.feed(a.data(0));
    r.start(c);
    unsigned ignoredBefore = (unsigned)r.rx.counters().retiredStream;
    r.feed(a.calibration(5));
    r.feed(a.status(0, 2, true));
    r.feed(a.report());
    r.feed(a.data(2));
    Calibration tmp;
    testOk(r.rx.counters().retiredStream == ignoredBefore + 4 && r.rx.current().stream == 0x22222222u &&
           !r.rx.current().hasFinal && !r.rx.calibration(5, &tmp),
           "A -> B -> late A: every packet type of A ignored");
    r.feed(c.calibration(3));
    r.feed(c.calibration(2));
    testOk(r.rx.current().latestGeneration == 3, "an old CALIBRATION after a newer one never becomes latest");
}

void testTooManyStreams()
{
    Rig r;
    std::vector<Brick> bricks;
    for (uint32_t i = 0; i < 10; ++i)
        bricks.push_back(Brick(0x1000u + i));
    for (size_t i = 0; i < bricks.size(); ++i)
        r.feed(bricks[i].context());
    testOk(r.feed(bricks[1].data(0)) == OUTCOME_IGNORED && r.rx.counters().retiredStream == 1,
           "the 8 most recent retired streams are remembered");
    testOk(r.feed(bricks[0].context()) == OUTCOME_ACCEPTED && r.rx.current().stream == 0x1000u,
           "the 9th oldest is forgotten: its CONTEXT becomes current again (accepted, 6.1)");
}

void testEquality()
{
    Rig r; Brick b;
    r.feed(b.context());
    r.feed(b.calibration(1, 0.0));
    testOk1(r.feed(b.calibration(1, 0.0)) == OUTCOME_ACCEPTED);   /* bit-identical repeat */
    /* generation 2 with HomePos +0.0, then the same generation with -0.0 (motor 1: home - 1) */
    r.feed(b.calibration(2, -1.0));
    Bytes neg = b.calibration(2, -1.0);
    /* the encoded HomePos is (-1.0 + 1) = +0.0; flip its sign bit to make -0.0 */
    size_t off = ES_CALIBRATION_HDR_LEN + 4 + 7;   /* record: u32 motor, f64 home (LE): byte 7 */
    testOk1(neg[off] == 0x00);
    neg[off] = 0x80;
    testOk(r.feed(neg) == OUTCOME_FAULTED && r.rx.current().fault == FAULT_CALIBRATION_REWRITTEN,
           "+0.0 then -0.0 in one generation: rewritten (bitwise, 6.6)");
}

/* ---- R3, R4: heartbeat and watchdogs ---------------------------------------------------------- */

void testHeartbeat()
{
    Rig r; Brick b;
    r.rx.onTimer(0);
    r.takeSends();
    testOk(r.subscribes.size() == 1 && (r.subscribes[0] & ES_SUB_SEND_META), "first SUBSCRIBE at once, asking for metadata");
    r.start(b);
    for (unsigned k = 0; k < 12000; ++k) {    /* a datagram every ms for 12 s, no onTimer */
        r.feed(b.data(2 * k));
        r.takeSends();
    }
    testOk(r.subscribes.size() == 13 && r.rx.counters().subscribesSent == 13,
           "heartbeat every second under continuous traffic: %u", (unsigned)r.subscribes.size());
    testOk(!(r.subscribes.back() & ES_SUB_SEND_META), "no metadata request once ready");
    r.rx.onTimer(r.t + 7 * S);                /* a 7 s pause */
    r.takeSends();
    testOk(r.subscribes.size() == 14 && r.rx.counters().subscribesCoalesced == 6,
           "after a 7 s pause: one SUBSCRIBE, 6 coalesced");
    r.rx.onTimer(r.t + 8 * S);
    const uint8_t *buf; size_t n;
    testOk1(r.rx.pendingSend(&buf, &n));
    r.rx.sent(false, r.t + 8 * S);
    testOk(!r.rx.pendingSend(&buf, &n) && r.rx.counters().subscribeSendFailures == 1,
           "a failed send is counted, not retried at once");
    es_packet p;
    r.rx.onTimer(r.t + 9 * S);
    testOk(r.rx.pendingSend(&buf, &n) && es_decode(buf, n, &p) == ES_OK && p.u.subscribe.id_len == 8 &&
           memcmp(p.u.subscribe.id, "ioc=test", 8) == 0, "the next heartbeat carries the identification");
    testOk(r.rx.nextDeadline() <= r.t + 10 * S, "next deadline no later than the next heartbeat");
}

void testContactWatchdog()
{
    Rig r; Brick b;
    r.rx.onTimer(0);
    r.takeSends();
    r.rx.onTimer(3 * S + 1);
    r.takeSends();
    testOk(r.sink.count("ContactLost") == 1 && r.rx.counters().contactLosses == 1,
           "no reply at all: contact lost (a second IOC that the Brick ignores)");
    r.t = 3 * S + 1;
    Bytes junk(30, 0x11);
    r.feed(junk);
    testOk(r.sink.count("ContactRestored") == 0, "malformed traffic does not restore contact");
    r.feed(b.status());
    testOk(r.sink.count("ContactRestored") == 1, "any valid packet does, even before CONTEXT");
    r.feed(b.context());
    r.feed(b.calibration());
    r.rx.onTimer(r.t + 3 * S + 1);
    testOk(r.sink.count("DataStalled") == 1 && r.sink.count("ContactLost") == 2,
           "metadata but no DATA: stalled; and contact lost again");
    Rig r2; Brick b2;
    r2.rx.onTimer(0);
    r2.takeSends();
    r2.start(b2);
    for (unsigned i = 0; i < 50; ++i)
        r2.feed(b2.status(), 100 * MS);       /* STATUS only for 5 s */
    testOk(r2.sink.count("ContactLost") == 0 && r2.sink.count("DataStalled") == 1,
           "STATUS-only traffic: contact kept, DATA stalled");
}

void testStallEndReasons()
{
    Rig r; Brick a(0x11111111u), c(0x22222222u);
    r.start(a);
    r.rx.onTimer(r.t + 4 * S);
    r.t += 4 * S;
    r.feed(c.context());
    testOk(r.sink.count("DataStallEnded") == 1 && r.sink.events[r.sink.events.size() - 3].stallEnd == STALL_END_REPLACED,
           "replacement ends a stall: %s", r.sink.sequence().c_str());
    Rig r2; Brick b2;
    r2.start(b2);
    r2.rx.onTimer(r2.t + 4 * S);
    r2.t += 4 * S;
    r2.feed(b2.status(ES_FAULT_NOT_ENABLED));
    testOk(r2.sink.events.back().type == EVENT_DATA_STALL_ENDED && r2.sink.events.back().stallEnd == STALL_END_FAULT,
           "a fault ends a stall");
}

/* ---- event log (9.2) -------------------------------------------------------------------------- */

void testEventLog()
{
    ReceiverConfig cfg = config();
    cfg.eventLogCapacity = 3;
    Rig r(cfg);
    for (uint32_t i = 0; i < 5; ++i) {
        Brick b(0x100u + i);
        r.feed(b.context());
    }
    testOk(r.rx.counters().eventsLogged == 9 && r.rx.counters().eventsLost == 6 && r.sink.events.size() == 9,
           "log overflow counted; every event still delivered to the sink");
    StreamEvent ev[5];
    size_t n = r.rx.recentEvents(ev, 5);
    testOk(n == 3 && ev[2].type == EVENT_STREAM_STARTED && ev[2].stream == 0x104u &&
           ev[1].type == EVENT_STREAM_REPLACED, "the most recent three, oldest first");
}

/* ---- configuration --------------------------------------------------------------------------- */

void testConfig()
{
    ReceiverConfig c = config();
    testOk1(checkConfig(c).empty());
    c.subscribePeriodNs = 2 * S + 1;
    testOk(!checkConfig(c).empty(), "heartbeat over 2 s refused");
    c = config(); c.identification = std::string(65, 'x');
    testOk(!checkConfig(c).empty(), "identification over 64 bytes refused");
    c = config(); c.identification = "a\tb";
    testOk(!checkConfig(c).empty(), "non-printable identification refused");
    c = config(); c.brick.port = 0;
    testOk(!checkConfig(c).empty(), "port 0 refused");
}

}

MAIN(testReceiver)
{
    testPlan(0);
    testCleanStream();
    testLateJoiner();
    testNoFinalAndTailLost();
    testBrickSkipAndTransportGap();
    testDataAfterFault();
    testFaultThenRestart();
    testChangedContext();
    testCalibrationMotors();
    testUnknownGenerationRecovers();
    testDelayedOldGeneration();
    testGenerationRewritten();
    testWrongRowSize();
    testLateDuplicateAndRetired();
    testDataStall();
    testStallDetectedBeforeEndingPacket();
    testNoStallAfterFinalOrFault();
    testOtherTrafficDoesNotHideStall();
    testFinalRules();
    testFaultAndFinalTogether();
    testPrefix();
    testClockWrapAndRange();
    testSequenceGaps();
    testDecoding();
    testCalibrationRetention();
    testRejectionsChangeNothing();
    testDatagramChecks();
    testMetadataLifecycle();
    testTooManyStreams();
    testEquality();
    testHeartbeat();
    testContactWatchdog();
    testStallEndReasons();
    testEventLog();
    testConfig();
    return testDone();
}
