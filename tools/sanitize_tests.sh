#!/bin/sh
# sanitize_tests.sh - build every test program of ADEncstreamApp/test with AddressSanitizer,
# UndefinedBehaviorSanitizer and float-cast-overflow, and run it (the conversions the
# ordinary build cannot check). For Linux development hosts with gcc or clang.
#     tools/sanitize_tests.sh            uses $EPICS_BASE (default ~/epics/epics-base)
# Exit 0 if every program passes with no sanitizer report.
set -eu
HERE=$(cd "$(dirname "$0")/.." && pwd)
BASE=${EPICS_BASE:-$HOME/epics/epics-base}
ADCORE=${ADCORE:-$HOME/epics/modules/areaDetector/ADCore}
ASYN=${ASYN:-$HOME/epics/modules/asyn}
BUSY=${BUSY:-$HOME/epics/modules/busy}
ARCH=${EPICS_HOST_ARCH:-linux-x86_64}
CC=${CC:-gcc}
CXX=${CXX:-g++}
SAN="-fsanitize=address,undefined,float-cast-overflow -fno-sanitize-recover=all -fno-omit-frame-pointer -g -O1"
INC="-I$HERE/ADEncstreamApp/src -I$HERE/ADEncstreamApp/test -I$HERE/vendor/encstream -I$BASE/include -I$BASE/include/os/Linux -I$BASE/include/compiler/gcc"
DEFS="-DES_GOLDEN_DIR=\"$HERE/vendor/encstream/golden\" -DES_TRACE_DIR=\"$HERE/ADEncstreamApp/test/traces\""
LIBS="-L$BASE/lib/$ARCH -lCom -Wl,-rpath,$BASE/lib/$ARCH"
W=$(mktemp -d)
trap 'rm -rf "$W"' EXIT
S=$HERE/ADEncstreamApp/src
T=$HERE/ADEncstreamApp/test
$CC $SAN $INC -c "$HERE/vendor/encstream/wire.c" -o "$W/wire.o"
CORE="$S/esReceiver.cpp $S/esTrace.cpp $S/esMonitor.cpp $S/esFramer.cpp"
# the driver's thread-crossing parts (libCom only)
PARTS="$S/esFrameQueue.cpp $S/esCommands.cpp $S/esEngine.cpp"
fail=0
for t in testWire testTrace testReceiver testReplay testMonitor testFramer testFanOut testDriverParts testEngine; do
    if [ "$t" = testWire ]; then
        $CC $SAN $INC $DEFS "$T/$t.c" "$W/wire.o" $LIBS -o "$W/$t"
    else
        $CXX -std=c++11 $SAN $INC $DEFS "$T/$t.cpp" $CORE $PARTS "$W/wire.o" $LIBS -lpthread -o "$W/$t"
    fi
    if (cd "$W" && ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=1} ./$t > "$W/$t.log" 2>&1) &&
       ! grep -qE 'runtime error|ERROR: AddressSanitizer|ERROR: LeakSanitizer' "$W/$t.log"; then
        echo "ok       $t ($(grep -cE '^ok' "$W/$t.log") checks)"
    else
        echo "FAILED   $t"; grep -m5 -E 'not ok|runtime error|ERROR' "$W/$t.log" | sed 's/^/         /'
        fail=1
    fi
done
# the driver itself, compiled with the sanitizers, against asyn and ADCore (testDriver)
DRV_INC="-I$ADCORE/include -I$ASYN/include"
DRV_LIBS="-L$ADCORE/lib/$ARCH -lADBase -Wl,-rpath,$ADCORE/lib/$ARCH -L$ASYN/lib/$ARCH -lasyn -Wl,-rpath,$ASYN/lib/$ARCH"
t=testDriver
$CXX -std=c++11 $SAN $INC $DRV_INC $DEFS "$T/$t.cpp" "$S/ADEncstream.cpp" $CORE $PARTS "$W/wire.o" \
    $DRV_LIBS -L$BASE/lib/$ARCH -ldbCore -lca $LIBS -lpthread -o "$W/$t"
if (cd "$W" && ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=0} ./$t > "$W/$t.log" 2>&1) &&
   ! grep -qE 'runtime error|ERROR: AddressSanitizer' "$W/$t.log"; then
    echo "ok       $t ($(grep -cE '^ok' "$W/$t.log") checks; leak check off: the ports live to exit)"
else
    echo "FAILED   $t"; grep -m5 -E 'not ok|runtime error|ERROR' "$W/$t.log" | sed 's/^/         /'
    fail=1
fi
# the driver with the real records in an in-process IOC (testDriverRecords): uses the database
# definition and record/device registration that `make` generated, so run `make` first
t=testDriverRecords
GEN=$T/O.$ARCH/${t}_registerRecordDeviceDriver.cpp
if [ ! -f "$GEN" ] || [ ! -f "$T/O.Common/$t.dbd" ]; then
    echo "FAILED   $t: run make first ($GEN and O.Common/$t.dbd are generated)"
    exit 1
fi
cp "$T/O.Common/$t.dbd" "$W/"
$CXX -std=c++11 $SAN $INC $DRV_INC $DEFS -DES_ADCORE_DB=\"$ADCORE/db\" "$T/$t.cpp" "$GEN" "$S/ADEncstream.cpp" \
    $CORE $PARTS "$W/wire.o" $DRV_LIBS -L$BUSY/lib/$ARCH -lbusy -Wl,-rpath,$BUSY/lib/$ARCH \
    -L$BASE/lib/$ARCH -ldbRecStd -ldbCore -lca $LIBS -lpthread -o "$W/$t"
if (cd "$W" && ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=0} ./$t > "$W/$t.log" 2>&1) &&
   ! grep -qE 'runtime error|ERROR: AddressSanitizer|^not ok' "$W/$t.log"; then
    echo "ok       $t ($(grep -cE '^ok' "$W/$t.log") checks; leak check off: the ports live to exit)"
else
    echo "FAILED   $t"; grep -m5 -E 'not ok|runtime error|ERROR' "$W/$t.log" | sed 's/^/         /'
    fail=1
fi
exit $fail
