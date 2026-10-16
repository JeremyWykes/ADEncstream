#!/bin/sh
# differential_replay.sh - replay traces through the receiver core and encstream_rx.py and compare
# the shared facts. For development hosts with Python 3.
#     tools/differential_replay.sh <encstream checkout> [trace ...]
# Default trace: ADEncstreamApp/test/traces/mock_stream.estrace. For each trace it also derives
# variants (loss, duplication, reordering, corruption, all four mixed) and compares those too.
# Needs esReplay built (make). Exit 0 if both receivers agree on every trace, else 1.
set -eu
ES=${1:?usage: tools/differential_replay.sh <encstream checkout> [trace ...]}
shift
HERE=$(cd "$(dirname "$0")/.." && pwd)
REPLAY=$(ls "$HERE"/ADEncstreamApp/test/O.*/esReplay 2>/dev/null | head -1)
[ -x "$REPLAY" ] || { echo "differential_replay: build first (esReplay not found)"; exit 2; }
[ $# -gt 0 ] || set -- "$HERE/ADEncstreamApp/test/traces/mock_stream.estrace"
W=$(mktemp -d)
trap 'rm -rf "$W"' EXIT
fail=0
for t in "$@"; do
    python3 - "$ES/host" "$t" "$W" <<'PY'
import os, sys
sys.path.insert(0, sys.argv[1])
import encstream_trace as tr
src, out = sys.argv[2], sys.argv[3]
raw = open(src, "rb").read()
recs, pos = [], 8
while pos < len(raw):
    n = raw[pos + 10] | (raw[pos + 11] << 8)
    recs.append(bytearray(raw[pos:pos + 12 + n]))
    pos += 12 + n
def is_data(r):
    return r[8] == tr.RECEIVED and len(r) > 38 and r[12 + 10 + 5] == 1
def timer(r):
    t = bytearray(r[:12]); t[8], t[9], t[10], t[11] = tr.TIMER, 0, 0, 0
    return t
def write(name, rs):
    with open(os.path.join(out, name + ".estrace"), "wb") as f:
        f.write(b"ESTRACE1" + b"".join(bytes(r) for r in rs))
def lose(rs, data):
    rs = list(rs)
    for k in range(25, len(data) - 1, 50):
        rs[data[k]] = timer(rs[data[k]])
    return rs
def duplicate(rs, data):
    dup = {data[k] for k in range(20, len(data), 40)}
    return [x for i, r in enumerate(rs) for x in ([r, r] if i in dup else [r])]
def reorder(rs, data):
    rs = list(rs)
    for k in range(15, len(data) - 2, 30):
        a, b = data[k], data[k + 1]
        ta, tb = rs[a][:8], rs[b][:8]
        rs[a], rs[b] = bytearray(ta + rs[b][8:]), bytearray(tb + rs[a][8:])
    return rs
def corrupt(rs, data):
    rs = [bytearray(r) for r in rs]
    for i in range(30, len(rs), 60):
        if rs[i][8] == tr.RECEIVED and i not in (data[0], data[-1]):
            rs[i][22] ^= 0xFF
    return rs
data = [i for i, r in enumerate(recs) if is_data(r)]
write("original", recs)
for name, f in (("loss", lose), ("duplication", duplicate), ("reordering", reorder),
                ("corruption", corrupt)):
    write(name, f(recs, data))
mixed = corrupt(reorder(lose(recs, data), data), data)
mixed_data = [i for i, r in enumerate(mixed) if is_data(r)]
write("mixed", duplicate(mixed, mixed_data))
PY
    for v in original loss duplication reordering corruption mixed; do
        f="$W/$v.estrace"
        "$REPLAY" "$f" > "$W/core.txt" 2> "$W/core.err" || true
        python3 "$ES/host/encstream_replay.py" "$f" > "$W/py.txt" 2> "$W/py.err" || true
        if cmp -s "$W/core.txt" "$W/py.txt" && [ ! -s "$W/core.err" ] && [ ! -s "$W/py.err" ]; then
            echo "agree     $(basename "$t") $v: $(grep -E '^(rows|rows_lost|late_or_duplicate|rejected)=' "$W/core.txt" | tr '\n' ' ')"
        else
            echo "DIFFER    $(basename "$t") $v"
            diff "$W/core.txt" "$W/py.txt" | sed 's/^/          /' || true
            sed 's/^/          core: /' "$W/core.err"; sed 's/^/          python: /' "$W/py.err"
            fail=1
        fi
    done
done
exit $fail
