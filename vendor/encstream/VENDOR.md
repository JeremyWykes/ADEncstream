# Vendored from encstream

The encstream packet codec and its golden packets, copied unchanged from the encstream code
repository so that the Brick sender and this receiver use one implementation of the wire format
(protocol version 1; the encstream wire-format spec).

| Here | encstream |
|---|---|
| `wire.h`, `wire.c` | `brick/src/wire.h`, `brick/src/wire.c` |
| `golden/*.hex`, `golden/README.md` | `brick/tests/golden/` |

Source commit: **`9b9a6e7c31ccad2ae362e404435b654bcb414ff5`** (the companion encstream public release;
`tools/check_vendor.sh <encstream checkout>` verifies the vendored files match it).

Rules:
- Never edit these files here. Change them in encstream, then copy them again and update the
  commit above in the same commit.
- `tools/check_vendor.sh <encstream checkout>` checks that every file here equals encstream's at
  the recorded commit, and reports whether encstream's current version differs (an update to
  take). `ADEncstreamApp/test/testWire` checks the copy against the golden packets on each
  compiler that builds this module.
