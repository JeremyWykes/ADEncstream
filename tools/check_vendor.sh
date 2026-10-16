#!/bin/sh
# check_vendor.sh - compare vendor/encstream with the encstream repository.
#     tools/check_vendor.sh <encstream checkout>
# Exit 0: every vendored file equals encstream's at the commit recorded in VENDOR.md, and
# encstream's HEAD has the same content. Exit 1: a file differs from the recorded commit (edited
# here, or VENDOR.md not updated). Exit 2: usage or repository error. Exit 3: the copy matches the
# recorded commit but encstream's HEAD has changed the files since (an update to take).
set -eu
ES=${1:?usage: tools/check_vendor.sh <encstream checkout>}
HERE=$(cd "$(dirname "$0")/.." && pwd)
V="$HERE/vendor/encstream"
REV=$(sed -n 's/^Source commit: \*\*`\([0-9a-f]\{40\}\)`\*\*.*/\1/p' "$V/VENDOR.md")
[ -n "$REV" ] || { echo "check_vendor: no source commit in VENDOR.md"; exit 2; }
git -C "$ES" cat-file -e "$REV^{commit}" 2>/dev/null || { echo "check_vendor: $ES has no commit $REV"; exit 2; }
bad=0; newer=0
check() {   # <vendored file> <path in encstream>
    if ! git -C "$ES" show "$REV:$2" | cmp -s - "$V/$1"; then
        echo "DIFFERS from $REV: $1 ($2)"; bad=1
    elif ! git -C "$ES" show "HEAD:$2" | cmp -s - "$V/$1"; then
        echo "newer in encstream HEAD: $2"; newer=1
    fi
}
check wire.h brick/src/wire.h
check wire.c brick/src/wire.c
for f in "$V"/golden/*; do check "golden/${f##*/}" "brick/tests/golden/${f##*/}"; done
# a golden file added in encstream since
for f in $(git -C "$ES" ls-tree --name-only HEAD brick/tests/golden/); do
    [ -e "$V/golden/${f##*/}" ] || { echo "new in encstream HEAD: $f"; newer=1; }
done
[ $bad = 0 ] || exit 1
[ $newer = 0 ] || exit 3
echo "check_vendor: vendor/encstream matches encstream $REV and HEAD"
