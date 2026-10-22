#!/bin/sh
# Runs every tests/cases/*.txt script and diffs against tests/expected/*.out
# Usage (from repo root):  sh tests/run_tests.sh          -> check
#                          sh tests/run_tests.sh --update -> regenerate expected
BIN=./quadmap
[ -x ./quadmap.exe ] && BIN=./quadmap.exe
mkdir -p tests/actual
pass=0; fail=0
for c in tests/cases/*.txt; do
    name=$(basename "$c" .txt)
    "$BIN" "$c" | tr -d '\r' > "tests/actual/$name.out"
    if [ "$1" = "--update" ]; then
        cp "tests/actual/$name.out" "tests/expected/$name.out"
        echo "updated $name"
    elif diff -q "tests/expected/$name.out" "tests/actual/$name.out" >/dev/null; then
        echo "PASS  $name"; pass=$((pass+1))
    else
        echo "FAIL  $name   (diff tests/expected/$name.out tests/actual/$name.out)"; fail=$((fail+1))
    fi
done
[ "$1" = "--update" ] || echo "$pass passed, $fail failed"
[ "$fail" -eq 0 ]
