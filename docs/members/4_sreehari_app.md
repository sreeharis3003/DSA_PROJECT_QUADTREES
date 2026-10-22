# Member 4 — Sreehari S (CB.SC.U4CSE25757) — LiDAR I/O, Visualisation, CLI, Tests

**Partition:** `src/app/`, `tests/`, `data/`, `demo/`  **Branch:** `member4-sreehari-app`

## Files you own
| File | Contents |
|---|---|
| `src/app/lidar_io.h/.c` | file loader, deterministic synthetic scene, ASCII height and depth maps, CSV export, benchmark |
| `src/app/main.c` | command interpreter (interactive + script mode), input validation |
| `tests/cases/*.txt`, `tests/expected/*.out`, `tests/run_tests.sh/.bat` | regression tests |
| `data/sample_scan.txt`, `demo/demo.txt` | sample LiDAR scan and the live-demo script |

## Commit plan (from YOUR GitHub account)
1. `app: LiDAR file loader and deterministic scene generator`
2. `app: ASCII height map, depth map and CSV export`
3. `app: command-line interface with input validation`
4. `app: benchmark command for complexity demonstration`
5. `tests: add 6 regression cases, expected outputs and runners`
6. `data/demo: sample scan and live demo script`

## What you must be able to explain
- **Input validation**: every number goes through `strtod`/`strtol` with a full-string check, so `abc`, `2.5` as an id, `nan` and `inf` are all rejected. Argument counts are checked per command, and lines over 1023 characters are discarded safely.
- **Script vs interactive mode**: `quadmap file.txt` echoes each command (`> ...`), which makes the output diffable for tests.
- **Deterministic RNG**: a custom LCG (`x = x·1103515245 + 12345`) instead of `rand()`, because `rand()` differs between Windows and Linux and expected outputs must match everywhere.
- **Scene generator**: building (2.5 m), pole (3 m), ramp (0–0.6 m slope), car id 1 (1.5 m), pedestrian id 2 (1.7 m). Ground points are not generated inside object footprints, which models occlusion.
- **Depth map** (`DEPTH`): prints each leaf's depth, giving visual proof of adaptive resolution. Flat ground shows 2–3, and obstacle edges show 6–10.
- **Benchmark** (`BENCH`): builds 10³…10⁶-point trees and measures average nodes per point query. It grows about 1.6 per ×10, which equals log₄ 10, confirming O(log n).
- **Tests**: `sh tests/run_tests.sh` diffs every case against `tests/expected`. `--update` regenerates them after an intended change.

## Complexity you should state
- Loading a file of n points: **O(n·D)** (n inserts).
- ASCII map of c×r characters: **O(c·r·D)** (one point query per character).
- CSV export: O(m).

## Likely viva questions
1. *How do you handle a malformed line in the scan file?* `sscanf` must read at least 3 numbers, and the id must be ≥ 0 and the point inside the map. Otherwise the line is counted as skipped (test 06 skips 4 lines).
2. *Why does the map render use the cell's `max_z` and not `mean_z`?* For safety, a cell is as tall as the tallest thing in it.
3. *How would faculty run their own test case?* Start `quadmap` and type commands, or put commands in a .txt file and run `quadmap file.txt`.
4. *Why a command interpreter instead of a menu?* Faculty test cases become one-line commands, scripts are repeatable, and the same code drives both the tests and the live demo.
