# Adaptive-Resolution 2.5D LiDAR Mapping for Dynamic Environment Perception

**Team 4 — CSE H** · DSA Lab Evaluation 3
Joel (CB.SC.U4CSE25720) · R Vishal (CB.SC.U4CSE25742) · Sai Ganesan (CB.SC.U4CSE25749) · Sreehari S (CB.SC.U4CSE25757)

## Problem
Autonomous vehicles and warehouse robots must convert dense LiDAR scans into a compact, real-time
**2.5D height map** that adapts its resolution and tracks moving obstacles.

## Data structure: adaptive region quadtree
The map is a square that is recursively split into 4 quadrants, **only where the heights inside a cell
disagree**. Each leaf is a ground cell of (nearly) uniform height. Every node keeps
`count / min / max / sum` of heights for its subtree, so queries can skip whole branches. When obstacles
move, cells split at the new position and **merge** back at the old one.

On the demo scene (5,000 returns, 64 m × 64 m), the map uses **475 leaf cells** where a fixed grid at the
same finest resolution would need **262,144 cells**.

## Build
Requires a C99 compiler (gcc / MinGW / TDM-GCC).
```bash
mingw32-make          # Windows (TDM-GCC)   |  make   on Linux/macOS/Git Bash
```
or on Windows without make (PowerShell needs the `.\` prefix):
```powershell
.\build.bat
```

## Run
Windows PowerShell:
```powershell
.\quadmap.exe                   # interactive (type HELP)
.\quadmap.exe demo\demo.txt     # scripted live-demo walkthrough
.\tests\run_tests.bat           # 6 regression tests
```
Git Bash / Linux / macOS:
```bash
./quadmap                  # interactive (type HELP)
./quadmap demo/demo.txt    # scripted live-demo walkthrough
sh tests/run_tests.sh      # 6 regression tests
```

## Commands
| Command | Description |
|---|---|
| `INIT x0 y0 size [ground_z]` | new empty map (default `0 0 64 0`) |
| `INSERT x y z [id]` | add one LiDAR return (`id > 0` = moving object) |
| `LOAD file` | load `x y z [id]` lines |
| `GEN [seed] [n]` | synthetic street scan (building, pole, ramp, car id 1, pedestrian id 2) |
| `HEIGHT x y` | height and occupancy of the cell at (x, y) |
| `RANGE x1 y1 x2 y2` | returns inside a rectangle |
| `COLLIDE x1 y1 x2 y2` | is the rectangle blocked? |
| `NEAREST x y` | nearest obstacle return |
| `DELETE x y` | delete the return at (x, y) |
| `OBJECTS` / `MOVE id dx dy` / `REMOVEOBJ id` / `CLEARDYN` | dynamic obstacle tracking |
| `STATS` | node and leaf counts, depth, memory vs. fixed grid |
| `MAP [c r]` / `DEPTH [c r]` | ASCII height map / leaf-depth map |
| `EXPORT file` | leaf cells to CSV |
| `BENCH` | scaling benchmark 10³ … 10⁶ points |

Every query prints **nodes visited** so the effect of pruning is visible.

## Complexity summary
| Operation | Time |
|---|---|
| Point query / insert | O(D) = O(log n), D ≤ 10 (insert amortised) |
| Delete point | O(D + b) |
| Range query | O(√m + k) typical |
| Nearest obstacle | O(log n) expected |
| Remove / move object | O(m + n) (+ k·D for move) |
| Space | O(m + n), cells ∝ obstacle boundary, not area |

## Reference
R. A. Finkel and J. L. Bentley, "Quad Trees: A Data Structure for Retrieval on Composite Keys,"
*Acta Informatica*, vol. 4, no. 1, pp. 1–9, 1974.
