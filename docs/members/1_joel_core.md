# Member 1 — Joel (CB.SC.U4CSE25720) — Core Quadtree

**Partition:** `src/core/`  **Branch:** `member1-joel-core`

## Files you own
| File | Contents |
|---|---|
| `src/core/quadtree.h` | `LidarPoint`, `QTNode`, `QuadTree`, tuning constants, core API |
| `src/core/quadtree.c` | node create/free, `qt_init`, `qt_insert`, `qt_split`, `qt_should_split`, `qt_recompute`, `qt_child_index`, `qt_leaf_push` |

## Commit plan (from YOUR GitHub account)
1. `core: add LidarPoint/QTNode types and tuning constants`, adding `quadtree.h`
2. `core: node lifecycle, geometry helpers and leaf bucket`, adding `quadtree.c` up to `qt_leaf_push`
3. `core: adaptive split rule and insertion`, completing `quadtree.c`
4. `core: document split rule and depth/size guards` (comments and tidy-ups)

> Tip: to create commits 2–3, paste the file in stages, or commit the full file and then make a follow-up improvement commit. Small, honest commits are better than one giant commit.

## What you must be able to explain
- **Why a region quadtree?** Space is split at fixed midpoints, so a leaf is a real ground cell, and delete/merge are local. A point quadtree (the reference paper) splits at data points, so its shape depends on insertion order.
- **Node layout**: why `child[4]` + `(north<<1)|east` indexing; why pointers instead of an array (an implicit array would need 1.4 M slots for depth 10).
- **Split rule**: split only if heights disagree (`max-min > 0.15 m`). Flat ground never splits, however many returns land on it.
- **Termination**: `QT_MAX_DEPTH` and `QT_MIN_CELL` mean 1,000 points at the same (x,y) can't recurse forever.
- **Aggregates**: insert updates `total/sum/min/max` on the way down in O(1) per level. `qt_recompute` rebuilds a node from its 4 children in O(1).
- **Bucket**: doubling dynamic array, so append is amortised O(1) (the total copy cost 8+16+32+… < 2·cap).

## Complexity you should state
- `qt_insert`: **O(D) amortised**, D ≤ 10 (= O(log n) for spread-out data). Each point moves down at most D levels over its lifetime through splits.
- `qt_split`: O(b) for a bucket of b points (one pass redistributes them).
- `qt_child_index`, `qt_contains`: O(1).

## Likely viva questions
1. *What happens if a point lies exactly on the midpoint line?* `>=` sends it east/north, so it goes to exactly one child.
2. *What if a point is exactly on the map's outer edge (x = x0+size)?* It is accepted, because the root uses a closed interval.
3. *Why is insertion amortised and not worst-case O(D)?* A single insert can trigger a split that moves b points. Summed over all inserts, each point is moved at most D times.
4. *Why store `sum_z` instead of `mean_z`?* The sum updates in O(1) and mean = sum/total. Keeping a mean would need division and rounding on every update.
5. *Memory per node?* About 112 bytes: 4 pointers, bounds, aggregates, plus the bucket.
