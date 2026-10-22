# Member 3 — Sai Ganesan (CB.SC.U4CSE25749) — Dynamic Updates (delete / merge / move)

**Partition:** `src/dynamic/`  **Branch:** `member3-sai-dynamic`

## Files you own
| File | Contents |
|---|---|
| `src/dynamic/dynamic.h` | `ObjectInfo`, `MoveResult`, dynamic-update API |
| `src/dynamic/dynamic.c` | `qt_try_merge`, `qt_delete_point`, `qt_remove_object`, `qt_move_object`, `qt_list_objects` |

## Commit plan (from YOUR GitHub account)
1. `dynamic: add API for delete/move/merge`, adding `dynamic.h`
2. `dynamic: merge rule (inverse of split)`, adding `qt_try_merge`
3. `dynamic: single point delete with upward merge cascade`
4. `dynamic: remove and move tracked objects by id`
5. `dynamic: list tracked objects with centroids`

## What you must be able to explain
- **Why merging matters**: without it, a cell subdivided for a car stays subdivided after the car leaves, and the map fills with garbage nodes. Merging keeps memory proportional to the *current* scene.
- **Merge rule = exact inverse of the split rule**: merge 4 leaf children iff the merged cell would not be split again (empty, 1 point, or spread ≤ 0.15 m). This makes the tree **canonical**: same points give the same shape, whatever the order of operations.
- **Delete point**: recursive descent to the leaf, swap-remove, then on the way back up `qt_recompute` (O(1) from the children) and `qt_try_merge`. Merges cascade upward automatically.
- **Remove object**: one post-order traversal. Each leaf filters out the object's points in place, then each internal node recomputes and tries to merge. Subtrees with `total == 0` are skipped.
- **Move object**: collect the object's points, remove them (old cells merge back), then re-insert shifted (new cells split). Points pushed outside the map are counted as `dropped`.
- **Why recompute instead of decrement?** `min`/`max` can't be undone by subtraction. If you delete the current max, the next max must come from the children.

## Complexity you should state
| Operation | Time | Why |
|---|---|---|
| Delete point | **O(D + b)** | O(D) descent, O(b) bucket scan, O(1) recompute per level, merge cost bounded by bucket sizes |
| Merge one node | O(points in the 4 children) | each point is copied once into the parent bucket |
| Remove object | **O(m + n)** | one traversal (empty subtrees skipped) |
| Move object (k points) | **O(m + n + k·D)** | traversal + k inserts |

## Likely viva questions
1. *Show split → merge live.* Run `quadmap tests/cases/04_split_merge.txt`: 1 node, then 5 nodes after the obstacle, then 1 node after deleting it.
2. *What if the object moves off the map?* Points outside are not inserted and are reported as "left the map" (test 05, pedestrian).
3. *Why not keep a hash map id → points to make removal O(k)?* It adds bookkeeping to every insert, delete and split. With few tracked objects, one traversal is simpler and fast enough. This is a conscious trade-off.
4. *Why does node count go UP after moving the car in the demo?* The car moved onto ground that was already scanned. Stale ground returns and car returns share cells, so those cells are genuinely mixed and split to the finest level. A real system would ray-trace and clear free space.
5. *Can merging cause the parent to need a merge too?* Yes. `delete_rec` calls `qt_try_merge` at every level as recursion unwinds, so merges cascade up to the root (test 03 collapses depth 10 back to 1 node).
