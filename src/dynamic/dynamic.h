/*
 * dynamic.h  --  Dynamic-environment updates: delete, move, merge
 * Owner: Member 3 (Sai Ganesan, CB.SC.U4CSE25749)
 *
 * When obstacles move or disappear, cells that were subdivided for them must
 * collapse back (merge) so the map stays compact. The merge rule is the exact
 * inverse of the split rule in quadtree.c:
 *     merge 4 leaf children  <=>  merged cell would NOT be split again
 *     (height spread of the merged cell <= QT_HEIGHT_TOL, or it is empty)
 * so the tree shape is always the same as if it had been built from scratch.
 */
#ifndef DYNAMIC_H
#define DYNAMIC_H

#include "quadtree.h"

#define QT_ALL_DYNAMIC (-1)     /* pass as id to remove every object with id > 0 */

typedef struct {
    int id;
    int count;
    double cx, cy;              /* centroid of the object's returns          */
    double max_z;
} ObjectInfo;

typedef struct {
    int moved;                  /* points re-inserted at the new position    */
    int dropped;                /* points that left the map boundary         */
} MoveResult;

/* Delete one return located exactly at (x, y). Returns 1 if deleted. */
int qt_delete_point(QuadTree *t, double x, double y, LidarPoint *removed);

/* Delete every return of object `id` (or all dynamic ones with QT_ALL_DYNAMIC).
 * Returns the number of points removed. */
int qt_remove_object(QuadTree *t, int id);

/* Shift every return of object `id` by (dx, dy). */
MoveResult qt_move_object(QuadTree *t, int id, double dx, double dy);

/* List dynamic objects (id > 0). Fills up to `max` entries; returns how many
 * distinct objects exist. */
int qt_list_objects(const QuadTree *t, ObjectInfo *out, int max);

/* Collapse n into a leaf if its 4 children are leaves and the merge rule
 * holds. Returns 1 if merged. */
int qt_try_merge(QTNode *n);

#endif
