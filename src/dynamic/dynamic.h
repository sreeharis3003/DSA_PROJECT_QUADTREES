
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
