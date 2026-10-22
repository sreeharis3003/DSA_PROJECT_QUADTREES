/*
 * query.c  --  Spatial queries on the 2.5D LiDAR quadtree
 */
#include "query.h"

#include <math.h>
#include <string.h>

/* ---- point query: O(depth) ---------------------------------------------- */

int qt_cell_at(const QuadTree *t, double x, double y, CellInfo *out)
{
    memset(out, 0, sizeof *out);
    if (!t || !t->root || !isfinite(x) || !isfinite(y) || !qt_contains(t->root, x, y))
        return 0;

    const QTNode *n = t->root;
    out->nodes_visited = 1;
    while (!n->is_leaf) {
        n = n->child[qt_child_index(n, x, y)];
        out->nodes_visited++;
    }

    out->x0 = n->x0;
    out->y0 = n->y0;
    out->size = n->size;
    out->depth = n->depth;
    out->count = n->count;
    out->known = n->count > 0;
    if (out->known) {
        out->min_z = n->min_z;
        out->max_z = n->max_z;
        out->mean_z = n->sum_z / n->count;
        out->occupied = n->max_z > qt_obstacle_threshold(t);
    }
    return 1;
}
