/*
 * query.c  --  Spatial queries on the 2.5D LiDAR quadtree
 */
#include "query.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* ---- rectangle helpers --------------------------------------------------- */

static void normalise_rect(double *x1, double *y1, double *x2, double *y2)
{
    double t;
    if (*x1 > *x2) { t = *x1; *x1 = *x2; *x2 = t; }
    if (*y1 > *y2) { t = *y1; *y1 = *y2; *y2 = t; }
}

static int box_intersects(const QTNode *n, double x1, double y1, double x2, double y2)
{
    return n->x0 <= x2 && n->x0 + n->size >= x1 &&
           n->y0 <= y2 && n->y0 + n->size >= y1;
}

static int box_inside(const QTNode *n, double x1, double y1, double x2, double y2)
{
    return n->x0 >= x1 && n->x0 + n->size <= x2 &&
           n->y0 >= y1 && n->y0 + n->size <= y2;
}

static int point_in_rect(const LidarPoint *p, double x1, double y1, double x2, double y2)
{
    return p->x >= x1 && p->x <= x2 && p->y >= y1 && p->y <= y2;
}

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

/* ---- range query --------------------------------------------------------- */

static void range_rec(const QTNode *n, double thr, double x1, double y1, double x2,
                      double y2, PointVisitor visit, void *ctx, RangeResult *r)
{
    r->nodes_visited++;
    if (n->total == 0 || !box_intersects(n, x1, y1, x2, y2)) return;   /* prune */

    if (n->is_leaf) {
        for (int k = 0; k < n->count; k++) {
            const LidarPoint *p = &n->pts[k];
            if (!point_in_rect(p, x1, y1, x2, y2)) continue;
            r->count++;
            if (p->z > thr) r->obstacle_count++;
            if (p->z > r->max_z) r->max_z = p->z;
            if (visit) visit(p, ctx);
        }
        return;
    }
    for (int i = 0; i < 4; i++)
        range_rec(n->child[i], thr, x1, y1, x2, y2, visit, ctx, r);
}

RangeResult qt_range_query(const QuadTree *t, double x1, double y1,
                           double x2, double y2, PointVisitor visit, void *ctx)
{
    RangeResult r = {0, 0, -DBL_MAX, 0};
    if (!t || !t->root) return r;
    normalise_rect(&x1, &y1, &x2, &y2);
    range_rec(t->root, qt_obstacle_threshold(t), x1, y1, x2, y2, visit, ctx, &r);
    return r;
}

/* ---- collision check ----------------------------------------------------- */

static int blocked_rec(const QTNode *n, double thr, double x1, double y1,
                       double x2, double y2, int *visited)
{
    (*visited)++;
    if (n->total == 0 || n->max_z <= thr) return 0;      /* no obstacle below here */
    if (!box_intersects(n, x1, y1, x2, y2)) return 0;
    if (box_inside(n, x1, y1, x2, y2)) return 1;          /* whole cell inside & has obstacle */

    if (n->is_leaf) {
        for (int k = 0; k < n->count; k++)
            if (n->pts[k].z > thr && point_in_rect(&n->pts[k], x1, y1, x2, y2))
                return 1;
        return 0;
    }
    for (int i = 0; i < 4; i++)
        if (blocked_rec(n->child[i], thr, x1, y1, x2, y2, visited)) return 1;
    return 0;
}

int qt_region_blocked(const QuadTree *t, double x1, double y1,
                      double x2, double y2, int *nodes_visited)
{
    int dummy = 0;
    if (!nodes_visited) nodes_visited = &dummy;
    *nodes_visited = 0;
    if (!t || !t->root) return 0;
    normalise_rect(&x1, &y1, &x2, &y2);
    return blocked_rec(t->root, qt_obstacle_threshold(t), x1, y1, x2, y2, nodes_visited);
}
