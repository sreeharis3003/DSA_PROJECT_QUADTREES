
#include "quadtree.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* ---- aggregate helpers --------------------------------------------------- */

static void stats_reset(QTNode *n)
{
    n->total = 0;
    n->sum_z = 0.0;
    n->min_z = DBL_MAX;
    n->max_z = -DBL_MAX;
}

static void stats_add(QTNode *n, double z)
{
    n->total++;
    n->sum_z += z;
    if (z < n->min_z) n->min_z = z;
    if (z > n->max_z) n->max_z = z;
}

static void *xrealloc(void *p, size_t bytes)
{
    void *q = realloc(p, bytes);
    if (!q) {
        fprintf(stderr, "Fatal: out of memory\n");
        exit(EXIT_FAILURE);
    }
    return q;
}

/* ---- node lifecycle ------------------------------------------------------ */

QTNode *qt_node_create(double x0, double y0, double size, int depth)
{
    QTNode *n = xrealloc(NULL, sizeof *n);
    n->x0 = x0;
    n->y0 = y0;
    n->size = size;
    n->depth = depth;
    n->is_leaf = 1;
    for (int i = 0; i < 4; i++) n->child[i] = NULL;
    n->pts = NULL;
    n->count = n->cap = 0;
    stats_reset(n);
    return n;
}

void qt_node_free(QTNode *n)
{
    if (!n) return;
    if (!n->is_leaf)
        for (int i = 0; i < 4; i++) qt_node_free(n->child[i]);
    free(n->pts);
    free(n);
}

int qt_init(QuadTree *t, double x0, double y0, double size, double ground_z)
{
    if (!t || !isfinite(x0) || !isfinite(y0) || !isfinite(ground_z) ||
        !isfinite(size) || size <= 0.0)
        return 0;
    t->root = qt_node_create(x0, y0, size, 0);
    t->ground_z = ground_z;
    return 1;
}

void qt_destroy(QuadTree *t)
{
    if (!t) return;
    qt_node_free(t->root);
    t->root = NULL;
}

/* ---- geometry ------------------------------------------------------------ */

/* Closed interval test: points on the far edge of the map are accepted. */
int qt_contains(const QTNode *n, double x, double y)
{
    return x >= n->x0 && x <= n->x0 + n->size &&
           y >= n->y0 && y <= n->y0 + n->size;
}

/* Which quadrant of n does (x, y) fall in?  O(1). */
int qt_child_index(const QTNode *n, double x, double y)
{
    double half = n->size / 2.0;
    int east  = x >= n->x0 + half;
    int north = y >= n->y0 + half;
    return (north << 1) | east;
}

double qt_obstacle_threshold(const QuadTree *t)
{
    return t->ground_z + QT_OBSTACLE_HEIGHT;
}

/* ---- leaf bucket (amortised O(1) append, doubling growth) ---------------- */

void qt_leaf_push(QTNode *leaf, LidarPoint p)
{
    if (leaf->count == leaf->cap) {
        leaf->cap = leaf->cap ? leaf->cap * 2 : QT_BUCKET_INIT;
        leaf->pts = xrealloc(leaf->pts, (size_t)leaf->cap * sizeof *leaf->pts);
    }
    leaf->pts[leaf->count++] = p;
}

/* ---- adaptive split rule ------------------------------------------------- */

int qt_should_split(const QTNode *n)
{
    return n->is_leaf &&
           n->count >= 2 &&
           (n->max_z - n->min_z) > QT_HEIGHT_TOL &&        /* heterogeneous */
           n->depth < QT_MAX_DEPTH &&                      /* depth limit   */
           n->size / 2.0 >= QT_MIN_CELL;                   /* size limit    */
}

/* Turn leaf n into an internal node with 4 children and push its bucket down.
 * Children that are themselves heterogeneous are split again;
 * recursion is bounded by QT_MAX_DEPTH, so identical (x,y) points can never
 * cause infinite subdivision. */
void qt_split(QTNode *n)
{
    double half = n->size / 2.0;
    for (int i = 0; i < 4; i++)
        n->child[i] = qt_node_create(n->x0 + (i & 1) * half,
                                     n->y0 + (i >> 1) * half,
                                     half, n->depth + 1);

    for (int k = 0; k < n->count; k++) {
        LidarPoint p = n->pts[k];
        QTNode *c = n->child[qt_child_index(n, p.x, p.y)];
        qt_leaf_push(c, p);
        stats_add(c, p.z);
    }

    free(n->pts);
    n->pts = NULL;
    n->count = n->cap = 0;
    n->is_leaf = 0;             /* aggregates of n are unchanged by a split */

    for (int i = 0; i < 4; i++)
        if (qt_should_split(n->child[i])) qt_split(n->child[i]);
}

/* Rebuild n's aggregates: O(count) for a leaf, O(1) for an internal node. */
void qt_recompute(QTNode *n)
{
    stats_reset(n);
    if (n->is_leaf) {
        for (int k = 0; k < n->count; k++) stats_add(n, n->pts[k].z);
        return;
    }
    for (int i = 0; i < 4; i++) {
        const QTNode *c = n->child[i];
        if (c->total == 0) continue;
        n->total += c->total;
        n->sum_z += c->sum_z;
        if (c->min_z < n->min_z) n->min_z = c->min_z;
        if (c->max_z > n->max_z) n->max_z = c->max_z;
    }
}

/* ---- insertion ----------------------------------------------------------- */

/* Walk root -> leaf (O(depth)), updating aggregates on the way down, append
 * to the leaf bucket, then split the leaf if the adaptive rule says so. */
int qt_insert(QuadTree *t, LidarPoint p)
{
    if (!t || !t->root) return 0;
    if (!isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z)) return 0;
    if (!qt_contains(t->root, p.x, p.y)) return 0;

    QTNode *n = t->root;
    while (!n->is_leaf) {
        stats_add(n, p.z);
        n = n->child[qt_child_index(n, p.x, p.y)];
    }
    qt_leaf_push(n, p);
    stats_add(n, p.z);

    if (qt_should_split(n)) qt_split(n);
    return 1;
}
