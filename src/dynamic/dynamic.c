/*
 * dynamic.c  --  Dynamic-environment updates: delete, move, merge
 * Owner: Member 3 (Sai Ganesan, CB.SC.U4CSE25749)
 */
#include "dynamic.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define POINT_EPS 1e-9

/* ---- merge (inverse of split) ------------------------------------------- */

int qt_try_merge(QTNode *n)
{
    if (!n || n->is_leaf) return 0;
    for (int i = 0; i < 4; i++)
        if (!n->child[i]->is_leaf) return 0;

    int flat = n->total <= 1 || (n->max_z - n->min_z) <= QT_HEIGHT_TOL;
    if (!flat) return 0;

    /* Pull every child's bucket up into n, then free the children. */
    n->is_leaf = 1;
    n->pts = NULL;
    n->count = n->cap = 0;
    for (int i = 0; i < 4; i++) {
        QTNode *c = n->child[i];
        for (int k = 0; k < c->count; k++) qt_leaf_push(n, c->pts[k]);
        qt_node_free(c);
        n->child[i] = NULL;
    }
    qt_recompute(n);
    return 1;
}

/* ---- single point delete: O(depth + bucket) ----------------------------- */

static int delete_rec(QTNode *n, double x, double y, LidarPoint *removed)
{
    if (n->is_leaf) {
        for (int k = 0; k < n->count; k++) {
            if (fabs(n->pts[k].x - x) <= POINT_EPS && fabs(n->pts[k].y - y) <= POINT_EPS) {
                if (removed) *removed = n->pts[k];
                n->pts[k] = n->pts[--n->count];      /* swap-remove, O(1) */
                qt_recompute(n);
                return 1;
            }
        }
        return 0;
    }
    if (!delete_rec(n->child[qt_child_index(n, x, y)], x, y, removed)) return 0;
    qt_recompute(n);          /* O(1) from children */
    qt_try_merge(n);          /* cascades upward as recursion unwinds */
    return 1;
}

int qt_delete_point(QuadTree *t, double x, double y, LidarPoint *removed)
{
    if (!t || !t->root || !isfinite(x) || !isfinite(y) || !qt_contains(t->root, x, y))
        return 0;
    return delete_rec(t->root, x, y, removed);
}

/* ---- object removal: one post-order pass, merges bottom-up -------------- */

static int matches(const LidarPoint *p, int id)
{
    return id == QT_ALL_DYNAMIC ? p->id > 0 : p->id == id;
}

static int remove_rec(QTNode *n, int id)
{
    if (n->total == 0) return 0;
    int removed = 0;

    if (n->is_leaf) {
        int w = 0;                                   /* in-place filter */
        for (int k = 0; k < n->count; k++)
            if (!matches(&n->pts[k], id)) n->pts[w++] = n->pts[k];
        removed = n->count - w;
        n->count = w;
        if (removed) qt_recompute(n);
        return removed;
    }
    for (int i = 0; i < 4; i++) removed += remove_rec(n->child[i], id);
    if (removed) {
        qt_recompute(n);
        qt_try_merge(n);
    }
    return removed;
}

int qt_remove_object(QuadTree *t, int id)
{
    if (!t || !t->root || (id <= 0 && id != QT_ALL_DYNAMIC)) return 0;
    return remove_rec(t->root, id);
}

/* ---- move object: collect, remove, re-insert shifted --------------------- */

typedef struct {
    LidarPoint *v;
    int n, cap;
} PointVec;

static void vec_push(PointVec *pv, LidarPoint p)
{
    if (pv->n == pv->cap) {
        pv->cap = pv->cap ? pv->cap * 2 : 16;
        LidarPoint *nv = realloc(pv->v, (size_t)pv->cap * sizeof *nv);
        if (!nv) {
            fprintf(stderr, "Fatal: out of memory\n");
            exit(EXIT_FAILURE);
        }
        pv->v = nv;
    }
    pv->v[pv->n++] = p;
}

static void collect_rec(const QTNode *n, int id, PointVec *pv)
{
    if (n->total == 0) return;
    if (n->is_leaf) {
        for (int k = 0; k < n->count; k++)
            if (n->pts[k].id == id) vec_push(pv, n->pts[k]);
        return;
    }
    for (int i = 0; i < 4; i++) collect_rec(n->child[i], id, pv);
}

MoveResult qt_move_object(QuadTree *t, int id, double dx, double dy)
{
    MoveResult r = {0, 0};
    if (!t || !t->root || id <= 0 || !isfinite(dx) || !isfinite(dy)) return r;

    PointVec pv = {NULL, 0, 0};
    collect_rec(t->root, id, &pv);
    qt_remove_object(t, id);                 /* old cells merge back */

    for (int k = 0; k < pv.n; k++) {
        LidarPoint p = pv.v[k];
        p.x += dx;
        p.y += dy;
        if (qt_insert(t, p)) r.moved++;      /* new cells split as needed */
        else r.dropped++;
    }
    free(pv.v);
    return r;
}

/* ---- object listing ------------------------------------------------------ */

typedef struct {
    ObjectInfo *v;
    int n, cap;
    double *sx, *sy;
} ObjTable;

static void objects_rec(const QTNode *n, ObjTable *tab)
{
    if (n->total == 0) return;
    if (!n->is_leaf) {
        for (int i = 0; i < 4; i++) objects_rec(n->child[i], tab);
        return;
    }
    for (int k = 0; k < n->count; k++) {
        const LidarPoint *p = &n->pts[k];
        if (p->id <= 0) continue;
        int j = 0;
        while (j < tab->n && tab->v[j].id != p->id) j++;
        if (j == tab->n) {
            if (tab->n == tab->cap) {
                tab->cap = tab->cap ? tab->cap * 2 : 8;
                tab->v = realloc(tab->v, (size_t)tab->cap * sizeof *tab->v);
                tab->sx = realloc(tab->sx, (size_t)tab->cap * sizeof *tab->sx);
                tab->sy = realloc(tab->sy, (size_t)tab->cap * sizeof *tab->sy);
                if (!tab->v || !tab->sx || !tab->sy) {
                    fprintf(stderr, "Fatal: out of memory\n");
                    exit(EXIT_FAILURE);
                }
            }
            tab->v[j].id = p->id;
            tab->v[j].count = 0;
            tab->v[j].max_z = p->z;
            tab->sx[j] = tab->sy[j] = 0.0;
            tab->n++;
        }
        tab->v[j].count++;
        tab->sx[j] += p->x;
        tab->sy[j] += p->y;
        if (p->z > tab->v[j].max_z) tab->v[j].max_z = p->z;
    }
}

int qt_list_objects(const QuadTree *t, ObjectInfo *out, int max)
{
    if (!t || !t->root) return 0;
    ObjTable tab = {NULL, 0, 0, NULL, NULL};
    objects_rec(t->root, &tab);

    /* sort by id (few objects -> insertion sort) */
    for (int i = 1; i < tab.n; i++) {
        ObjectInfo o = tab.v[i];
        double sx = tab.sx[i], sy = tab.sy[i];
        int j = i - 1;
        while (j >= 0 && tab.v[j].id > o.id) {
            tab.v[j + 1] = tab.v[j];
            tab.sx[j + 1] = tab.sx[j];
            tab.sy[j + 1] = tab.sy[j];
            j--;
        }
        tab.v[j + 1] = o;
        tab.sx[j + 1] = sx;
        tab.sy[j + 1] = sy;
    }
    for (int i = 0; i < tab.n && i < max; i++) {
        out[i] = tab.v[i];
        out[i].cx = tab.sx[i] / tab.v[i].count;
        out[i].cy = tab.sy[i] / tab.v[i].count;
    }
    int total = tab.n;
    free(tab.v);
    free(tab.sx);
    free(tab.sy);
    return total;
}
