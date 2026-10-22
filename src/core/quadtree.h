/*
 * quadtree.h  --  Core adaptive-resolution quadtree for 2.5D LiDAR mapping
 * Owner: Member 1 (Joel, CB.SC.U4CSE25720)
 *
 * A region quadtree over a square ground area. Every node covers the square
 * [x0, x0+size] x [y0, y0+size]. Leaves hold the raw LiDAR returns that fall
 * in their cell (a "bucket"); every node also keeps aggregate height
 * statistics (count / min / max / sum of z) for its whole subtree, which the
 * query module uses to prune entire branches.
 *
 * Adaptive rule: a leaf is split into 4 children ONLY when it is
 *   heterogeneous -> height spread (max_z - min_z) > QT_HEIGHT_TOL
 * and it is still allowed to go deeper (depth / minimum cell size limits).
 * So every leaf is (as far as the limits allow) a cell of uniform height:
 * flat ground stays as a few big cells no matter how many returns hit it,
 * while obstacle edges get fine cells -> the number of cells grows with
 * scene complexity, not with area or with the number of LiDAR returns.
 */
#ifndef QUADTREE_H
#define QUADTREE_H

#include <stddef.h>

/* ---- Tuning parameters (metres) ---------------------------------------- */
#define QT_BUCKET_INIT     8     /* initial capacity of a leaf bucket (doubles on demand) */
#define QT_MAX_DEPTH       10    /* hard depth limit -> bounds every root-to-leaf walk   */
#define QT_MIN_CELL        0.05  /* never create cells smaller than 5 cm                  */
#define QT_HEIGHT_TOL      0.15  /* a cell is "flat" if its height spread is <= 15 cm     */
#define QT_OBSTACLE_HEIGHT 0.30  /* a return > 30 cm above ground is an obstacle          */

/* Child order: index = (north << 1) | east */
enum { QT_SW = 0, QT_SE = 1, QT_NW = 2, QT_NE = 3 };

/* One LiDAR return. id == 0 -> static scene, id > 0 -> tracked dynamic object. */
typedef struct {
    double x, y, z;
    int id;
} LidarPoint;

typedef struct QTNode {
    double x0, y0, size;        /* square region covered by this node        */
    int depth;                  /* root = 0                                  */
    int is_leaf;

    struct QTNode *child[4];    /* valid only when !is_leaf                  */

    LidarPoint *pts;            /* leaf bucket (dynamic array), NULL if internal */
    int count, cap;

    /* Aggregates over the whole subtree (kept for internal nodes too). */
    int total;                  /* number of points in subtree               */
    double min_z, max_z, sum_z;
} QTNode;

typedef struct {
    QTNode *root;
    double ground_z;            /* reference ground height                   */
} QuadTree;

/* ---- Tree lifecycle ----------------------------------------------------- */
int  qt_init(QuadTree *t, double x0, double y0, double size, double ground_z);
void qt_destroy(QuadTree *t);

/* Insert one point. Returns 1 on success, 0 if outside the map / invalid. */
int  qt_insert(QuadTree *t, LidarPoint p);

/* ---- Helpers shared with the query / dynamic modules -------------------- */
QTNode *qt_node_create(double x0, double y0, double size, int depth);
void    qt_node_free(QTNode *n);
int     qt_contains(const QTNode *n, double x, double y);
int     qt_child_index(const QTNode *n, double x, double y);
void    qt_leaf_push(QTNode *leaf, LidarPoint p);
int     qt_should_split(const QTNode *n);
void    qt_split(QTNode *n);
void    qt_recompute(QTNode *n);          /* rebuild aggregates from bucket / children */
double  qt_obstacle_threshold(const QuadTree *t);   /* absolute z above which = obstacle */

#endif
