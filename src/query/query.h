/*
 * query.h  --  Spatial queries on the 2.5D LiDAR quadtree
 *
 * Every query reports `nodes_visited` so the team can demonstrate the
 * pruning (and hence the complexity) live during the evaluation.
 */
#ifndef QUERY_H
#define QUERY_H

#include "quadtree.h"

/* Height-map cell (leaf) that contains a query location. */
typedef struct {
    int known;                  /* 0 -> no LiDAR returns in this cell        */
    double x0, y0, size;        /* leaf square                               */
    int depth;
    int count;
    double min_z, max_z, mean_z;
    int occupied;               /* max_z above the obstacle threshold        */
    int nodes_visited;
} CellInfo;

typedef void (*PointVisitor)(const LidarPoint *p, void *ctx);

typedef struct {
    int count;                  /* points inside the rectangle               */
    int obstacle_count;         /* of which are obstacle returns             */
    double max_z;
    int nodes_visited;
} RangeResult;

typedef struct {
    size_t nodes, leaves, empty_leaves;
    int max_depth;
    int points;
    double min_leaf_size;
    size_t tree_bytes;          /* node structures (cells) only              */
    size_t point_bytes;         /* raw LiDAR returns held in leaf buckets    */
    double grid_cells;          /* fixed grid needed for the same finest resolution */
    double grid_bytes;
} TreeStats;

/* Point query: height + occupancy at (x, y). O(depth). Returns 0 if outside. */
int qt_cell_at(const QuadTree *t, double x, double y, CellInfo *out);

/* Orthogonal range query over [x1,x2] x [y1,y2]; visit may be NULL. */
RangeResult qt_range_query(const QuadTree *t, double x1, double y1,
                           double x2, double y2, PointVisitor visit, void *ctx);

/* Collision check: is there ANY obstacle return in the rectangle?
 * Uses subtree max_z to prune obstacle-free branches. */
int qt_region_blocked(const QuadTree *t, double x1, double y1,
                      double x2, double y2, int *nodes_visited);

/* Nearest obstacle return to (x, y). Returns 0 if the map has none. */
int qt_nearest_obstacle(const QuadTree *t, double x, double y,
                        LidarPoint *out, double *dist, int *nodes_visited);

void qt_compute_stats(const QuadTree *t, TreeStats *s);

#endif
