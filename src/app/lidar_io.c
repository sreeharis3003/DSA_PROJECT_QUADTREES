/*
 * lidar_io.c  --  LiDAR input, synthetic scenes, visualisation, benchmark
 */
#include "lidar_io.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "query.h"

/* ---- portable deterministic RNG (rand() differs between compilers) ------- */

static unsigned rng_state = 1;

static void rng_seed(unsigned s) { rng_state = s ? s : 1; }

static double rng01(void)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return (double)(rng_state >> 8) / 16777216.0;      /* [0, 1) */
}

static double rng_range(double lo, double hi) { return lo + (hi - lo) * rng01(); }

/* ---- file loading -------------------------------------------------------- */

int lidar_load_file(QuadTree *t, const char *path, int *skipped)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;

    char line[512];
    int inserted = 0, bad = 0;
    while (fgets(line, sizeof line, f)) {
        char *s = line;
        while (*s == ' ' || *s == '\t') s++;
        if (*s == '#' || *s == '\n' || *s == '\r' || *s == '\0') continue;

        LidarPoint p = {0, 0, 0, 0};
        int got = sscanf(s, "%lf %lf %lf %d", &p.x, &p.y, &p.z, &p.id);
        if (got < 3 || p.id < 0 || !qt_insert(t, p)) bad++;
        else inserted++;
    }
    fclose(f);
    if (skipped) *skipped = bad;
    return inserted;
}

/* ---- synthetic scene ----------------------------------------------------- */

typedef struct {
    double fx0, fy0, fx1, fy1;   /* footprint as fraction of the map */
    double h0, h1;               /* top height: h0 at west edge, h1 at east edge */
    int id;
    double share;                /* fraction of all points */
} SceneObject;

static const SceneObject SCENE[] = {
    {0.10, 0.65, 0.30, 0.90, 2.50, 2.50, 0, 0.14},   /* building           */
    {0.70, 0.70, 0.73, 0.73, 3.00, 3.00, 0, 0.03},   /* pole / tree        */
    {0.55, 0.45, 0.85, 0.55, 0.00, 0.60, 0, 0.06},   /* ramp (gradual)     */
    {0.40, 0.20, 0.55, 0.28, 1.50, 1.50, 1, 0.10},   /* car        (id 1)  */
    {0.78, 0.30, 0.80, 0.32, 1.70, 1.70, 2, 0.03},   /* pedestrian (id 2)  */
};
#define SCENE_COUNT ((int)(sizeof SCENE / sizeof SCENE[0]))

static int in_any_footprint(double fx, double fy)
{
    for (int i = 0; i < SCENE_COUNT; i++)
        if (fx >= SCENE[i].fx0 && fx <= SCENE[i].fx1 &&
            fy >= SCENE[i].fy0 && fy <= SCENE[i].fy1)
            return 1;
    return 0;
}

int lidar_generate_scene(QuadTree *t, unsigned seed, int n)
{
    if (!t || !t->root || n <= 0) return 0;
    rng_seed(seed);
    const QTNode *r = t->root;
    double g = t->ground_z;
    int inserted = 0, made = 0;

    for (int i = 0; i < SCENE_COUNT; i++) {
        const SceneObject *o = &SCENE[i];
        int k = (int)(n * o->share);
        for (int j = 0; j < k; j++, made++) {
            double fx = rng_range(o->fx0, o->fx1), fy = rng_range(o->fy0, o->fy1);
            double ramp = (fx - o->fx0) / (o->fx1 - o->fx0);
            LidarPoint p;
            p.x = r->x0 + fx * r->size;
            p.y = r->y0 + fy * r->size;
            p.z = g + o->h0 + (o->h1 - o->h0) * ramp + rng_range(-0.02, 0.02);
            p.id = o->id;
            inserted += qt_insert(t, p);
        }
    }
    for (; made < n; made++) {                      /* remaining = ground */
        double fx, fy;
        int tries = 0;
        do {
            fx = rng01();
            fy = rng01();
        } while (in_any_footprint(fx, fy) && ++tries < 20);
        LidarPoint p = {r->x0 + fx * r->size, r->y0 + fy * r->size,
                        g + rng_range(-0.03, 0.03), 0};
        inserted += qt_insert(t, p);
    }
    return inserted;
}

/* ---- visualisation ------------------------------------------------------- */

static char height_char(const QuadTree *t, const CellInfo *c)
{
    if (!c->known) return ' ';
    double h = c->max_z - t->ground_z;
    if (h <= QT_OBSTACLE_HEIGHT) return '.';
    if (h <= 1.0) return ':';
    if (h <= 2.0) return '+';
    return '#';
}

static void render(const QuadTree *t, int cols, int rows, FILE *out, int depth_mode)
{
    if (!t || !t->root) return;
    const QTNode *r = t->root;
    double cw = r->size / cols, ch = r->size / rows;

    fprintf(out, "   y=%-8.2f+", r->y0 + r->size);
    for (int c = 0; c < cols; c++) fputc('-', out);
    fputs("+\n", out);
    for (int row = 0; row < rows; row++) {
        double y = r->y0 + r->size - (row + 0.5) * ch;
        fputs("              |", out);
        for (int c = 0; c < cols; c++) {
            CellInfo ci;
            qt_cell_at(t, r->x0 + (c + 0.5) * cw, y, &ci);
            if (depth_mode)
                fputc("0123456789ABCDEF"[ci.depth & 15], out);
            else
                fputc(height_char(t, &ci), out);
        }
        fputs("|\n", out);
    }
    fprintf(out, "   y=%-8.2f+", r->y0);
    for (int c = 0; c < cols; c++) fputc('-', out);
    fprintf(out, "+\n              x=%.2f .. %.2f\n", r->x0, r->x0 + r->size);
}

void lidar_render_height(const QuadTree *t, int cols, int rows, FILE *out)
{
    render(t, cols, rows, out, 0);
    fprintf(out, "Legend: ' ' no data  '.' ground(<=%.2fm)  ':' <=1m  '+' <=2m  '#' >2m\n",
            QT_OBSTACLE_HEIGHT);
}

void lidar_render_depth(const QuadTree *t, int cols, int rows, FILE *out)
{
    render(t, cols, rows, out, 1);
    fputs("Legend: digit = leaf depth (higher = finer cell; cell side = map / 2^depth)\n", out);
}

/* ---- CSV export ---------------------------------------------------------- */

static int export_rec(const QTNode *n, double thr, FILE *f)
{
    if (!n->is_leaf) {
        int w = 0;
        for (int i = 0; i < 4; i++) w += export_rec(n->child[i], thr, f);
        return w;
    }
    if (n->count == 0)
        fprintf(f, "%.4f,%.4f,%.4f,%d,0,,,,0\n", n->x0, n->y0, n->size, n->depth);
    else
        fprintf(f, "%.4f,%.4f,%.4f,%d,%d,%.4f,%.4f,%.4f,%d\n", n->x0, n->y0, n->size,
                n->depth, n->count, n->min_z, n->max_z, n->sum_z / n->count,
                n->max_z > thr);
    return 1;
}

int lidar_export_leaves(const QuadTree *t, const char *path)
{
    if (!t || !t->root) return -1;
    FILE *f = fopen(path, "w");
    if (!f) return -1;
    fputs("x0,y0,size,depth,count,min_z,max_z,mean_z,occupied\n", f);
    int w = export_rec(t->root, qt_obstacle_threshold(t), f);
    fclose(f);
    return w;
}

/* ---- benchmark ----------------------------------------------------------- */

void lidar_benchmark(FILE *out)
{
    static const int sizes[] = {1000, 10000, 100000, 1000000};
    fprintf(out, "%10s %8s %6s %12s %14s %14s\n", "points", "leaves", "depth",
            "build(ms)", "us/insert", "avg nodes/qry");
    for (int s = 0; s < 4; s++) {
        int n = sizes[s];
        QuadTree t;
        qt_init(&t, 0, 0, 1024, 0);
        rng_seed(42);

        clock_t c0 = clock();
        for (int i = 0; i < n; i++) {
            LidarPoint p = {rng01() * 1024, rng01() * 1024, rng01() * 3.0, 0};
            qt_insert(&t, p);
        }
        double ms = 1000.0 * (double)(clock() - c0) / CLOCKS_PER_SEC;

        long visited = 0;
        const int Q = 10000;
        for (int q = 0; q < Q; q++) {
            CellInfo ci;
            qt_cell_at(&t, rng01() * 1024, rng01() * 1024, &ci);
            visited += ci.nodes_visited;
        }
        TreeStats st;
        qt_compute_stats(&t, &st);
        fprintf(out, "%10d %8lu %6d %12.1f %14.3f %14.2f\n", n, (unsigned long)st.leaves,
                st.max_depth, ms, 1000.0 * ms / n, (double)visited / Q);
        qt_destroy(&t);
    }
    fputs("Point-query cost grows by ~1 node per 4x more points -> O(log4 n).\n", out);
}
