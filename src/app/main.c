/*
 * main.c  --  Command-line driver for the adaptive 2.5D LiDAR map
 * Owner: Member 4 (Sreehari S, CB.SC.U4CSE25757)
 *
 * Usage:
 *   quadmap               interactive prompt
 *   quadmap script.txt    run commands from a file (each command is echoed)
 *
 * Type HELP for the command list. Commands are case-insensitive.
 */
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dynamic.h"
#include "lidar_io.h"
#include "query.h"
#include "quadtree.h"

#define MAX_TOKENS     8
#define RANGE_PRINT_MAX 10

static QuadTree g_map;

/* ---- parsing helpers ----------------------------------------------------- */

static int parse_double(const char *s, double *out)
{
    char *end;
    double v = strtod(s, &end);
    if (end == s || *end != '\0' || !isfinite(v)) {
        printf("Error: '%s' is not a valid number\n", s);
        return 0;
    }
    *out = v;
    return 1;
}

static int parse_int(const char *s, int *out)
{
    char *end;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0' || v < -1000000000L || v > 1000000000L) {
        printf("Error: '%s' is not a valid integer\n", s);
        return 0;
    }
    *out = (int)v;
    return 1;
}

/* Parse argv[from .. from+n-1] as doubles. */
static int parse_doubles(char **argv, int from, int n, double *out)
{
    for (int i = 0; i < n; i++)
        if (!parse_double(argv[from + i], &out[i])) return 0;
    return 1;
}

static int need_args(int argc, int lo, int hi, const char *usage)
{
    if (argc - 1 < lo || argc - 1 > hi) {
        printf("Usage: %s\n", usage);
        return 0;
    }
    return 1;
}

static void print_bounds(void)
{
    const QTNode *r = g_map.root;
    printf("Map: [%.2f, %.2f] x [%.2f, %.2f] m, ground z = %.2f m\n",
           r->x0, r->x0 + r->size, r->y0, r->y0 + r->size, g_map.ground_z);
}

/* ---- commands ------------------------------------------------------------ */

static void cmd_help(void)
{
    puts("Map setup");
    puts("  INIT x0 y0 size [ground_z]   new empty map (default: 0 0 64 0)");
    puts("  CLEAR                        remove all points, keep bounds");
    puts("Input");
    puts("  INSERT x y z [id]            add one LiDAR return (id>0 = moving object)");
    puts("  LOAD file                    load 'x y z [id]' lines from a file");
    puts("  GEN [seed] [n]               synthetic street scan (default 7 5000)");
    puts("Queries");
    puts("  HEIGHT x y                   height / occupancy of the cell at (x,y)");
    puts("  RANGE x1 y1 x2 y2            returns inside a rectangle");
    puts("  COLLIDE x1 y1 x2 y2          is the rectangle blocked by an obstacle?");
    puts("  NEAREST x y                  nearest obstacle return to (x,y)");
    puts("Dynamic updates");
    puts("  DELETE x y                   delete the return at exactly (x,y)");
    puts("  OBJECTS                      list moving objects");
    puts("  MOVE id dx dy                shift object id by (dx,dy)");
    puts("  REMOVEOBJ id                 delete object id");
    puts("  CLEARDYN                     delete all moving objects");
    puts("Output");
    puts("  STATS                        tree size vs. fixed grid");
    puts("  MAP [cols rows]              ASCII height map (default 64 32)");
    puts("  DEPTH [cols rows]            ASCII leaf-depth map (adaptive resolution)");
    puts("  EXPORT file                  write leaf cells to CSV");
    puts("  BENCH                        scaling benchmark (1k .. 1M points)");
    puts("  HELP | QUIT");
}

static void cmd_init(int argc, char **argv)
{
    if (!need_args(argc, 3, 4, "INIT x0 y0 size [ground_z]")) return;
    double v[4] = {0, 0, 0, 0};
    if (!parse_doubles(argv, 1, argc - 1, v)) return;
    if (v[2] <= 0) {
        puts("Error: size must be positive");
        return;
    }
    QuadTree fresh;
    if (!qt_init(&fresh, v[0], v[1], v[2], v[3])) {
        puts("Error: invalid map parameters");
        return;
    }
    qt_destroy(&g_map);
    g_map = fresh;
    print_bounds();
}

static void cmd_clear(void)
{
    const QTNode *r = g_map.root;
    QuadTree fresh;
    qt_init(&fresh, r->x0, r->y0, r->size, g_map.ground_z);
    qt_destroy(&g_map);
    g_map = fresh;
    puts("Map cleared");
}

static void cmd_insert(int argc, char **argv)
{
    if (!need_args(argc, 3, 4, "INSERT x y z [id]")) return;
    double v[3];
    int id = 0;
    if (!parse_doubles(argv, 1, 3, v)) return;
    if (argc == 5 && !parse_int(argv[4], &id)) return;
    if (id < 0) {
        puts("Error: id must be >= 0");
        return;
    }
    LidarPoint p = {v[0], v[1], v[2], id};
    if (qt_insert(&g_map, p))
        printf("Inserted (%.2f, %.2f, %.2f) id=%d\n", p.x, p.y, p.z, id);
    else {
        printf("Rejected: (%.2f, %.2f) is outside the map. ", p.x, p.y);
        print_bounds();
    }
}

static void cmd_load(int argc, char **argv)
{
    if (!need_args(argc, 1, 1, "LOAD file")) return;
    int skipped = 0;
    int n = lidar_load_file(&g_map, argv[1], &skipped);
    if (n < 0) printf("Error: cannot open '%s'\n", argv[1]);
    else printf("Loaded %d points (%d lines skipped)\n", n, skipped);
}

static void cmd_gen(int argc, char **argv)
{
    if (!need_args(argc, 0, 2, "GEN [seed] [n]")) return;
    int seed = 7, n = 5000;
    if (argc >= 2 && !parse_int(argv[1], &seed)) return;
    if (argc >= 3 && !parse_int(argv[2], &n)) return;
    if (n <= 0 || n > 5000000) {
        puts("Error: n must be in 1 .. 5000000");
        return;
    }
    int got = lidar_generate_scene(&g_map, (unsigned)seed, n);
    printf("Generated %d points (seed %d): building, pole, ramp, car id=1, pedestrian id=2\n",
           got, seed);
}

static void cmd_height(int argc, char **argv)
{
    if (!need_args(argc, 2, 2, "HEIGHT x y")) return;
    double v[2];
    if (!parse_doubles(argv, 1, 2, v)) return;
    CellInfo c;
    if (!qt_cell_at(&g_map, v[0], v[1], &c)) {
        printf("(%.2f, %.2f) is outside the map\n", v[0], v[1]);
        return;
    }
    printf("Cell [%.3f, %.3f] size %.3f m, depth %d, %d returns\n",
           c.x0, c.y0, c.size, c.depth, c.count);
    if (!c.known)
        puts("  No data (unknown cell)");
    else
        printf("  height min %.2f  max %.2f  mean %.2f  -> %s\n", c.min_z, c.max_z,
               c.mean_z, c.occupied ? "OCCUPIED" : "free");
    printf("  nodes visited: %d\n", c.nodes_visited);
}

static void print_point(const LidarPoint *p, void *ctx)
{
    int *shown = ctx;
    if (++*shown <= RANGE_PRINT_MAX)
        printf("  (%.2f, %.2f, %.2f) id=%d\n", p->x, p->y, p->z, p->id);
}

static void cmd_range(int argc, char **argv)
{
    if (!need_args(argc, 4, 4, "RANGE x1 y1 x2 y2")) return;
    double v[4];
    if (!parse_doubles(argv, 1, 4, v)) return;
    int shown = 0;
    RangeResult r = qt_range_query(&g_map, v[0], v[1], v[2], v[3], print_point, &shown);
    if (r.count > RANGE_PRINT_MAX) printf("  ... %d more\n", r.count - RANGE_PRINT_MAX);
    printf("%d returns in range, %d obstacle returns", r.count, r.obstacle_count);
    if (r.count) printf(", max height %.2f", r.max_z);
    printf("\n  nodes visited: %d\n", r.nodes_visited);
}

static void cmd_collide(int argc, char **argv)
{
    if (!need_args(argc, 4, 4, "COLLIDE x1 y1 x2 y2")) return;
    double v[4];
    if (!parse_doubles(argv, 1, 4, v)) return;
    int visited;
    int b = qt_region_blocked(&g_map, v[0], v[1], v[2], v[3], &visited);
    printf("%s\n  nodes visited: %d\n", b ? "BLOCKED - obstacle in region" : "CLEAR - region is free",
           visited);
}

static void cmd_nearest(int argc, char **argv)
{
    if (!need_args(argc, 2, 2, "NEAREST x y")) return;
    double v[2];
    if (!parse_doubles(argv, 1, 2, v)) return;
    LidarPoint p;
    double d;
    int visited;
    if (qt_nearest_obstacle(&g_map, v[0], v[1], &p, &d, &visited))
        printf("Nearest obstacle (%.2f, %.2f, %.2f) id=%d at distance %.3f m\n",
               p.x, p.y, p.z, p.id, d);
    else
        puts("No obstacles in the map");
    printf("  nodes visited: %d\n", visited);
}

static void cmd_delete(int argc, char **argv)
{
    if (!need_args(argc, 2, 2, "DELETE x y")) return;
    double v[2];
    if (!parse_doubles(argv, 1, 2, v)) return;
    LidarPoint p;
    if (qt_delete_point(&g_map, v[0], v[1], &p))
        printf("Deleted (%.2f, %.2f, %.2f) id=%d\n", p.x, p.y, p.z, p.id);
    else
        printf("No return at (%.2f, %.2f)\n", v[0], v[1]);
}

static void cmd_objects(void)
{
    ObjectInfo obj[64];
    int n = qt_list_objects(&g_map, obj, 64);
    if (n == 0) {
        puts("No moving objects");
        return;
    }
    printf("%4s %7s %18s %8s\n", "id", "points", "centroid", "max z");
    for (int i = 0; i < n && i < 64; i++)
        printf("%4d %7d   (%6.2f, %6.2f) %8.2f\n", obj[i].id, obj[i].count, obj[i].cx,
               obj[i].cy, obj[i].max_z);
    if (n > 64) printf("... %d more\n", n - 64);
}

static void cmd_move(int argc, char **argv)
{
    if (!need_args(argc, 3, 3, "MOVE id dx dy")) return;
    int id;
    double d[2];
    if (!parse_int(argv[1], &id) || !parse_doubles(argv, 2, 2, d)) return;
    if (id <= 0) {
        puts("Error: only moving objects (id > 0) can be moved");
        return;
    }
    MoveResult r = qt_move_object(&g_map, id, d[0], d[1]);
    if (r.moved + r.dropped == 0)
        printf("No object with id %d\n", id);
    else
        printf("Moved object %d by (%.2f, %.2f): %d returns moved, %d left the map\n", id,
               d[0], d[1], r.moved, r.dropped);
}

static void cmd_removeobj(int argc, char **argv)
{
    if (!need_args(argc, 1, 1, "REMOVEOBJ id")) return;
    int id;
    if (!parse_int(argv[1], &id)) return;
    if (id <= 0) {
        puts("Error: id must be > 0 (id 0 is the static scene)");
        return;
    }
    int n = qt_remove_object(&g_map, id);
    if (n) printf("Removed object %d (%d returns)\n", id, n);
    else printf("No object with id %d\n", id);
}

static void cmd_stats(void)
{
    TreeStats s;
    qt_compute_stats(&g_map, &s);
    print_bounds();
    printf("  points          : %d\n", s.points);
    printf("  nodes / leaves  : %lu / %lu (%lu empty)\n", (unsigned long)s.nodes,
           (unsigned long)s.leaves, (unsigned long)s.empty_leaves);
    printf("  max depth       : %d (finest cell %.3f m)\n", s.max_depth, s.min_leaf_size);
    printf("  quadtree cells  : %.1f KB (+ %.1f KB raw returns)\n", s.tree_bytes / 1024.0,
           s.point_bytes / 1024.0);
    printf("  fixed grid at finest resolution: %.0f cells, %.1f KB\n", s.grid_cells,
           s.grid_bytes / 1024.0);
    printf("  leaf cells vs grid cells: %.3f%%\n", 100.0 * s.leaves / s.grid_cells);
}

static int read_dims(int argc, char **argv, int *cols, int *rows, const char *usage)
{
    *cols = 64;
    *rows = 32;
    if (argc != 1 && argc != 3) {
        printf("Usage: %s\n", usage);
        return 0;
    }
    if (argc == 3 && (!parse_int(argv[1], cols) || !parse_int(argv[2], rows))) return 0;
    if (*cols < 1 || *cols > 200 || *rows < 1 || *rows > 200) {
        puts("Error: cols and rows must be in 1 .. 200");
        return 0;
    }
    return 1;
}

static void cmd_export(int argc, char **argv)
{
    if (!need_args(argc, 1, 1, "EXPORT file")) return;
    int n = lidar_export_leaves(&g_map, argv[1]);
    if (n < 0) printf("Error: cannot write '%s'\n", argv[1]);
    else printf("Exported %d leaf cells to %s\n", n, argv[1]);
}

/* Returns 0 when the user asked to quit. */
static int execute(char *line)
{
    char *argv[MAX_TOKENS + 1];
    int argc = 0;
    for (char *tok = strtok(line, " \t\r\n"); tok; tok = strtok(NULL, " \t\r\n")) {
        if (tok[0] == '#') break;                       /* comment */
        if (argc == MAX_TOKENS) {
            puts("Error: too many arguments");
            return 1;
        }
        argv[argc++] = tok;
    }
    if (argc == 0) return 1;
    for (char *c = argv[0]; *c; c++) *c = (char)toupper((unsigned char)*c);

    const char *cmd = argv[0];
    int cols, rows;
    if (!strcmp(cmd, "QUIT") || !strcmp(cmd, "EXIT")) return 0;
    else if (!strcmp(cmd, "HELP")) cmd_help();
    else if (!strcmp(cmd, "INIT")) cmd_init(argc, argv);
    else if (!strcmp(cmd, "CLEAR")) cmd_clear();
    else if (!strcmp(cmd, "INSERT")) cmd_insert(argc, argv);
    else if (!strcmp(cmd, "LOAD")) cmd_load(argc, argv);
    else if (!strcmp(cmd, "GEN")) cmd_gen(argc, argv);
    else if (!strcmp(cmd, "HEIGHT")) cmd_height(argc, argv);
    else if (!strcmp(cmd, "RANGE")) cmd_range(argc, argv);
    else if (!strcmp(cmd, "COLLIDE")) cmd_collide(argc, argv);
    else if (!strcmp(cmd, "NEAREST")) cmd_nearest(argc, argv);
    else if (!strcmp(cmd, "DELETE")) cmd_delete(argc, argv);
    else if (!strcmp(cmd, "OBJECTS")) cmd_objects();
    else if (!strcmp(cmd, "MOVE")) cmd_move(argc, argv);
    else if (!strcmp(cmd, "REMOVEOBJ")) cmd_removeobj(argc, argv);
    else if (!strcmp(cmd, "CLEARDYN")) {
        printf("Removed %d dynamic returns\n", qt_remove_object(&g_map, QT_ALL_DYNAMIC));
    }
    else if (!strcmp(cmd, "STATS")) cmd_stats();
    else if (!strcmp(cmd, "MAP")) {
        if (read_dims(argc, argv, &cols, &rows, "MAP [cols rows]"))
            lidar_render_height(&g_map, cols, rows, stdout);
    }
    else if (!strcmp(cmd, "DEPTH")) {
        if (read_dims(argc, argv, &cols, &rows, "DEPTH [cols rows]"))
            lidar_render_depth(&g_map, cols, rows, stdout);
    }
    else if (!strcmp(cmd, "EXPORT")) cmd_export(argc, argv);
    else if (!strcmp(cmd, "BENCH")) lidar_benchmark(stdout);
    else printf("Unknown command '%s' (type HELP)\n", argv[0]);
    return 1;
}

int main(int argc, char **argv)
{
    FILE *in = stdin;
    int script = 0;
    if (argc > 1) {
        in = fopen(argv[1], "r");
        if (!in) {
            fprintf(stderr, "Error: cannot open script '%s'\n", argv[1]);
            return 1;
        }
        script = 1;
    }

    qt_init(&g_map, 0, 0, 64, 0);
    if (!script) {
        puts("Adaptive-Resolution 2.5D LiDAR Map (Quadtree) - Team 4, CSE H");
        print_bounds();
        puts("Type HELP for commands.");
    }

    char line[1024];
    for (;;) {
        if (!script) {
            printf("quadmap> ");
            fflush(stdout);
        }
        if (!fgets(line, sizeof line, in)) break;
        if (!strchr(line, '\n') && !feof(in)) {          /* over-long line */
            int ch;
            while ((ch = fgetc(in)) != '\n' && ch != EOF) {}
            puts("Error: line too long (max 1023 characters)");
            continue;
        }
        /* Drop a UTF-8 byte-order mark (added by Notepad / PowerShell pipes). */
        if ((unsigned char)line[0] == 0xEF && (unsigned char)line[1] == 0xBB &&
            (unsigned char)line[2] == 0xBF)
            memmove(line, line + 3, strlen(line + 3) + 1);
        if (script) {
            line[strcspn(line, "\r\n")] = '\0';
            if (line[0] == '\0') continue;
            printf("> %s\n", line);
        }
        if (!execute(line)) break;
        fflush(stdout);
    }

    if (script) fclose(in);
    qt_destroy(&g_map);
    return 0;
}
