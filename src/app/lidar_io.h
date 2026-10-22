/*
 * lidar_io.h  --  LiDAR input, synthetic scenes, visualisation, benchmark
 */
#ifndef LIDAR_IO_H
#define LIDAR_IO_H

#include <stdio.h>
#include "quadtree.h"

/* Load "x y z [id]" lines ('#' = comment). Returns points inserted, or -1 if
 * the file cannot be opened. *skipped counts malformed / out-of-map lines. */
int lidar_load_file(QuadTree *t, const char *path, int *skipped);

/* Deterministic synthetic street scene (same seed -> same scan on any OS):
 * flat ground, a building, a pole, a ramp, a car (id 1) and a pedestrian (id 2). */
int lidar_generate_scene(QuadTree *t, unsigned seed, int n);

/* ASCII 2.5D height map, sampled at cols x rows. */
void lidar_render_height(const QuadTree *t, int cols, int rows, FILE *out);

/* ASCII map of leaf depth -> shows where resolution adapted. */
void lidar_render_depth(const QuadTree *t, int cols, int rows, FILE *out);

/* Write every leaf cell as CSV. Returns leaves written, or -1 on error. */
int lidar_export_leaves(const QuadTree *t, const char *path);

/* Build trees of growing size from random scans and time insert / query. */
void lidar_benchmark(FILE *out);

#endif
