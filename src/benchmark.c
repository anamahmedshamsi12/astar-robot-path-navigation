/*
 * benchmark.c
 *
 * Empirical performance comparison between A* and Dijkstra.
 * Measures nodes expanded and average runtime (microseconds) for each
 * algorithm across six grid sizes and prints the results as CSV.
 * Output is saved in outputs/benchmark_results.csv.
 *
 * To run: make bench
 */

#include "astar.h"
#include <stdio.h>
#include <time.h>

/*
 * build_corridor_grid() creates a grid with two vertical walls and gaps
 * that scale proportionally with the grid size.
 *
 * Wall 1: column = size/3,   rows 1 to size-2, gap at row size-3
 * Wall 2: column = 2*size/3, rows 2 to size-2, gap at row 2
 *
 * We use a structured obstacle layout instead of an open grid because
 * on an open grid both algorithms expand a very similar band of cells
 * and the heuristic advantage of A* is barely visible. The two corridors
 * force Dijkstra to waste effort exploring cells in the wrong direction
 * while A*'s heuristic keeps it focused toward the goal. This makes the
 * difference in nodes_expanded between the two algorithms clearly measurable
 * and consistent across all six grid sizes.
 *
 * Integer division (size/3 and 2*size/3) scales the wall positions with
 * the grid size so the layout stays proportionally consistent whether the
 * grid is 10x10 or 60x60.
 */
static void build_corridor_grid(Grid* grid, int size) {
    int row;
    grid_init(grid, size, size);

    for (row = 1; row < size - 1; row++) {
        if (row != size - 3) {
            grid_set_cell(grid, (Point){row, size / 3}, 1);
        }
    }
    for (row = 2; row < size - 2; row++) {
        if (row != 2) {
            grid_set_cell(grid, (Point){row, (2 * size) / 3}, 1);
        }
    }
}

/*
 * elapsed_seconds() converts two clock_t tick counts into elapsed time
 * as a double-precision floating-point number in seconds.
 *
 * clock() returns CPU ticks. Dividing by CLOCKS_PER_SEC converts to seconds.
 * We cast to double before dividing to avoid integer division truncation —
 * if both operands were integers, any elapsed time shorter than one second
 * would round down to zero, making the measurement useless.
 */
static double elapsed_seconds(clock_t start, clock_t end) {
    return (double)(end - start) / (double)CLOCKS_PER_SEC;
}

/*
 * main() benchmarks A* and Dijkstra across 6 grid sizes and prints
 * one CSV row per size matching the header:
 *   grid_size, astar_nodes, dijkstra_nodes, astar_time_us, dijkstra_time_us
 *
 * For each grid size we:
 *   1. Build the corridor grid.
 *   2. Run each algorithm once (untimed) to capture node counts.
 *   3. Run each algorithm 2000 times inside a clock() bracket.
 *   4. Compute average microseconds per run and print the CSV row.
 *
 * We use 2000 repetitions because a single search on a small grid runs in
 * roughly 5-25 microseconds — well below the ~1ms resolution of many system
 * clocks. Averaging over 2000 runs gives a total measurement of 10-50ms,
 * which clock() can measure accurately. This reduces noise from OS scheduling
 * and gives stable, reproducible timing data.
 *
 * nodes_expanded is the primary metric because it directly counts algorithmic
 * work independent of hardware speed. Timing confirms the pattern on real
 * hardware and accounts for constant factors that Big-O analysis abstracts away.
 * Together they give a complete picture of the performance difference.
 */
int main(void) {
    int sizes[] = {10, 20, 30, 40, 50, 60};
    int count = (int)(sizeof(sizes) / sizeof(sizes[0]));
    int i;

    printf("grid_size,astar_nodes,dijkstra_nodes,astar_time_us,dijkstra_time_us\n");

    for (i = 0; i < count; i++) {
        Grid grid;
        SearchResult a_result;
        SearchResult d_result;
        Point start = {0, 0};
        Point goal  = {sizes[i] - 1, sizes[i] - 1};
        int repeats = 2000;
        int r;
        clock_t a_begin, a_end;
        clock_t d_begin, d_end;
        double a_us, d_us;

        build_corridor_grid(&grid, sizes[i]);

        /* one untimed run to record node counts without timing overhead */
        astar_search(&grid, start, goal, &a_result);
        dijkstra_search(&grid, start, goal, &d_result);

        /* time A*: record start tick, run 2000 times, record end tick */
        a_begin = clock();
        for (r = 0; r < repeats; r++) {
            astar_search(&grid, start, goal, &a_result);
        }
        a_end = clock();

        /* time Dijkstra: same structure for a fair side-by-side comparison */
        d_begin = clock();
        for (r = 0; r < repeats; r++) {
            dijkstra_search(&grid, start, goal, &d_result);
        }
        d_end = clock();

        /*
         * Convert total ticks to average microseconds per run:
         *   total_seconds = (end - start) / CLOCKS_PER_SEC
         *   total_us      = total_seconds * 1,000,000
         *   average_us    = total_us / repeats
         *
         * Reporting in microseconds gives more readable numbers than
         * seconds for operations this fast, and makes it easier to compare
         * A* and Dijkstra side by side in the paper's empirical table.
         */
        a_us = elapsed_seconds(a_begin, a_end) * 1000000.0 / repeats;
        d_us = elapsed_seconds(d_begin, d_end) * 1000000.0 / repeats;

        printf("%d,%d,%d,%.3f,%.3f\n",
               sizes[i],
               a_result.nodes_expanded,
               d_result.nodes_expanded,
               a_us,
               d_us);
    }

    return 0;
}