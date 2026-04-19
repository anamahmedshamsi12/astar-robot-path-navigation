/*
 * benchmark.c
 *
 * Runs three empirical experiments comparing A* with dynamic weight
 * coefficient against Dijkstra's algorithm:
 *
 *   Experiment 1: Grid size scaling (10x10 to 60x60 in steps of 5)
 *     Output: outputs/benchmark_grid_size.csv
 *
 *   Experiment 2: Obstacle density scaling (0% to 40% on a 30x30 grid)
 *     Output: outputs/benchmark_obstacle_density.csv
 *
 *   Experiment 3: Initial planning vs replanning timing
 *     Inspired by the rerouting experiments in Hu et al. [2].
 *     Output: outputs/benchmark_replan.csv
 *
 * To run: make bench
 */

#include "astar.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static double elapsed_us(clock_t start, clock_t end) {
    return (double)(end - start) / (double)CLOCKS_PER_SEC * 1000000.0;
}

/*
 * build_corridor_grid() creates a structured grid with two vertical walls
 * and gaps that scale with size. Used for Experiments 1 and 3.
 * The corridor layout forces both algorithms to navigate through specific
 * openings, making the heuristic advantage clearly measurable.
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
 * build_random_grid() creates a 30x30 grid with randomly placed obstacles
 * at the given density. A fixed seed ensures results are reproducible.
 * Start (0,0) and goal (size-1, size-1) are always kept open.
 * Used for Experiment 2. This matches the experimental approach of
 * Chatzisavvas et al. [1] who test on grids with about 20% obstacle density.
 */
static void build_random_grid(Grid* grid, int size, double density, unsigned int seed) {
    int r, c;
    srand(seed);
    grid_init(grid, size, size);
    for (r = 0; r < size; r++) {
        for (c = 0; c < size; c++) {
            if ((r == 0 && c == 0) || (r == size-1 && c == size-1)) {
                continue;
            }
            if ((double)rand() / RAND_MAX < density) {
                grid_set_cell(grid, (Point){r, c}, 1);
            }
        }
    }
}

int main(void) {
    int r, i;
    int repeats = 2000;

    /* Experiment 1: Grid Size Scaling */
    {
        int sizes[] = {10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60};
        int num_sizes = (int)(sizeof(sizes) / sizeof(sizes[0]));
        FILE* f = fopen("outputs/benchmark_grid_size.csv", "w");
        if (!f) { printf("ERROR: cannot open benchmark_grid_size.csv\n"); return 1; }
        fprintf(f, "grid_size,astar_nodes,dijkstra_nodes,astar_time_us,dijkstra_time_us\n");

        for (i = 0; i < num_sizes; i++) {
            Grid grid;
            SearchResult a_result, d_result;
            Point start = {0, 0};
            Point goal  = {sizes[i]-1, sizes[i]-1};
            clock_t a_begin, a_end, d_begin, d_end;

            build_corridor_grid(&grid, sizes[i]);
            astar_search(&grid, start, goal, &a_result);
            dijkstra_search(&grid, start, goal, &d_result);

            a_begin = clock();
            for (r = 0; r < repeats; r++) astar_search(&grid, start, goal, &a_result);
            a_end = clock();

            d_begin = clock();
            for (r = 0; r < repeats; r++) dijkstra_search(&grid, start, goal, &d_result);
            d_end = clock();

            fprintf(f, "%d,%d,%d,%.3f,%.3f\n",
                    sizes[i],
                    a_result.nodes_expanded,
                    d_result.nodes_expanded,
                    elapsed_us(a_begin, a_end) / repeats,
                    elapsed_us(d_begin, d_end) / repeats);
        }
        fclose(f);
        printf("Saved outputs/benchmark_grid_size.csv\n");
    }

    /* Experiment 2: Obstacle Density Scaling */
    {
        double densities[] = {0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40};
        int num_densities = (int)(sizeof(densities) / sizeof(densities[0]));
        int grid_size = 30;
        FILE* f = fopen("outputs/benchmark_obstacle_density.csv", "w");
        if (!f) { printf("ERROR: cannot open benchmark_obstacle_density.csv\n"); return 1; }
        fprintf(f, "obstacle_pct,astar_nodes,dijkstra_nodes,astar_time_us,dijkstra_time_us\n");

        for (i = 0; i < num_densities; i++) {
            Grid grid;
            SearchResult a_result, d_result;
            Point start = {0, 0};
            Point goal  = {grid_size-1, grid_size-1};
            clock_t a_begin, a_end, d_begin, d_end;

            build_random_grid(&grid, grid_size, densities[i], 42);

            if (!astar_search(&grid, start, goal, &a_result)) {
                fprintf(f, "%d,0,0,0,0\n", (int)(densities[i] * 100));
                continue;
            }
            dijkstra_search(&grid, start, goal, &d_result);

            a_begin = clock();
            for (r = 0; r < repeats; r++) astar_search(&grid, start, goal, &a_result);
            a_end = clock();

            d_begin = clock();
            for (r = 0; r < repeats; r++) dijkstra_search(&grid, start, goal, &d_result);
            d_end = clock();

            fprintf(f, "%d,%d,%d,%.3f,%.3f\n",
                    (int)(densities[i] * 100),
                    a_result.nodes_expanded,
                    d_result.nodes_expanded,
                    elapsed_us(a_begin, a_end) / repeats,
                    elapsed_us(d_begin, d_end) / repeats);
        }
        fclose(f);
        printf("Saved outputs/benchmark_obstacle_density.csv\n");
    }

    /* Experiment 3: Initial Planning vs Replanning */
    /*
     * Measures how long A* takes for an initial search versus replanning
     * after a new obstacle appears mid-route. Directly corresponds to the
     * rerouting experiments in Hu et al. [2] and demonstrates that A* is
     * fast enough for real-time dynamic replanning in robot navigation.
     */
    {
        int sizes[] = {20, 30, 40, 50};
        int num_sizes = (int)(sizeof(sizes) / sizeof(sizes[0]));
        FILE* f = fopen("outputs/benchmark_replan.csv", "w");
        if (!f) { printf("ERROR: cannot open benchmark_replan.csv\n"); return 1; }
        fprintf(f, "grid_size,initial_nodes,replan_nodes,initial_time_us,replan_time_us\n");

        for (i = 0; i < num_sizes; i++) {
            Grid grid;
            Grid grid_blocked;
            SearchResult initial_result, replan_result;
            Point start = {0, 0};
            Point goal  = {sizes[i]-1, sizes[i]-1};
            Point replan_start, blocked;
            clock_t t_begin, t_end;
            double initial_us, replan_us;
            int j;

            build_corridor_grid(&grid, sizes[i]);
            astar_search(&grid, start, goal, &initial_result);

            t_begin = clock();
            for (r = 0; r < repeats; r++) astar_search(&grid, start, goal, &initial_result);
            t_end = clock();
            initial_us = elapsed_us(t_begin, t_end) / repeats;

            /* Make a copy of the grid, add a new obstacle, and replan */
            grid_blocked = grid;
            if (initial_result.path_length >= 6) {
                replan_start = initial_result.path[3];
                blocked      = initial_result.path[4];
            } else {
                replan_start = start;
                blocked      = initial_result.path[1];
            }
            grid_set_cell(&grid_blocked, blocked, 1);

            astar_search(&grid_blocked, replan_start, goal, &replan_result);

            t_begin = clock();
            for (j = 0; j < repeats; j++) astar_search(&grid_blocked, replan_start, goal, &replan_result);
            t_end = clock();
            replan_us = elapsed_us(t_begin, t_end) / repeats;

            fprintf(f, "%d,%d,%d,%.3f,%.3f\n",
                    sizes[i],
                    initial_result.nodes_expanded,
                    replan_result.nodes_expanded,
                    initial_us,
                    replan_us);
        }
        fclose(f);
        printf("Saved outputs/benchmark_replan.csv\n");
    }

    printf("\nAll 3 benchmark experiments complete.\n");
    return 0;
}
