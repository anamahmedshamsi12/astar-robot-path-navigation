/*
 * main.c
 *
 * Demo program for A* robot navigation with dynamic weight coefficient.
 * Shows A* and Dijkstra finding a path on an 8x8 grid, then demonstrates
 * rerouting when a new obstacle appears mid-path.
 *
 * The A* implementation here uses the dynamic weight coefficient from
 * Chatzisavvas et al. [1]: f(n) = g(n) + k * h(n) where k adapts
 * based on the estimated remaining cost to the goal. The rerouting
 * behavior is inspired by Hu et al. [2], which specifically studies
 * how robots should replan when unexpected obstacles appear on route.
 *
 * Output is saved to outputs/sample_run.txt as evidence of correct execution.
 * To compile and run: make run
 */

#include "astar.h"
#include <stdio.h>

/*
 * load_demo_grid() builds the 8x8 obstacle layout for the demo.
 * The walls simulate an indoor environment a robot would navigate,
 * consistent with the grid-based occupancy map representation used
 * in all three source papers.
 *
 * Layout (S=start, G=goal, #=wall, .=open):
 *   S . . . . . . .
 *   . . # . . # . .
 *   . . # . . # . .
 *   . . # . . # . .
 *   . . # # # # . .
 *   . . . . . . . .
 *   . # # # . . . .
 *   . . . . . . . G
 *
 * walls[] is a stack-allocated array of Point structs. We use
 * sizeof(walls)/sizeof(walls[0]) to count elements — the standard
 * C idiom for stack arrays without a separate length variable.
 */
static void load_demo_grid(Grid* grid) {
    Point walls[] = {
        {1, 2}, {2, 2}, {3, 2}, {4, 2},
        {4, 3}, {4, 4}, {4, 5},
        {1, 5}, {2, 5}, {3, 5},
        {6, 1}, {6, 2}, {6, 3}
    };
    int wall_count = (int)(sizeof(walls) / sizeof(walls[0]));
    int i;

    grid_init(grid, 8, 8);
    for (i = 0; i < wall_count; i++) {
        grid_set_cell(grid, walls[i], 1);
    }
}

/*
 * point_in_path() performs a linear search through the path array.
 * Returns 1 on the first match, 0 if the point is not in the path.
 * Linear search is appropriate here because path lengths are small.
 */
static int point_in_path(const SearchResult* result, Point p) {
    int i;
    for (i = 0; i < result->path_length; i++) {
        if (point_equal(result->path[i], p)) { return 1; }
    }
    return 0;
}

/*
 * print_grid() prints a visual map of the grid with an optional path overlay.
 * Symbols: S=start, G=goal, #=obstacle, *=path cell, .=open cell.
 *
 * The nested loop iterates row by row then column by column, matching
 * the row-major layout of the flat cells[] array in Grid. A local
 * Point struct is built per cell to use our helper functions without
 * tracking two separate integers.
 *
 * Passing result=NULL shows the map before any search has run.
 */
static void print_grid(const Grid* grid, const SearchResult* result, Point start, Point goal) {
    int row;
    int col;

    for (row = 0; row < grid->rows; row++) {
        for (col = 0; col < grid->cols; col++) {
            Point p = {row, col};
            char symbol = '.';

            if      (point_equal(p, start))                                       { symbol = 'S'; }
            else if (point_equal(p, goal))                                        { symbol = 'G'; }
            else if (grid_is_blocked(grid, p))                                    { symbol = '#'; }
            else if (result != NULL && result->found && point_in_path(result, p)) { symbol = '*'; }

            printf("%c ", symbol);
        }
        printf("\n");
    }
}

/*
 * print_path_coordinates() prints the path as "(row,col) -> (row,col) -> ..."
 * so we can verify the sequence is a valid connected chain of adjacent cells.
 */
static void print_path_coordinates(const SearchResult* result) {
    int i;
    if (!result->found) {
        printf("No path exists.\n");
        return;
    }
    for (i = 0; i < result->path_length; i++) {
        printf("(%d,%d)", result->path[i].row, result->path[i].col);
        if (i < result->path_length - 1) { printf(" -> "); }
    }
    printf("\n");
}

/*
 * main() runs the demo in three parts:
 *
 * Part 1: Show the initial map before any search.
 *
 * Part 2: Run both A* (with dynamic weight from [1]) and Dijkstra on
 * the same grid. The nodes_expanded comparison directly shows the
 * benefit of the weighted heuristic — A* reaches the goal while
 * processing far fewer cells than Dijkstra's blind outward expansion.
 *
 * Part 3: Simulate rerouting as described in Hu et al. [2]. The robot
 * has already moved partway along its planned route when a new obstacle
 * appears ahead. A* replans from the robot's current position, finding
 * a new optimal route around the new obstacle.
 */
int main(void) {
    Grid grid;
    SearchResult astar_result;
    SearchResult dijkstra_result;
    SearchResult reroute_result;
    Point start = {0, 0};
    Point goal  = {7, 7};
    Point current_position;
    Point new_block;

    load_demo_grid(&grid);

    printf("A* Robot Navigation Demo\n");
    printf("Dynamic Weight Coefficient: k=3.0 (far), k=0.85 (near), threshold=18\n");
    printf("Based on Chatzisavvas et al. [1], Hu et al. [2], Mai Jialing et al. [3]\n");
    printf("========================================================================\n\n");
    printf("Initial map:\n");
    print_grid(&grid, NULL, start, goal);
    printf("\n");

    /* run both algorithms on the same grid for direct comparison */
    astar_search(&grid, start, goal, &astar_result);
    dijkstra_search(&grid, start, goal, &dijkstra_result);

    print_result_summary("A* result (dynamic weight)", &astar_result);
    print_result_summary("Dijkstra result (no heuristic)", &dijkstra_result);
    printf("\nA* path:\n");
    print_path_coordinates(&astar_result);
    printf("\nGrid with A* path:\n");
    print_grid(&grid, &astar_result, start, goal);

    printf("\nRerouting example (inspired by Hu et al. [2])\n");
    printf("---------------------------------------------\n");

    /*
     * Simulate the robot having moved 3 steps along the initial path.
     * path[3] is the 4th cell (index 0 = start).
     * We block path[4] to simulate a new unexpected obstacle.
     * A* reruns from path[3] and finds a new route avoiding it.
     * This mirrors the rerouting experiment in Hu et al. [2] where
     * the robot encounters choke points mid-journey and must replan.
     */
    current_position = astar_result.path[3];
    new_block        = astar_result.path[4];
    grid_set_cell(&grid, new_block, 1);

    printf("Robot moved to (%d,%d).\n", current_position.row, current_position.col);
    printf("New obstacle appeared at (%d,%d).\n\n", new_block.row, new_block.col);

    astar_search(&grid, current_position, goal, &reroute_result);
    print_result_summary("Rerouted A* result", &reroute_result);
    printf("\nRerouted path:\n");
    print_path_coordinates(&reroute_result);
    printf("\nGrid after rerouting:\n");
    print_grid(&grid, &reroute_result, current_position, goal);

    return 0;
}