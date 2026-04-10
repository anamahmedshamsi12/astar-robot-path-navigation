/*
 * main.c
 *
 * Demo program for A* Humanoid Robot Navigation.
 * Runs A* and Dijkstra on an 8x8 grid, then shows rerouting when a
 * new obstacle appears mid-path. Output is captured in outputs/sample_run.txt
 * as evidence the code compiles and runs correctly.
 *
 * To compile and run: make run
 */

#include "astar.h"
#include <stdio.h>

/*
 * load_demo_grid() builds the 8x8 obstacle layout for the demo.
 * The walls simulate an indoor room a humanoid robot would navigate:
 * a partial wall structure in the middle of the space.
 *
 * Printed layout (S=start, G=goal, #=wall, .=open):
 *   S . . . . . . .
 *   . . # . . # . .
 *   . . # . . # . .
 *   . . # . . # . .
 *   . . # # # # . .
 *   . . . . . . . .
 *   . # # # . . . .
 *   . . . . . . . G
 *
 * walls[] is a fixed-size array of Point structs on the stack. We use
 * sizeof(walls)/sizeof(walls[0]) to get the element count — the standard
 * C idiom for stack-allocated arrays where the length is not stored separately.
 * Obstacle cells are set to 1 via grid_set_cell(), which removes them as
 * valid nodes in the grid graph during neighbor expansion.
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
 * point_in_path() performs a linear search through the path array
 * to check whether a given Point appears anywhere in the result.
 * Returns 1 on the first match, 0 if the point is not in the path.
 *
 * This is a sequential search — O(n) where n = path_length. Path lengths
 * are small (at most MAX_CELLS), so linear search is appropriate here.
 * We use this in print_grid() to decide whether to draw a '*' symbol.
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
 * Each cell is one of five symbols:
 *   S = start    G = goal    # = obstacle    * = path cell    . = open
 *
 * The nested loop iterates row by row then column by column, matching the
 * row-major layout of the flat cells[] array in Grid. A local Point struct
 * is constructed for each cell so we can use our helper functions like
 * grid_is_blocked() and point_in_path() without tracking two separate integers.
 *
 * Passing result=NULL lets us call this function before the search to show
 * the initial map, and after to show the found path overlaid on the grid.
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
 * print_path_coordinates() prints the path as a sequence of (row,col)
 * coordinate pairs connected by " -> " arrows so we can verify the
 * path is a valid connected sequence of adjacent cells.
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
 * Part 1: Print the initial map before any search.
 *
 * Part 2: Run both A* and Dijkstra on the same grid. Print the result
 * summary for each so we can directly compare nodes_expanded. Both
 * algorithms find the same shortest path, but A*'s heuristic lets it
 * do so while expanding fewer cells than Dijkstra's blind outward search.
 *
 * Part 3: Simulate rerouting. We treat path[3] as the robot's current
 * position (it has already moved there) and block path[4] to simulate
 * a new obstacle appearing ahead. Running A* again from path[3] finds
 * a valid alternate route, demonstrating that A* can replan efficiently
 * when the environment changes.
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

    printf("A* Humanoid Navigation Demo\n");
    printf("===========================\n\n");
    printf("Initial map:\n");
    print_grid(&grid, NULL, start, goal);
    printf("\n");

    /* run both algorithms on the same grid for direct comparison */
    astar_search(&grid, start, goal, &astar_result);
    dijkstra_search(&grid, start, goal, &dijkstra_result);

    print_result_summary("A* result", &astar_result);
    print_result_summary("Dijkstra result", &dijkstra_result);
    printf("\nA* path:\n");
    print_path_coordinates(&astar_result);
    printf("\nGrid with A* path:\n");
    print_grid(&grid, &astar_result, start, goal);

    printf("\nRerouting example\n");
    printf("-----------------\n");

    /*
     * Simulate the robot having moved 3 steps along the initial path.
     * path[3] is the 4th cell in the path (index 0 = start).
     * We block path[4] — the very next step — to simulate a new obstacle.
     * grid_set_cell() marks it as 1, removing it from the graph.
     * Running A* again from path[3] finds a new route that avoids it.
     */
    current_position = astar_result.path[3];
    new_block        = astar_result.path[4];
    grid_set_cell(&grid, new_block, 1);

    printf("Robot moved to (%d,%d).\n", current_position.row, current_position.col);
    printf("A new obstacle appeared at (%d,%d).\n\n", new_block.row, new_block.col);

    astar_search(&grid, current_position, goal, &reroute_result);
    print_result_summary("Rerouted A* result", &reroute_result);
    printf("\nRerouted path:\n");
    print_path_coordinates(&reroute_result);
    printf("\nGrid after rerouting:\n");
    print_grid(&grid, &reroute_result, current_position, goal);

    return 0;
}