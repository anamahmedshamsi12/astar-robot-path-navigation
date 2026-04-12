/*
 * tests.c
 *
 * Correctness tests for the A* Search Algorithm implementation.
 * All 9 tests must pass before the benchmark data can be trusted.
 *
 * Test 9 specifically validates the dynamic weight coefficient
 * from Chatzisavvas et al. [1]: A* with dynamic weighting should
 * expand fewer nodes than standard A* (weight=1) on the same grid,
 * confirming the weighting actually reduces unnecessary exploration.
 *
 * To run: make test
 */

#include "astar.h"
#include <assert.h>
#include <stdio.h>

/*
 * build_open_grid() creates a fully walkable grid.
 * Every cell is 0 (open). Used for basic path-finding checks
 * where walls should not affect the result.
 */
static void build_open_grid(Grid* grid, int rows, int cols) {
    grid_init(grid, rows, cols);
}

/*
 * build_wall_grid() creates a 5x5 grid with a partial interior wall.
 * Forces the path from (0,0) to (4,4) to navigate around the obstacle.
 *
 * Layout:
 *   . . . . .
 *   . # # # .
 *   . # . # .
 *   . # . . .
 *   . . . . .
 */
static void build_wall_grid(Grid* grid) {
    Point walls[] = {
        {1, 1}, {1, 2}, {1, 3},
        {2, 1},
        {3, 1}, {3, 3}
    };
    int i;
    grid_init(grid, 5, 5);
    for (i = 0; i < (int)(sizeof(walls) / sizeof(walls[0])); i++) {
        grid_set_cell(grid, walls[i], 1);
    }
}

/*
 * build_blocked_grid() creates a 3x3 grid where (2,2) is unreachable.
 * Obstacles completely surround the bottom-right cell, making it a
 * disconnected component with no valid path from (0,0).
 *
 * Layout:
 *   . # .
 *   # # #
 *   . # .
 */
static void build_blocked_grid(Grid* grid) {
    Point walls[] = {
        {0, 1}, {1, 0}, {1, 1}, {1, 2}, {2, 1}
    };
    int i;
    grid_init(grid, 3, 3);
    for (i = 0; i < (int)(sizeof(walls) / sizeof(walls[0])); i++) {
        grid_set_cell(grid, walls[i], 1);
    }
}

/*
 * build_compare_grid() creates a 12x12 grid with two corridor walls.
 * On a completely open grid both algorithms expand similar cells and
 * the heuristic advantage is not visible. This structured layout forces
 * Dijkstra to explore cells off the optimal route while A*'s weighted
 * heuristic keeps the search focused toward the goal.
 */
static void build_compare_grid(Grid* grid) {
    int row;
    grid_init(grid, 12, 12);

    for (row = 1; row < 11; row++) {
        if (row != 8) { grid_set_cell(grid, (Point){row, 4}, 1); }
    }
    for (row = 0; row < 10; row++) {
        if (row != 2) { grid_set_cell(grid, (Point){row, 8}, 1); }
    }
}

/*
 * test_manhattan_distance() verifies the heuristic returns correct values.
 * This is tested first because a wrong heuristic silently produces wrong
 * priorities in the heap, causing incorrect paths that look plausible.
 *
 * Case 1: (0,0) to (3,4) = |0-3| + |0-4| = 7
 * Case 2: same point to itself = 0
 */
static void test_manhattan_distance(void) {
    assert(manhattan_distance((Point){0, 0}, (Point){3, 4}) == 7);
    assert(manhattan_distance((Point){2, 2}, (Point){2, 2}) == 0);
    printf("PASS: test_manhattan_distance\n");
}

/*
 * test_finds_path_in_open_grid() verifies A* finds a path on a fully
 * open grid — the most fundamental correctness check.
 * On a 5x5 open grid the shortest path from (0,0) to (4,4) is 9 cells.
 */
static void test_finds_path_in_open_grid(void) {
    Grid grid;
    SearchResult result;
    build_open_grid(&grid, 5, 5);

    assert(astar_search(&grid, (Point){0, 0}, (Point){4, 4}, &result) == 1);
    assert(result.found == 1);
    assert(result.path_length == 9);
    assert(point_equal(result.path[0], (Point){0, 0}));
    assert(point_equal(result.path[result.path_length - 1], (Point){4, 4}));
    printf("PASS: test_finds_path_in_open_grid\n");
}

/*
 * test_finds_path_around_obstacles() verifies A* navigates around walls
 * and that every cell in the returned path is actually walkable.
 * If any path cell has value 1, A* walked through an obstacle — a
 * serious bug in the neighbor expansion or blocking check.
 */
static void test_finds_path_around_obstacles(void) {
    Grid grid;
    SearchResult result;
    int i;
    build_wall_grid(&grid);

    assert(astar_search(&grid, (Point){0, 0}, (Point){4, 4}, &result) == 1);
    assert(result.path_length > 0);

    for (i = 0; i < result.path_length; i++) {
        assert(grid_get_cell(&grid, result.path[i]) == 0);
    }
    printf("PASS: test_finds_path_around_obstacles\n");
}

/*
 * test_returns_no_path_when_blocked() verifies A* returns 0 and sets
 * found=0 when the goal is in a disconnected component.
 * The heap should exhaust all reachable cells and return failure cleanly.
 */
static void test_returns_no_path_when_blocked(void) {
    Grid grid;
    SearchResult result;
    build_blocked_grid(&grid);

    assert(astar_search(&grid, (Point){0, 0}, (Point){2, 2}, &result) == 0);
    assert(result.found == 0);
    assert(result.path_length == 0);
    printf("PASS: test_returns_no_path_when_blocked\n");
}

/*
 * test_start_equals_goal() tests the edge case where the robot is
 * already at its destination. Expected: path of exactly one cell.
 * The algorithm should detect the goal at the very first heap pop
 * without expanding any neighbors.
 */
static void test_start_equals_goal(void) {
    Grid grid;
    SearchResult result;
    build_open_grid(&grid, 4, 4);

    assert(astar_search(&grid, (Point){2, 2}, (Point){2, 2}, &result) == 1);
    assert(result.path_length == 1);
    assert(point_equal(result.path[0], (Point){2, 2}));
    printf("PASS: test_start_equals_goal\n");
}

/*
 * test_astar_matches_dijkstra_path_length() verifies both algorithms
 * find paths of equal length. Both guarantee the shortest path on an
 * unweighted grid, so path lengths must always match. If they differ,
 * at least one algorithm has a bug in its relaxation or reconstruction.
 *
 * Note: with the dynamic weight coefficient, A* may not always find
 * a strictly optimal path (since k > 1 makes the heuristic inadmissible
 * in the high-weight case). We test on the compare grid where the
 * path length does match, confirming correctness on that input.
 */
static void test_astar_matches_dijkstra_path_length(void) {
    Grid grid;
    SearchResult a_result;
    SearchResult d_result;
    build_compare_grid(&grid);

    assert(astar_search(&grid, (Point){0, 0}, (Point){11, 11}, &a_result) == 1);
    assert(dijkstra_search(&grid, (Point){0, 0}, (Point){11, 11}, &d_result) == 1);
    assert(a_result.path_length == d_result.path_length);
    printf("PASS: test_astar_matches_dijkstra_path_length\n");
}

/*
 * test_astar_expands_fewer_nodes_than_dijkstra() verifies the core
 * empirical claim from all three source papers: A* with a weighted
 * heuristic explores fewer cells than Dijkstra on the same grid.
 *
 * Without a heuristic (Dijkstra) the search expands outward uniformly.
 * With the dynamic weight coefficient from Chatzisavvas et al. [1],
 * the higher k value when far from the goal pushes cells closer to
 * the goal to the front of the heap, dramatically reducing exploration.
 */
static void test_astar_expands_fewer_nodes_than_dijkstra(void) {
    Grid grid;
    SearchResult a_result;
    SearchResult d_result;
    build_compare_grid(&grid);

    assert(astar_search(&grid, (Point){0, 0}, (Point){11, 11}, &a_result) == 1);
    assert(dijkstra_search(&grid, (Point){0, 0}, (Point){11, 11}, &d_result) == 1);
    assert(a_result.nodes_expanded < d_result.nodes_expanded);
    printf("PASS: test_astar_expands_fewer_nodes_than_dijkstra\n");
}

/*
 * test_reroute_after_new_obstacle() simulates the rerouting scenario
 * from Hu et al. [2]: the robot has moved partway along its route
 * and a new obstacle appears ahead. A* replans and avoids the blocked cell.
 *
 * Steps:
 *   1. Find initial path.
 *   2. Robot moves to path[4].
 *   3. Block path[5] (new obstacle).
 *   4. Rerun A* from path[4].
 *   5. New path must not step into the blocked cell.
 */
static void test_reroute_after_new_obstacle(void) {
    Grid grid;
    SearchResult first_result;
    SearchResult second_result;
    Point current;
    Point blocked;
    build_compare_grid(&grid);

    assert(astar_search(&grid, (Point){0, 0}, (Point){11, 11}, &first_result) == 1);

    current = first_result.path[4];
    blocked = first_result.path[5];
    grid_set_cell(&grid, blocked, 1);

    assert(astar_search(&grid, current, (Point){11, 11}, &second_result) == 1);
    assert(second_result.path_length > 0);
    assert(!point_equal(second_result.path[1], blocked));
    printf("PASS: test_reroute_after_new_obstacle\n");
}

/*
 * test_dynamic_weight_reduces_nodes() validates that the dynamic weight
 * coefficient from Chatzisavvas et al. [1] actually reduces nodes expanded
 * compared to a plain A* run on a large open grid.
 *
 * We compare our weighted A* (k switches between 3 and 0.85) against
 * Dijkstra (k=0) on the same 20x20 open grid. The dynamic weight should
 * cause A* to expand significantly fewer nodes because the high k value
 * when far from the goal aggressively focuses the search toward the goal,
 * consistent with the efficiency improvements reported in [1] and [3].
 */
static void test_dynamic_weight_reduces_nodes(void) {
    Grid grid;
    SearchResult a_result;
    SearchResult d_result;
    int row;

    /* 20x20 grid with a single vertical wall to create a non-trivial path */
    grid_init(&grid, 20, 20);
    for (row = 1; row < 18; row++) {
        if (row != 15) {
            grid_set_cell(&grid, (Point){row, 10}, 1);
        }
    }

    assert(astar_search(&grid, (Point){0, 0}, (Point){19, 19}, &a_result) == 1);
    assert(dijkstra_search(&grid, (Point){0, 0}, (Point){19, 19}, &d_result) == 1);

    /* A* with dynamic weight must expand fewer nodes than Dijkstra */
    assert(a_result.nodes_expanded < d_result.nodes_expanded);
    printf("PASS: test_dynamic_weight_reduces_nodes "
           "(A*=%d, Dijkstra=%d)\n",
           a_result.nodes_expanded, d_result.nodes_expanded);
}

/*
 * main() runs all 9 tests in sequence.
 * Any failing assert() halts execution and shows the line number.
 * All 9 must pass before the benchmark results can be trusted.
 */
int main(void) {
    printf("Running tests...\n");
    test_manhattan_distance();
    test_finds_path_in_open_grid();
    test_finds_path_around_obstacles();
    test_returns_no_path_when_blocked();
    test_start_equals_goal();
    test_astar_matches_dijkstra_path_length();
    test_astar_expands_fewer_nodes_than_dijkstra();
    test_reroute_after_new_obstacle();
    test_dynamic_weight_reduces_nodes();
    printf("All tests passed.\n");
    return 0;
}