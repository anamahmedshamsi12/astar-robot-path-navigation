/*
 * tests.c
 *
 * Correctness tests for the A* Search Algorithm implementation.
 * All 8 tests must pass before the benchmark data in the paper
 * can be considered meaningful — a buggy algorithm would produce
 * wrong node counts and wrong paths that only look plausible.
 *
 * Each test uses assert() from assert.h. If the condition inside
 * assert() is false, the program immediately prints the file name
 * and line number where the failure occurred and stops execution.
 * If all asserts pass, the test prints "PASS".
 *
 * To run: make test
 */

#include "astar.h"
#include <assert.h>
#include <stdio.h>

/*
 * build_open_grid() creates a fully walkable grid of the given size.
 * Every cell is 0 (open). Used for tests where obstacles should not
 * affect the result, so we can check basic path-finding in isolation.
 */
static void build_open_grid(Grid* grid, int rows, int cols) {
    grid_init(grid, rows, cols);
}

/*
 * build_wall_grid() creates a 5x5 grid with a partial interior wall.
 * The wall forces the path from (0,0) to (4,4) to navigate around
 * the obstacle rather than passing straight through the middle.
 *
 * Layout:
 *   . . . . .
 *   . # # # .
 *   . # . # .
 *   . # . . .
 *   . . . . .
 *
 * Obstacles are set to 1, which removes them as valid nodes in the
 * grid graph. The path must route around the left or right side
 * of the wall to reach the goal.
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
 * build_blocked_grid() creates a 3x3 grid where the goal cell (2,2)
 * is completely unreachable. A ring of obstacles surrounds the
 * bottom-right corner, making it a disconnected component with no
 * valid edges connecting it to the rest of the graph.
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
 * build_compare_grid() creates a 12x12 grid with two vertical corridor
 * walls and gaps at specific rows. Used in tests comparing A* vs Dijkstra.
 *
 * On a completely open grid both algorithms explore nearly the same cells
 * and the heuristic advantage of A* is not visible. This structured layout
 * forces Dijkstra to waste time exploring cells off the correct route while
 * A*'s heuristic keeps it focused toward the goal. The difference in
 * nodes_expanded becomes clearly measurable.
 */
static void build_compare_grid(Grid* grid) {
    int row;
    grid_init(grid, 12, 12);

    /* first wall at column 4, gap at row 8 */
    for (row = 1; row < 11; row++) {
        if (row != 8) { grid_set_cell(grid, (Point){row, 4}, 1); }
    }

    /* second wall at column 8, gap at row 2 */
    for (row = 0; row < 10; row++) {
        if (row != 2) { grid_set_cell(grid, (Point){row, 8}, 1); }
    }
}

/*
 * test_manhattan_distance() verifies the heuristic function returns
 * correct values for two known inputs.
 *
 * We test this first because the heuristic is the foundation of A*.
 * A wrong heuristic silently produces wrong priorities in the binary
 * min-heap, leading to incorrect paths that look plausible. Confirming
 * the heuristic is correct before testing the full algorithm means we
 * can trust that any failures in later tests are not caused by bad h(n).
 *
 * Case 1: (0,0) to (3,4) = |0-3| + |0-4| = 3 + 4 = 7
 * Case 2: same point to itself = 0
 */
static void test_manhattan_distance(void) {
    assert(manhattan_distance((Point){0, 0}, (Point){3, 4}) == 7);
    assert(manhattan_distance((Point){2, 2}, (Point){2, 2}) == 0);
    printf("PASS: test_manhattan_distance\n");
}

/*
 * test_finds_path_in_open_grid() verifies A* finds a path when there
 * are no obstacles — the most fundamental correctness check.
 *
 * On a 5x5 open grid from (0,0) to (4,4), the shortest path is always
 * 9 cells: 4 moves right + 4 moves down passes through 9 cells total.
 * We verify the return value, the found flag, the path length, and that
 * the path starts and ends at the correct cells.
 *
 * An open grid is a fully connected graph where every cell has 4 edges
 * to its neighbors. If A* cannot find a path here, something fundamental
 * is broken in the algorithm — the heap, the relaxation step, or the
 * path reconstruction.
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
 *
 * We loop through every cell in the path and assert its grid value is 0.
 * If any cell has value 1 it means A* walked through an obstacle, which
 * would be a serious correctness bug in the neighbor expansion logic.
 * This test directly validates that grid_is_blocked() is being checked
 * during neighbor expansion and that blocked cells are never entered.
 */
static void test_finds_path_around_obstacles(void) {
    Grid grid;
    SearchResult result;
    int i;
    build_wall_grid(&grid);

    assert(astar_search(&grid, (Point){0, 0}, (Point){4, 4}, &result) == 1);
    assert(result.path_length > 0);

    /* verify no cell in the path is an obstacle */
    for (i = 0; i < result.path_length; i++) {
        assert(grid_get_cell(&grid, result.path[i]) == 0);
    }
    printf("PASS: test_finds_path_around_obstacles\n");
}

/*
 * test_returns_no_path_when_blocked() verifies A* returns 0 and sets
 * found=0 when the goal is in a disconnected component of the grid graph.
 *
 * Without this test we cannot confirm the algorithm terminates cleanly
 * instead of looping forever or returning garbage data when no valid
 * route exists. The heap should exhaust all reachable cells and then
 * return failure because the goal was never reached.
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
 * test_start_equals_goal() tests the edge case where the robot is already
 * at its destination. Expected result: a path of exactly one cell.
 *
 * Edge cases are easy to miss and often cause off-by-one errors. When
 * start == goal, the algorithm should detect the goal at the very first
 * heap pop and return immediately without expanding any neighbors.
 * The returned path should contain only the start/goal cell itself.
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
 * return paths of equal length on the same grid.
 *
 * Both A* and Dijkstra are guaranteed to find the shortest path on an
 * unweighted grid, so their path lengths must always match. If they
 * differ, at least one algorithm has a bug in its relaxation step or
 * path reconstruction. This cross-validation test uses two independent
 * algorithm implementations to confirm both produce optimal results.
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
 * empirical claim of the paper: A*'s heuristic causes it to explore
 * fewer cells than Dijkstra on the same grid.
 *
 * Without a heuristic (Dijkstra, h=0), the priority queue is ordered
 * purely by g — actual steps taken — and the search expands outward
 * like concentric rings from the start, exploring many cells that are
 * not on the way to the goal. With the Manhattan distance heuristic
 * (A*), cells closer to the goal get pushed to the front of the binary
 * min-heap, so the search focuses in the right direction and never wastes
 * time on cells far from the optimal route.
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
 * test_reroute_after_new_obstacle() simulates what happens when an obstacle
 * appears on the planned path after the robot has already started moving.
 *
 * Steps:
 *   1. Find the initial path from (0,0) to (11,11).
 *   2. Mark path[4] as the robot's current position.
 *   3. Block path[5] — a new obstacle appeared right ahead.
 *   4. Run A* again from path[4] to (11,11).
 *   5. Verify the new path avoids the newly blocked cell.
 *
 * Blocking a cell changes the grid graph by removing edges to that cell.
 * A* running again on the updated graph finds the new shortest path in
 * the modified environment. The assertion on path[1] confirms the robot's
 * very first step on the new route does not enter the blocked cell.
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
 * main() runs all 8 tests in sequence. Any failing assert() halts the
 * program immediately and shows which line failed. All 8 tests must pass
 * before the benchmark numbers in the paper can be trusted as meaningful.
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
    printf("All tests passed.\n");
    return 0;
}