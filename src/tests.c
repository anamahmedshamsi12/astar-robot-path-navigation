/*
 * tests.c
 *
 * Correctness and timing tests for A* with dynamic weight coefficient.
 * Prints live progress as each test runs, with pass/fail status and
 * timing information — similar to the speed comparison assignment format.
 *
 * Usage:
 *   ./tests_runner           — run all tests
 *   ./tests_runner -v        — verbose, show extra detail per test
 *   ./tests_runner -h        — print help
 *
 * To run: make test
 */

#include "astar.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── Global flags ─────────────────────────────────────────────────────────── */
static int VERBOSE    = 0;
static int tests_run  = 0;
static int tests_pass = 0;
static int tests_fail = 0;

/* ── Helpers ──────────────────────────────────────────────────────────────── */
static double elapsed_us(clock_t start, clock_t end) {
    return (double)(end - start) / (double)CLOCKS_PER_SEC * 1000000.0;
}

static void print_help(void) {
    printf("Usage: ./tests_runner [OPTIONS]\n\n");
    printf("Runs all correctness and timing tests for the A* implementation.\n\n");
    printf("Options:\n");
    printf("  -h, --help     Print this help message\n");
    printf("  -v, --verbose  Show extra detail for each test\n\n");
    printf("Tests:\n");
    printf("  1. test_manhattan_distance\n");
    printf("  2. test_finds_path_in_open_grid\n");
    printf("  3. test_finds_path_around_obstacles\n");
    printf("  4. test_returns_no_path_when_blocked\n");
    printf("  5. test_start_equals_goal\n");
    printf("  6. test_astar_matches_dijkstra_path_length\n");
    printf("  7. test_astar_expands_fewer_nodes_than_dijkstra\n");
    printf("  8. test_reroute_after_new_obstacle\n");
    printf("  9. test_dynamic_weight_reduces_nodes\n\n");
}

/* ── Test runner macro ────────────────────────────────────────────────────── */
/*
 * RUN_TEST times the test function, prints live pass/fail status,
 * and updates the global counters. This mirrors the live output style
 * from the speed comparison assignment.
 */
#define RUN_TEST(name, func) do {                                    \
    clock_t _start, _end;                                            \
    int _passed = 1;                                                 \
    printf("  [%2d] %-45s ", ++tests_run, #name);                   \
    fflush(stdout);                                                  \
    _start = clock();                                                \
    func(&_passed);                                                  \
    _end = clock();                                                  \
    if (_passed) {                                                   \
        printf("PASS   (%.1f us)\n", elapsed_us(_start, _end));     \
        tests_pass++;                                                \
    } else {                                                         \
        printf("FAIL\n");                                            \
        tests_fail++;                                                \
    }                                                                \
} while(0)

/* ── Grid builders ────────────────────────────────────────────────────────── */
static void build_open_grid(Grid* grid, int rows, int cols) {
    grid_init(grid, rows, cols);
}

static void build_wall_grid(Grid* grid) {
    Point walls[] = {
        {1,1},{1,2},{1,3},{2,1},{3,1},{3,3}
    };
    int i;
    grid_init(grid, 5, 5);
    for (i = 0; i < (int)(sizeof(walls)/sizeof(walls[0])); i++)
        grid_set_cell(grid, walls[i], 1);
}

static void build_blocked_grid(Grid* grid) {
    Point walls[] = {
        {0,1},{1,0},{1,1},{1,2},{2,1}
    };
    int i;
    grid_init(grid, 3, 3);
    for (i = 0; i < (int)(sizeof(walls)/sizeof(walls[0])); i++)
        grid_set_cell(grid, walls[i], 1);
}

static void build_compare_grid(Grid* grid) {
    int row;
    grid_init(grid, 12, 12);
    for (row = 1; row < 11; row++)
        if (row != 8) grid_set_cell(grid, (Point){row, 4}, 1);
    for (row = 0; row < 10; row++)
        if (row != 2) grid_set_cell(grid, (Point){row, 8}, 1);
}

/* ── Individual tests ─────────────────────────────────────────────────────── */
static void test_manhattan_distance(int* passed) {
    if (manhattan_distance((Point){0,0}, (Point){3,4}) != 7) { *passed=0; return; }
    if (manhattan_distance((Point){2,2}, (Point){2,2}) != 0) { *passed=0; return; }
    if (VERBOSE)
        printf("\n       (0,0)->(3,4)=7, (2,2)->(2,2)=0  ");
}

static void test_finds_path_in_open_grid(int* passed) {
    Grid grid;
    SearchResult result;
    build_open_grid(&grid, 5, 5);
    if (!astar_search(&grid, (Point){0,0}, (Point){4,4}, &result)) { *passed=0; return; }
    if (result.path_length != 9) { *passed=0; return; }
    if (!point_equal(result.path[0], (Point){0,0})) { *passed=0; return; }
    if (!point_equal(result.path[result.path_length-1], (Point){4,4})) { *passed=0; return; }
    if (VERBOSE)
        printf("\n       path_length=%d, start=(%d,%d), goal=(%d,%d)  ",
               result.path_length, result.path[0].row, result.path[0].col,
               result.path[result.path_length-1].row,
               result.path[result.path_length-1].col);
}

static void test_finds_path_around_obstacles(int* passed) {
    Grid grid;
    SearchResult result;
    int i;
    build_wall_grid(&grid);
    if (!astar_search(&grid, (Point){0,0}, (Point){4,4}, &result)) { *passed=0; return; }
    for (i = 0; i < result.path_length; i++) {
        if (grid_get_cell(&grid, result.path[i]) != 0) { *passed=0; return; }
    }
    if (VERBOSE)
        printf("\n       path_length=%d, all %d cells walkable  ",
               result.path_length, result.path_length);
}

static void test_returns_no_path_when_blocked(int* passed) {
    Grid grid;
    SearchResult result;
    build_blocked_grid(&grid);
    if (astar_search(&grid, (Point){0,0}, (Point){2,2}, &result)) { *passed=0; return; }
    if (result.found != 0 || result.path_length != 0) { *passed=0; return; }
    if (VERBOSE)
        printf("\n       found=%d, path_length=%d  ",
               result.found, result.path_length);
}

static void test_start_equals_goal(int* passed) {
    Grid grid;
    SearchResult result;
    build_open_grid(&grid, 4, 4);
    if (!astar_search(&grid, (Point){2,2}, (Point){2,2}, &result)) { *passed=0; return; }
    if (result.path_length != 1) { *passed=0; return; }
    if (!point_equal(result.path[0], (Point){2,2})) { *passed=0; return; }
    if (VERBOSE)
        printf("\n       path_length=%d (expected 1)  ", result.path_length);
}

static void test_astar_matches_dijkstra_path_length(int* passed) {
    Grid grid;
    SearchResult a_result, d_result;
    build_compare_grid(&grid);
    astar_search(&grid, (Point){0,0}, (Point){11,11}, &a_result);
    dijkstra_search(&grid, (Point){0,0}, (Point){11,11}, &d_result);
    if (a_result.path_length != d_result.path_length) { *passed=0; return; }
    if (VERBOSE)
        printf("\n       A*=%d cells, Dijkstra=%d cells  ",
               a_result.path_length, d_result.path_length);
}

static void test_astar_expands_fewer_nodes_than_dijkstra(int* passed) {
    Grid grid;
    SearchResult a_result, d_result;
    build_compare_grid(&grid);
    astar_search(&grid, (Point){0,0}, (Point){11,11}, &a_result);
    dijkstra_search(&grid, (Point){0,0}, (Point){11,11}, &d_result);
    if (a_result.nodes_expanded >= d_result.nodes_expanded) { *passed=0; return; }
    if (VERBOSE)
        printf("\n       A*=%d nodes, Dijkstra=%d nodes, reduction=%.1f%%  ",
               a_result.nodes_expanded, d_result.nodes_expanded,
               100.0*(d_result.nodes_expanded-a_result.nodes_expanded)
               /d_result.nodes_expanded);
}

static void test_reroute_after_new_obstacle(int* passed) {
    Grid grid;
    SearchResult first, second;
    Point current, blocked;
    build_compare_grid(&grid);
    astar_search(&grid, (Point){0,0}, (Point){11,11}, &first);
    current = first.path[4];
    blocked = first.path[5];
    grid_set_cell(&grid, blocked, 1);
    if (!astar_search(&grid, current, (Point){11,11}, &second)) { *passed=0; return; }
    if (point_equal(second.path[1], blocked)) { *passed=0; return; }
    if (VERBOSE)
        printf("\n       blocked=(%d,%d), new_path_length=%d  ",
               blocked.row, blocked.col, second.path_length);
}

static void test_dynamic_weight_reduces_nodes(int* passed) {
    Grid grid;
    SearchResult a_result, d_result;
    int row;
    grid_init(&grid, 20, 20);
    for (row = 1; row < 18; row++)
        if (row != 15) grid_set_cell(&grid, (Point){row, 10}, 1);
    astar_search(&grid, (Point){0,0}, (Point){19,19}, &a_result);
    dijkstra_search(&grid, (Point){0,0}, (Point){19,19}, &d_result);
    if (a_result.nodes_expanded >= d_result.nodes_expanded) { *passed=0; return; }
    if (VERBOSE)
        printf("\n       A*=%d nodes, Dijkstra=%d nodes, reduction=%.1f%%  ",
               a_result.nodes_expanded, d_result.nodes_expanded,
               100.0*(d_result.nodes_expanded-a_result.nodes_expanded)
               /d_result.nodes_expanded);
}

/* ── Timing benchmark (runs each search many times and reports avg) ────────── */
static void run_timing_summary(void) {
    int repeats = 5000;
    int r;
    Grid grid;
    SearchResult result;
    Point start = {0, 0};
    Point goal  = {19, 19};
    clock_t a_begin, a_end, d_begin, d_end;

    grid_init(&grid, 20, 20);
    int row;
    for (row = 1; row < 18; row++)
        if (row != 15) grid_set_cell(&grid, (Point){row, 10}, 1);

    printf("\n  Timing summary (20x20 grid, %d runs each):\n", repeats);
    printf("  %-30s ", "A* with dynamic weight:");
    fflush(stdout);
    a_begin = clock();
    for (r = 0; r < repeats; r++) astar_search(&grid, start, goal, &result);
    a_end = clock();
    printf("avg %.2f us/run\n", elapsed_us(a_begin, a_end) / repeats);

    printf("  %-30s ", "Dijkstra (no heuristic):");
    fflush(stdout);
    d_begin = clock();
    for (r = 0; r < repeats; r++) dijkstra_search(&grid, start, goal, &result);
    d_end = clock();
    printf("avg %.2f us/run\n", elapsed_us(d_begin, d_end) / repeats);

    printf("  Speedup: %.1fx faster\n",
           elapsed_us(d_begin, d_end) / elapsed_us(a_begin, a_end));
}

/* ── Argument parsing ─────────────────────────────────────────────────────── */
static void process_args(int argc, char** argv) {
    int i;
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0)
            VERBOSE = 1;
        else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_help();
            exit(0);
        }
    }
}

/* ── Main ─────────────────────────────────────────────────────────────────── */
int main(int argc, char** argv) {
    process_args(argc, argv);

    printf("A* Search Algorithm -- Test Suite\n");
    printf("Dynamic weight coefficient: k=%d (far), k=0.%d (near), threshold=%d\n",
           WEIGHT_HIGH, WEIGHT_LOW_NUM, EC_THRESHOLD);
    printf("Verbose mode: %s\n", VERBOSE ? "on (use -v to toggle)" : "off");
    printf("------------------------------------------------------------\n");
    printf("  Running %s tests...\n\n", "9");

    RUN_TEST(test_manhattan_distance,                  test_manhattan_distance);
    RUN_TEST(test_finds_path_in_open_grid,             test_finds_path_in_open_grid);
    RUN_TEST(test_finds_path_around_obstacles,         test_finds_path_around_obstacles);
    RUN_TEST(test_returns_no_path_when_blocked,        test_returns_no_path_when_blocked);
    RUN_TEST(test_start_equals_goal,                   test_start_equals_goal);
    RUN_TEST(test_astar_matches_dijkstra_path_length,  test_astar_matches_dijkstra_path_length);
    RUN_TEST(test_astar_expands_fewer_nodes,           test_astar_expands_fewer_nodes_than_dijkstra);
    RUN_TEST(test_reroute_after_new_obstacle,          test_reroute_after_new_obstacle);
    RUN_TEST(test_dynamic_weight_reduces_nodes,        test_dynamic_weight_reduces_nodes);

    printf("\n------------------------------------------------------------\n");
    printf("  Results: %d run, %d passed, %d failed\n",
           tests_run, tests_pass, tests_fail);

    if (tests_fail == 0) {
        printf("  All tests passed.\n");
    } else {
        printf("  SOME TESTS FAILED. Check output above.\n");
    }

    run_timing_summary();
    printf("------------------------------------------------------------\n");

    return tests_fail > 0 ? 1 : 0;
}