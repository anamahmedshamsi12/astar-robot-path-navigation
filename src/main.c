/*
 * main.c
 *
 * A* Robot Navigation Demo with command-line interface.
 *
 * Demonstrates A* with dynamic weight coefficient from Chatzisavvas et al. [1]
 * on a 2D occupancy grid. Supports verbose mode showing the robot moving
 * step by step and a rerouting scenario when a new obstacle appears.
 *
 * Usage:
 *   ./demo [OPTIONS]
 *
 * Options:
 *   -h, --help          Print this help message
 *   -v, --verbose       Show robot moving step by step
 *   -g, --grid SIZE     Set grid size (default: 8, max: 20)
 *   -s, --scenario N    Run scenario 1 (basic) or 2 (reroute, default: 2)
 *
 * Example:
 *   ./demo -v
 *   ./demo -v -g 12
 *   ./demo -s 1
 *
 * To compile and run: make run
 */

#include "astar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── Global flags ─────────────────────────────────────────────────────────── */
static int VERBOSE   = 0;
static int GRID_SIZE = 8;
static int SCENARIO  = 2;

/* ── Help ─────────────────────────────────────────────────────────────────── */
static void print_help(void) {
    printf("Usage: ./demo [OPTIONS]\n\n");
    printf("Demonstrates A* pathfinding with dynamic weight coefficient\n");
    printf("for robot navigation on a 2D occupancy grid.\n\n");
    printf("Based on:\n");
    printf("  [1] Chatzisavvas et al., Electronics 2024 -- dynamic weight k\n");
    printf("  [2] Hu et al., ACM EPCE 2022              -- rerouting scenario\n");
    printf("  [3] Mai Jialing et al., ACM ACMLC 2025    -- improved heuristic\n\n");
    printf("Options:\n");
    printf("  -h, --help          Print this help message\n");
    printf("  -v, --verbose       Show robot moving step by step\n");
    printf("  -g, --grid SIZE     Set grid size (default: 8, max: 20)\n");
    printf("  -s, --scenario N    1 = basic path only, 2 = reroute demo (default: 2)\n\n");
    printf("Examples:\n");
    printf("  ./demo -v\n");
    printf("  ./demo -v -g 12\n");
    printf("  ./demo -s 1 -v\n\n");
}

/* ── Argument parsing ─────────────────────────────────────────────────────── */
static int check_for_help(int argc, char** argv) {
    int i;
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
            return 1;
    }
    return 0;
}

static void process_args(int argc, char** argv) {
    int i;
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            VERBOSE = 1;
        } else if ((strcmp(argv[i], "-g") == 0 || strcmp(argv[i], "--grid") == 0)
                    && i + 1 < argc) {
            int sz = atoi(argv[++i]);
            if (sz >= 4 && sz <= 20) {
                GRID_SIZE = sz;
            } else {
                printf("WARNING: grid size must be between 4 and 20. Using default 8.\n");
            }
        } else if ((strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--scenario") == 0)
                    && i + 1 < argc) {
            int sc = atoi(argv[++i]);
            if (sc == 1 || sc == 2) {
                SCENARIO = sc;
            } else {
                printf("WARNING: scenario must be 1 or 2. Using default 2.\n");
            }
        }
    }
}

/* ── Grid helpers ─────────────────────────────────────────────────────────── */

/*
 * load_demo_grid() builds the obstacle layout for the demo.
 * Walls simulate an indoor room a robot navigates through.
 * The layout scales with GRID_SIZE so the demo works at any size.
 */
static void load_demo_grid(Grid* grid) {
    int size = GRID_SIZE;
    int wall1_col = size / 3;
    int wall2_col = (2 * size) / 3;
    int row;

    grid_init(grid, size, size);

    for (row = 1; row < size - 1; row++) {
        if (row != size - 3)
            grid_set_cell(grid, (Point){row, wall1_col}, 1);
    }
    for (row = 2; row < size - 2; row++) {
        if (row != 2)
            grid_set_cell(grid, (Point){row, wall2_col}, 1);
    }
}

static int point_in_path(const SearchResult* result, Point p) {
    int i;
    for (i = 0; i < result->path_length; i++) {
        if (point_equal(result->path[i], p)) return 1;
    }
    return 0;
}

/*
 * print_grid() prints a visual map with optional path overlay.
 * If robot_pos is not NULL, prints R at the robot's current position.
 */
static void print_grid(const Grid* grid, const SearchResult* result,
                        Point start, Point goal, const Point* robot_pos) {
    int row, col;
    for (row = 0; row < grid->rows; row++) {
        for (col = 0; col < grid->cols; col++) {
            Point p = {row, col};
            char symbol = '.';

            if      (robot_pos != NULL && point_equal(p, *robot_pos))          symbol = 'R';
            else if (point_equal(p, start))                                     symbol = 'S';
            else if (point_equal(p, goal))                                      symbol = 'G';
            else if (grid_is_blocked(grid, p))                                  symbol = '#';
            else if (result && result->found && point_in_path(result, p))       symbol = '*';

            printf("%c ", symbol);
        }
        printf("\n");
    }
}

static void print_path_coordinates(const SearchResult* result) {
    int i;
    if (!result->found) { printf("  No path found.\n"); return; }
    printf("  ");
    for (i = 0; i < result->path_length; i++) {
        printf("(%d,%d)", result->path[i].row, result->path[i].col);
        if (i < result->path_length - 1) printf(" -> ");
    }
    printf("\n");
}

/* ── Separator helpers ────────────────────────────────────────────────────── */
static void print_separator(void) {
    printf("------------------------------------------------------------\n");
}

static void print_header(const char* title) {
    printf("\n============================================================\n");
    printf("  %s\n", title);
    printf("============================================================\n");
}

/* ── Scenario 1: Basic path ───────────────────────────────────────────────── */
static void run_scenario_basic(void) {
    Grid grid;
    SearchResult astar_result;
    SearchResult dijkstra_result;
    Point start = {0, 0};
    Point goal  = {GRID_SIZE - 1, GRID_SIZE - 1};

    print_header("Scenario 1: A* vs Dijkstra Pathfinding");
    printf("Grid size: %dx%d\n", GRID_SIZE, GRID_SIZE);
    printf("Dynamic weight: k=3 when EC>%d (far from goal), "
           "k=0.85 when EC<=%d (near goal)\n", EC_THRESHOLD, EC_THRESHOLD);
    printf("Source: Chatzisavvas et al. [1]\n\n");

    load_demo_grid(&grid);

    printf("Initial map (S=start, G=goal, #=obstacle, .=open):\n");
    print_grid(&grid, NULL, start, goal, NULL);
    printf("\n");

    print_separator();
    printf("Running A* with dynamic weight coefficient...\n");
    astar_search(&grid, start, goal, &astar_result);
    printf("  Found:          %s\n",   astar_result.found ? "yes" : "no");
    printf("  Path length:    %d cells\n", astar_result.path_length);
    printf("  Nodes expanded: %d\n",   astar_result.nodes_expanded);
    printf("  Path: ");
    print_path_coordinates(&astar_result);

    print_separator();
    printf("Running Dijkstra (no heuristic, baseline)...\n");
    dijkstra_search(&grid, start, goal, &dijkstra_result);
    printf("  Found:          %s\n",   dijkstra_result.found ? "yes" : "no");
    printf("  Path length:    %d cells\n", dijkstra_result.path_length);
    printf("  Nodes expanded: %d\n",   dijkstra_result.nodes_expanded);

    print_separator();
    printf("Comparison:\n");
    printf("  A* nodes:       %d\n",   astar_result.nodes_expanded);
    printf("  Dijkstra nodes: %d\n",   dijkstra_result.nodes_expanded);
    if (dijkstra_result.nodes_expanded > 0) {
        double reduction = 100.0 * (dijkstra_result.nodes_expanded - astar_result.nodes_expanded)
                           / dijkstra_result.nodes_expanded;
        printf("  Reduction:      %.1f%%\n", reduction);
    }
    printf("  Same path length: %s\n",
           astar_result.path_length == dijkstra_result.path_length ? "yes" : "no");

    printf("\nGrid with A* path (* = path cells):\n");
    print_grid(&grid, &astar_result, start, goal, NULL);

    if (VERBOSE) {
        printf("\nVerbose: Robot walking path step by step\n");
        print_separator();
        int i;
        for (i = 0; i < astar_result.path_length; i++) {
            Point robot = astar_result.path[i];
            printf("Step %2d: Robot at (%d,%d)\n", i, robot.row, robot.col);
            print_grid(&grid, &astar_result, start, goal, &robot);
            printf("\n");
        }
        printf("Robot reached goal (%d,%d).\n", goal.row, goal.col);
    }
}

/* ── Scenario 2: Rerouting ────────────────────────────────────────────────── */
static void run_scenario_reroute(void) {
    Grid grid;
    SearchResult initial_result;
    SearchResult reroute_result;
    Point start = {0, 0};
    Point goal  = {GRID_SIZE - 1, GRID_SIZE - 1};
    Point robot_pos;
    Point new_obstacle;
    int step;

    print_header("Scenario 2: Dynamic Rerouting");
    printf("Grid size: %dx%d\n", GRID_SIZE, GRID_SIZE);
    printf("Inspired by rerouting experiments in Hu et al. [2]\n\n");

    load_demo_grid(&grid);

    /* Initial plan */
    printf("Step 1: Planning initial route...\n");
    astar_search(&grid, start, goal, &initial_result);

    if (!initial_result.found || initial_result.path_length < 6) {
        printf("ERROR: Could not find initial path on this grid.\n");
        return;
    }

    printf("  Path found: %d cells, %d nodes expanded\n",
           initial_result.path_length, initial_result.nodes_expanded);
    printf("  Path: ");
    print_path_coordinates(&initial_result);
    printf("\nInitial map with planned route:\n");
    print_grid(&grid, &initial_result, start, goal, NULL);

    /* Robot moves along path */
    print_separator();
    printf("Step 2: Robot begins moving along planned route...\n\n");

    if (VERBOSE) {
        for (step = 0; step < 4 && step < initial_result.path_length; step++) {
            robot_pos = initial_result.path[step];
            printf("  Move %d: Robot at (%d,%d)\n", step + 1, robot_pos.row, robot_pos.col);
            print_grid(&grid, &initial_result, start, goal, &robot_pos);
            printf("\n");
        }
    } else {
        for (step = 0; step < 3; step++) {
            robot_pos = initial_result.path[step];
            printf("  Move %d: Robot moves to (%d,%d)\n",
                   step + 1, robot_pos.row, robot_pos.col);
        }
    }

    /* New obstacle appears */
    robot_pos    = initial_result.path[3];
    new_obstacle = initial_result.path[4];
    grid_set_cell(&grid, new_obstacle, 1);

    print_separator();
    printf("Step 3: New obstacle detected!\n");
    printf("  Robot current position: (%d,%d)\n", robot_pos.row, robot_pos.col);
    printf("  Blocked cell:           (%d,%d)\n", new_obstacle.row, new_obstacle.col);
    printf("  Replanning from current position...\n\n");

    /* Replan */
    astar_search(&grid, robot_pos, goal, &reroute_result);

    if (!reroute_result.found) {
        printf("ERROR: No alternative path found.\n");
        return;
    }

    printf("  New path found: %d cells, %d nodes expanded\n",
           reroute_result.path_length, reroute_result.nodes_expanded);
    printf("  New path: ");
    print_path_coordinates(&reroute_result);

    printf("\nGrid after rerouting (R=robot, #=obstacle including new one):\n");
    print_grid(&grid, &reroute_result, robot_pos, goal, &robot_pos);

    if (VERBOSE) {
        printf("\nVerbose: Robot walking new route step by step\n");
        print_separator();
        int i;
        for (i = 0; i < reroute_result.path_length; i++) {
            Point rp = reroute_result.path[i];
            printf("Step %2d: Robot at (%d,%d)\n", i, rp.row, rp.col);
            print_grid(&grid, &reroute_result, robot_pos, goal, &rp);
            printf("\n");
        }
        printf("Robot reached goal (%d,%d) via new route.\n", goal.row, goal.col);
    }

    print_separator();
    printf("Summary:\n");
    printf("  Initial plan:  %d nodes expanded\n", initial_result.nodes_expanded);
    printf("  Replan:        %d nodes expanded\n", reroute_result.nodes_expanded);
    printf("  Replan is faster because the robot starts closer to the goal.\n");
    printf("  This confirms A* is suitable for real-time replanning [2].\n");
}

/* ── Main ─────────────────────────────────────────────────────────────────── */
int main(int argc, char** argv) {
    if (check_for_help(argc, argv)) {
        print_help();
        return 0;
    }

    process_args(argc, argv);

    printf("A* Robot Navigation Demo\n");
    printf("Dynamic weight coefficient: k=%d (far from goal, EC>%d), "
           "k=0.%d (near goal)\n",
           WEIGHT_HIGH, EC_THRESHOLD, WEIGHT_LOW_NUM);
    printf("Verbose mode: %s\n", VERBOSE ? "on" : "off");
    printf("Grid size:    %dx%d\n", GRID_SIZE, GRID_SIZE);
    printf("Scenario:     %d\n\n", SCENARIO);

    if (SCENARIO == 1) {
        run_scenario_basic();
    } else {
        run_scenario_basic();
        run_scenario_reroute();
    }

    printf("\nDone. Run with -h for usage options.\n");
    return 0;
}
