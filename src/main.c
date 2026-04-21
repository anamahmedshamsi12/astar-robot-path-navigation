/*
 * main.c
 *
 * Demo program for A* Search Algorithm with dynamic weight coefficient.
 * Runs A* and Dijkstra on a grid, prints results, and shows rerouting
 * when a new obstacle appears mid-path. Inspired by the rerouting
 * experiments described in Hu et al. [2].
 *
 * Supports command line flags similar to the speed comparison assignment:
 *   -h, --help         print help and exit
 *   -v, --verbose      show robot moving step by step
 *   -g, --grid SIZE    set grid size between 4 and 20 (default 8)
 *   -s, --scenario N   1 = basic path only, 2 = reroute demo (default 2)
 *
 * Examples:
 *   ./demo -h
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

/*
 * Global flags set by command line arguments.
 * VERBOSE controls whether the robot moves are printed step by step.
 * GRID_SIZE sets the dimensions of the demo grid.
 * SCENARIO selects which demo to run.
 */
static int VERBOSE   = 0;
static int GRID_SIZE = 8;
static int SCENARIO  = 3;

/*
 * print_help() prints usage information and exits.
 * Called when -h or --help is passed on the command line.
 */
static void print_help(void) {
    printf("Usage: ./demo [OPTIONS]\n\n");
    printf("Demonstrates A* pathfinding with dynamic weight coefficient\n");
    printf("for robot navigation on a 2D occupancy grid.\n\n");
    printf("Based on:\n");
    printf("  [1] Chatzisavvas et al., Electronics 2024\n");
    printf("  [2] Hu et al., ACM EPCE 2022\n");
    printf("  [3] Mai Jialing et al., ACM ACMLC 2025\n\n");
    printf("Options:\n");
    printf("  -h, --help          Print this help message\n");
    printf("  -v, --verbose       Show robot moving step by step\n");
    printf("  -g, --grid SIZE     Set grid size (default: 8, max: 20)\n");
    printf("  -s, --scenario N    1 = basic A* vs Dijkstra, 2 = reroute demo, 3 = heuristic comparison (default: 3)\n\n");
    printf("Examples:\n");
    printf("  ./demo -v\n");
    printf("  ./demo -s 1 -v\n");
    printf("  ./demo -s 3 -g 15\n\n");
}

/*
 * check_for_help() scans argv for -h or --help.
 * Returns 1 if found, 0 otherwise.
 */
static int check_for_help(int argc, char** argv) {
    int i;
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
            return 1;
    }
    return 0;
}

/*
 * process_args() parses the command line and sets the global flags.
 * Unrecognized arguments are ignored silently.
 */
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
            if (sc >= 1 && sc <= 3) {
                SCENARIO = sc;
            } else {
                printf("WARNING: scenario must be 1, 2, or 3. Using default 3.\n");
            }
        }
    }
}

/*
 * load_demo_grid() builds the obstacle layout for the demo.
 * Two vertical walls with gaps scale proportionally with GRID_SIZE
 * so the layout stays consistent at any grid size between 4 and 20.
 * Wall positions use integer division to stay proportional.
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

/*
 * point_in_path() does a linear search through the path array.
 * Returns 1 if Point p appears anywhere in the result path, 0 otherwise.
 * Used by print_grid to decide whether to draw a '*' at each cell.
 */
static int point_in_path(const SearchResult* result, Point p) {
    int i;
    for (i = 0; i < result->path_length; i++) {
        if (point_equal(result->path[i], p)) return 1;
    }
    return 0;
}

/*
 * print_grid() prints a visual map of the grid with an optional path overlay.
 * Symbols: S=start, G=goal, R=robot current position, #=obstacle, *=path, .=open.
 * Passing result=NULL shows the map before any search.
 * Passing robot_pos=NULL skips the robot marker.
 */
static void print_grid(const Grid* grid, const SearchResult* result,
                        Point start, Point goal, const Point* robot_pos) {
    int row, col;
    for (row = 0; row < grid->rows; row++) {
        for (col = 0; col < grid->cols; col++) {
            Point p = {row, col};
            char symbol = '.';

            if      (robot_pos != NULL && point_equal(p, *robot_pos))       symbol = 'R';
            else if (point_equal(p, start))                                  symbol = 'S';
            else if (point_equal(p, goal))                                   symbol = 'G';
            else if (grid_is_blocked(grid, p))                               symbol = '#';
            else if (result && result->found && point_in_path(result, p))    symbol = '*';

            printf("%c ", symbol);
        }
        printf("\n");
    }
}

/*
 * print_path_coordinates() prints the path as a sequence of (row,col) pairs
 * separated by arrows so we can verify the path is a connected sequence.
 */
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

/*
 * run_scenario_basic() runs A* and Dijkstra on the demo grid and prints
 * the results side by side. This directly shows the efficiency advantage
 * of the dynamic weight coefficient from Chatzisavvas et al. [1].
 * In verbose mode, the robot walks the path step by step.
 */
static void run_scenario_basic(void) {
    Grid grid;
    SearchResult astar_result;
    SearchResult dijkstra_result;
    Point start = {0, 0};
    Point goal  = {GRID_SIZE - 1, GRID_SIZE - 1};

    printf("\nScenario 1: A* vs Dijkstra Pathfinding\n");
    printf("Grid size: %dx%d\n", GRID_SIZE, GRID_SIZE);
    printf("Dynamic weight: k=3 when EC>%d (far from goal), "
           "k=0.85 when EC<=%d (near goal)\n", EC_THRESHOLD, EC_THRESHOLD);
    printf("Source: Chatzisavvas et al. [1]\n\n");

    load_demo_grid(&grid);

    printf("Initial map (S=start, G=goal, #=obstacle, .=open):\n");
    print_grid(&grid, NULL, start, goal, NULL);
    printf("\n");

    printf("Running A* with dynamic weight coefficient...\n");
    astar_search(&grid, start, goal, &astar_result);
    printf("  Found:          %s\n",       astar_result.found ? "yes" : "no");
    printf("  Path length:    %d cells\n", astar_result.path_length);
    printf("  Nodes expanded: %d\n",       astar_result.nodes_expanded);
    printf("  Path: ");
    print_path_coordinates(&astar_result);

    printf("\nRunning Dijkstra (no heuristic, baseline)...\n");
    dijkstra_search(&grid, start, goal, &dijkstra_result);
    printf("  Found:          %s\n",       dijkstra_result.found ? "yes" : "no");
    printf("  Path length:    %d cells\n", dijkstra_result.path_length);
    printf("  Nodes expanded: %d\n",       dijkstra_result.nodes_expanded);

    printf("\nComparison:\n");
    printf("  A* nodes:         %d\n", astar_result.nodes_expanded);
    printf("  Dijkstra nodes:   %d\n", dijkstra_result.nodes_expanded);
    if (dijkstra_result.nodes_expanded > 0) {
        double reduction = 100.0 * (dijkstra_result.nodes_expanded - astar_result.nodes_expanded)
                           / dijkstra_result.nodes_expanded;
        printf("  Reduction:        %.1f%%\n", reduction);
    }
    printf("  Same path length: %s\n",
           astar_result.path_length == dijkstra_result.path_length ? "yes" : "no");

    printf("\nGrid with A* path (* = path cells):\n");
    print_grid(&grid, &astar_result, start, goal, NULL);

    if (VERBOSE) {
        int i;
        printf("\nVerbose: Robot walking path step by step\n\n");
        for (i = 0; i < astar_result.path_length; i++) {
            Point robot = astar_result.path[i];
            printf("Step %2d: Robot at (%d,%d)\n", i, robot.row, robot.col);
            print_grid(&grid, &astar_result, start, goal, &robot);
            printf("\n");
        }
        printf("Robot reached goal (%d,%d).\n", goal.row, goal.col);
    }
}

/*
 * run_scenario_reroute() simulates the robot moving partway along its planned
 * route when a new obstacle appears directly ahead. A* replans from the
 * robot's current position and finds an alternative route. This mirrors the
 * rerouting experiment in Hu et al. [2] where delivery robots encounter
 * unexpected choke points mid-journey and must dynamically replan.
 */
static void run_scenario_reroute(void) {
    Grid grid;
    SearchResult initial_result;
    SearchResult reroute_result;
    Point start = {0, 0};
    Point goal  = {GRID_SIZE - 1, GRID_SIZE - 1};
    Point robot_pos;
    Point new_obstacle;
    int step;

    printf("\nScenario 2: Dynamic Rerouting\n");
    printf("Grid size: %dx%d\n", GRID_SIZE, GRID_SIZE);
    printf("Inspired by rerouting experiments in Hu et al. [2]\n\n");

    load_demo_grid(&grid);

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

    printf("\nStep 2: Robot begins moving along planned route...\n\n");

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

    /*
     * Simulate a new obstacle appearing on the planned path.
     * path[3] is the robot's current position after 3 moves.
     * path[4] is the next cell, which we block to force a replan.
     * grid_set_cell() marks it as 1, removing it from the graph.
     */
    robot_pos    = initial_result.path[3];
    new_obstacle = initial_result.path[4];
    grid_set_cell(&grid, new_obstacle, 1);

    printf("\nStep 3: New obstacle detected!\n");
    printf("  Robot current position: (%d,%d)\n", robot_pos.row, robot_pos.col);
    printf("  Blocked cell:           (%d,%d)\n", new_obstacle.row, new_obstacle.col);
    printf("  Replanning from current position...\n\n");

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
        int i;
        printf("\nVerbose: Robot walking new route step by step\n\n");
        for (i = 0; i < reroute_result.path_length; i++) {
            Point rp = reroute_result.path[i];
            printf("Step %2d: Robot at (%d,%d)\n", i, rp.row, rp.col);
            print_grid(&grid, &reroute_result, robot_pos, goal, &rp);
            printf("\n");
        }
        printf("Robot reached goal (%d,%d) via new route.\n", goal.row, goal.col);
    }

    printf("\nSummary:\n");
    printf("  Initial plan:  %d nodes expanded\n", initial_result.nodes_expanded);
    printf("  Replan:        %d nodes expanded\n", reroute_result.nodes_expanded);
    printf("  Replan is faster because the robot starts closer to the goal.\n");
    printf("  This confirms A* is suitable for real-time replanning [2].\n");
}

/*
 * run_scenario_heuristics() compares four search configurations on the
 * same grid and prints results side by side. This directly shows how
 * heuristic choice affects the number of nodes expanded and path quality.
 *
 * The four configurations are:
 *   1. A* with Manhattan distance + dynamic weight k (this implementation)
 *   2. A* with Manhattan distance + fixed k = 1 (standard A* from Hart et al.)
 *   3. A* with Euclidean distance + fixed k = 1
 *   4. Dijkstra with no heuristic (h = 0)
 *
 * All four find the same optimal path length, confirming that all three
 * admissible heuristics produce correct results. The node counts show
 * that tighter heuristics expand fewer nodes: Manhattan < Euclidean < Dijkstra.
 * The dynamic weight on top of Manhattan is the most efficient overall.
 *
 * This scenario is inspired by the comparative analysis in
 * Gudari and Vadivu [7] and Yin et al. [6].
 */
static void run_scenario_heuristics(void) {
    Grid grid;
    SearchResult r_dyn, r_man, r_euc, r_dijk;
    Point start = {0, 0};
    Point goal  = {GRID_SIZE - 1, GRID_SIZE - 1};
    double reduction_man, reduction_euc, reduction_dijk;

    printf("\nScenario 3: Heuristic Comparison\n");
    printf("Grid size: %dx%d\n", GRID_SIZE, GRID_SIZE);
    printf("Comparing all four search configurations on the same grid.\n");
    printf("Based on Gudari and Vadivu [7] and Yin et al. [6].\n\n");

    load_demo_grid(&grid);

    printf("Grid map (S=start, G=goal, #=obstacle, .=open):\n");
    print_grid(&grid, NULL, start, goal, NULL);
    printf("\n");

    /* Run all four configurations */
    astar_search(&grid, start, goal, &r_dyn);
    astar_manhattan_standard(&grid, start, goal, &r_man);
    astar_euclidean_standard(&grid, start, goal, &r_euc);
    dijkstra_search(&grid, start, goal, &r_dijk);

    /* Print comparison table */
    printf("Results:\n");
    printf("  %-40s  %6s  %6s  %s\n",
           "Algorithm", "Nodes", "Path", "Found");
    printf("  %-40s  %6s  %6s  %s\n",
           "-----------------------------------------",
           "------", "------", "-----");
    printf("  %-40s  %6d  %6d  %s\n",
           "A* Manhattan + dynamic k (Chatzisavvas [1])",
           r_dyn.nodes_expanded, r_dyn.path_length,
           r_dyn.found ? "yes" : "no");
    printf("  %-40s  %6d  %6d  %s\n",
           "A* Manhattan + fixed k=1 (standard A*)",
           r_man.nodes_expanded, r_man.path_length,
           r_man.found ? "yes" : "no");
    printf("  %-40s  %6d  %6d  %s\n",
           "A* Euclidean + fixed k=1",
           r_euc.nodes_expanded, r_euc.path_length,
           r_euc.found ? "yes" : "no");
    printf("  %-40s  %6d  %6d  %s\n",
           "Dijkstra (h=0, no heuristic)",
           r_dijk.nodes_expanded, r_dijk.path_length,
           r_dijk.found ? "yes" : "no");

    /* Compute reductions relative to Dijkstra */
    if (r_dijk.nodes_expanded > 0) {
        reduction_man  = 100.0 * (r_dijk.nodes_expanded - r_man.nodes_expanded)
                         / r_dijk.nodes_expanded;
        reduction_euc  = 100.0 * (r_dijk.nodes_expanded - r_euc.nodes_expanded)
                         / r_dijk.nodes_expanded;
        reduction_dijk = 100.0 * (r_dijk.nodes_expanded - r_dyn.nodes_expanded)
                         / r_dijk.nodes_expanded;
        printf("\nNode reduction vs Dijkstra:\n");
        printf("  A* Manhattan + dynamic k: %.1f%% fewer nodes\n", reduction_dijk);
        printf("  A* Manhattan + fixed k=1: %.1f%% fewer nodes\n", reduction_man);
        printf("  A* Euclidean + fixed k=1: %.1f%% fewer nodes\n", reduction_euc);
    }

    printf("\nKey observation:\n");
    printf("  All four found paths of the same length: %s\n",
           (r_dyn.path_length == r_man.path_length &&
            r_man.path_length == r_euc.path_length &&
            r_euc.path_length == r_dijk.path_length) ? "yes" : "no");
    printf("  Manhattan is tighter than Euclidean on a 4-direction grid.\n");
    printf("  Tighter heuristic = fewer nodes expanded = faster search.\n");
    printf("  Dynamic weight amplifies this further when far from the goal.\n");
}

/*
 * main() parses arguments, prints the configuration, and runs the selected
 * scenario. Returns 0 on success. Exits early if -h is passed.
 */
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
    printf("Scenario:     %d\n", SCENARIO);

    if (SCENARIO == 1) {
        run_scenario_basic();
    } else if (SCENARIO == 2) {
        run_scenario_basic();
        run_scenario_reroute();
    } else {
        run_scenario_basic();
        run_scenario_reroute();
        run_scenario_heuristics();
    }

    printf("\nDone. Run with -h for usage options.\n");
    return 0;
}
