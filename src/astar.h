/*
 * astar.h
 *
 * Header file for the A* Search Algorithm implementation for
 * robot navigation on a 2D occupancy grid.
 *
 * This implementation is grounded in three peer-reviewed papers:
 *
 *   [1] Chatzisavvas et al., "Optimizing Mobile Robot Navigation Based
 *       on A-Star Algorithm for Obstacle Avoidance in Smart Agriculture,"
 *       Electronics, MDPI, 2024. DOI: 10.3390/electronics13112057
 *
 *   [2] Hu et al., "An Improved A-Star Algorithm for Path Planning of
 *       Outdoor Distribution Robots," ACM EPCE, 2022.
 *       DOI: 10.1145/3529299.3533400
 *
 *   [3] Mai Jialing and Zhang Xiaohua, "Research on Path Planning for
 *       Intelligent Robots Based on Improved A Star Algorithm,"
 *       ACM ACMLC, 2025. DOI: 10.1145/3772673.3772688
 *
 * The core algorithmic improvement from [1] implemented here is a
 * dynamic weight coefficient k applied to the heuristic:
 *
 *     f(n) = g(n) + k * h(n)
 *
 * k is chosen based on the estimated cost (EC = Manhattan distance
 * from the current neighbor to the goal):
 *   EC > EC_THRESHOLD  ->  k = WEIGHT_HIGH (3.0)
 *     The robot is far from the goal. A higher weight pushes the
 *     search aggressively toward the goal, reducing nodes expanded.
 *   EC <= EC_THRESHOLD ->  k = WEIGHT_LOW (0.85)
 *     The robot is near the goal. A lower weight makes the search
 *     more cautious to avoid missing the optimal final approach.
 *
 * Paper [2] also supports a weighted heuristic f(n) = g(n) + a*h(n)
 * with a > 1 to reduce unnecessary round-trip searching. Paper [3]
 * confirms that dynamically adjusting the heuristic weight based on
 * distance to the goal improves both efficiency and path quality.
 */

#ifndef ASTAR_H
#define ASTAR_H

#include <stddef.h>

/*
 * Maximum grid size supported. Fixed-size arrays are used throughout
 * so the upper bound must be known at compile time.
 * 64 * 64 = 4096 cells maximum.
 */
#define MAX_ROWS 64
#define MAX_COLS 64
#define MAX_CELLS (MAX_ROWS * MAX_COLS)

/*
 * Represents "not yet reached" for g_score initialization.
 * Any real path cost found during search will always be smaller,
 * so the first update to any cell always takes effect.
 */
#define INF_COST 1000000000

/*
 * Dynamic weight coefficient constants from Chatzisavvas et al. [1].
 *
 * EC_THRESHOLD = 18: the boundary between far-from-goal and near-goal
 * search modes. Taken directly from Algorithm 1 in [1].
 *
 * WEIGHT_HIGH = 3: applied when EC > 18 (robot far from goal).
 * WEIGHT_LOW = 0.85: applied when EC <= 18 (robot near goal).
 *
 * Because C integer arithmetic truncates, we represent 0.85 as the
 * fraction WEIGHT_LOW_NUM / WEIGHT_LOW_DEN = 85 / 100.
 * The weighted f_score is then computed as:
 *   f = g + (h * WEIGHT_LOW_NUM) / WEIGHT_LOW_DEN
 * This avoids floating-point arithmetic while preserving the intent
 * of the 0.85 weight from the paper.
 */
#define EC_THRESHOLD    18
#define WEIGHT_HIGH      3
#define WEIGHT_LOW_NUM  85
#define WEIGHT_LOW_DEN 100

/*
 * Point represents one cell location as (row, col).
 * Row/col rather than x/y matches C's row-major 2D array indexing:
 *     flat_index = row * num_cols + col
 * Grouping both values in a struct keeps function signatures clean.
 */
typedef struct {
    int row;
    int col;
} Point;

/*
 * Grid stores the robot's occupancy map as a flat 1D array.
 * cells[i] = 0 means the cell is walkable open floor.
 * cells[i] = 1 means the cell is an obstacle.
 *
 * Flat 1D storage with row-major order is equivalent to a 2D array
 * but simpler to pass between functions. We always pass Grid* to
 * avoid copying the entire cells[] array on every function call.
 */
typedef struct {
    int rows;
    int cols;
    int cells[MAX_CELLS];
} Grid;

/*
 * SearchResult bundles all output from one search call.
 * Since C functions can only return one value, a struct is the
 * standard way to return multiple related pieces of data.
 *
 * nodes_expanded is used in the empirical analysis to compare A*
 * with Dijkstra, consistent with the comparison methodology used
 * in all three source papers.
 */
typedef struct {
    int found;
    int path_length;
    int nodes_expanded;
    Point path[MAX_CELLS];
} SearchResult;

/* Initializes a grid to the given size with all cells set to 0 */
void  grid_init(Grid* grid, int rows, int cols);

/* Returns 1 if p is inside the grid boundaries, 0 if out of bounds */
int   grid_in_bounds(const Grid* grid, Point p);

/* Returns 1 if the cell at p is blocked (non-zero), 0 if walkable */
int   grid_is_blocked(const Grid* grid, Point p);

/* Sets the cell at p to value (0 = open, 1 = blocked) */
void  grid_set_cell(Grid* grid, Point p, int value);

/* Returns the raw integer cell value at p */
int   grid_get_cell(const Grid* grid, Point p);

/* Returns 1 if a and b represent the same grid cell */
int   point_equal(Point a, Point b);

/* Converts (row, col) to a flat array index: row * cols + col */
int   point_to_index(const Grid* grid, Point p);

/* Converts a flat array index back to a (row, col) Point */
Point index_to_point(const Grid* grid, int index);

/*
 * Computes the Manhattan distance heuristic h(n) between two points:
 *     h = |a.row - b.row| + |a.col - b.col|
 *
 * This is the base h(n) before the dynamic weight k is applied.
 * Manhattan distance is admissible on a 4-direction grid with step
 * cost 1: the real path can never be shorter than the grid-straight-
 * line distance, so h(n) never overestimates the true remaining cost.
 * Both Chatzisavvas et al. [1] and Hu et al. [2] use Manhattan
 * distance as their grid-based heuristic for the same reason.
 */
int   manhattan_distance(Point a, Point b);

/*
 * Runs A* with the dynamic weight coefficient from [1].
 * Evaluation function: f(n) = g(n) + k * h(n)
 * where k = WEIGHT_HIGH when EC > EC_THRESHOLD,
 *       k = WEIGHT_LOW  when EC <= EC_THRESHOLD,
 * and EC = manhattan_distance(neighbor, goal).
 * Returns 1 if a path was found, 0 if the goal is unreachable.
 */
int astar_search(const Grid* grid, Point start, Point goal, SearchResult* result);

/*
 * Runs Dijkstra's algorithm with h = 0 (no heuristic, no weight).
 * Used as the baseline comparison, consistent with all three papers
 * which compare their improved A* variants against standard Dijkstra.
 */
int dijkstra_search(const Grid* grid, Point start, Point goal, SearchResult* result);

/* Prints found status, path length, and nodes expanded to stdout */
void print_result_summary(const char* label, const SearchResult* result);

#endif
