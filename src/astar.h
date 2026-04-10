/*
 * astar.h
 *
 * Header file for the A* Search Algorithm implementation.
 * Declares all structs, constants, and function prototypes used
 * across astar.c, main.c, tests.c, and benchmark.c.
 *
 * The real-world context: a humanoid robot navigates an indoor
 * environment modeled as a flat 2D grid. Cells marked 0 are open
 * floor, cells marked 1 are obstacles like furniture or walls.
 * A* finds the shortest valid path from the robot's start cell
 * to its goal cell.
 */

/*
 * Include guard — prevents this header from being processed more
 * than once if multiple .c files include it. Without this the
 * compiler would see duplicate struct definitions and fail.
 */
#ifndef ASTAR_H
#define ASTAR_H

#include <stddef.h>

/*
 * Maximum grid dimensions this program supports.
 * We use fixed-size arrays throughout, so the largest possible
 * grid must be known at compile time. Both values are set to 64,
 * giving a maximum map of 64 x 64 = 4096 cells.
 *
 * These are compile-time constants defined with #define. The
 * compiler replaces every use of MAX_ROWS or MAX_COLS with the
 * literal value before generating any code.
 */
#define MAX_ROWS 64
#define MAX_COLS 64

/*
 * Total cell count for the largest possible grid (64 * 64 = 4096).
 * Every per-cell array in the search — g_score, parent, closed,
 * and path — uses MAX_CELLS as its size so all array accesses
 * stay within bounds regardless of the actual grid dimensions.
 */
#define MAX_CELLS (MAX_ROWS * MAX_COLS)

/*
 * INF_COST represents "not yet reached" for g_score values.
 * We initialize every cell's cost to this value (one billion)
 * before the search begins. Any real path length discovered will
 * always be smaller, so the first update to any cell always wins.
 * This is the standard infinity initialization pattern used in
 * shortest-path algorithms like Dijkstra's.
 */
#define INF_COST 1000000000

/*
 * Point is a struct that holds one grid cell location as (row, col).
 * We use row/col rather than x/y because 2D arrays in C are indexed
 * row-first, which matches the flat index formula:
 *     index = row * num_cols + col
 *
 * Grouping row and col into a single struct keeps function signatures
 * clean — we pass one Point instead of two separate integers everywhere
 * a grid position is needed.
 */
typedef struct {
    int row;   /* vertical axis: 0 = top, increases downward */
    int col;   /* horizontal axis: 0 = left, increases rightward */
} Point;

/*
 * Grid is a struct that stores the robot's occupancy map.
 * Each cell in cells[] is either 0 (walkable floor) or 1 (obstacle).
 *
 * We store the 2D grid as a flat 1D array using row-major order:
 *     index = row * cols + col
 * This is equivalent to a 2D array but easier to pass to functions.
 * We always pass Grid* (a pointer to the struct) rather than copying
 * it, because copying a 4096-element array on every function call
 * would be wasteful and slow.
 *
 * rows and cols store the actual dimensions of this specific grid
 * so bounds checking works correctly regardless of the grid size.
 */
typedef struct {
    int rows;
    int cols;
    int cells[MAX_CELLS];  /* flat array: 0 = open, 1 = blocked */
} Grid;

/*
 * SearchResult bundles everything returned by a search into one struct.
 * C functions can only return one value, so a struct is the standard
 * way to return multiple related pieces of data to the caller.
 *
 * found         : 1 if a valid path exists, 0 if the goal is unreachable
 * path_length   : total cells in the path including start and goal
 * nodes_expanded: total cells fully processed during the search.
 *                 This counter exists specifically to compare how many
 *                 cells A* explores versus Dijkstra on the same grid,
 *                 which is the core of the empirical analysis in the paper.
 * path[]        : the sequence of cells from start to goal
 */
typedef struct {
    int found;
    int path_length;
    int nodes_expanded;
    Point path[MAX_CELLS];
} SearchResult;

/*
 * Function prototypes — these declarations let any .c file that
 * includes this header call these functions before the compiler
 * has seen their full definitions in astar.c.
 * The linker connects each call site to the implementation at link time.
 */

/* Initializes a grid to the given size with all cells set to 0 */
void  grid_init(Grid* grid, int rows, int cols);

/* Returns 1 if point p is inside the grid boundaries, 0 if out of bounds */
int   grid_in_bounds(const Grid* grid, Point p);

/* Returns 1 if the cell at p is an obstacle (non-zero), 0 if walkable */
int   grid_is_blocked(const Grid* grid, Point p);

/* Writes value to the cell at p (0 = open, 1 = blocked) */
void  grid_set_cell(Grid* grid, Point p, int value);

/* Returns the raw cell value at p */
int   grid_get_cell(const Grid* grid, Point p);

/* Returns 1 if points a and b share the same row and col */
int   point_equal(Point a, Point b);

/* Converts a (row, col) Point to its flat array index: row * cols + col */
int   point_to_index(const Grid* grid, Point p);

/* Converts a flat index back to a (row, col) Point */
Point index_to_point(const Grid* grid, int index);

/*
 * Computes the Manhattan distance heuristic between two points:
 *     h = |a.row - b.row| + |a.col - b.col|
 *
 * This is the heuristic function h(n) used by A* to estimate the
 * remaining cost from any cell to the goal. Manhattan distance is
 * admissible on a 4-direction grid with step cost 1 because obstacles
 * can only make the path longer — the real cost is always >= h(n).
 * An admissible heuristic guarantees A* finds the optimal path.
 */
int   manhattan_distance(Point a, Point b);

/*
 * Runs A* search from start to goal on the given grid.
 * Fills result with the path, path length, and nodes expanded.
 * Returns 1 if a path was found, 0 if the goal is unreachable.
 *
 * A* treats the grid as an implicit graph where cells are nodes and
 * edges connect adjacent walkable cells. It uses a binary min-heap
 * as a priority queue to always process the most promising cell next,
 * ordered by f = g + h (actual cost so far + heuristic estimate).
 */
int astar_search(const Grid* grid, Point start, Point goal, SearchResult* result);

/*
 * Runs Dijkstra's shortest path algorithm from start to goal.
 * Internally identical to A* but with the heuristic permanently set
 * to zero, so the priority is just g (actual cost). Without a heuristic
 * the search expands outward in all directions from the start instead
 * of focusing toward the goal, which causes it to explore more cells.
 * Included for direct performance comparison with A* in the paper.
 */
int dijkstra_search(const Grid* grid, Point start, Point goal, SearchResult* result);

/* Prints a one-line summary of found status, path length, nodes expanded */
void print_result_summary(const char* label, const SearchResult* result);

#endif