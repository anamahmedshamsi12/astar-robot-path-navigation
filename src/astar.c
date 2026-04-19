/*
 * astar.c
 *
 * Implementation of A* Search and Dijkstra's Shortest Path for
 * robot navigation on a 2D occupancy grid.
 *
 * The key change from a standard A* implementation is the dynamic
 * weight coefficient applied to the heuristic, based on:
 *
 *   Chatzisavvas et al. [1]: f(n) = g(n) + k * h(n)
 *     where k = 3 when the estimated remaining cost is high (far from goal)
 *     and   k = 0.85 when the estimated remaining cost is low (near goal).
 *
 *   Hu et al. [2] also uses a weighted heuristic f(n) = g(n) + a*h(n)
 *   with a > 1 to prevent the algorithm from wasting time on round-trip
 *   searches — a problem noted in standard A* on large grids.
 *
 *   Mai Jialing et al. [3] confirms that dynamically adjusting the
 *   heuristic weight improves search efficiency in complex environments.
 */

#include "astar.h"
#include <stdio.h>
#include <string.h>

/*
 * HeapEntry is one item in the priority queue.
 * A* uses a priority queue so it always processes the cell with
 * the lowest f_score first — the cell most likely to be on the
 * optimal path based on cost so far and estimated remaining cost.
 *
 * cell_index: flat array index of the grid cell this entry represents.
 *   Stored as an integer rather than a Point because integers are
 *   cheaper to copy and compare inside the heap.
 *
 * f_score: the priority key. For A*, f = g + k*h. For Dijkstra, f = g.
 *   The heap always gives us the entry with the smallest f_score first.
 *
 * h_score: stored separately for tie-breaking. When two entries have
 *   equal f_score we prefer the one with the smaller h_score — the
 *   cell estimated closer to the goal — keeping the search directional.
 */
typedef struct {
    int cell_index;
    int f_score;
    int h_score;
} HeapEntry;

/*
 * MinHeap is the binary min-heap used as the priority queue for A*.
 *
 * A binary min-heap stores entries in an array such that:
 *   - Index 0 always holds the entry with the smallest f_score
 *   - Parent of node at index i is at (i-1)/2
 *   - Children of node at index i are at 2i+1 and 2i+2
 *
 * Both push and pop run in O(log n) time, giving A* its overall
 * O(V log V) time complexity where V is the number of grid cells.
 *
 * We size data[] at MAX_CELLS * 4 because the same cell can be pushed
 * multiple times with different f_scores as better paths are found.
 * Lazy deletion (never removing stale entries) is simpler than
 * updating existing entries in place. The extra space prevents overflow.
 */
typedef struct {
    HeapEntry data[MAX_CELLS * 4];
    int size;
} MinHeap;

/*
 * result_reset() clears a SearchResult to its zero/empty state.
 * Called at the start of every search to prevent data from a previous
 * call from contaminating the new result. C has no constructors so
 * we zero every field manually. Path slots are set to (-1,-1) as
 * a clear "not set" sentinel useful for debugging.
 */
static void result_reset(SearchResult* result) {
    int i;
    result->found = 0;
    result->path_length = 0;
    result->nodes_expanded = 0;
    for (i = 0; i < MAX_CELLS; i++) {
        result->path[i].row = -1;
        result->path[i].col = -1;
    }
}

/*
 * grid_init() sets up a Grid with the given dimensions and zeros all
 * cells (walkable). Must be called before placing any obstacles.
 * We loop through all MAX_CELLS entries rather than just rows*cols
 * to ensure no leftover garbage from previous memory contents.
 */
void grid_init(Grid* grid, int rows, int cols) {
    int i;
    grid->rows = rows;
    grid->cols = cols;
    for (i = 0; i < MAX_CELLS; i++) {
        grid->cells[i] = 0;
    }
}

/*
 * grid_in_bounds() checks that Point p is inside the grid.
 * C does not check array bounds automatically — reading outside
 * an array causes undefined behavior. This function is the explicit
 * guard called before every cell access to prevent that.
 */
int grid_in_bounds(const Grid* grid, Point p) {
    if (p.row < 0 || p.row >= grid->rows) { return 0; }
    if (p.col < 0 || p.col >= grid->cols) { return 0; }
    return 1;
}

/*
 * point_equal() compares two Points field by field.
 * C structs cannot be compared with == directly so we check
 * each integer field individually.
 */
int point_equal(Point a, Point b) {
    return a.row == b.row && a.col == b.col;
}

/*
 * point_to_index() converts (row, col) to a flat array index
 * using row-major order: index = row * cols + col.
 * Every complete row occupies exactly "cols" slots, so multiplying
 * row by cols skips all prior rows, and adding col lands on the
 * correct position within that row.
 */
int point_to_index(const Grid* grid, Point p) {
    return p.row * grid->cols + p.col;
}

/*
 * index_to_point() reverses point_to_index().
 * Integer division (index / cols) gives the row.
 * The remainder (index % cols) gives the column within that row.
 */
Point index_to_point(const Grid* grid, int index) {
    Point p;
    p.row = index / grid->cols;
    p.col = index % grid->cols;
    return p;
}

/*
 * grid_get_cell() returns the raw cell value (0 or 1) at Point p.
 */
int grid_get_cell(const Grid* grid, Point p) {
    return grid->cells[point_to_index(grid, p)];
}

/*
 * grid_is_blocked() returns 1 if the cell is an obstacle.
 * Any non-zero value is treated as blocked so the grid can later
 * support multiple obstacle types without changing this function.
 */
int grid_is_blocked(const Grid* grid, Point p) {
    return grid_get_cell(grid, p) != 0;
}

/*
 * grid_set_cell() writes a value to the cell at Point p.
 * Bounds are checked first to prevent writing outside cells[].
 */
void grid_set_cell(Grid* grid, Point p, int value) {
    if (grid_in_bounds(grid, p)) {
        grid->cells[point_to_index(grid, p)] = value;
    }
}

/*
 * manhattan_distance() computes the base heuristic h(n):
 *     h = |a.row - b.row| + |a.col - b.col|
 *
 * This is the estimated number of steps from a to b ignoring obstacles.
 * We compute absolute value manually with if-statements rather than
 * calling abs() from stdlib.h to keep dependencies minimal.
 *
 * The heuristic is admissible on a 4-direction grid with step cost 1:
 * the real shortest path can never be shorter than the Manhattan
 * distance because obstacles can only increase the path length.
 * An admissible heuristic is required for A* to guarantee it finds
 * the optimal path. Chatzisavvas et al. [1] and Hu et al. [2] both
 * use Manhattan distance as their grid-based heuristic for this reason.
 */
int manhattan_distance(Point a, Point b) {
    int row_diff = a.row - b.row;
    int col_diff = a.col - b.col;
    if (row_diff < 0) { row_diff = -row_diff; }
    if (col_diff < 0) { col_diff = -col_diff; }
    return row_diff + col_diff;
}

/*
 * compute_weighted_f() applies the dynamic weight coefficient from
 * Chatzisavvas et al. [1] to compute f = g + k * h.
 *
 * The estimated cost EC is the Manhattan distance from the current
 * cell to the goal, representing how far away the goal still is.
 *
 * When EC > EC_THRESHOLD (robot is far from goal):
 *   k = WEIGHT_HIGH = 3
 *   f = g + 3 * h
 *   A higher weight makes the heuristic dominate the priority,
 *   pushing the search aggressively toward the goal and reducing
 *   the number of nodes expanded in open areas.
 *
 * When EC <= EC_THRESHOLD (robot is near goal):
 *   k = WEIGHT_LOW = 0.85 (stored as 85/100 for integer math)
 *   f = g + (h * 85) / 100
 *   A lower weight makes the search more cautious near the goal,
 *   ensuring the actual cost g(n) has more influence so the algorithm
 *   does not overshoot or take a suboptimal final approach.
 *
 * This adaptive behavior is described in Algorithm 1 of [1] and
 * supported by the efficiency analysis in [3].
 */
static int compute_weighted_f(int g, int h, int ec) {
    if (ec > EC_THRESHOLD) {
        return g + WEIGHT_HIGH * h;
    } else {
        return g + (h * WEIGHT_LOW_NUM) / WEIGHT_LOW_DEN;
    }
}

/*
 * heap_has_higher_priority() defines the ordering rule for the heap.
 * Returns 1 if entry a should be processed before entry b.
 * Primary sort key: f_score (lower = higher priority).
 * Tiebreaker 1: h_score (lower = closer estimated to goal).
 * Tiebreaker 2: cell_index (ensures deterministic ordering).
 */
static int heap_has_higher_priority(HeapEntry a, HeapEntry b) {
    if (a.f_score != b.f_score) { return a.f_score < b.f_score; }
    if (a.h_score != b.h_score) { return a.h_score < b.h_score; }
    return a.cell_index < b.cell_index;
}

/*
 * heap_swap() exchanges two HeapEntry values via a temporary variable.
 * Takes pointers to modify the actual array entries rather than copies.
 * Standard three-step swap: save a, overwrite a with b, overwrite b.
 */
static void heap_swap(HeapEntry* a, HeapEntry* b) {
    HeapEntry temp = *a;
    *a = *b;
    *b = temp;
}

/*
 * heap_init() resets the heap to empty by setting size to 0.
 * No need to clear data[] since size tracks the valid boundary.
 */
static void heap_init(MinHeap* heap) {
    heap->size = 0;
}

/*
 * heap_is_empty() returns 1 if no entries remain in the heap.
 * The main search loop uses this as its stopping condition: if the
 * heap empties without reaching the goal, the goal is unreachable.
 */
static int heap_is_empty(const MinHeap* heap) {
    return heap->size == 0;
}

/*
 * heap_push() inserts a new entry and restores the heap property
 * using bubble-up.
 *
 * Steps:
 *   1. Place the new entry at the end of the array (index = size).
 *   2. Increment size.
 *   3. Compare with parent at (child-1)/2. If higher priority, swap.
 *   4. Move up one level and repeat until settled or at root.
 *
 * Time complexity: O(log n) — tree height is log(n) levels, one swap max per level.
 */
static void heap_push(MinHeap* heap, HeapEntry entry) {
    int child;
    int parent;

    heap->data[heap->size] = entry;
    child = heap->size;
    heap->size++;

    while (child > 0) {
        parent = (child - 1) / 2;
        if (heap_has_higher_priority(heap->data[child], heap->data[parent])) {
            heap_swap(&heap->data[child], &heap->data[parent]);
            child = parent;
        } else {
            break;
        }
    }
}

/*
 * heap_pop() removes and returns the root (highest-priority entry)
 * and restores the heap property using bubble-down.
 *
 * Steps:
 *   1. Save root to return.
 *   2. Move last entry to root slot, decrement size.
 *   3. Compare with children at 2*parent+1 and 2*parent+2.
 *   4. Swap with higher-priority child if one exists.
 *   5. Move down one level and repeat until settled.
 *
 * Time complexity: O(log n)
 */
static HeapEntry heap_pop(MinHeap* heap) {
    HeapEntry top = heap->data[0];
    int parent = 0;

    heap->size--;
    heap->data[0] = heap->data[heap->size];

    while (1) {
        int left     = (2 * parent) + 1;
        int right    = (2 * parent) + 2;
        int smallest = parent;

        if (left < heap->size &&
            heap_has_higher_priority(heap->data[left], heap->data[smallest])) {
            smallest = left;
        }
        if (right < heap->size &&
            heap_has_higher_priority(heap->data[right], heap->data[smallest])) {
            smallest = right;
        }
        if (smallest == parent) { break; }

        heap_swap(&heap->data[parent], &heap->data[smallest]);
        parent = smallest;
    }

    return top;
}

/*
 * build_path() reconstructs the start-to-goal path by following
 * parent[] links backward from goal to start, then reversing.
 *
 * During search, whenever we find a better path to a cell we record
 * parent[cell] = the flat index of the cell we came from. The start
 * cell has parent[start] = -1 as a terminal sentinel. This creates
 * a chain of predecessor links from goal back to start, the same
 * structure as a singly linked list traversed back toward the head.
 *
 * We collect cells into reversed[] (goal first), then copy in reverse
 * order into result->path[] so index 0 = start, last index = goal.
 */
static void build_path(const Grid* grid, int parent[], int goal_index, SearchResult* result) {
    Point reversed[MAX_CELLS];
    int count = 0;
    int current = goal_index;
    int i;

    while (current != -1) {
        reversed[count] = index_to_point(grid, current);
        count++;
        current = parent[current];
    }

    result->path_length = count;
    for (i = 0; i < count; i++) {
        result->path[i] = reversed[count - 1 - i];
    }
}

/*
 * search_internal() is the shared implementation used by both A* and Dijkstra.
 *
 * The use_heuristic flag controls which algorithm runs:
 *   use_heuristic = 1  ->  A* with dynamic weight coefficient (from [1])
 *   use_heuristic = 0  ->  Dijkstra (h = 0, no weight, pure cost-based)
 *
 * Sharing one function keeps the comparison fair: both algorithms use
 * the same grid representation, same heap, and same update rules. The
 * only variable is whether the heuristic and its weight are applied.
 *
 * Algorithm overview:
 *   1. Initialize g_score[] to INF_COST and parent[] to -1 for all cells.
 *   2. Push start into the binary min-heap with f = k * h(start, goal).
 *   3. Pop the cell with the lowest f_score from the heap.
 *   4. If it is the goal, reconstruct and return the path.
 *   5. For each of the 4 neighbors: compute tentative_g = g + 1.
 *      If better than known g_score for that neighbor, update and push.
 *      For A*: compute EC = manhattan(neighbor, goal), then apply
 *      compute_weighted_f(g, h, ec) to get the priority.
 *   6. Repeat until goal is reached or heap is empty.
 *
 * Time complexity:  O(V log V)  where V = rows * cols
 * Space complexity: O(V) for g_score, parent, closed, and the heap
 */
static int search_internal(const Grid* grid, Point start, Point goal,
                           int use_heuristic, SearchResult* result) {

    int total_cells = grid->rows * grid->cols;

    /*
     * g_score[i] = best known actual cost from start to cell i.
     * Initialized to INF_COST so the first real distance always wins.
     * This is the standard shortest-path initialization pattern.
     */
    int g_score[MAX_CELLS];

    /*
     * parent[i] = flat index of the cell we came from to reach cell i.
     * Initialized to -1 (no parent). Following these links backward
     * from the goal reconstructs the complete path, the same
     * predecessor-chain technique used in linked list traversal.
     */
    int parent[MAX_CELLS];

    /*
     * closed[i] = 1 if cell i has been fully processed, 0 otherwise.
     * Once a cell is closed its g_score is finalized and it will not
     * be reprocessed even if it reappears in the heap. This is the
     * visited set from graph traversal — the same role visited[] plays
     * in BFS and DFS to prevent revisiting finalized nodes.
     */
    int closed[MAX_CELLS];

    MinHeap open_set;
    int i;
    int start_index;
    int goal_index;

    /*
     * Four movement directions: up, down, left, right.
     * Stored as an array and looped over rather than writing four
     * separate blocks of duplicate code. This is the standard
     * direction-array pattern in grid-based graph traversal.
     * Only 4-directional movement is used to match the Manhattan
     * distance heuristic, which assumes no diagonal movement.
     */
    Point directions[4] = {
        {-1,  0},
        { 1,  0},
        { 0, -1},
        { 0,  1}
    };

    result_reset(result);

    if (!grid_in_bounds(grid, start) || !grid_in_bounds(grid, goal)) { return 0; }
    if (grid_is_blocked(grid, start) || grid_is_blocked(grid, goal)) { return 0; }

    start_index = point_to_index(grid, start);
    goal_index  = point_to_index(grid, goal);

    for (i = 0; i < total_cells; i++) {
        g_score[i] = INF_COST;
        parent[i]  = -1;
        closed[i]  = 0;
    }

    heap_init(&open_set);
    g_score[start_index] = 0;

    /*
     * Push start into the heap with its initial f_score.
     * For A*: apply dynamic weight to h(start, goal).
     * For Dijkstra: f = 0 (no heuristic at all).
     */
    if (use_heuristic) {
        int h_start = manhattan_distance(start, goal);
        int f_start = compute_weighted_f(0, h_start, h_start);
        heap_push(&open_set, (HeapEntry){start_index, f_start, h_start});
    } else {
        heap_push(&open_set, (HeapEntry){start_index, 0, 0});
    }

    while (!heap_is_empty(&open_set)) {

        HeapEntry current_entry = heap_pop(&open_set);
        int current_index = current_entry.cell_index;
        Point current_point = index_to_point(grid, current_index);
        int d;

        /*
         * Skip stale heap entries. Because we use lazy deletion,
         * the same cell may appear multiple times with different f_scores.
         * If we already closed this cell with the optimal g_score, skip it.
         * This is the same "already visited" check used in BFS and DFS.
         */
        if (closed[current_index]) { continue; }

        /*
         * Mark this cell as fully processed. After this point its
         * g_score is finalized and the loop invariant holds: every
         * closed cell has the optimal cost from start to that cell.
         */
        closed[current_index] = 1;
        result->nodes_expanded++;

        if (current_index == goal_index) {
            result->found = 1;
            build_path(grid, parent, goal_index, result);
            return 1;
        }

        for (d = 0; d < 4; d++) {
            Point neighbor;
            int neighbor_index;
            int tentative_g;
            int h_val;
            int ec;
            int f_val;

            neighbor.row = current_point.row + directions[d].row;
            neighbor.col = current_point.col + directions[d].col;

            if (!grid_in_bounds(grid, neighbor)) { continue; }
            if (grid_is_blocked(grid, neighbor))  { continue; }

            neighbor_index = point_to_index(grid, neighbor);
            if (closed[neighbor_index])           { continue; }

            /*
             * Relaxation step: if going through the current cell gives
             * a shorter path to this neighbor, update and push.
             * Every step costs 1 on this unweighted grid.
             * This is the same edge relaxation used in Dijkstra's algorithm:
             * only update when we have found a strictly better path.
             */
            tentative_g = g_score[current_index] + 1;

            if (tentative_g < g_score[neighbor_index]) {
                g_score[neighbor_index] = tentative_g;
                parent[neighbor_index]  = current_index;

                if (use_heuristic) {
                    /*
                     * Compute the weighted f_score using the dynamic weight
                     * coefficient from Chatzisavvas et al. [1].
                     *
                     * h_val is the base Manhattan distance heuristic.
                     * ec    is the estimated cost = manhattan(neighbor, goal),
                     *       which determines whether k = WEIGHT_HIGH or WEIGHT_LOW.
                     *
                     * compute_weighted_f() applies:
                     *   f = g + 3 * h    when ec > EC_THRESHOLD (far from goal)
                     *   f = g + 0.85 * h when ec <= EC_THRESHOLD (near goal)
                     *
                     * This is the core algorithmic improvement over standard A*
                     * described in Algorithm 1 of [1] and supported by [2] and [3].
                     */
                    h_val = manhattan_distance(neighbor, goal);
                    ec    = h_val;
                    f_val = compute_weighted_f(tentative_g, h_val, ec);
                } else {
                    /*
                     * Dijkstra: no heuristic, no weight.
                     * Priority is just the actual cost g, so the search
                     * expands outward uniformly from the start with no
                     * directional guidance toward the goal.
                     */
                    h_val = 0;
                    f_val = tentative_g;
                }

                heap_push(&open_set, (HeapEntry){neighbor_index, f_val, h_val});
            }
        }
    }

    return 0;
}

/*
 * astar_search() runs A* with the dynamic weight coefficient.
 * Passes use_heuristic=1 so compute_weighted_f() is applied.
 */
int astar_search(const Grid* grid, Point start, Point goal, SearchResult* result) {
    return search_internal(grid, start, goal, 1, result);
}

/*
 * dijkstra_search() runs Dijkstra with no heuristic (h = 0).
 * Passes use_heuristic=0 so f_score = g_score only.
 * Included as the baseline comparison from all three source papers.
 */
int dijkstra_search(const Grid* grid, Point start, Point goal, SearchResult* result) {
    return search_internal(grid, start, goal, 0, result);
}

/*
 * print_result_summary() prints found status, path length, and
 * nodes expanded. The ternary operator prints "yes"/"no" for readability.
 */
void print_result_summary(const char* label, const SearchResult* result) {
    printf("%s\n", label);
    printf("  found: %s\n", result->found ? "yes" : "no");
    printf("  path length: %d\n", result->path_length);
    printf("  nodes expanded: %d\n", result->nodes_expanded);
}
