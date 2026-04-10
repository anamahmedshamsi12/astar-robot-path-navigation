/*
 * astar.c
 *
 * Implementation of A* Search and Dijkstra's Shortest Path for
 * humanoid robot navigation on a 2D occupancy grid.
 */

#include "astar.h"
#include <stdio.h>
#include <string.h>

/*
 * HeapEntry is one item stored inside the priority queue.
 * A* needs a priority queue so it always processes the cell with
 * the lowest estimated total cost f = g + h next.
 *
 * cell_index is the flat array index of the grid cell this entry
 * represents. We store an index rather than a Point because integers
 * are cheaper to copy and compare than structs.
 *
 * f_score is the priority key: g (steps taken so far) + h (estimated
 * steps remaining). The heap always gives us the entry with the
 * smallest f_score first, which is what drives A* to always expand
 * the most promising cell.
 *
 * h_score is stored separately only for tie-breaking. When two entries
 * share the same f_score we prefer the one with the smaller h_score —
 * the cell estimated closer to the goal — so the search stays directional.
 */
typedef struct {
    int cell_index;
    int f_score;
    int h_score;
} HeapEntry;

/*
 * MinHeap is the binary min-heap used as the priority queue for A*.
 *
 * A binary min-heap stores entries in an array where:
 *   - Index 0 always holds the entry with the smallest f_score
 *   - For any entry at index i, its parent is at (i-1)/2
 *   - For any entry at index i, its children are at 2i+1 and 2i+2
 *
 * The heap property means push and pop both run in O(log n) time,
 * which is what keeps A*'s overall runtime at O(V log V).
 *
 * We size data[] at MAX_CELLS * 4 because the same cell can be pushed
 * multiple times with different f_scores as better paths are discovered.
 * Using lazy deletion (never removing stale entries) is simpler than
 * updating existing entries in place, and the extra space handles the
 * worst-case number of pushes without overflow.
 *
 * size tracks how many valid entries are currently in the heap.
 * Entries at indices 0 through size-1 are valid; the rest are ignored.
 */
typedef struct {
    HeapEntry data[MAX_CELLS * 4];
    int size;
} MinHeap;

/*
 * result_reset() clears a SearchResult to its initial empty state.
 * Called at the start of every search so stale data from a previous
 * call does not bleed into the new result.
 *
 * In C there are no constructors, so we manually zero every field
 * we care about. Setting path entries to (-1, -1) gives us a clear
 * "not set" sentinel that is easy to spot during debugging.
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
 * grid_init() sets up a Grid struct with the given dimensions and
 * clears every cell to 0 (walkable). Must be called before placing
 * any obstacles with grid_set_cell().
 *
 * We loop through the full MAX_CELLS entries even if the grid is
 * smaller than 64x64 so there is no leftover garbage from whatever
 * was previously in that memory location. In C, local variables and
 * struct fields are not automatically zeroed — we have to do it manually.
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
 * grid_in_bounds() checks whether Point p is inside the valid area
 * of the grid. We call this before every cell access to prevent
 * reading or writing outside the cells[] array.
 *
 * C does not check array bounds automatically — accessing outside
 * the array causes undefined behavior (usually a crash or silent
 * memory corruption). This function is the explicit guard that keeps
 * every array access inside the valid range.
 */
int grid_in_bounds(const Grid* grid, Point p) {
    if (p.row < 0 || p.row >= grid->rows) { return 0; }
    if (p.col < 0 || p.col >= grid->cols) { return 0; }
    return 1;
}

/*
 * point_equal() compares two Points field by field.
 * C structs cannot be compared with == directly, so we compare
 * each field individually. Returns 1 only if both row and col match.
 */
int point_equal(Point a, Point b) {
    return a.row == b.row && a.col == b.col;
}

/*
 * point_to_index() converts a 2D (row, col) position to a flat index
 * using row-major order: index = row * cols + col.
 *
 * Row-major order is how 2D data is laid out in a flat array in C.
 * Every complete row takes up exactly "cols" slots, so multiplying
 * the row by cols skips past all previous rows, and adding col lands
 * on the exact slot within that row.
 *
 * We use this formula to access g_score[], parent[], closed[], and
 * cells[] — all of which are flat arrays indexed by cell position.
 */
int point_to_index(const Grid* grid, Point p) {
    return p.row * grid->cols + p.col;
}

/*
 * index_to_point() reverses point_to_index().
 * Integer division (index / cols) gives the row because every
 * complete row has exactly "cols" cells. The remainder (index % cols)
 * gives the column position within that row.
 *
 * We use this when we pop a flat index from the heap and need to
 * convert it back to a (row, col) position to expand its neighbors.
 */
Point index_to_point(const Grid* grid, int index) {
    Point p;
    p.row = index / grid->cols;
    p.col = index % grid->cols;
    return p;
}

/*
 * grid_get_cell() returns the raw value (0 or 1) stored at Point p.
 */
int grid_get_cell(const Grid* grid, Point p) {
    return grid->cells[point_to_index(grid, p)];
}

/*
 * grid_is_blocked() returns 1 if the cell is an obstacle.
 * Any non-zero value counts as blocked so the grid can later support
 * multiple obstacle types without changing this function.
 */
int grid_is_blocked(const Grid* grid, Point p) {
    return grid_get_cell(grid, p) != 0;
}

/*
 * grid_set_cell() writes a value to the cell at Point p.
 * We check bounds before writing to prevent corrupting memory
 * outside the cells[] array.
 */
void grid_set_cell(Grid* grid, Point p, int value) {
    if (grid_in_bounds(grid, p)) {
        grid->cells[point_to_index(grid, p)] = value;
    }
}

/*
 * manhattan_distance() computes the heuristic estimate h(n):
 * the number of steps it would take to reach b from a if there
 * were no obstacles at all.
 *
 *     h = |a.row - b.row| + |a.col - b.col|
 *
 * We compute absolute value manually using an if statement rather
 * than calling abs() from stdlib.h, keeping the logic explicit and
 * the dependencies minimal.
 *
 * This heuristic is admissible because it never overestimates the
 * real cost. On a 4-direction grid where every step costs 1, the
 * true shortest path can never be shorter than the Manhattan distance.
 * Obstacles can only make the path longer. An admissible heuristic
 * is required for A* to guarantee it finds the optimal path.
 */
int manhattan_distance(Point a, Point b) {
    int row_diff = a.row - b.row;
    int col_diff = a.col - b.col;
    if (row_diff < 0) { row_diff = -row_diff; }
    if (col_diff < 0) { col_diff = -col_diff; }
    return row_diff + col_diff;
}

/*
 * heap_has_higher_priority() defines the ordering rule for the binary
 * min-heap. Returns 1 if entry a should be processed before entry b.
 *
 * Primary key: f_score — lower means higher priority.
 * Tiebreaker 1: h_score — lower means closer estimated to goal.
 * Tiebreaker 2: cell_index — ensures deterministic ordering.
 *
 * Every parent in the heap must satisfy heap_has_higher_priority(parent, child).
 */
static int heap_has_higher_priority(HeapEntry a, HeapEntry b) {
    if (a.f_score != b.f_score) { return a.f_score < b.f_score; }
    if (a.h_score != b.h_score) { return a.h_score < b.h_score; }
    return a.cell_index < b.cell_index;
}

/*
 * heap_swap() exchanges two HeapEntry values using a temp variable.
 * We take pointers so we can modify the actual array entries rather
 * than working on local copies. This is the standard three-step
 * swap: save a, overwrite a with b, overwrite b with saved a.
 */
static void heap_swap(HeapEntry* a, HeapEntry* b) {
    HeapEntry temp = *a;
    *a = *b;
    *b = temp;
}

/*
 * heap_init() resets the heap to empty by setting size to 0.
 * The data[] array does not need to be cleared because size tracks
 * the boundary of valid entries — we never read past index size-1.
 */
static void heap_init(MinHeap* heap) {
    heap->size = 0;
}

/*
 * heap_is_empty() returns 1 if there are no entries in the heap.
 * The main search loop uses this as its stopping condition: if the
 * heap empties before the goal is reached, the goal is unreachable.
 */
static int heap_is_empty(const MinHeap* heap) {
    return heap->size == 0;
}

/*
 * heap_push() inserts a new entry into the binary min-heap and
 * restores the heap property using bubble-up.
 *
 * Bubble-up works as follows:
 *   1. Place the new entry at the end of the array (index = size).
 *   2. Increment size to include it.
 *   3. Compare the new entry with its parent at index (child-1)/2.
 *   4. If the new entry has higher priority than its parent, swap them.
 *   5. Repeat from step 3, moving upward, until the entry is in
 *      the correct position or reaches the root (index 0).
 *
 * The parent index formula (child - 1) / 2 uses integer division,
 * which automatically floors the result for odd child indices.
 *
 * Time complexity: O(log n) because the tree has at most log(n) levels
 * and we do at most one swap per level.
 */
static void heap_push(MinHeap* heap, HeapEntry entry) {
    int child;
    int parent;

    heap->data[heap->size] = entry;  /* place at end of array */
    child = heap->size;
    heap->size++;

    /* bubble up: swap with parent while this entry has higher priority */
    while (child > 0) {
        parent = (child - 1) / 2;   /* parent index formula for binary heap */
        if (heap_has_higher_priority(heap->data[child], heap->data[parent])) {
            heap_swap(&heap->data[child], &heap->data[parent]);
            child = parent;          /* move up one level and repeat */
        } else {
            break;                   /* heap property restored, stop */
        }
    }
}

/*
 * heap_pop() removes and returns the highest-priority entry (the root
 * at index 0) and restores the heap property using bubble-down.
 *
 * Bubble-down works as follows:
 *   1. Save the root entry to return at the end.
 *   2. Move the last entry in the array into the root slot.
 *   3. Decrement size to remove the last slot.
 *   4. Compare the new root with its children:
 *        left child  = 2*parent + 1
 *        right child = 2*parent + 2
 *   5. Swap with the higher-priority child if one exists.
 *   6. Repeat from step 4 moving downward until settled.
 *
 * Time complexity: O(log n) — at most one swap per level of the tree.
 */
static HeapEntry heap_pop(MinHeap* heap) {
    HeapEntry top = heap->data[0];   /* save root to return */
    int parent = 0;

    heap->size--;
    heap->data[0] = heap->data[heap->size];  /* move last entry to root */

    /* bubble down: swap with the higher-priority child until settled */
    while (1) {
        int left     = (2 * parent) + 1;   /* left child index formula  */
        int right    = (2 * parent) + 2;   /* right child index formula */
        int smallest = parent;

        if (left < heap->size &&
            heap_has_higher_priority(heap->data[left], heap->data[smallest])) {
            smallest = left;
        }
        if (right < heap->size &&
            heap_has_higher_priority(heap->data[right], heap->data[smallest])) {
            smallest = right;
        }
        if (smallest == parent) { break; }  /* heap property restored */

        heap_swap(&heap->data[parent], &heap->data[smallest]);
        parent = smallest;   /* move down one level and repeat */
    }

    return top;
}

/*
 * build_path() reconstructs the full start-to-goal path after the
 * search has successfully reached the goal cell.
 *
 * During the search, every time a better path to a cell is found we
 * record parent[cell] = the flat index of the cell we came from. The
 * start cell has parent[start] = -1 as a sentinel meaning no parent.
 * This creates a chain of predecessor links from goal back to start,
 * similar to how a singly linked list links nodes back toward the head.
 *
 * Reconstruction steps:
 *   1. Start at goal_index and follow parent[] links backward.
 *   2. Collect each cell into a local reversed[] array (goal first).
 *   3. Copy reversed[] into result->path[] in the correct order
 *      (start first) by reading reversed[] from back to front.
 *
 * We build the path in reverse first because it is natural to follow
 * predecessor links backward, and reversing a fixed-size array is
 * a straightforward O(n) operation.
 */
static void build_path(const Grid* grid, int parent[], int goal_index, SearchResult* result) {
    Point reversed[MAX_CELLS];
    int count = 0;
    int current = goal_index;
    int i;

    /* follow predecessor links from goal back to start */
    while (current != -1) {
        reversed[count] = index_to_point(grid, current);
        count++;
        current = parent[current];   /* step back toward start */
    }

    result->path_length = count;

    /* copy in reverse so result->path[0] = start, result->path[last] = goal */
    for (i = 0; i < count; i++) {
        result->path[i] = reversed[count - 1 - i];
    }
}

/*
 * search_internal() is the main search loop shared by A* and Dijkstra.
 *
 * The only difference between the two algorithms is the use_heuristic flag:
 *   use_heuristic = 1  ->  A*        (priority = g + h)
 *   use_heuristic = 0  ->  Dijkstra  (priority = g,  h always 0)
 *
 * Sharing one function means the comparison between algorithms is fair:
 * the same grid, same heap, same update logic. The heuristic is the
 * only variable.
 *
 * Overview of the algorithm:
 *   1. Initialize g_score[] to INF_COST for all cells, parent[] to -1.
 *   2. Push the start cell into the binary min-heap with f = h(start).
 *   3. Pop the cell with the lowest f_score from the heap.
 *   4. If it is the goal, reconstruct and return the path.
 *   5. For each of its 4 neighbors: if a shorter path through the
 *      current cell exists, update g_score and push the neighbor.
 *   6. Repeat from step 3 until the goal is reached or the heap empties.
 *
 * Time complexity:  O(V log V)  where V = rows * cols
 * Space complexity: O(V) for g_score, parent, closed, and path arrays
 */
static int search_internal(const Grid* grid, Point start, Point goal,
                           int use_heuristic, SearchResult* result) {

    int total_cells = grid->rows * grid->cols;

    /*
     * g_score[i] = best known number of steps from start to cell i.
     * Initialized to INF_COST (one billion) for all cells so the
     * first real distance found always improves on it. This is the
     * standard infinity initialization for shortest-path algorithms.
     */
    int g_score[MAX_CELLS];

    /*
     * parent[i] = flat index of the cell we came from to reach cell i
     * on the best known path. Initialized to -1 (no parent).
     * After the search, following these predecessor links backward from
     * the goal reconstructs the complete path, the same chain-following
     * technique used with linked list traversal.
     */
    int parent[MAX_CELLS];

    /*
     * closed[i] = 1 if cell i has been fully processed, 0 if not.
     * Once a cell is closed we skip it even if it appears again in the
     * heap. This is the visited set used in graph traversal — the same
     * role a visited[] array plays in BFS and DFS to prevent revisiting
     * nodes. Closing a cell also maintains the loop invariant: every
     * closed cell has its optimal g_score finalized.
     */
    int closed[MAX_CELLS];

    MinHeap open_set;  /* binary min-heap — the priority queue for A* */
    int i;
    int start_index;
    int goal_index;

    /*
     * The four movement directions: up, down, left, right.
     * We store them as an array of Points and loop through all four
     * when expanding neighbors, rather than writing four separate blocks
     * of duplicate code. This is the same direction-array pattern used
     * in grid-based graph traversal.
     */
    Point directions[4] = {
        {-1,  0},
        { 1,  0},
        { 0, -1},
        { 0,  1}
    };

    result_reset(result);

    /* reject start or goal that is out of bounds or inside an obstacle */
    if (!grid_in_bounds(grid, start) || !grid_in_bounds(grid, goal)) { return 0; }
    if (grid_is_blocked(grid, start) || grid_is_blocked(grid, goal)) { return 0; }

    start_index = point_to_index(grid, start);
    goal_index  = point_to_index(grid, goal);

    /* initialization phase: set all costs to infinity, all parents to -1 */
    for (i = 0; i < total_cells; i++) {
        g_score[i] = INF_COST;
        parent[i]  = -1;
        closed[i]  = 0;
    }

    /* seed the heap with the start cell at cost 0 */
    heap_init(&open_set);
    g_score[start_index] = 0;

    /*
     * Push start into the binary min-heap.
     * For A* (use_heuristic=1): f = g + h = 0 + manhattan_distance(start, goal)
     * For Dijkstra (use_heuristic=0): f = 0 (no heuristic, just g)
     * The ternary operator selects between the two based on the flag.
     */
    heap_push(&open_set, (HeapEntry){
        start_index,
        use_heuristic ? manhattan_distance(start, goal) : 0,
        use_heuristic ? manhattan_distance(start, goal) : 0
    });

    /* main search loop: process cells in order of lowest f_score */
    while (!heap_is_empty(&open_set)) {

        /* pop the highest-priority entry from the binary min-heap */
        HeapEntry current_entry = heap_pop(&open_set);
        int current_index = current_entry.cell_index;
        Point current_point = index_to_point(grid, current_index);
        int d;

        /*
         * Skip stale heap entries. Because we use lazy deletion, the
         * same cell can be in the heap multiple times with different
         * f_scores. If we already closed this cell with a better score,
         * skip it — the same "already visited" check used in BFS and DFS.
         */
        if (closed[current_index]) { continue; }

        /*
         * Close this cell: mark it as fully processed.
         * After this point the loop invariant holds for this cell —
         * its g_score is optimal and will not be updated again.
         */
        closed[current_index] = 1;
        result->nodes_expanded++;   /* count for empirical comparison */

        /* goal reached — reconstruct the path and return */
        if (current_index == goal_index) {
            result->found = 1;
            build_path(grid, parent, goal_index, result);
            return 1;
        }

        /* expand all 4 neighbors */
        for (d = 0; d < 4; d++) {
            Point neighbor;
            int neighbor_index;
            int tentative_g;
            int h_value;
            int f_value;

            neighbor.row = current_point.row + directions[d].row;
            neighbor.col = current_point.col + directions[d].col;

            if (!grid_in_bounds(grid, neighbor)) { continue; }  /* bounds check */
            if (grid_is_blocked(grid, neighbor))  { continue; }  /* skip walls  */

            neighbor_index = point_to_index(grid, neighbor);
            if (closed[neighbor_index])           { continue; }  /* already done */

            /*
             * Relaxation step: check if going through the current cell
             * gives a shorter path to this neighbor than previously known.
             * Every step costs 1 on this unweighted grid, so the new cost
             * is simply the current cell's g_score plus 1.
             *
             * This is the same edge relaxation used in Dijkstra's algorithm:
             * only update if the new path is strictly shorter than the best
             * path found so far. The first time we reach a cell its cost is
             * INF_COST, so the first update always triggers.
             */
            tentative_g = g_score[current_index] + 1;

            if (tentative_g < g_score[neighbor_index]) {
                g_score[neighbor_index] = tentative_g;
                parent[neighbor_index]  = current_index;

                /*
                 * Compute h for this neighbor.
                 * A* (use_heuristic=1): h = Manhattan distance to goal.
                 * Dijkstra (use_heuristic=0): h = 0 always.
                 *
                 * This single ternary line is the only algorithmic difference
                 * between A* and Dijkstra in the entire implementation.
                 * With h=0 the priority is just g and the search spreads
                 * outward in all directions. With h=Manhattan the priority
                 * includes an estimate of remaining distance and the search
                 * focuses toward the goal.
                 */
                h_value = use_heuristic ? manhattan_distance(neighbor, goal) : 0;
                f_value = tentative_g + h_value;

                /* push neighbor into the binary min-heap with its new priority */
                heap_push(&open_set, (HeapEntry){neighbor_index, f_value, h_value});
            }
        }
    }

    return 0;  /* heap exhausted — goal is unreachable */
}

/*
 * astar_search() runs A* by passing use_heuristic=1 to search_internal.
 * The heuristic is active: priority = g + h (Manhattan distance to goal).
 */
int astar_search(const Grid* grid, Point start, Point goal, SearchResult* result) {
    return search_internal(grid, start, goal, 1, result);
}

/*
 * dijkstra_search() runs Dijkstra by passing use_heuristic=0.
 * The heuristic is off: priority = g only, h is always 0.
 * This is the same Dijkstra's algorithm used for shortest paths,
 * included here as a direct performance baseline for A*.
 */
int dijkstra_search(const Grid* grid, Point start, Point goal, SearchResult* result) {
    return search_internal(grid, start, goal, 0, result);
}

/*
 * print_result_summary() prints found, path length, and nodes expanded.
 * The ternary operator prints "yes"/"no" instead of 1/0 for readability.
 */
void print_result_summary(const char* label, const SearchResult* result) {
    printf("%s\n", label);
    printf("  found: %s\n", result->found ? "yes" : "no");
    printf("  path length: %d\n", result->path_length);
    printf("  nodes expanded: %d\n", result->nodes_expanded);
}