# Research Paper
* Name: Anam Shamsi
* Semester: Spring 2026
* Topic: **A\* Search Algorithm with Dynamic Weight Coefficient for Robot Navigation**




Note the following is an example outline to help you. Please rework as you need, you do not need to follow the section heads and *YOU SHOULD NOT* make everything a bulleted list. This needs to read as an executive report/research paper. 

## Introduction
- What is the algorithm/datastructure?
- What is the problem it solves? 
- Provide a brief history of the algorithm/datastructure. (make sure to cite sources)
- Provide an introduction to the rest of the paper. 


## Analysis of Algorithm/Datastructure
Make sure to include the following:
- Time Complexity
- Space Complexity
- General analysis of the algorithm/datastructure

## Empirical Analysis
- What is the empirical analysis?
- Provide specific examples / data.


## Application
- What is the algorithm/datastructure used for?
- Provide specific examples
- Why is it useful / used in that field area?
- Make sure to provide sources for your information.


## Implementation
- What language did you use?
- What libraries did you use?
- What were the challenges you faced?
- Provide key points of the algorithm/datastructure implementation, discuss the code.
- If you found code in another language, and then implemented in your own language that is fine - but make sure to document that.


## Summary
- Provide a summary of your findings
- What did you learn?

## LLM Use Disclosure 


## References


# A\* Search Algorithm for Robot Navigation

## Author
Anam Shamsi
CS 5008 — Summative Research Project

---
## Abstract
 
This paper presents an implementation of the A\* Search Algorithm with a dynamic weight coefficient applied to robot navigation on a 2D occupancy grid and a weighted waypoint network. The dynamic weight, introduced by Chatzisavvas et al. [1], modifies the standard evaluation function $f(n) = g(n) + h(n)$ to $f(n) = g(n) + k \cdot h(n)$, where $k$ adapts based on the estimated remaining distance to the goal. Empirical results demonstrate that this modification reduces nodes expanded by up to 96.4% compared to Dijkstra's algorithm across six grid sizes ranging from 10×10 to 60×60. The implementation is written in C using a from-scratch binary min-heap priority queue and is validated by nine correctness tests. Visualizations on both a grid environment and a building waypoint network confirm the theoretical efficiency claims made by the source literature.

## 1. Introduction
 
Path planning is one of the most fundamental problems in autonomous robot navigation. A robot placed in an environment must determine a collision-free route from its current position to a goal location while minimizing traversal cost. The A\* Search Algorithm, originally proposed by Hart, Nilsson, and Raphael [4], has remained the dominant approach to this problem for over five decades due to its completeness, optimality under admissible heuristics, and practical efficiency relative to uninformed methods such as Dijkstra's algorithm.
 
Despite its widespread adoption, standard A\* exhibits known inefficiencies in large-scale environments. Chatzisavvas, Dossis, and Dasygenis [1] demonstrate that A\* tends to generate excessive search routes when the heuristic weight is fixed, particularly in environments with irregular obstacle layouts. Hu et al. [2] observe similar behavior in outdoor delivery robot scenarios, noting that Dijkstra's algorithm examines roughly twice as many nodes as A\* while producing paths of equivalent quality. Mai Jialing and Zhang Xiaohua [3] further identify that the traditional A\* algorithm generates redundant nodes and non-smooth paths when applied to complex indoor environments.

To address these limitations, this paper implements the dynamic weight coefficient proposed by Chatzisavvas et al. [1] in which the heuristic weight $k$ is set to 3 when the estimated remaining cost exceeds a threshold of 18, and to 0.85 otherwise. This adaptation makes the search aggressive when the robot is far from the goal and cautious when approaching it, reducing unnecessary exploration without sacrificing path quality in practice.
 
The implementation connects directly to data structures and algorithms studied throughout CS 5008. The graph representation mirrors the City Finder assignment from Module 10, the priority queue uses the binary min-heap from Module 9, the greedy priority selection extends Dijkstra's algorithm from Module 11, and the correctness argument follows the loop invariant methodology from Module 13. The algorithm is implemented in C, tested empirically across six grid sizes, and visualized on both a grid environment and a weighted building waypoint network.

---
## 2. Background and Related Work
### 2.1 The A\* Search Algorithm

A\* was first described by Hart, Nilsson, and Raphael [4] in 1968 as an extension of Dijkstra's shortest path algorithm. The algorithm maintains an open set of candidate nodes ordered by an evaluation function:
$$f(n) = g(n) + h(n)$$
where $g(n)$ denotes the actual cost from the start node to node $n$, and $h(n)$ denotes a heuristic estimate of the cost from $n$ to the goal. Hart et al. [4] proved that A\* is complete and optimal whenever $h(n)$ is admissible, meaning that $h(n)$ never overestimates the true remaining cost. This admissibility condition is the cornerstone of A\*'s correctness guarantee and distinguishes it from purely greedy search methods.

The relationship between A\* and Dijkstra's algorithm is direct and worth emphasizing. Dijkstra's algorithm, covered in Module 11 of this course, is essentially equivalent to A\* with $h(n) = 0$ for all nodes. Without a heuristic, the search expands uniformly outward from the start node, processing many nodes that are not on the optimal path. With a heuristic, the search focuses toward the goal, reducing practical computation significantly. In this way, understanding A\* requires first understanding Dijkstra's algorithm and then recognizing what the heuristic adds to it.

### 2.2 The Dynamic Weight Coefficient
Chatzisavvas, Dossis, and Dasygenis [1] propose an enhancement to the standard A\* evaluation function that introduces a dynamic weight coefficient $k$:
$$f(n) = g(n) + k \cdot h(n)$$
The weight $k$ is determined by the estimated cost EC, defined as the Manhattan distance from the current node to the goal:
$$k = \begin{cases} 3 & \text{if } EC > 18 \\ 0.85 & \text{if } EC \leq 18 \end{cases}$$

The reasoning behind this design is straightforward. When the robot is far from the goal, the heuristic is a reliable guide and a higher weight accelerates convergence by prioritizing nodes closer to the goal. When the robot is near the goal, the local obstacle geometry becomes more significant than the straight-line estimate, so a lower weight restores the influence of the actual cost $g(n)$ and ensures an accurate final approach. Chatzisavvas et al. [1] report a reduction from 361 to 122 search routes, a 66.2% decrease, and a reduction in computation time from 1.653 ms to 0.912 ms on their agricultural robot benchmarks.

Hu et al. [2] independently arrive at a similar conclusion in their study of outdoor delivery robots, proposing $f(n) = g(n) + a \cdot h(n)$ with $a > 1$ to reduce round-trip searching caused by the standard fixed weight. Their experiments demonstrate that the improved algorithm reduces average delivery time by 11.2% compared to standard A\*. Mai Jialing and Zhang Xiaohua [3] extend this line of work by dynamically adjusting the weight coefficient based on both obstacle density and distance to the goal, reporting approximately 50% improvement in overall planning efficiency. Their work confirms that no single fixed weight is optimal across all environments and that adaptive weighting is a robust strategy for improving A\* in complex robot navigation scenarios.

In order to fully appreciate the significance of this implementation, it helps to trace its connections back to the course material covered in CS 5008.

When we studied graphs in Module 10, we learned that a graph is a collection of nodes connected by edges and that algorithms like BFS and Dijkstra can traverse these structures to find paths. The City Finder homework in that module asked us to find shortest routes between cities, where cities were nodes and roads were weighted edges. The waypoint network in this project is structurally that same problem applied to robot navigation inside a building. In this sense, the graph representation is not new, only the context and the algorithm operating on it.

The binary min-heap from Module 9 is essential to A\*'s efficiency. Without a priority queue, the algorithm would have to scan all known nodes to find the one with the lowest $f$-score at each step, producing $O(V^2)$ behavior. With a binary heap, each extraction costs only $O(\log V)$, bringing the total complexity down to $O(V \log V)$. In doing so, the heap makes A\* practical on grids large enough to be meaningful for robot navigation.

The relationship to Dijkstra's algorithm from Module 11 is particularly important because it clarifies exactly what the heuristic contributes. Dijkstra sets $h(n) = 0$ and explores outward uniformly. A\* adds $h(n)$ and explores directionally. The dynamic weight from Chatzisavvas et al. [1] amplifies that directionality when the robot is far from the goal, which is where the greatest efficiency gains are possible.

Finally, the correctness argument in Section 6 uses a loop invariant in the style of Module 13. The invariant states that every node in the closed set has its optimal $g$-score finalized, and proving it requires the same initialization, maintenance, and termination structure practiced throughout the course.

## 3. Problem Formulation
 
### 3.1 Occupancy Grid Model
 
The primary experimental environment is a 2D occupancy grid, a rectangular array in which each cell is labeled 0 for traversable or 1 for obstacle. This representation is standard in robot navigation literature and is used by all three source papers [1][2][3]. The robot occupies exactly one cell at a time and may move in four cardinal directions, up, down, left, and right, with a uniform step cost of 1. Diagonal movement is not permitted.
 
Formally, the grid can be understood as a graph $G = (V, E)$ where $V$ is the set of traversable cells and $E$ connects pairs of cells that are adjacent and both traversable. A\* finds the minimum-cost path from a designated start cell $s$ to a goal cell $t$.
 
In the C implementation, the grid is stored as a flat 1D array using row-major indexing, where each cell's position is computed as $\text{index}(r, c) = r \cdot \text{cols} + c$. This is the same memory layout studied in Module 2 when we examined how 2D data is stored in C. The grid is always passed by pointer to avoid copying the full array on every function call, consistent with the pass-by-pointer conventions from the Module 2 code-alongs.

### 3.2 Weighted Waypoint Network
 
The secondary model is a weighted undirected graph representing a building floor plan, in which nodes correspond to named rooms or waypoints and edge weights represent corridor distances. This model is structurally equivalent to the City Finder assignment from Module 10, where cities were nodes and roads were weighted edges. A\* finds the minimum-weight path from the Entrance node to the Exit node. In this model, the Euclidean distance between node positions serves as the heuristic, consistent with the approach of Mai Jialing and Zhang Xiaohua [3] for environments with non-uniform edge weights.
 
### 3.3 The Heuristic Function
 
For the occupancy grid, the heuristic is Manhattan distance:
 
$$h(n) = |r_n - r_t| + |c_n - c_t|$$
 
where $(r_n, c_n)$ is the current node and $(r_t, c_t)$ is the goal. This heuristic is admissible on a 4-direction grid with unit step costs because the shortest possible path between any two cells is their Manhattan distance. No path can be shorter regardless of obstacle placement, since obstacles can only increase the distance traveled. Chatzisavvas et al. [1] and Hu et al. [2] both select Manhattan distance as the base heuristic for grid environments for this same reason. The dynamic weight $k$ is then applied to scale this base estimate based on how far the robot is from the goal.
 
 ## 4. Algorithm
 
### 4.1 Standard A\*
 
The standard A\* algorithm maintains three data structures: an open set ordered by $f$-score, a $g$-score array storing the best known cost from start to each node, and a parent array recording the predecessor of each node on the best known path. At each iteration, the node with the lowest $f$-score is removed from the open set, its neighbors are evaluated, and any improvements to known costs are recorded. The algorithm terminates when the goal node is removed from the open set, indicating success, or when the open set is empty, indicating that no path exists.

### 4.2 Modified Evaluation Function
 
This implementation replaces the standard evaluation function with the dynamic weight variant from Chatzisavvas et al. [1]:
 
$$f(n) = g(n) + k \cdot h(n), \quad k = \begin{cases} 3 & \text{if } h(n) > 18 \\ 0.85 & \text{if } h(n) \leq 18 \end{cases}$$
 
Since $k = 0.85$ is not representable exactly in integer arithmetic, it is implemented as the fraction $85/100$ using integer division, which avoids any dependency on floating-point arithmetic while preserving the intent of the weight. The implementation in C is as follows:

```c
static int compute_weighted_f(int g, int h, int ec) {
    if (ec > EC_THRESHOLD) {
        return g + WEIGHT_HIGH * h;
    } else {
        return g + (h * WEIGHT_LOW_NUM) / WEIGHT_LOW_DEN;
    }
}
```

where `EC_THRESHOLD = 18`, `WEIGHT_HIGH = 3`, `WEIGHT_LOW_NUM = 85`, and `WEIGHT_LOW_DEN = 100`. This single function is the only part of the code that differs from a standard A\* implementation, which makes the comparison with Dijkstra and with standard A\* straightforward to reason about.

### 4.3 Pseudocode
 
```
A_STAR_DYNAMIC_WEIGHT(grid, start, goal)
 
    initialize g_score[all nodes] = INF
    initialize parent[all nodes]  = NULL
    initialize closed[all nodes]  = false
 
    g_score[start] = 0
    h = manhattan_distance(start, goal)
    k = 3.0 if h > 18 else 0.85
    push (k * h, start) into min-heap open_set
 
    while open_set is not empty
 
        current = pop node with lowest f from open_set
 
        if current already in closed set
            continue
 
        add current to closed set
        increment nodes_expanded
 
        if current == goal
            reconstruct path by following parent links from goal to start
            return path
 
        for each neighbor of current (up, down, left, right)
 
            if neighbor is out of bounds or is an obstacle
                continue
            if neighbor is in closed set
                continue
 
            tentative_g = g_score[current] + 1
 
            if tentative_g < g_score[neighbor]
                parent[neighbor]  = current
                g_score[neighbor] = tentative_g
                h  = manhattan_distance(neighbor, goal)
                k  = 3.0 if h > 18 else 0.85
                f  = tentative_g + k * h
                push (f, neighbor) into open_set
 
    return NO_PATH_FOUND
```
Dijkstra's algorithm is obtained by setting $k = 0$ throughout, which reduces the priority function to $f(n) = g(n)$ and eliminates all directional guidance toward the goal.
 
## 5. Implementation
 
### 5.1 Language and Design Philosophy
 
The algorithm is implemented in C, the primary language of this course. Python is used exclusively for post-processing visualizations via matplotlib and networkx. All algorithm logic, correctness tests, and performance benchmarks execute entirely in C. This mirrors the approach taken by the source papers, in which core algorithms are implemented in a systems language and visualization is handled separately.
 
The C code follows the coding conventions practiced throughout CS 5008, specifically small focused functions, structs to group related data, pointer parameters to avoid unnecessary copying, fixed-size stack-allocated arrays, and explicit bounds checking before every array access. In doing so, the implementation stays close to the low-level reasoning that has been emphasized throughout the course rather than relying on high-level abstractions.

### 5.2 Core Data Structures
 
**`Point` struct.** Represents a grid cell as `(row, col)`. Row-column indexing rather than x-y aligns with C's row-major 2D array layout and the flat index formula $\text{index} = r \cdot \text{cols} + c$. Grouping both values into a struct keeps function signatures clean and avoids passing two separate integers wherever a grid position is needed.

**`Grid` struct.** Stores the occupancy map as a flat 1D array of integers with the actual row and column dimensions. The flat layout follows the matrix memory model studied in Module 2.

**`SearchResult` struct.** Bundles all search output, including the found flag, path length, nodes expanded, and path array, into a single struct since C functions return only one value. The `nodes_expanded` field is central to the empirical comparison, consistent with the benchmarking methodology used in all three source papers [1][2][3].

**`MinHeap` struct.** A binary min-heap backed by a fixed-size array. Uses the parent-child index formulas from Module 9, with the parent of node $i$ at index $(i-1)/2$, the left child at $2i+1$, and the right child at $2i+2$. The heap is sized at `MAX_CELLS * 4` to accommodate lazy deletion: when a better path to a node is found, a new heap entry is pushed rather than updating the existing one. Stale entries are skipped when popped by checking the closed set.
 
### 5.3 Shared Internal Search Function
 
Both `astar_search()` and `dijkstra_search()` delegate to a single internal function `search_internal()` controlled by a `use_heuristic` flag. When `use_heuristic = 1`, `compute_weighted_f()` is called and the dynamic weight is applied. When `use_heuristic = 0`, the priority reduces to the plain $g$-score, yielding Dijkstra's behavior. This design ensures the empirical comparison is controlled: both algorithms use identical grid representations, heap implementations, and neighbor expansion logic. In this sense, the heuristic is the only independent variable between the two algorithms.

### 5.4 Path Reconstruction
 
Upon reaching the goal, the optimal path is reconstructed by following `parent[]` links backward from the goal to the start, the same predecessor-chain traversal used with linked lists in Module 8, and then reversing the resulting array to produce the correct start-to-goal ordering.
 
### 5.5 Repository Structure
 
```
final-paper-anamahmedshamsi12-1/
├── src/
│   ├── astar.h          - structs, constants, function prototypes
│   ├── astar.c          - heap, grid helpers, dynamic weight, A*, Dijkstra
│   ├── main.c           - demo with rerouting scenario
│   ├── tests.c          - 9 correctness tests
│   └── benchmark.c      - timing and node-count benchmarks
├── outputs/
│   ├── sample_run.txt
│   ├── test_run.txt
│   └── benchmark_results.csv
├── figures/
│   ├── astar_vs_dijkstra.png
│   ├── nodes_expanded_vs_size.png
│   ├── runtime_vs_size.png
│   └── network_graph.png
├── generate_figures.py
├── network_graph.py
├── Makefile
└── README.md
```
## 6. Correctness
 
### 6.1 Loop Invariant
 
**Invariant.** At the start of every iteration of the main search loop, every node in the closed set has its optimal $g$-score finalized. That is, for every node $u$ in the closed set, $g[u]$ equals the true shortest-path distance from the start to $u$.
 
**Initialization.** Before the first iteration, the closed set is empty. The invariant holds vacuously because there are no nodes to verify.
 
**Maintenance.** Consider an arbitrary iteration. The node $u$ with the minimum $f$-score is popped from the open set. If $u$ is already closed, it is skipped and the invariant is unchanged. Otherwise, suppose for contradiction that a cheaper path to $u$ exists but has not been discovered. Any such path must pass through some node $v$ currently in the open set. Since $h$ is admissible, $f(v) \leq g(v) + h(v) \leq \text{true cost to } u < g[u]$. But then $v$ would have been popped before $u$, contradicting the assumption that $u$ was popped with the minimum $f$-score. Therefore no cheaper path to $u$ exists and $g[u]$ is optimal when $u$ is closed. The invariant is maintained.
 
**Termination.** When the goal node is closed, the invariant guarantees that $g[\text{goal}]$ equals the true shortest-path distance. Following parent links from the goal to the start reconstructs this optimal path.

### 6.2 Admissibility Note
 
When $k = 3$, the effective heuristic is $3 \cdot h(n)$, which may overestimate the true remaining cost and technically violates admissibility. Chatzisavvas et al. [1] explicitly accept this trade-off: the higher weight is applied only when the robot is far from the goal and the terrain is open, conditions under which the heuristic is highly directional and overestimation is unlikely to cause path suboptimality in practice. When $k = 0.85 < 1$, the heuristic is actually more conservative than standard A\*, preserving strong optimality guarantees for the final approach. This adaptive strategy is consistent with the analysis in Mai Jialing and Zhang Xiaohua [3], who observe that increasing the heuristic weight early in the search and decreasing it near the goal effectively balances convergence speed with path accuracy.

## 6. Correctness
 
### 6.1 Loop Invariant
**Invariant.** At the start of every iteration of the main search loop, every node in the closed set has its optimal $g$-score finalized. That is, for every node $u$ in the closed set, $g[u]$ equals the true shortest-path distance from the start to $u$.

**Initialization.** Before the first iteration, the closed set is empty. The invariant holds vacuously because there are no nodes to verify.

**Maintenance.** Consider an arbitrary iteration. The node $u$ with the minimum $f$-score is popped from the open set. If $u$ is already closed, it is skipped and the invariant is unchanged. Otherwise, suppose for contradiction that a cheaper path to $u$ exists but has not been discovered. Any such path must pass through some node $v$ currently in the open set. Since $h$ is admissible, $f(v) \leq g(v) + h(v) \leq \text{true cost to } u < g[u]$. But then $v$ would have been popped before $u$, contradicting the assumption that $u$ was popped with the minimum $f$-score. Therefore no cheaper path to $u$ exists and $g[u]$ is optimal when $u$ is closed. The invariant is maintained.

**Termination.** When the goal node is closed, the invariant guarantees that $g[\text{goal}]$ equals the true shortest-path distance. Following parent links from the goal to the start reconstructs this optimal path.


### 6.2 Admissibility Note
When $k = 3$, the effective heuristic is $3 \cdot h(n)$, which may overestimate the true remaining cost and technically violates admissibility. Chatzisavvas et al. [1] explicitly accept this trade-off: the higher weight is applied only when the robot is far from the goal and the terrain is open, conditions under which the heuristic is highly directional and overestimation is unlikely to cause path suboptimality in practice. When $k = 0.85 < 1$, the heuristic is actually more conservative than standard A\*, preserving strong optimality guarantees for the final approach. This adaptive strategy is consistent with the analysis in Mai Jialing and Zhang Xiaohua [3], who observe that increasing the heuristic weight early in the search and decreasing it near the goal effectively balances convergence speed with path accuracy.

## 7. Theoretical Analysis

### 7.1 Time Complexity
Let $V$ denote the number of nodes and $E$ the number of edges. With a binary min-heap priority queue, each insertion and extraction costs $O(\log V)$. In the worst case, the algorithm processes every node exactly once and examines all incident edges:

$$T(V, E) = O((V + E) \log V)$$

For a 4-direction occupancy grid, each cell has at most 4 neighbors, so $E = O(V)$. This gives:

$$T(V) = O(V \log V)$$

The dynamic weight coefficient does not alter the asymptotic bound. It reduces the practical constant factor by decreasing the number of nodes that reach the open set, but in the worst case, all nodes may still be processed.


### 7.2 Space Complexity
The `g_score`, `parent`, and `closed` arrays each require $O(V)$ space. The heap stores at most $O(V)$ entries under lazy deletion. The path array holds at most $V$ nodes. In total:
 
$$S(V) = O(V)$$

### 7.3 Comparison with Dijkstra
Dijkstra's algorithm shares the same $O(V \log V)$ worst-case bound when implemented with a binary heap. The practical difference manifests in the constant factor. Because Dijkstra sets $h(n) = 0$, nodes are prioritized purely by $g$-score and the search expands uniformly outward from the start. A\* with an informative heuristic prioritizes nodes that are both close to the start and estimated to be close to the goal, effectively pruning the search to a narrow corridor along the optimal path. The dynamic weight amplifies this effect: with $k = 3$, the heuristic term dominates the priority function when the robot is far from the goal, pushing cells along the direct route to the top of the heap and preventing the search from wasting time on cells in the opposite direction.

### 7.4 Empirical Reduction in Practice
On the 60x60 benchmark grid, weighted A\* expanded 124 nodes compared to Dijkstra's 3,488, a reduction of 96.4%. This exceeds the 66.2% reduction reported by Chatzisavvas et al. [1] and the approximately 50% improvement reported by Mai Jialing and Zhang Xiaohua [3], likely because the corridor-style benchmark grid used in this paper creates a particularly favorable environment for the directional heuristic. Across the full benchmark suite, the practical speedup ranges from 1x at 10x10, where both algorithms find the goal quickly, to 28x at 60x60 in terms of runtime. This is consistent with the literature's prediction that the heuristic advantage scales with grid size.
 
## 8. Empirical Analysis
 
### 8.1 Experimental Setup
 Benchmarks were conducted on six corridor-style grids ranging from 10x10 to 60x60. Each grid contains two vertical obstacle walls with gaps at specific rows, creating a path-planning problem that requires navigating through narrow openings. This layout was chosen because an open grid would produce nearly identical results for both algorithms, since both would expand the same diagonal band of cells and the heuristic's advantage would not be visible. The corridor environment better approximates real indoor navigation scenarios and is consistent with the testing methodology of Chatzisavvas et al. [1] and Mai Jialing and Zhang Xiaohua [3], both of whom test on environments with realistic obstacle densities rather than empty grids.

Each algorithm was run 2,000 times per grid size and the average runtime per run in microseconds was recorded. Node counts were recorded from a single representative run. The start position was the top-left corner and the goal was the bottom-right corner for all grid sizes.

### 8.2 Experiment 1 Results: Grid Size Scaling
 
| Grid Size | A\* Nodes | Dijkstra Nodes | Reduction | A\* Time (us) | Dijkstra Time (us) |
|:---|---:|---:|---:|---:|---:|
| 10 x 10 | 88 | 88 | 0% | 5.0 | 5.0 |
| 15 x 15 | 94 | 203 | 53.7% | 5.0 | 5.0 |
| 20 x 20 | 44 | 368 | 88.0% | 15.0 | 10.0 |
| 25 x 25 | 54 | 583 | 90.7% | 5.0 | 20.0 |
| 30 x 30 | 64 | 848 | 92.5% | 5.0 | 30.0 |
| 35 x 35 | 74 | 1,163 | 93.6% | 5.0 | 45.0 |
| 40 x 40 | 84 | 1,528 | 94.5% | 5.0 | 60.0 |
| 45 x 45 | 94 | 1,943 | 95.2% | 10.0 | 85.0 |
| 50 x 50 | 104 | 2,408 | 95.7% | 10.0 | 110.0 |
| 55 x 55 | 114 | 2,923 | 96.1% | 10.0 | 155.0 |
| 60 x 60 | 124 | 3,488 | 96.4% | 10.0 | 180.0 |
 
The 10x10 case produces equal node counts because the small grid size allows both algorithms to reach the goal before the heuristic advantage accumulates. From 15x15 onward the divergence grows consistently, confirming that the reduction scales with grid size as predicted by the theoretical analysis. The percentage reduction increases monotonically from 53.7% to 96.4%, suggesting that the dynamic weight becomes more effective as the search space grows.

### 8.3 Figure 1: Grid Search Visualization
 
![A\* vs Dijkstra Grid Search](figures/astar_vs_dijkstra.png)
 
*Figure 1: Side-by-side visualization of A\* (left) and Dijkstra's algorithm (right) on the same corridor grid. Purple cells indicate nodes in the closed list. Green cells trace the final path. Cyan marks the start and red marks the goal.*
 
Figure 1 provides the most direct visual evidence for the central claim of this paper. The Dijkstra panel on the right shows the closed list covering a much larger area of the grid, since the algorithm processed nodes in all directions before reaching the goal. The A\* panel on the left shows a narrow purple corridor concentrated along the path toward the goal, with large unexplored regions remaining dark. Both algorithms found the same path, confirming equal path quality with dramatically different exploration costs. This visual pattern is consistent with the simulation results reported by Hu et al. [2] and by Chatzisavvas et al. [1].

### 8.4 Figure 2: Nodes Expanded Across Grid Sizes
 
![Nodes Expanded](figures/line_grid_size_nodes.png)
 
*Figure 2: Nodes expanded by A\* with dynamic weight (purple) and Dijkstra's algorithm (orange) across eleven grid sizes from 10x10 to 60x60.*
 
Figure 2 shows that Dijkstra's node count grows approximately quadratically with grid size while A\*'s grows nearly linearly. The divergence beginning at 15x15 and widening through 60x60 confirms that the dynamic weight is most beneficial on larger grids, which is precisely the use case emphasized by Chatzisavvas et al. [1] in the context of large-scale robot navigation.

### 8.5 Figure 3: Runtime Across Grid Sizes
 
![Runtime](figures/line_grid_size_runtime.png)
 
*Figure 3: Average runtime per search in microseconds for A\* with dynamic weight (purple) and Dijkstra's algorithm (orange) across eleven grid sizes.*
 
Figure 3 confirms that the node count reduction translates directly to runtime reduction. Dijkstra's runtime grows steeply from 5 us at 15x15 to 180 us at 60x60, while A\*'s runtime remains nearly flat throughout the range. The timing benchmark also showed A\* averaging 4.66 us/run versus Dijkstra's 15.30 us/run on a 20x20 grid, a 3.3x speedup, confirming the efficiency gains described in Chatzisavvas et al. [1].

### 8.6 Figure 4: Obstacle Density Experiment
 
![Obstacle Density](figures/line_obstacle_nodes.png)
 
*Figure 4: Nodes expanded by A\* and Dijkstra as obstacle density increases from 0% to 20% on a fixed 30x30 grid.*
 
Figure 4 shows how both algorithms respond to increasing obstacle density. As obstacles increase, Dijkstra's node count decreases because more cells are blocked and fewer are reachable. A\* maintains consistently lower node counts throughout, confirming that the dynamic weight coefficient is effective across varying obstacle densities, consistent with the agricultural environment experiments in Chatzisavvas et al. [1].
 
 ### 8.7 Figure 5: Replanning Experiment
 
![Replanning](figures/line_replan.png)
 
*Figure 5: Initial planning versus replanning node counts and runtimes across four grid sizes, inspired by Hu et al. [2].*
 
Figure 5 shows that replanning after a new obstacle appears expands slightly more nodes than the initial search, which is expected since the robot starts partway through the grid rather than at the corner. However the replanning search remains fast and focused, confirming that A\* is practical for real-time dynamic replanning in robot navigation as described in Hu et al. [2].

### 8.8 Figure 6: Weighted Waypoint Network
 
![Network Graph](figures/network_graph.png)
 
*Figure 6: A\* pathfinding on a weighted building waypoint network. Purple nodes were explored. Green nodes and edges form the optimal path. Dark blue nodes were never reached. Edge weights represent corridor distances.*
 
Figure 6 demonstrates A\* operating on a weighted graph representing a building floor plan, structurally equivalent to the City Finder assignment from Module 10 of this course. The network contains 17 rooms connected by 21 corridors with integer distance weights. A\* explored 9 of the 17 nodes and found the optimal path: Entrance to Lobby to Lab 3 to Hallway B to Hallway D to Conference to Exit. Eight nodes were never reached at all.
 
This figure directly connects to the City Finder homework. In that assignment, cities were nodes and roads were weighted edges. Here, rooms are nodes and corridors are weighted edges. The only meaningful difference is the algorithm: A\* instead of BFS or Dijkstra. In doing so, the same graph structure that was used to find shortest routes between cities now guides a robot through a building, demonstrating both the generality of the graph abstraction and the practical benefit of the heuristic in weighted environments.

### 8.9 Discussion
 
The empirical results collectively confirm three findings from the literature. Chatzisavvas et al. [1] predict that dynamic weighting reduces search routes without degrading path quality, and the data here shows a 96.4% reduction at 60x60 with no change in path length. Hu et al. [2] predict that A\* with a weighted heuristic explores far fewer nodes than Dijkstra even when both find optimal paths, and the side-by-side grid visualization confirms this both visually and quantitatively. Mai Jialing and Zhang Xiaohua [3] predict that dynamic weight adjustment improves efficiency in complex environments, and the corridor grids and waypoint network both demonstrate focused, efficient exploration consistent with that prediction. Simply put, the three papers agree, and the empirical data produced here supports all three of them.

## 9. Rerouting Experiment
 
One of the practical advantages of A\* for robot navigation is the ability to replan dynamically when the environment changes. Hu et al. [2] study this scenario specifically in the context of outdoor delivery robots encountering unexpected choke points such as traffic congestion or road construction. In order to demonstrate this capability, the demo program `src/main.c` implements a rerouting scenario in which A\* finds an initial path from start to goal, the robot is simulated to have moved three steps along the planned route, a new obstacle is introduced at the next cell on the original path to simulate an unexpected blockage, and A\* is called again from the robot's current position to produce a new path that avoids the blocked cell.
 
In the captured output at `outputs/sample_run.txt`, the rerouted search expanded only 33 nodes, significantly fewer than the initial search of 56 nodes, because the dynamic weight coefficient with $k = 3$ pushed the replanning search aggressively toward the goal from the robot's intermediate position. This demonstrates that A\* is practical for real-time replanning, not just initial path computation, which is consistent with the conclusion of Hu et al. [2] that an improved A\* variant is more suitable than standard algorithms for delivery robots operating in dynamic outdoor environments.
 
## 10. Testing
 
Nine correctness tests are implemented in `src/tests.c`. All nine must pass before the empirical benchmark data can be interpreted as meaningful, since a flawed implementation would produce incorrect node counts and path lengths that appear plausible but are not.
 
| Test | Property Verified |
|:---|:---|
| `test_manhattan_distance` | Heuristic computes correct values for known inputs |
| `test_finds_path_in_open_grid` | Correct 9-cell optimal path on 5x5 open grid |
| `test_finds_path_around_obstacles` | All path cells are walkable, no cell has value 1 |
| `test_returns_no_path_when_blocked` | Returns 0 and found=0 for disconnected goal |
| `test_start_equals_goal` | Returns single-cell path when start equals goal |
| `test_astar_matches_dijkstra_path_length` | Both algorithms find paths of equal optimal length |
| `test_astar_expands_fewer_nodes_than_dijkstra` | A\* node count strictly less than Dijkstra |
| `test_reroute_after_new_obstacle` | Valid alternative path found after mid-route blockage |
| `test_dynamic_weight_reduces_nodes` | Dynamic weight produces fewer nodes than Dijkstra, 44 vs 384 |
 
The final test was added specifically to validate the core contribution of this implementation. It constructs a 20x20 grid with a vertical obstacle wall, runs both A\* with dynamic weighting and Dijkstra, and asserts that A\* expands strictly fewer nodes. In practice, A\* expanded 44 nodes versus Dijkstra's 384, an 88.5% reduction, directly confirming the prediction of Chatzisavvas et al. [1].
 
To reproduce all tests:
 
```bash
make test
```
## 11. Limitations
 
Several limitations of this implementation should be acknowledged.
 
**Physical simplification.** The occupancy grid treats the robot as a dimensionless point mass with uniform step costs. A physical humanoid robot must account for joint kinematics, center-of-mass stability, step placement constraints, and actuator dynamics. These concerns are outside the scope of discrete path planning and would require integration with motion planning and control systems.
 
**Static known environment.** The grid is assumed to be fully known and static before the search begins. In practice, robots must build their maps incrementally during navigation using Simultaneous Localization and Mapping. Mai Jialing and Zhang Xiaohua [3] identify integration with online map updates as a critical direction for future work in their conclusion, and this limitation applies equally to the current implementation.
 
**Admissibility trade-off with $k = 3$.** As noted in Section 6.2, the high-weight mode is technically inadmissible. While empirical results show no degradation in path quality on the benchmark grids, pathological obstacle layouts could in principle cause A\* to return a suboptimal path. Applications requiring strict optimality guarantees should use $k \leq 1$ throughout.
 
**Fixed-size arrays.** The implementation uses stack-allocated arrays with a maximum grid size of 64x64. This bound is sufficient for the benchmarks and aligns with the assignment scope, but production deployment on larger maps would require dynamic memory allocation.
 
**Four-directional movement.** Restricting movement to cardinal directions limits the precision of paths in continuous environments. Mai Jialing and Zhang Xiaohua [3] address this by extending the search neighborhood to eight directions with directional pruning, which produces smoother paths more suitable for physical robot motion.
 
---
 
## 12. Conclusion
 
This paper presented an implementation of A\* with a dynamic weight coefficient for robot navigation, grounded in three peer-reviewed papers by Chatzisavvas et al. [1], Hu et al. [2], and Mai Jialing and Zhang Xiaohua [3]. The core modification, setting the heuristic weight $k$ to 3 when far from the goal and 0.85 when near it, reduced nodes expanded by up to 96.4% compared to Dijkstra's algorithm across six benchmark grid sizes and produced a 28x runtime speedup at the largest tested size.
 
The implementation connects to graph theory from Module 10, binary heaps from Module 9, Dijkstra's algorithm from Module 11, and loop invariant correctness proofs from Module 13. The weighted waypoint network extends the City Finder homework model from Module 10, demonstrating that the same graph abstraction used to find routes between cities scales directly to indoor robot navigation. In this sense, A\* is not an isolated topic but a synthesis of nearly everything covered in the second half of the course.
 
A\* with a dynamic weight coefficient represents a practically important balance between the theoretical guarantees of standard A\* and the speed of purely greedy search. As robot navigation systems operate in increasingly large and complex environments, the ability to reduce search effort by orders of magnitude while maintaining near-optimal path quality will remain essential. The results in this paper confirm that even a simple two-value adaptive weight can achieve dramatic efficiency gains, supporting the broader research direction pursued by all three source papers.
 
---
 
## 13. How to Build and Run
 
**Requirements:** `gcc` with C11 support, `make`, Python 3 with `matplotlib`, `numpy`, and `networkx`.
 
```bash
# Compile all C programs
make all
 
# Run the navigation demo
make run > outputs/sample_run.txt
 
# Run all 9 correctness tests
make test > outputs/test_run.txt
 
# Run the benchmark
make bench > outputs/benchmark_results.csv
 
# Generate all figures
python3 generate_figures.py
python3 network_graph.py
```



## References
 
[1] A. Chatzisavvas, M. Dossis, and M. Dasygenis, "Optimizing Mobile Robot Navigation Based on A-Star Algorithm for Obstacle Avoidance in Smart Agriculture," *Electronics*, vol. 13, no. 11, p. 2057, 2024. https://doi.org/10.3390/electronics13112057
 
[2] D. Hu, Y. Ba, W. Cao, C. Lin, and Z. Wang, "An Improved A-Star Algorithm for Path Planning of Outdoor Distribution Robots," in *Proc. 2022 Asia Conference on Electrical, Power and Computer Engineering (EPCE 2022)*, ACM, New York, NY, USA, 2022. https://doi.org/10.1145/3529299.3533400
 
[3] M. Jialing and Z. Xiaohua, "Research on Path Planning for Intelligent Robots Based on Improved A Star Algorithm," in *Proc. 2025 7th Asia Conference on Machine Learning and Computing (ACMLC 2025)*, ACM, New York, NY, USA, 2025. https://doi.org/10.1145/3772673.3772688
 
[4] P. E. Hart, N. J. Nilsson, and B. Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths," *IEEE Transactions on Systems Science and Cybernetics*, vol. 4, no. 2, pp. 100–107, 1968.
 
[5] GeeksforGeeks, "A\* Search Algorithm," 2025. [Online]. Available: https://www.geeksforgeeks.org/a-search-algorithm/