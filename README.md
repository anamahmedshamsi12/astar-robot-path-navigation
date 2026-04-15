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




## References
 
[1] A. Chatzisavvas, M. Dossis, and M. Dasygenis, "Optimizing Mobile Robot Navigation Based on A-Star Algorithm for Obstacle Avoidance in Smart Agriculture," *Electronics*, vol. 13, no. 11, p. 2057, 2024. https://doi.org/10.3390/electronics13112057
 
[2] D. Hu, Y. Ba, W. Cao, C. Lin, and Z. Wang, "An Improved A-Star Algorithm for Path Planning of Outdoor Distribution Robots," in *Proc. 2022 Asia Conference on Electrical, Power and Computer Engineering (EPCE 2022)*, ACM, New York, NY, USA, 2022. https://doi.org/10.1145/3529299.3533400
 
[3] M. Jialing and Z. Xiaohua, "Research on Path Planning for Intelligent Robots Based on Improved A Star Algorithm," in *Proc. 2025 7th Asia Conference on Machine Learning and Computing (ACMLC 2025)*, ACM, New York, NY, USA, 2025. https://doi.org/10.1145/3772673.3772688
 
[4] P. E. Hart, N. J. Nilsson, and B. Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths," *IEEE Transactions on Systems Science and Cybernetics*, vol. 4, no. 2, pp. 100–107, 1968.
 
[5] GeeksforGeeks, "A\* Search Algorithm," 2025. [Online]. Available: https://www.geeksforgeeks.org/a-search-algorithm/