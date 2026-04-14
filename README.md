# Research Paper
* Name: Anam Shamsi
* Semester: Spring 2026
* Topic: 



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
