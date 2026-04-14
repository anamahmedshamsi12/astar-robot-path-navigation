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

This project implements the A\* Search Algorithm in C and applies it to robot navigation on both a 2D occupancy grid and a weighted waypoint network. The core idea is that a robot needs to find the shortest collision-free path from a starting location to a goal location in an environment that contains obstacles such as walls, furniture, or restricted zones.

What makes this implementation different from a standard A\* is the inclusion of a **dynamic weight coefficient** on the heuristic function, taken directly from the research by Chatzisavvas et al. [1]. In standard A\*, the evaluation function is:

$$f(n) = g(n) + h(n)$$

This implementation uses:

$$f(n) = g(n) + k \cdot h(n)$$

where $k$ is chosen dynamically based on how far the robot is from the goal. When the robot is far away, $k = 3$ to push the search aggressively toward the goal. When the robot is close, $k = 0.85$ to be more cautious and accurate. This single change significantly reduces the number of nodes the algorithm has to explore, which is critical for real-time robot navigation where replanning must happen quickly.

The project is grounded in three peer-reviewed ACM and MDPI papers, all of which study improved variants of A\* for robot path planning. The algorithm is implemented in C, tested with 9 correctness tests, benchmarked empirically across six grid sizes, and visualized using matplotlib and networkx in Python.

---

## 2. Why I Chose This Topic

I chose A\* for three reasons.

First, it connects directly to algorithms and data structures covered in this course. A\* is built on graphs (Module 10), uses a binary min-heap as a priority queue (Module 9), applies the same greedy selection principle as Dijkstra's algorithm (Module 11), and can be proven correct using a loop invariant (Module 13). Studying it meant revisiting almost every major topic from the second half of the course in a unified context.

Second, the connection to the City Finder homework from Module 10 was immediately obvious. In that assignment, cities were nodes, roads were weighted edges, and the task was to find a path between two cities. Robot navigation is the exact same problem — rooms or waypoints replace cities, corridors replace roads, and the goal is still to find the shortest path. A\* improves on that assignment's approach by adding a heuristic that focuses the search toward the goal instead of exploring in all directions blindly.

Third, all three of my source papers are recent ACM and MDPI publications that study A\* specifically in the context of robot navigation, which gave me high-quality academic grounding for both the background section and the empirical analysis.

---

