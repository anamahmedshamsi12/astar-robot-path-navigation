"""
network_graph.py

Generates a weighted waypoint network graph showing A* pathfinding
on a building floor plan. Nodes represent rooms or waypoints a robot
must navigate between. Edge weights represent corridor distances.

This visualization demonstrates A* operating on a weighted graph
rather than a grid, which is structurally identical to the City Finder
assignment from Module 10 of CS 5008. In that assignment, cities were
nodes and roads were weighted edges. Here, rooms are nodes and
corridors are weighted edges. The only difference is the algorithm:
A* with dynamic weight coefficient instead of BFS or Dijkstra.

The dynamic weight coefficient from Chatzisavvas et al. [1] is applied
here using Euclidean distance as the heuristic instead of Manhattan
distance, since node positions are continuous (x, y) coordinates
rather than discrete grid cells.

The figure shows:
  - All nodes and edges in the building network
  - Explored nodes (purple) -- rooms A* processed during the search
  - Unexplored nodes (dark blue) -- rooms never reached
  - Final path edges (bright green) -- the optimal route
  - Start node (cyan) and goal node (red)

To run: python3 network_graph.py
Output: figures/network_graph.png
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import networkx as nx
import heapq
import os

os.makedirs("figures", exist_ok=True)

"""
Building waypoint network definition.

Each node is a named room with a (x, y) position used for graph layout.
Positions are chosen to roughly match a building floor plan where the
Entrance is at the bottom left and the Exit is at the top right.

Edges connect adjacent rooms with an integer distance weight representing
the estimated corridor length between them. This is the same weighted
graph structure used in the City Finder homework from Module 10.
"""
nodes = {
    "Entrance":     (1.0, 0.0),
    "Lobby":        (2.5, 0.5),
    "Hallway A":    (2.5, 2.0),
    "Hallway B":    (4.5, 2.0),
    "Hallway C":    (2.5, 4.0),
    "Hallway D":    (4.5, 4.0),
    "Hallway E":    (6.5, 2.0),
    "Lab 1":        (1.0, 2.5),
    "Lab 2":        (1.0, 4.5),
    "Lab 3":        (4.5, 0.5),
    "Office 1":     (6.5, 0.5),
    "Office 2":     (6.5, 4.0),
    "Conference":   (4.5, 5.5),
    "Storage":      (2.5, 5.5),
    "Charging":     (8.0, 2.0),
    "Server Room":  (8.0, 4.0),
    "Exit":         (8.0, 5.5),
}

"""
Edge list: each tuple is (room_a, room_b, distance_weight).
The distance weight represents how far apart two connected rooms are.
A* uses these weights as the actual cost g(n) when computing
tentative path costs during the search.
"""
edges = [
    ("Entrance",    "Lobby",       2),
    ("Lobby",       "Hallway A",   2),
    ("Lobby",       "Lab 3",       3),
    ("Hallway A",   "Lab 1",       2),
    ("Hallway A",   "Hallway B",   3),
    ("Hallway A",   "Hallway C",   2),
    ("Hallway B",   "Lab 3",       2),
    ("Hallway B",   "Hallway E",   3),
    ("Hallway B",   "Hallway D",   2),
    ("Hallway C",   "Lab 2",       2),
    ("Hallway C",   "Hallway D",   3),
    ("Hallway C",   "Storage",     2),
    ("Hallway D",   "Conference",  2),
    ("Hallway D",   "Office 2",    3),
    ("Hallway E",   "Office 1",    2),
    ("Hallway E",   "Charging",    2),
    ("Hallway E",   "Office 2",    3),
    ("Office 2",    "Server Room", 2),
    ("Server Room", "Exit",        2),
    ("Conference",  "Exit",        3),
    ("Lab 3",       "Office 1",    3),
    ("Storage",     "Conference",  2),
    ("Charging",    "Server Room", 3),
]

START = "Entrance"
GOAL  = "Exit"

"""
Dynamic weight coefficient constants from Chatzisavvas et al. [1].
EC_THRESHOLD is the boundary between aggressive and cautious search modes.
When the estimated cost to the goal exceeds this value, k = WEIGHT_HIGH.
When at or below this value, k = WEIGHT_LOW.
The threshold is set lower here (5 vs 18 in the grid version) because
the waypoint network is much smaller than a 20x20 grid and Euclidean
distances between nodes are in the range of 1 to 8 units.
"""
EC_THRESHOLD = 5
WEIGHT_HIGH  = 3
WEIGHT_LOW   = 0.85

"""
Build the networkx graph from the node and edge definitions above.
networkx provides graph data structures and drawing utilities.
We use an undirected Graph since the robot can traverse corridors
in either direction, matching the 4-direction grid model where
movement is symmetric in all directions.
"""
G = nx.Graph()
for name, pos in nodes.items():
    G.add_node(name, pos=pos)
for u, v, w in edges:
    G.add_edge(u, v, weight=w)

pos = nx.get_node_attributes(G, 'pos')


def euclidean(a, b):
    """
    Computes the Euclidean distance between two named nodes.
    Used as the heuristic h(n) for A* on this network graph.

    For a grid the heuristic is Manhattan distance since movement
    is restricted to cardinal directions. For a continuous waypoint
    network where nodes can be in any spatial arrangement, Euclidean
    distance is the appropriate admissible heuristic because the
    straight-line distance between two points is always less than or
    equal to the shortest path through the network.

    Parameters:
        a: name of the first node (string key in the nodes dict)
        b: name of the second node (string key in the nodes dict)

    Returns:
        float: straight-line Euclidean distance between the two nodes
    """
    ax, ay = nodes[a]
    bx, by = nodes[b]
    return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5


def astar_network(graph, start, goal):
    """
    Runs A* with the dynamic weight coefficient on the waypoint network.

    This is the same algorithm as in astar.c but adapted for a weighted
    graph instead of a grid. The key differences are:
      - Neighbors come from graph.neighbors() instead of the 4-direction array
      - Edge costs come from graph[u][v]['weight'] instead of a fixed step cost of 1
      - The heuristic is Euclidean distance instead of Manhattan distance

    The dynamic weight coefficient is applied the same way as in the C
    implementation: k = WEIGHT_HIGH when the estimated cost exceeds
    EC_THRESHOLD, and k = WEIGHT_LOW otherwise.

    Parameters:
        graph: the networkx Graph representing the building
        start: name of the starting room (string)
        goal:  name of the destination room (string)

    Returns:
        path:     list of room names from start to goal in order
        explored: list of all rooms processed during the search
    """
    open_heap = []
    g_score   = {start: 0}
    came_from = {}
    closed    = set()
    explored  = []

    # Seed the heap with the start node using the dynamic weight heuristic.
    # The initial priority is k * h(start, goal) since g(start) = 0.
    h0 = euclidean(start, goal)
    k  = WEIGHT_HIGH if h0 > EC_THRESHOLD else WEIGHT_LOW
    heapq.heappush(open_heap, (k * h0, start))

    while open_heap:
        f, current = heapq.heappop(open_heap)

        # Skip stale heap entries using lazy deletion.
        # The same node may appear multiple times with different priorities
        # as better paths are found. The closed set detects duplicates.
        if current in closed:
            continue

        closed.add(current)
        explored.append(current)

        # Goal reached: reconstruct the path by following came_from links
        # backward from the goal to the start, then reverse the list.
        # This is the same predecessor-chain traversal used in astar.c.
        if current == goal:
            path = []
            node = goal
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path, explored

        # Expand all neighbors connected by edges in the graph.
        # For each neighbor, compute the tentative cost through current
        # and update if it improves the best known cost to that neighbor.
        for neighbor in graph.neighbors(current):
            if neighbor in closed:
                continue

            # The actual cost of this edge from the edge weight attribute.
            edge_w = graph[current][neighbor]['weight']
            tg     = g_score[current] + edge_w

            if tg < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor]   = tg

                # Apply the dynamic weight coefficient from Chatzisavvas et al. [1].
                # EC is the Euclidean distance from this neighbor to the goal.
                # If EC > EC_THRESHOLD the robot is far away so use k = 3.
                # If EC <= EC_THRESHOLD the robot is close so use k = 0.85.
                h_val = euclidean(neighbor, goal)
                k     = WEIGHT_HIGH if h_val > EC_THRESHOLD else WEIGHT_LOW
                f_val = tg + k * h_val
                heapq.heappush(open_heap, (f_val, neighbor))

    # If the heap empties without reaching the goal, no path exists.
    return [], explored


# Run A* on the building network and collect results.
path, explored = astar_network(G, START, GOAL)
path_edges     = list(zip(path[:-1], path[1:]))
path_set       = set(path)

"""
Color scheme matches the dark style used in the grid visualizations
so all figures in the report look consistent.
Purple = explored nodes, green = path, cyan = start, red = goal.
"""
COL_BG       = '#0a0a14'
COL_NODE     = '#16213e'
COL_EXPLORED = '#7c4dff'
COL_PATH_N   = '#00e676'
COL_START    = '#00b0ff'
COL_GOAL     = '#ff1744'
COL_EDGE     = '#2a2a4a'
COL_PATH_E   = '#00e676'
COL_LABEL    = '#cccccc'

"""
Assign a color and size to each node based on its role in the search.
Start and goal nodes are larger so they stand out in the figure.
Nodes on the optimal path are green.
Nodes that were explored but not on the path are purple.
Nodes that were never reached remain dark blue.
"""
node_colors = []
node_sizes  = []
for n in G.nodes():
    if n == START:
        node_colors.append(COL_START)
        node_sizes.append(600)
    elif n == GOAL:
        node_colors.append(COL_GOAL)
        node_sizes.append(600)
    elif n in path_set:
        node_colors.append(COL_PATH_N)
        node_sizes.append(500)
    elif n in explored:
        node_colors.append(COL_EXPLORED)
        node_sizes.append(400)
    else:
        node_colors.append(COL_NODE)
        node_sizes.append(350)

"""
Assign a color and width to each edge based on whether it is part
of the optimal path. Path edges are drawn thicker and in bright green
so the route is immediately visible against the dark background.
"""
edge_colors = []
edge_widths = []
for u, v in G.edges():
    if (u, v) in path_edges or (v, u) in path_edges:
        edge_colors.append(COL_PATH_E)
        edge_widths.append(4.0)
    else:
        edge_colors.append(COL_EDGE)
        edge_widths.append(1.2)

# Draw the graph using networkx and matplotlib.
# The pos dictionary provides the (x, y) layout coordinates for each node.
fig, ax = plt.subplots(figsize=(13, 9), facecolor=COL_BG)
ax.set_facecolor(COL_BG)

nx.draw_networkx_edges(G, pos, ax=ax,
                       edge_color=edge_colors,
                       width=edge_widths,
                       alpha=0.9)

nx.draw_networkx_nodes(G, pos, ax=ax,
                       node_color=node_colors,
                       node_size=node_sizes)

nx.draw_networkx_labels(G, pos, ax=ax,
                        font_size=7.5,
                        font_color=COL_LABEL,
                        font_weight='bold')

# Draw edge weight labels so the reader can see the corridor distances
# that A* uses when computing the actual cost g(n) for each path.
edge_labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels,
                              ax=ax,
                              font_size=7,
                              font_color='#888888',
                              bbox=dict(boxstyle='round,pad=0.1',
                                        fc=COL_BG, ec='none', alpha=0.7))

ax.set_title(
    f"A* Pathfinding on Building Waypoint Network\n"
    f"Robot navigates from {START} to {GOAL}  |  "
    f"Path length: {len(path)} waypoints  |  "
    f"Nodes explored: {len(explored)} of {len(G.nodes())}",
    color='white', fontsize=12, fontweight='bold', pad=14
)

legend_items = [
    mpatches.Patch(color=COL_START,    label=f'Start ({START})'),
    mpatches.Patch(color=COL_GOAL,     label=f'Goal ({GOAL})'),
    mpatches.Patch(color=COL_PATH_N,   label='On optimal path'),
    mpatches.Patch(color=COL_EXPLORED, label='Explored by A*'),
    mpatches.Patch(color=COL_NODE,     label='Not reached'),
    mpatches.Patch(color=COL_PATH_E,   label='Path edges'),
]
ax.legend(handles=legend_items, loc='lower left',
          facecolor='#1a1a2e', edgecolor='none',
          labelcolor='white', fontsize=9,
          framealpha=0.85)

ax.axis('off')
plt.tight_layout()
plt.savefig('figures/network_graph.png', dpi=150,
            bbox_inches='tight', facecolor=COL_BG)
plt.close()
print(f"Saved figures/network_graph.png")
print(f"Path found: {' -> '.join(path)}")
print(f"Nodes explored: {len(explored)} / {len(G.nodes())}")