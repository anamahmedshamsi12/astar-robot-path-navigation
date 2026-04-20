"""
network_graph.py

Generates a weighted waypoint network graph showing A* pathfinding
on a building floor plan. Nodes represent rooms/waypoints a humanoid
robot must navigate between. Edge weights represent corridor distances.

A* with dynamic weight coefficient from Chatzisavvas et al. [1] finds
the shortest path. The figure shows:
  - All nodes and edges in the building network
  - Explored nodes (purple) — cells A* processed
  - Unexplored nodes (dark) — never reached
  - Final path edges (bright green) — the optimal route
  - Start node (cyan) and goal node (red)
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import networkx as nx
import heapq
import os

os.makedirs("figures", exist_ok=True)

# ── Building waypoint network ──────────────────────────────────────────────────
# Each node is a room/waypoint with a (x, y) position for layout.
# Edges connect adjacent rooms with a distance weight.

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

# ── Build graph ────────────────────────────────────────────────────────────────
G = nx.Graph()
for name, pos in nodes.items():
    G.add_node(name, pos=pos)
for u, v, w in edges:
    G.add_edge(u, v, weight=w)

pos = nx.get_node_attributes(G, 'pos')

# ── A* on the network graph ────────────────────────────────────────────────────
EC_THRESHOLD = 5
WEIGHT_HIGH  = 3
WEIGHT_LOW   = 0.85

def euclidean(a, b):
    ax, ay = nodes[a]
    bx, by = nodes[b]
    return ((ax-bx)**2 + (ay-by)**2) ** 0.5

def astar_network(graph, start, goal):
    open_heap = []
    g_score   = {start: 0}
    came_from = {}
    closed    = set()
    explored  = []

    h0 = euclidean(start, goal)
    k  = WEIGHT_HIGH if h0 > EC_THRESHOLD else WEIGHT_LOW
    heapq.heappush(open_heap, (k * h0, start))

    while open_heap:
        f, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        explored.append(current)

        if current == goal:
            path = []
            node = goal
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path, explored

        for neighbor in graph.neighbors(current):
            if neighbor in closed:
                continue
            edge_w = graph[current][neighbor]['weight']
            tg     = g_score[current] + edge_w
            if tg < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor]   = tg
                h_val = euclidean(neighbor, goal)
                k     = WEIGHT_HIGH if h_val > EC_THRESHOLD else WEIGHT_LOW
                f_val = tg + k * h_val
                heapq.heappush(open_heap, (f_val, neighbor))

    return [], explored

path, explored = astar_network(G, START, GOAL)
path_edges = list(zip(path[:-1], path[1:]))
path_set   = set(path)

# ── Colors ─────────────────────────────────────────────────────────────────────
COL_BG       = '#0a0a14'
COL_NODE     = '#16213e'
COL_EXPLORED = '#7c4dff'
COL_PATH_N   = '#00e676'
COL_START    = '#00b0ff'
COL_GOAL     = '#ff1744'
COL_EDGE     = '#2a2a4a'
COL_PATH_E   = '#00e676'
COL_LABEL    = '#cccccc'

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

edge_colors = []
edge_widths = []
for u, v in G.edges():
    if (u, v) in path_edges or (v, u) in path_edges:
        edge_colors.append(COL_PATH_E)
        edge_widths.append(4.0)
    else:
        edge_colors.append(COL_EDGE)
        edge_widths.append(1.2)

# ── Draw ───────────────────────────────────────────────────────────────────────
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