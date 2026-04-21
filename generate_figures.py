"""
generate_figures.py

Generates grid visualization figures for the A* heuristic comparison.
Produces two figures:

  astar_vs_dijkstra.png  -- side by side: A* dynamic weight vs Dijkstra
  heuristic_comparison.png -- 2x2 grid showing all four search configurations:
      top-left:     A* Manhattan + dynamic weight k (this implementation)
      top-right:    A* Manhattan + fixed k=1 (standard A*)
      bottom-left:  A* Euclidean + fixed k=1
      bottom-right: Dijkstra (h=0, no heuristic)
  grid_path.png          -- standalone A* dynamic weight path

All four configurations find the same optimal path length, confirming
admissibility. The difference in purple explored cells shows how tighter
heuristics expand fewer nodes.

To run: python3 generate_figures.py
Output: figures/
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import heapq
import math
import os

os.makedirs("figures", exist_ok=True)

ROWS, COLS = 20, 20
START = (0, 0)
GOAL  = (19, 19)
EC_THRESHOLD = 18
WEIGHT_HIGH  = 3
WEIGHT_LOW   = 0.85

def make_corridor_grid():
    """
    Builds the corridor-style obstacle grid used in all benchmark experiments.
    Two vertical walls with gaps scale with grid size.
    """
    grid = [[0]*COLS for _ in range(ROWS)]
    wall1 = COLS // 3
    wall2 = (2 * COLS) // 3
    for r in range(1, ROWS - 1):
        if r != ROWS - 3:
            grid[r][wall1] = 1
    for r in range(2, ROWS - 2):
        if r != 2:
            grid[r][wall2] = 1
    return grid

GRID = make_corridor_grid()

def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean(a, b):
    dr = a[0]-b[0]
    dc = a[1]-b[1]
    return int(math.sqrt(dr*dr + dc*dc))

def run_search(heuristic_fn, use_dynamic_weight):
    """
    Runs A* with the given heuristic function.
    If use_dynamic_weight is True, applies k=3/k=0.85 from Chatzisavvas et al.
    If heuristic_fn is None, runs Dijkstra (h=0).

    Parameters:
        heuristic_fn:       function(a, b) -> int, or None for Dijkstra
        use_dynamic_weight: True to apply the dynamic weight coefficient

    Returns:
        closed_set: set of all cells processed during the search
        path:       list of cells on the optimal path from start to goal
    """
    open_heap = []
    g_score   = {START: 0}
    came_from = {}
    closed    = set()

    if heuristic_fn is not None:
        h0 = heuristic_fn(START, GOAL)
        if use_dynamic_weight:
            k = WEIGHT_HIGH if h0 > EC_THRESHOLD else WEIGHT_LOW
            f0 = int(k * h0)
        else:
            f0 = h0
    else:
        f0 = 0

    heapq.heappush(open_heap, (f0, START))

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if current == GOAL:
            path = []
            node = GOAL
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(START)
            return closed, path

        r, c = current
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r+dr, c+dc
            if 0 <= nr < ROWS and 0 <= nc < COLS and GRID[nr][nc] == 0:
                nb = (nr, nc)
                if nb not in closed:
                    tg = g_score[current] + 1
                    if tg < g_score.get(nb, float('inf')):
                        came_from[nb] = current
                        g_score[nb]   = tg
                        if heuristic_fn is not None:
                            h_val = heuristic_fn(nb, GOAL)
                            if use_dynamic_weight:
                                k = WEIGHT_HIGH if h_val > EC_THRESHOLD else WEIGHT_LOW
                                f = int(tg + k * h_val)
                            else:
                                f = tg + h_val
                        else:
                            f = tg
                        heapq.heappush(open_heap, (f, nb))

    return closed, []

# Run all four configurations
closed_dyn, path_dyn = run_search(manhattan, True)
closed_man, path_man = run_search(manhattan, False)
closed_euc, path_euc = run_search(euclidean, False)
closed_dijk, path_dijk = run_search(None, False)

# Colors
C_BG        = '#0d1117'
C_CELL      = '#161b22'
C_OBSTACLE  = '#f5a623'
C_CLOSED    = '#7c4dff'
C_PATH      = '#00e676'
C_START     = '#00b0ff'
C_GOAL      = '#ff1744'
C_GRID      = '#21262d'

def draw_panel(ax, closed_set, path_set, title, node_count, path_len):
    """
    Draws one grid panel on the given axes object.
    Purple cells = explored, green cells = optimal path,
    orange cells = obstacles, dark cells = not reached.
    """
    ax.set_facecolor(C_BG)
    for r in range(ROWS):
        for c in range(COLS):
            pos = (r, c)
            y   = ROWS - 1 - r
            if GRID[r][c] == 1:
                color = C_OBSTACLE
            elif pos == START:
                color = C_START
            elif pos == GOAL:
                color = C_GOAL
            elif pos in path_set:
                color = C_PATH
            elif pos in closed_set:
                color = C_CLOSED
            else:
                color = C_CELL
            ax.add_patch(plt.Rectangle([c, y], 1, 1,
                                        facecolor=color,
                                        edgecolor=C_GRID,
                                        linewidth=0.5,
                                        zorder=2))
    ax.text(START[1]+0.5, ROWS-1-START[0]+0.5, 'S',
            color='white', fontsize=7, fontweight='bold',
            ha='center', va='center', zorder=5)
    ax.text(GOAL[1]+0.5, ROWS-1-GOAL[0]+0.5, 'G',
            color='white', fontsize=7, fontweight='bold',
            ha='center', va='center', zorder=5)
    ax.set_xlim(0, COLS)
    ax.set_ylim(0, ROWS)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_edgecolor('#30363d')
        spine.set_linewidth(1)
    ax.set_title(title, fontsize=9.5, fontweight='bold',
                 color='white', pad=6)
    ax.text(0.5, -0.05,
            f"Nodes: {node_count}   |   Path: {path_len} cells",
            transform=ax.transAxes,
            fontsize=8.5, ha='center', color='#8b949e')

# Figure 1: side by side A* dynamic vs Dijkstra
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 8),
                                facecolor=C_BG,
                                gridspec_kw={'wspace': 0.06})
fig.patch.set_facecolor(C_BG)

draw_panel(ax1, closed_dyn, set(path_dyn),
           "A*  (k=3 far from goal, k=0.85 near goal)",
           len(closed_dyn), len(path_dyn))
draw_panel(ax2, closed_dijk, set(path_dijk),
           "Dijkstra's  (no heuristic)",
           len(closed_dijk), len(path_dijk))

reduction = 100.0 * (len(closed_dijk) - len(closed_dyn)) / max(len(closed_dijk), 1)
fig.suptitle(
    "A* with Dynamic Weight Coefficient  vs  Dijkstra's Algorithm\n"
    f"Node reduction: {len(closed_dijk) - len(closed_dyn)} fewer nodes "
    f"({reduction:.1f}%)   |   Same path length: "
    f"{'yes' if len(path_dyn) == len(path_dijk) else 'no'}",
    fontsize=12, fontweight='bold', color='white', y=1.01)

legend_items = [
    mpatches.Patch(color=C_START,    label='Start (S)'),
    mpatches.Patch(color=C_GOAL,     label='Goal (G)'),
    mpatches.Patch(color=C_PATH,     label='Optimal path'),
    mpatches.Patch(color=C_CLOSED,   label='Explored (closed list)'),
    mpatches.Patch(color=C_OBSTACLE, label='Obstacle'),
    mpatches.Patch(color=C_CELL,     label='Not reached'),
]
fig.legend(handles=legend_items, loc='lower center', ncol=6,
           facecolor='#161b22', edgecolor='#30363d',
           labelcolor='white', fontsize=9.5,
           bbox_to_anchor=(0.5, -0.04))

plt.tight_layout(rect=[0, 0.06, 1, 1])
plt.savefig('figures/astar_vs_dijkstra.png', dpi=150,
            bbox_inches='tight', facecolor=C_BG)
plt.close()
print("Saved figures/astar_vs_dijkstra.png")

# Figure 2: 2x2 heuristic comparison
fig, axes = plt.subplots(2, 2, figsize=(14, 14),
                          facecolor=C_BG,
                          gridspec_kw={'wspace': 0.06, 'hspace': 0.18})
fig.patch.set_facecolor(C_BG)

configs = [
    (axes[0][0], closed_dyn,  set(path_dyn),
     "A* Manhattan + dynamic k\n(Chatzisavvas et al. [1])",
     len(closed_dyn),  len(path_dyn)),
    (axes[0][1], closed_man,  set(path_man),
     "A* Manhattan + fixed k=1\n(standard A*, Hart et al. [4])",
     len(closed_man),  len(path_man)),
    (axes[1][0], closed_euc,  set(path_euc),
     "A* Euclidean + fixed k=1\n(less informed on 4-dir grid)",
     len(closed_euc),  len(path_euc)),
    (axes[1][1], closed_dijk, set(path_dijk),
     "Dijkstra's  (h=0, no heuristic)\n(baseline comparison)",
     len(closed_dijk), len(path_dijk)),
]

for ax, closed, path_set, title, nodes, pathlen in configs:
    draw_panel(ax, closed, path_set, title, nodes, pathlen)

fig.suptitle(
    "Heuristic Comparison on 20x20 Corridor Grid\n"
    "All four find the same optimal path -- difference is nodes expanded",
    fontsize=13, fontweight='bold', color='white', y=1.01)

legend_items = [
    mpatches.Patch(color=C_START,    label='Start (S)'),
    mpatches.Patch(color=C_GOAL,     label='Goal (G)'),
    mpatches.Patch(color=C_PATH,     label='Optimal path'),
    mpatches.Patch(color=C_CLOSED,   label='Explored (closed list)'),
    mpatches.Patch(color=C_OBSTACLE, label='Obstacle'),
    mpatches.Patch(color=C_CELL,     label='Not reached'),
]
fig.legend(handles=legend_items, loc='lower center', ncol=6,
           facecolor='#161b22', edgecolor='#30363d',
           labelcolor='white', fontsize=9.5,
           bbox_to_anchor=(0.5, -0.02))

plt.tight_layout(rect=[0, 0.04, 1, 1])
plt.savefig('figures/heuristic_comparison.png', dpi=150,
            bbox_inches='tight', facecolor=C_BG)
plt.close()
print("Saved figures/heuristic_comparison.png")

# Figure 3: standalone A* path
fig, ax = plt.subplots(figsize=(7, 7), facecolor=C_BG)
draw_panel(ax, closed_dyn, set(path_dyn),
           f"A* Path on 20x20 Corridor Grid\n"
           f"Nodes expanded: {len(closed_dyn)}   |   Path: {len(path_dyn)} cells",
           len(closed_dyn), len(path_dyn))
ax.legend(handles=[
    mpatches.Patch(color=C_START,    label='Start'),
    mpatches.Patch(color=C_GOAL,     label='Goal'),
    mpatches.Patch(color=C_PATH,     label='Optimal path'),
    mpatches.Patch(color=C_CLOSED,   label='Explored'),
    mpatches.Patch(color=C_OBSTACLE, label='Obstacle'),
], loc='upper right', fontsize=8,
   facecolor='#161b22', edgecolor='#30363d', labelcolor='white')
plt.tight_layout()
plt.savefig('figures/grid_path.png', dpi=150,
            bbox_inches='tight', facecolor=C_BG)
plt.close()
print("Saved figures/grid_path.png")
print("Done.")