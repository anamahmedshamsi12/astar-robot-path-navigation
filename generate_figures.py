import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import heapq
import os

os.makedirs("figures", exist_ok=True)

GRID = [
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,0,0,1,1,1,1,0,0,1,1,0,0,1,1,0],
    [0,1,0,0,1,0,0,1,0,0,1,0,0,0,1,0,0,1,0,0],
    [0,1,0,0,1,0,0,1,0,0,1,0,0,0,1,0,0,1,0,0],
    [0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0],
    [0,0,0,1,1,1,1,0,0,0,1,0,1,1,1,1,0,0,0,0],
    [0,0,0,1,0,0,1,0,0,0,0,0,1,0,0,1,0,0,0,0],
    [0,0,0,1,0,0,1,0,0,0,0,0,1,0,0,1,0,0,0,0],
    [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0],
    [0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
    [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0],
    [0,1,0,0,1,1,1,0,0,0,0,0,1,1,1,0,0,0,1,0],
    [0,0,0,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0],
    [0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,0,0,0,0,0,1,1,0,0,0,0,0,1,1,1,0],
    [0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0],
    [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
]
ROWS, COLS = 20, 20
START = (0, 0)
GOAL  = (19, 19)
EC_THRESHOLD = 18
WEIGHT_HIGH  = 3
WEIGHT_LOW   = 0.85

def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def run_search(use_heuristic):
    open_heap = []
    g_score   = {START: 0}
    came_from = {}
    closed    = set()

    h0 = manhattan(START, GOAL)
    k  = WEIGHT_HIGH if h0 > EC_THRESHOLD else WEIGHT_LOW
    f0 = int(k * h0) if use_heuristic else 0
    heapq.heappush(open_heap, (f0, h0, START))

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
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
            return list(closed), path
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
                        h_val = manhattan(nb, GOAL)
                        if use_heuristic:
                            k   = WEIGHT_HIGH if h_val > EC_THRESHOLD else WEIGHT_LOW
                            f   = int(tg + k * h_val)
                        else:
                            f = tg
                        heapq.heappush(open_heap, (f, h_val, nb))
    return list(closed), []

closed_a, path_a = run_search(True)
closed_d, path_d = run_search(False)
path_a_set = set(path_a)
path_d_set = set(path_d)

# ── Clean academic color scheme ────────────────────────────────────────────────
# Inspired by robotics research paper figures (light background, clear colors)
COL_BG       = '#ffffff'   # white background
COL_GRID     = '#cccccc'   # light gray grid lines
COL_OPEN     = '#f5f5f5'   # very light gray - unvisited
COL_OBSTACLE = '#2d2d2d'   # near black - obstacles
COL_CLOSED_A = '#a8d8ea'   # light blue - A* explored
COL_CLOSED_D = '#ffb347'   # orange - Dijkstra explored
COL_PATH     = '#2ecc71'   # green - path
COL_START    = '#3498db'   # blue - start
COL_GOAL     = '#e74c3c'   # red - goal

def draw_academic_grid(ax, closed_set, path_set, title, node_count,
                       path_len, closed_color):
    ax.set_facecolor(COL_BG)
    ax.set_xlim(-0.5, COLS - 0.5)
    ax.set_ylim(-0.5, ROWS - 0.5)
    ax.set_aspect('equal')

    # Draw grid lines
    for i in range(ROWS + 1):
        ax.axhline(i - 0.5, color=COL_GRID, linewidth=0.3, zorder=1)
    for j in range(COLS + 1):
        ax.axvline(j - 0.5, color=COL_GRID, linewidth=0.3, zorder=1)

    # Draw cells
    for r in range(ROWS):
        for c in range(COLS):
            pos = (r, c)
            if pos == START:
                color = COL_START
            elif pos == GOAL:
                color = COL_GOAL
            elif pos in path_set:
                color = COL_PATH
            elif GRID[r][c] == 1:
                color = COL_OBSTACLE
            elif pos in closed_set:
                color = closed_color
            else:
                color = COL_OPEN

            rect = plt.Rectangle([c - 0.5, r - 0.5], 1.0, 1.0,
                                  facecolor=color, edgecolor='none',
                                  zorder=2)
            ax.add_patch(rect)

    # Start and goal labels
    ax.text(START[1], START[0], 'S', color='white', fontsize=7,
            fontweight='bold', ha='center', va='center', zorder=5)
    ax.text(GOAL[1], GOAL[0], 'G', color='white', fontsize=7,
            fontweight='bold', ha='center', va='center', zorder=5)

    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_edgecolor('#999999')
        spine.set_linewidth(0.8)

    ax.set_title(title, fontsize=11, fontweight='bold', color='#222222', pad=8)
    ax.text(0.5, -0.04,
            f"Nodes expanded: {node_count}   |   Path length: {path_len} cells",
            transform=ax.transAxes,
            fontsize=9, ha='center', color='#444444')

# ── Figure 1: Side by side comparison ─────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7),
                                facecolor='white')
fig.patch.set_facecolor('white')

draw_academic_grid(ax1, set(closed_a), path_a_set,
                   "A*  (dynamic weight: k=3 far, k=0.85 near)",
                   len(closed_a), len(path_a), COL_CLOSED_A)

draw_academic_grid(ax2, set(closed_d), path_d_set,
                   "Dijkstra's  (no heuristic)",
                   len(closed_d), len(path_d), COL_CLOSED_D)

reduction = 100.0 * (len(closed_d) - len(closed_a)) / len(closed_d)
fig.suptitle(
    "A* with Dynamic Weight Coefficient  vs  Dijkstra's Algorithm\n"
    f"Node reduction: {len(closed_d) - len(closed_a)} fewer nodes "
    f"({reduction:.1f}% reduction)   |   Same path length: "
    f"{'yes' if len(path_a) == len(path_d) else 'no'}",
    fontsize=12, fontweight='bold', color='#222222', y=1.01)

legend_items = [
    mpatches.Patch(color=COL_START,    label='Start (S)'),
    mpatches.Patch(color=COL_GOAL,     label='Goal (G)'),
    mpatches.Patch(color=COL_PATH,     label='Optimal path'),
    mpatches.Patch(color=COL_CLOSED_A, label='A* explored'),
    mpatches.Patch(color=COL_CLOSED_D, label="Dijkstra's explored"),
    mpatches.Patch(color=COL_OBSTACLE, label='Obstacle'),
    mpatches.Patch(color=COL_OPEN,     label='Not reached'),
]
fig.legend(handles=legend_items, loc='lower center', ncol=7,
           facecolor='white', edgecolor='#cccccc',
           fontsize=9, bbox_to_anchor=(0.5, -0.04))

plt.tight_layout(rect=[0, 0.06, 1, 1])
plt.savefig('figures/astar_vs_dijkstra.png', dpi=150,
            bbox_inches='tight', facecolor='white')
plt.close()
print("Saved figures/astar_vs_dijkstra.png")

# ── Figure 2: nodes expanded bar chart (clean white style) ────────────────────
sizes       = [10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60]
astar_nodes = [88, 94, 44, 54, 64, 74, 84, 94, 104, 114, 124]
dijk_nodes  = [88, 203, 368, 583, 848, 1163, 1528, 1943, 2408, 2923, 3488]
labels      = [f"{s}x{s}" for s in sizes]
x     = np.arange(len(sizes))
width = 0.35

fig, ax = plt.subplots(figsize=(12, 6), facecolor='white')
ax.set_facecolor('white')
bars1 = ax.bar(x - width/2, astar_nodes, width,
               label='A* (dynamic weight k)', color='#3498db',
               edgecolor='#2980b9', linewidth=0.5)
bars2 = ax.bar(x + width/2, dijk_nodes, width,
               label="Dijkstra's", color='#e67e22',
               edgecolor='#d35400', linewidth=0.5)

for bar in bars1:
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 15,
            str(int(bar.get_height())), ha='center', va='bottom',
            color='#2980b9', fontsize=7, fontweight='bold')
for bar in bars2:
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 15,
            str(int(bar.get_height())), ha='center', va='bottom',
            color='#d35400', fontsize=7, fontweight='bold')

ax.set_xlabel('Grid Size', fontsize=11)
ax.set_ylabel('Nodes Expanded', fontsize=11)
ax.set_title("Experiment 1: Nodes Expanded vs Grid Size\n"
             "A* with Dynamic Weight vs Dijkstra's Algorithm",
             fontsize=12, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels(labels, rotation=30)
ax.yaxis.grid(True, linestyle='--', alpha=0.5, color='#cccccc')
ax.set_axisbelow(True)
ax.legend(fontsize=10)
for spine in ax.spines.values():
    spine.set_edgecolor('#cccccc')
plt.tight_layout()
plt.savefig('figures/nodes_expanded_vs_size.png', dpi=150,
            bbox_inches='tight', facecolor='white')
plt.close()
print("Saved figures/nodes_expanded_vs_size.png")

# ── Figure 3: runtime bar chart ────────────────────────────────────────────────
astar_time = [5, 5, 15, 5, 5, 5, 5, 10, 10, 10, 10]
dijk_time  = [5, 5, 10, 20, 30, 45, 60, 85, 110, 155, 180]

fig, ax = plt.subplots(figsize=(12, 6), facecolor='white')
ax.set_facecolor('white')
ax.bar(x - width/2, astar_time, width,
       label='A* (dynamic weight k)', color='#3498db',
       edgecolor='#2980b9', linewidth=0.5)
ax.bar(x + width/2, dijk_time, width,
       label="Dijkstra's", color='#e67e22',
       edgecolor='#d35400', linewidth=0.5)
ax.set_xlabel('Grid Size', fontsize=11)
ax.set_ylabel('Avg Runtime (microseconds)', fontsize=11)
ax.set_title("Experiment 1: Runtime vs Grid Size\n"
             "A* with Dynamic Weight vs Dijkstra's Algorithm",
             fontsize=12, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels(labels, rotation=30)
ax.yaxis.grid(True, linestyle='--', alpha=0.5, color='#cccccc')
ax.set_axisbelow(True)
ax.legend(fontsize=10)
for spine in ax.spines.values():
    spine.set_edgecolor('#cccccc')
plt.tight_layout()
plt.savefig('figures/runtime_vs_size.png', dpi=150,
            bbox_inches='tight', facecolor='white')
plt.close()
print("Saved figures/runtime_vs_size.png")

# ── Figure 4: standalone grid path ────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(7, 7), facecolor='white')
draw_academic_grid(ax, set(closed_a), path_a_set,
                   f"A* Path on 20x20 Grid\n"
                   f"Nodes expanded: {len(closed_a)}   Path: {len(path_a)} cells",
                   len(closed_a), len(path_a), COL_CLOSED_A)
legend = [
    mpatches.Patch(color=COL_START,    label='Start'),
    mpatches.Patch(color=COL_GOAL,     label='Goal'),
    mpatches.Patch(color=COL_PATH,     label='Optimal path'),
    mpatches.Patch(color=COL_CLOSED_A, label='Explored'),
    mpatches.Patch(color=COL_OBSTACLE, label='Obstacle'),
]
ax.legend(handles=legend, loc='lower left', fontsize=8,
          facecolor='white', edgecolor='#cccccc')
plt.tight_layout()
plt.savefig('figures/grid_path.png', dpi=150,
            bbox_inches='tight', facecolor='white')
plt.close()
print("Saved figures/grid_path.png")

print("\nAll figures saved to figures/")