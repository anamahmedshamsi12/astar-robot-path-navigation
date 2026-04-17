import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
import numpy as np
import heapq
import os

os.makedirs("figures", exist_ok=True)

# ── Grid ──────────────────────────────────────────────────────────────────────
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
START = (0,  0)
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
    open_set  = set([START])

    h0 = manhattan(START, GOAL)
    k  = WEIGHT_HIGH if h0 > EC_THRESHOLD else WEIGHT_LOW
    f0 = int(0 + k * h0) if use_heuristic else 0
    heapq.heappush(open_heap, (f0, h0, START))

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)
        open_set.discard(current)

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
                        open_set.add(nb)

    return list(closed), []

# Run both
closed_a, path_a = run_search(True)
closed_d, path_d = run_search(False)
path_a_set = set(path_a)
path_d_set = set(path_d)

# ── Colors ────────────────────────────────────────────────────────────────────
COL_OBSTACLE = '#1a1a2e'
COL_OPEN     = '#16213e'
COL_CLOSED   = '#7c4dff'
COL_PATH     = '#00e676'
COL_START    = '#00b0ff'
COL_GOAL     = '#ff1744'
COL_BG       = '#0a0a14'

def cell_color(r, c, closed_set, path_set):
    pos = (r, c)
    if pos == START:       return COL_START
    if pos == GOAL:        return COL_GOAL
    if pos in path_set:    return COL_PATH
    if pos in closed_set:  return COL_CLOSED
    if GRID[r][c] == 1:    return COL_OBSTACLE
    return COL_OPEN

def draw_grid(ax, closed_set, path_set, title, node_count, path_len):
    ax.set_facecolor('#0a0a14')
    for r in range(ROWS):
        for c in range(COLS):
            color = cell_color(r, c, closed_set, path_set)
            # Make path cells slightly larger/brighter by drawing them on top
            if (r, c) in path_set and (r, c) != START and (r, c) != GOAL:
                rect = plt.Rectangle([c, ROWS-r-1], 0.95, 0.95,
                                     facecolor=color, edgecolor='#00ff88',
                                     linewidth=0.8, zorder=3)
            else:
                rect = plt.Rectangle([c, ROWS-r-1], 0.92, 0.92,
                                     facecolor=color, edgecolor='#0a0a14',
                                     linewidth=0.4, zorder=2)
            ax.add_patch(rect)

    ax.set_xlim(0, COLS)
    ax.set_ylim(0, ROWS)
    ax.set_aspect('equal')
    ax.axis('off')

    # Title with node count annotation
    ax.set_title(title, color='white', fontsize=12,
                 fontweight='bold', pad=6)

    # Annotation box bottom center — nodes expanded
    ax.text(COLS / 2, -0.8,
            f"Nodes expanded: {node_count}   |   Path length: {path_len} cells",
            color='white', fontsize=9.5, ha='center', va='top',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#1a1a2e',
                      edgecolor='#444466', alpha=0.9))

    # Mark start and goal with text labels
    ax.text(0.5, ROWS - 0 - 0.5, 'S', color='white', fontsize=8,
            fontweight='bold', ha='center', va='center', zorder=5)
    ax.text(COLS - 0.5, 0.5, 'G', color='white', fontsize=8,
            fontweight='bold', ha='center', va='center', zorder=5)

# ── Figure 1: Side by side A* vs Dijkstra ─────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 8),
                                facecolor=COL_BG)
fig.suptitle(
    "A* with Dynamic Weight Coefficient  vs  Dijkstra's Algorithm\n"
    "Purple = nodes processed   Bright green = optimal path   "
    "Cyan = start   Red = goal",
    color='white', fontsize=12, fontweight='bold', y=1.01)

draw_grid(ax1, set(closed_a), path_a_set,
          "A*  (k=3 far from goal, k=0.85 near goal)",
          len(closed_a), len(path_a))

draw_grid(ax2, set(closed_d), path_d_set,
          "Dijkstra's  (no heuristic, k=0)",
          len(closed_d), len(path_d))

# Reduction annotation between panels
reduction = 100.0 * (len(closed_d) - len(closed_a)) / len(closed_d)
fig.text(0.5, 0.01,
         f"Node reduction: {len(closed_d) - len(closed_a)} fewer nodes  "
         f"({reduction:.1f}% reduction)  |  Same path length: "
         f"{'yes' if len(path_a) == len(path_d) else 'no'}",
         color='#00e676', fontsize=11, fontweight='bold',
         ha='center', va='bottom',
         bbox=dict(boxstyle='round,pad=0.5', facecolor='#0f0f1a',
                   edgecolor='#00e676', alpha=0.9))

legend_items = [
    mpatches.Patch(color=COL_START,   label='Start (S)'),
    mpatches.Patch(color=COL_GOAL,    label='Goal (G)'),
    mpatches.Patch(color=COL_PATH,    label='Optimal path'),
    mpatches.Patch(color=COL_CLOSED,  label='Nodes processed'),
    mpatches.Patch(color=COL_OBSTACLE,label='Obstacle'),
    mpatches.Patch(color=COL_OPEN,    label='Not reached'),
]
fig.legend(handles=legend_items, loc='lower center', ncol=6,
           facecolor='#1a1a2e', edgecolor='none',
           labelcolor='white', fontsize=9.5,
           bbox_to_anchor=(0.5, 0.06))

plt.tight_layout(rect=[0, 0.12, 1, 1])
plt.savefig('figures/astar_vs_dijkstra.png', dpi=150,
            bbox_inches='tight', facecolor=COL_BG)
plt.close()
print("Saved figures/astar_vs_dijkstra.png")

# ── Figure 2: nodes expanded bar chart ────────────────────────────────────────
sizes       = [10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60]
astar_nodes = [88, 94, 44, 54, 64, 74, 84, 94, 104, 114, 124]
dijk_nodes  = [88, 203, 368, 583, 848, 1163, 1528, 1943, 2408, 2923, 3488]
labels      = [f"{s}x{s}" for s in sizes]
x     = np.arange(len(sizes))
width = 0.35

fig, ax = plt.subplots(figsize=(12, 6), facecolor=COL_BG)
ax.set_facecolor('#0f0f1a')
bars1 = ax.bar(x - width/2, astar_nodes, width,
               label='A* (dynamic weight k)', color='#7c4dff', zorder=3)
bars2 = ax.bar(x + width/2, dijk_nodes,  width,
               label="Dijkstra's", color='#f5a623', zorder=3)

# Annotate top of each bar
for bar in bars1:
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 20,
            str(int(bar.get_height())), ha='center', va='bottom',
            color='#7c4dff', fontsize=7, fontweight='bold')
for bar in bars2:
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 20,
            str(int(bar.get_height())), ha='center', va='bottom',
            color='#f5a623', fontsize=7, fontweight='bold')

ax.set_xlabel('Grid Size', color='white', fontsize=11)
ax.set_ylabel('Nodes Expanded', color='white', fontsize=11)
ax.set_title("Experiment 1: Nodes Expanded vs Grid Size\n"
             "A* with Dynamic Weight vs Dijkstra's Algorithm",
             color='white', fontweight='bold', fontsize=12)
ax.set_xticks(x)
ax.set_xticklabels(labels, color='white', rotation=30)
ax.tick_params(colors='white')
ax.yaxis.grid(True, linestyle='--', alpha=0.3, color='white', zorder=0)
ax.set_axisbelow(True)
ax.legend(facecolor='#1a1a2e', edgecolor='none', labelcolor='white', fontsize=10)
for spine in ax.spines.values():
    spine.set_edgecolor('#333355')
plt.tight_layout()
plt.savefig('figures/nodes_expanded_vs_size.png', dpi=150,
            bbox_inches='tight', facecolor=COL_BG)
plt.close()
print("Saved figures/nodes_expanded_vs_size.png")

# ── Figure 3: runtime bar chart ────────────────────────────────────────────────
astar_time = [5, 5, 15, 5, 5, 5, 5, 10, 10, 10, 10]
dijk_time  = [5, 5, 10, 20, 30, 45, 60, 85, 110, 155, 180]

fig, ax = plt.subplots(figsize=(12, 6), facecolor=COL_BG)
ax.set_facecolor('#0f0f1a')
bars1 = ax.bar(x - width/2, astar_time, width,
               label='A* (dynamic weight k)', color='#7c4dff', zorder=3)
bars2 = ax.bar(x + width/2, dijk_time,  width,
               label="Dijkstra's", color='#f5a623', zorder=3)

for bar in bars2:
    h = bar.get_height()
    if h > 0:
        ax.text(bar.get_x() + bar.get_width()/2, h + 1.5,
                f"{h:.0f}", ha='center', va='bottom',
                color='#f5a623', fontsize=7, fontweight='bold')

ax.set_xlabel('Grid Size', color='white', fontsize=11)
ax.set_ylabel('Avg Runtime (microseconds)', color='white', fontsize=11)
ax.set_title("Experiment 1: Runtime vs Grid Size\n"
             "A* with Dynamic Weight vs Dijkstra's Algorithm",
             color='white', fontweight='bold', fontsize=12)
ax.set_xticks(x)
ax.set_xticklabels(labels, color='white', rotation=30)
ax.tick_params(colors='white')
ax.yaxis.grid(True, linestyle='--', alpha=0.3, color='white', zorder=0)
ax.set_axisbelow(True)
ax.legend(facecolor='#1a1a2e', edgecolor='none', labelcolor='white', fontsize=10)
for spine in ax.spines.values():
    spine.set_edgecolor('#333355')
plt.tight_layout()
plt.savefig('figures/runtime_vs_size.png', dpi=150,
            bbox_inches='tight', facecolor=COL_BG)
plt.close()
print("Saved figures/runtime_vs_size.png")

# ── Figure 4: grid path only ───────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(7, 7), facecolor=COL_BG)
ax.set_facecolor(COL_BG)
for r in range(ROWS):
    for c in range(COLS):
        color = cell_color(r, c, set(closed_a), path_a_set)
        if (r,c) in path_a_set and (r,c) != START and (r,c) != GOAL:
            rect = plt.Rectangle([c, ROWS-r-1], 0.95, 0.95,
                                 facecolor=color, edgecolor='#00ff88',
                                 linewidth=0.8, zorder=3)
        else:
            rect = plt.Rectangle([c, ROWS-r-1], 0.92, 0.92,
                                 facecolor=color, edgecolor='#0a0a14',
                                 linewidth=0.4, zorder=2)
        ax.add_patch(rect)

ax.text(0.5, ROWS - 0.5, 'S', color='white', fontsize=9,
        fontweight='bold', ha='center', va='center', zorder=5)
ax.text(COLS - 0.5, 0.5, 'G', color='white', fontsize=9,
        fontweight='bold', ha='center', va='center', zorder=5)

ax.set_xlim(0, COLS)
ax.set_ylim(0, ROWS)
ax.set_aspect('equal')
ax.axis('off')
ax.set_title(f"A* Path on 20x20 Grid\n"
             f"Nodes expanded: {len(closed_a)}   Path length: {len(path_a)} cells",
             color='white', fontsize=11, fontweight='bold', pad=10)

legend = [
    mpatches.Patch(color=COL_START,   label='Start'),
    mpatches.Patch(color=COL_GOAL,    label='Goal'),
    mpatches.Patch(color=COL_PATH,    label='Optimal path'),
    mpatches.Patch(color=COL_CLOSED,  label='Nodes processed'),
    mpatches.Patch(color=COL_OBSTACLE,label='Obstacle'),
]
ax.legend(handles=legend, loc='lower left', fontsize=8,
          facecolor='#1a1a2e', edgecolor='none', labelcolor='white')
plt.tight_layout()
plt.savefig('figures/grid_path.png', dpi=150,
            bbox_inches='tight', facecolor=COL_BG)
plt.close()
print("Saved figures/grid_path.png")

print("\nAll figures saved to figures/")