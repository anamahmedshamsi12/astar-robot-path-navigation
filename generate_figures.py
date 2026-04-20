import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import heapq
import os

os.makedirs("figures", exist_ok=True)

ROWS, COLS = 20, 20
START = (0, 0)
GOAL  = (19, 19)
EC_THRESHOLD = 18
WEIGHT_HIGH  = 3
WEIGHT_LOW   = 0.85

def make_corridor_grid():
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

C_BG        = '#0d1117'
C_CELL      = '#161b22'
C_OBSTACLE  = '#f5a623'
C_CLOSED    = '#7c4dff'
C_PATH      = '#00e676'
C_START     = '#00b0ff'
C_GOAL      = '#ff1744'
C_GRID      = '#21262d'

def draw_grid(ax, closed_set, path_set, title, node_count, path_len):
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
            color='white', fontsize=8, fontweight='bold',
            ha='center', va='center', zorder=5)
    ax.text(GOAL[1]+0.5, ROWS-1-GOAL[0]+0.5, 'G',
            color='white', fontsize=8, fontweight='bold',
            ha='center', va='center', zorder=5)
    ax.set_xlim(0, COLS)
    ax.set_ylim(0, ROWS)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_edgecolor('#30363d')
        spine.set_linewidth(1)
    ax.set_title(title, fontsize=11, fontweight='bold', pad=8, color='white')
    ax.text(0.5, -0.04,
            f"Nodes expanded: {node_count}   |   Path length: {path_len} cells",
            transform=ax.transAxes, fontsize=9, ha='center', color='#8b949e')

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 8),
                                facecolor=C_BG,
                                gridspec_kw={'wspace': 0.06})
fig.patch.set_facecolor(C_BG)

draw_grid(ax1, set(closed_a), path_a_set,
          "A*  (k=3 far from goal, k=0.85 near goal)",
          len(closed_a), len(path_a))
draw_grid(ax2, set(closed_d), path_d_set,
          "Dijkstra's  (no heuristic)",
          len(closed_d), len(path_d))

reduction = 100.0 * (len(closed_d) - len(closed_a)) / max(len(closed_d), 1)
fig.suptitle(
    "A* with Dynamic Weight Coefficient  vs  Dijkstra's Algorithm\n"
    f"Node reduction: {len(closed_d) - len(closed_a)} fewer nodes "
    f"({reduction:.1f}%)   |   Same path length: "
    f"{'yes' if len(path_a) == len(path_d) else 'no'}",
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

fig, ax = plt.subplots(figsize=(7, 7), facecolor=C_BG)
draw_grid(ax, set(closed_a), path_a_set,
          f"A* Path on 20x20 Corridor Grid\n"
          f"Nodes expanded: {len(closed_a)}   |   Path: {len(path_a)} cells",
          len(closed_a), len(path_a))
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