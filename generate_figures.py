import matplotlib.pyplot as plt
import numpy as np
import os

sizes        = [10,   20,   30,   40,   50,   60  ]
astar_nodes  = [88,   44,   64,   84,   104,  124 ]
dijk_nodes   = [88,   368,  848,  1528, 2408, 3488]
astar_time   = [5.0,  5.0,  15.0, 5.0,  10.0, 5.0 ]
dijk_time    = [0.0,  10.0, 20.0, 45.0, 90.0, 140.0]
labels       = ['10x10','20x20','30x30','40x40','50x50','60x60']

os.makedirs('figures', exist_ok=True)

x     = np.arange(len(sizes))
width = 0.35

fig, ax = plt.subplots(figsize=(9, 5))
ax.bar(x - width/2, astar_nodes, width, label='A* (dynamic weight k)', color='#3498db')
ax.bar(x + width/2, dijk_nodes,  width, label="Dijkstra's (no heuristic)", color='#e67e22')
ax.set_xlabel('Grid Size')
ax.set_ylabel('Nodes Expanded')
ax.set_title("Nodes Expanded: A* with Dynamic Weight vs Dijkstra's")
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend()
ax.yaxis.grid(True, linestyle='--', alpha=0.5)
ax.set_axisbelow(True)
plt.tight_layout()
plt.savefig('figures/nodes_expanded_vs_size.png', dpi=150)
plt.close()
print("Saved figures/nodes_expanded_vs_size.png")

fig, ax = plt.subplots(figsize=(9, 5))
ax.bar(x - width/2, astar_time, width, label='A* (dynamic weight k)', color='#3498db')
ax.bar(x + width/2, dijk_time,  width, label="Dijkstra's (no heuristic)", color='#e67e22')
ax.set_xlabel('Grid Size')
ax.set_ylabel('Avg Runtime (microseconds)')
ax.set_title("Runtime: A* with Dynamic Weight vs Dijkstra's")
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend()
ax.yaxis.grid(True, linestyle='--', alpha=0.5)
ax.set_axisbelow(True)
plt.tight_layout()
plt.savefig('figures/runtime_vs_size.png', dpi=150)
plt.close()
print("Saved figures/runtime_vs_size.png")

import matplotlib.patches as mpatches
grid = [
    [0,0,0,0,0,0,0,0],
    [0,0,1,0,0,1,0,0],
    [0,0,1,0,0,1,0,0],
    [0,0,1,0,0,1,0,0],
    [0,0,1,1,1,1,0,0],
    [0,0,0,0,0,0,0,0],
    [0,1,1,1,0,0,0,0],
    [0,0,0,0,0,0,0,0],
]
path  = {(0,0),(0,1),(0,2),(0,3),(0,4),(0,5),(0,6),(0,7),
         (1,7),(2,7),(3,7),(4,7),(5,7),(6,7),(7,7)}
fig, ax = plt.subplots(figsize=(6, 6))
for r in range(8):
    for c in range(8):
        if   grid[r][c] == 1: color = '#2c2c2c'
        elif (r,c) == (0,0):  color = '#2ecc71'
        elif (r,c) == (7,7):  color = '#e74c3c'
        elif (r,c) in path:   color = '#3498db'
        else:                 color = '#f0f0f0'
        ax.add_patch(plt.Rectangle([c, 7-r], 1, 1,
                     facecolor=color, edgecolor='#aaaaaa', linewidth=0.8))
ax.set_xlim(0,8)
ax.set_ylim(0,8)
ax.set_aspect('equal')
ax.axis('off')
ax.set_title("A* Path on Demo Grid\nGreen=Start  Red=Goal  Blue=Path  Dark=Obstacle", fontsize=11, pad=10)
legend = [
    mpatches.Patch(color='#2ecc71', label='Start'),
    mpatches.Patch(color='#e74c3c', label='Goal'),
    mpatches.Patch(color='#3498db', label='A* Path'),
    mpatches.Patch(color='#2c2c2c', label='Obstacle'),
]
ax.legend(handles=legend, loc='lower left', fontsize=9)
plt.tight_layout()
plt.savefig('figures/grid_path.png', dpi=150, bbox_inches='tight')
plt.close()
print("Saved figures/grid_path.png")
