"""
line_graphs.py

Generates line graphs from all three benchmark CSV files produced by
running make bench. Each experiment is read from its own CSV file
and plotted as a line graph comparing A* with dynamic weight coefficient
against Dijkstra's algorithm.

The three experiments are:
  Experiment 1: Grid size scaling (10x10 to 60x60 in steps of 5)
    Reads: outputs/benchmark_grid_size.csv
    Saves: figures/line_grid_size_nodes.png
           figures/line_grid_size_runtime.png

  Experiment 2: Obstacle density scaling (0% to 20% on a 30x30 grid)
    Reads: outputs/benchmark_obstacle_density.csv
    Saves: figures/line_obstacle_nodes.png
           figures/line_obstacle_runtime.png

  Experiment 3: Initial planning vs replanning after a new obstacle
    Reads: outputs/benchmark_replan.csv
    Saves: figures/line_replan.png

All figures use the same dark color scheme as the grid visualizations
so the report looks visually consistent. Purple is always A* and orange
is always Dijkstra. Green is used for the replanning line in Experiment 3
to distinguish it from the initial planning line.

To run: python3 line_graphs.py
Prerequisite: make bench must be run first to generate the CSV files.
"""

import matplotlib.pyplot as plt
import csv
import os

os.makedirs("figures", exist_ok=True)

# Color scheme used across all figures in the report.
# COL_BG is the figure background, COL_PANEL is the axes background.
# COL_A is always A* (purple), COL_D is always Dijkstra (orange),
# COL_C is used for the replanning line in Experiment 3 (green).
COL_BG    = '#0a0a14'
COL_PANEL = '#0f0f1a'
COL_A     = '#7c4dff'
COL_D     = '#f5a623'
COL_C     = '#00e676'


def styled_ax(ax):
    """
    Applies the consistent dark theme to a matplotlib axes object.
    Sets the background color, tick colors, grid lines, and spine colors
    so every figure in the report looks visually consistent.

    Parameters:
        ax: the matplotlib Axes object to style
    """
    ax.set_facecolor(COL_PANEL)
    ax.tick_params(colors='white')
    ax.yaxis.grid(True, linestyle='--', alpha=0.3, color='white')
    ax.xaxis.grid(True, linestyle='--', alpha=0.15, color='white')
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_edgecolor('#333355')


def read_csv(path):
    """
    Reads a CSV file and returns all rows as a list of dictionaries.
    Each dictionary maps column header names to string values for one row.
    The csv.DictReader uses the first row as the header automatically.

    Parameters:
        path: string path to the CSV file

    Returns:
        list of dicts, one dict per data row
    """
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


# EXPERIMENT 1: Grid Size Scaling
# Reads benchmark_grid_size.csv which contains nodes expanded and runtime
# for both A* and Dijkstra across 11 grid sizes from 10x10 to 60x60.
# Each algorithm was run 2000 times per grid size to get stable averages.
rows   = read_csv("outputs/benchmark_grid_size.csv")
sizes  = [int(r["grid_size"])          for r in rows]
a_nodes = [int(r["astar_nodes"])       for r in rows]
d_nodes = [int(r["dijkstra_nodes"])    for r in rows]
a_time  = [float(r["astar_time_us"])   for r in rows]
d_time  = [float(r["dijkstra_time_us"]) for r in rows]
labels  = [f"{s}x{s}" for s in sizes]

# Nodes expanded line graph for Experiment 1.
# The shaded area between the two lines makes the growing gap
# between A* and Dijkstra immediately visible as grid size increases.
fig, ax = plt.subplots(figsize=(11, 5), facecolor=COL_BG)
styled_ax(ax)
ax.plot(labels, a_nodes, color=COL_A, marker='o', linewidth=2.5,
        markersize=6, label='A* (dynamic weight k)')
ax.plot(labels, d_nodes, color=COL_D, marker='s', linewidth=2.5,
        markersize=6, label="Dijkstra's (no heuristic)")
ax.fill_between(labels, a_nodes, d_nodes, alpha=0.07, color=COL_D)
ax.set_xlabel('Grid Size', color='white', fontsize=11)
ax.set_ylabel('Nodes Expanded', color='white', fontsize=11)
ax.set_title("Experiment 1: Nodes Expanded vs Grid Size\n"
             "A* with Dynamic Weight vs Dijkstra's",
             color='white', fontweight='bold')
ax.legend(facecolor='#1a1a2e', edgecolor='none', labelcolor='white')

# Annotate each data point with its exact value so the reader does not
# have to estimate from the axis scale. A* values are above the line
# in purple and Dijkstra values are above the line in orange.
for i, (y1, y2) in enumerate(zip(a_nodes, d_nodes)):
    ax.annotate(str(y1), (labels[i], y1),
                textcoords="offset points", xytext=(0, 7),
                ha='center', color=COL_A, fontsize=7)
    ax.annotate(str(y2), (labels[i], y2),
                textcoords="offset points", xytext=(0, 7),
                ha='center', color=COL_D, fontsize=7)

plt.xticks(rotation=45, color='white')
plt.tight_layout()
plt.savefig('figures/line_grid_size_nodes.png', dpi=150,
            bbox_inches='tight', facecolor=COL_BG)
plt.close()
print("Saved figures/line_grid_size_nodes.png")

# Runtime line graph for Experiment 1.
# Shows that the node count reduction translates directly to runtime
# reduction: Dijkstra grows steeply while A* stays nearly flat.
fig, ax = plt.subplots(figsize=(11, 5), facecolor=COL_BG)
styled_ax(ax)
ax.plot(labels, a_time, color=COL_A, marker='o', linewidth=2.5,
        markersize=6, label='A* (dynamic weight k)')
ax.plot(labels, d_time, color=COL_D, marker='s', linewidth=2.5,
        markersize=6, label="Dijkstra's (no heuristic)")
ax.fill_between(labels, a_time, d_time, alpha=0.07, color=COL_D)
ax.set_xlabel('Grid Size', color='white', fontsize=11)
ax.set_ylabel('Avg Runtime (microseconds)', color='white', fontsize=11)
ax.set_title("Experiment 1: Runtime vs Grid Size\n"
             "A* with Dynamic Weight vs Dijkstra's",
             color='white', fontweight='bold')
ax.legend(facecolor='#1a1a2e', edgecolor='none', labelcolor='white')
plt.xticks(rotation=45, color='white')
plt.tight_layout()
plt.savefig('figures/line_grid_size_runtime.png', dpi=150,
            bbox_inches='tight', facecolor=COL_BG)
plt.close()
print("Saved figures/line_grid_size_runtime.png")


# EXPERIMENT 2: Obstacle Density Scaling
# Reads benchmark_obstacle_density.csv which contains nodes expanded
# and runtime for both algorithms as obstacle density increases from
# 0% to 20% on a fixed 30x30 grid with random obstacles (seed=42).
# Rows where no path was found (astar_nodes = 0) are filtered out
# because those configurations have no meaningful comparison data.
rows2      = read_csv("outputs/benchmark_obstacle_density.csv")
rows2      = [r for r in rows2 if int(r["astar_nodes"]) > 0]
densities  = [int(r["obstacle_pct"])         for r in rows2]
a_nodes2   = [int(r["astar_nodes"])          for r in rows2]
d_nodes2   = [int(r["dijkstra_nodes"])       for r in rows2]
a_time2    = [float(r["astar_time_us"])      for r in rows2]
d_time2    = [float(r["dijkstra_time_us"])   for r in rows2]
dlabels    = [f"{d}%" for d in densities]

# Nodes expanded line graph for Experiment 2.
fig, ax = plt.subplots(figsize=(10, 5), facecolor=COL_BG)
styled_ax(ax)
ax.plot(dlabels, a_nodes2, color=COL_A, marker='o', linewidth=2.5,
        markersize=7, label='A* (dynamic weight k)')
ax.plot(dlabels, d_nodes2, color=COL_D, marker='s', linewidth=2.5,
        markersize=7, label="Dijkstra's (no heuristic)")
ax.fill_between(dlabels, a_nodes2, d_nodes2, alpha=0.07, color=COL_D)
ax.set_xlabel('Obstacle Density', color='white', fontsize=11)
ax.set_ylabel('Nodes Expanded', color='white', fontsize=11)
ax.set_title("Experiment 2: Nodes Expanded vs Obstacle Density (30x30 Grid)\n"
             "A* with Dynamic Weight vs Dijkstra's",
             color='white', fontweight='bold')
ax.legend(facecolor='#1a1a2e', edgecolor='none', labelcolor='white')
for i, (y1, y2) in enumerate(zip(a_nodes2, d_nodes2)):
    ax.annotate(str(y1), (dlabels[i], y1),
                textcoords="offset points", xytext=(0, 7),
                ha='center', color=COL_A, fontsize=8)
    ax.annotate(str(y2), (dlabels[i], y2),
                textcoords="offset points", xytext=(0, 7),
                ha='center', color=COL_D, fontsize=8)
plt.tight_layout()
plt.savefig('figures/line_obstacle_nodes.png', dpi=150,
            bbox_inches='tight', facecolor=COL_BG)
plt.close()
print("Saved figures/line_obstacle_nodes.png")

# Runtime line graph for Experiment 2.
fig, ax = plt.subplots(figsize=(10, 5), facecolor=COL_BG)
styled_ax(ax)
ax.plot(dlabels, a_time2, color=COL_A, marker='o', linewidth=2.5,
        markersize=7, label='A* (dynamic weight k)')
ax.plot(dlabels, d_time2, color=COL_D, marker='s', linewidth=2.5,
        markersize=7, label="Dijkstra's (no heuristic)")
ax.fill_between(dlabels, a_time2, d_time2, alpha=0.07, color=COL_D)
ax.set_xlabel('Obstacle Density', color='white', fontsize=11)
ax.set_ylabel('Avg Runtime (microseconds)', color='white', fontsize=11)
ax.set_title("Experiment 2: Runtime vs Obstacle Density (30x30 Grid)",
             color='white', fontweight='bold')
ax.legend(facecolor='#1a1a2e', edgecolor='none', labelcolor='white')
plt.tight_layout()
plt.savefig('figures/line_obstacle_runtime.png', dpi=150,
            bbox_inches='tight', facecolor=COL_BG)
plt.close()
print("Saved figures/line_obstacle_runtime.png")


# EXPERIMENT 3: Initial Planning vs Replanning
# Reads benchmark_replan.csv which contains node counts and runtimes
# for the initial A* search and for replanning after a new obstacle
# appears on the planned path. Directly inspired by Hu et al. [2].
# Uses a dual-panel figure so nodes and runtime can be compared side by side.
rows3   = read_csv("outputs/benchmark_replan.csv")
sizes3  = [int(r["grid_size"])          for r in rows3]
i_nodes = [int(r["initial_nodes"])      for r in rows3]
r_nodes = [int(r["replan_nodes"])       for r in rows3]
i_time  = [float(r["initial_time_us"])  for r in rows3]
r_time  = [float(r["replan_time_us"])   for r in rows3]
rlabels = [f"{s}x{s}" for s in sizes3]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5), facecolor=COL_BG)
fig.suptitle("Experiment 3: Initial Planning vs Replanning After New Obstacle\n"
             "Inspired by Hu et al. [2] rerouting experiments",
             color='white', fontweight='bold', fontsize=12)

# Left panel: nodes expanded comparison.
# Replanning expands slightly more nodes than the initial search because
# the robot starts partway through the grid rather than at the corner.
# Both remain much lower than Dijkstra would expand on the same grid.
styled_ax(ax1)
ax1.plot(rlabels, i_nodes, color=COL_A, marker='o', linewidth=2.5,
         markersize=7, label='Initial search')
ax1.plot(rlabels, r_nodes, color=COL_C, marker='^', linewidth=2.5,
         markersize=7, label='Replanning search')
ax1.set_xlabel('Grid Size', color='white')
ax1.set_ylabel('Nodes Expanded', color='white')
ax1.set_title('Nodes Expanded', color='white')
ax1.legend(facecolor='#1a1a2e', edgecolor='none', labelcolor='white')
for i, (y1, y2) in enumerate(zip(i_nodes, r_nodes)):
    ax1.annotate(str(y1), (rlabels[i], y1),
                 textcoords="offset points", xytext=(-12, 5),
                 color=COL_A, fontsize=8)
    ax1.annotate(str(y2), (rlabels[i], y2),
                 textcoords="offset points", xytext=(5, 5),
                 color=COL_C, fontsize=8)

# Right panel: runtime comparison.
# Both initial and replan searches are fast, confirming A* is suitable
# for real-time dynamic replanning as described in Hu et al. [2].
styled_ax(ax2)
ax2.plot(rlabels, i_time, color=COL_A, marker='o', linewidth=2.5,
         markersize=7, label='Initial search')
ax2.plot(rlabels, r_time, color=COL_C, marker='^', linewidth=2.5,
         markersize=7, label='Replanning search')
ax2.set_xlabel('Grid Size', color='white')
ax2.set_ylabel('Avg Runtime (microseconds)', color='white')
ax2.set_title('Runtime', color='white')
ax2.legend(facecolor='#1a1a2e', edgecolor='none', labelcolor='white')

plt.tight_layout()
plt.savefig('figures/line_replan.png', dpi=150,
            bbox_inches='tight', facecolor=COL_BG)
plt.close()
print("Saved figures/line_replan.png")

print("\nAll line graphs saved to figures/")