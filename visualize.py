"""
visualize.py

Pygame visualization of A* (with dynamic weight coefficient) vs Dijkstra
on the same grid. Saves screenshots to figures/ folder.

Color scheme:
  #1a1a2e  dark navy    = obstacle
  #16213e  midnight     = unvisited open cell
  #f5a623  orange       = open list (discovered, not yet processed)
  #7c4dff  purple       = closed list (fully processed)
  #00e676  bright green = final path
  #00b0ff  cyan         = start cell
  #ff1744  red          = goal cell

Based on:
  [1] Chatzisavvas et al., Electronics 2024
  [2] Hu et al., ACM EPCE 2022
  [3] Mai Jialing et al., ACM ACMLC 2025
"""

import pygame
import heapq
import time
import os

os.makedirs("figures", exist_ok=True)

# ── Colors ────────────────────────────────────────────────────────────────────
C_OBSTACLE   = (26,  26,  46)   # dark navy
C_OPEN_CELL  = (22,  33,  62)   # midnight blue
C_OPEN_LIST  = (245, 166,  35)  # orange
C_CLOSED     = (124,  77, 255)  # purple
C_PATH       = (0,  230, 118)   # bright green
C_START      = (0,  176, 255)   # cyan
C_GOAL       = (255,  23,  68)  # red
C_BG         = (10,  10,  20)   # near-black background
C_TEXT       = (220, 220, 220)  # light gray text

# ── Grid settings ─────────────────────────────────────────────────────────────
ROWS        = 20
COLS        = 20
CELL        = 28        # pixels per cell
MARGIN      = 2         # gap between cells
PANEL_H     = 60        # header panel height
WIN_W       = COLS * (CELL + MARGIN) + MARGIN
WIN_H       = ROWS * (CELL + MARGIN) + MARGIN + PANEL_H

# Dynamic weight constants from Chatzisavvas et al. [1]
EC_THRESHOLD  = 18
WEIGHT_HIGH   = 3
WEIGHT_LOW    = 0.85

# ── Grid definition ───────────────────────────────────────────────────────────
# 0 = open, 1 = obstacle
# Same layout as the C implementation demo grid (scaled to 20x20)
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

START = (0,  0)
GOAL  = (19, 19)


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def cell_rect(row, col):
    x = MARGIN + col * (CELL + MARGIN)
    y = PANEL_H + MARGIN + row * (CELL + MARGIN)
    return pygame.Rect(x, y, CELL, CELL)


def draw_panel(surf, font, title, nodes_expanded, done):
    surf.fill(C_BG, (0, 0, WIN_W, PANEL_H))
    label = font.render(title, True, C_TEXT)
    surf.blit(label, (10, 8))
    if done:
        info = font.render(f"Nodes expanded: {nodes_expanded}", True, C_PATH)
    else:
        info = font.render(f"Nodes expanded: {nodes_expanded}", True, C_OPEN_LIST)
    surf.blit(info, (10, 32))


def draw_legend(surf, small_font):
    items = [
        (C_START,     "Start"),
        (C_GOAL,      "Goal"),
        (C_PATH,      "Path"),
        (C_OPEN_LIST, "Open list"),
        (C_CLOSED,    "Closed list"),
        (C_OBSTACLE,  "Obstacle"),
    ]
    x = WIN_W - 160
    y = 6
    for color, label in items:
        pygame.draw.rect(surf, color, (x, y, 12, 12))
        text = small_font.render(label, True, C_TEXT)
        surf.blit(text, (x + 16, y))
        y += 18


def draw_grid(surf, grid, state, path_set):
    for r in range(ROWS):
        for c in range(COLS):
            rect = cell_rect(r, c)
            pos  = (r, c)

            if pos == START:
                color = C_START
            elif pos == GOAL:
                color = C_GOAL
            elif pos in path_set:
                color = C_PATH
            elif grid[r][c] == 1:
                color = C_OBSTACLE
            elif state.get(pos) == 'closed':
                color = C_CLOSED
            elif state.get(pos) == 'open':
                color = C_OPEN_LIST
            else:
                color = C_OPEN_CELL

            pygame.draw.rect(surf, color, rect)
            pygame.draw.rect(surf, C_BG, rect, 1)


def run_visualization(use_heuristic):
    """
    Runs one visualization (A* or Dijkstra) in a pygame window.
    Animates the search step by step, then highlights the final path.
    Returns (path, nodes_expanded, screenshot_path).
    """
    title  = "A* with Dynamic Weight  (k=3 far, k=0.85 near)" if use_heuristic \
             else "Dijkstra's Algorithm  (no heuristic)"
    fname  = "figures/astar_search.png" if use_heuristic \
             else "figures/dijkstra_search.png"

    pygame.init()
    surf  = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption(title)
    font       = pygame.font.SysFont("monospace", 13, bold=True)
    small_font = pygame.font.SysFont("monospace", 11)
    clock = pygame.time.Clock()

    # Search state
    state = {}          # pos -> 'open' | 'closed'
    path_set = set()
    nodes_expanded = 0
    done = False
    found_path = []

    # Priority queue: (f, h, pos)
    open_heap = []
    g_score   = {START: 0}
    came_from = {}

    h0 = manhattan(START, GOAL)
    if use_heuristic:
        k  = WEIGHT_HIGH if h0 > EC_THRESHOLD else WEIGHT_LOW
        f0 = int(0 + k * h0)
    else:
        f0 = 0

    heapq.heappush(open_heap, (f0, h0, START))
    state[START] = 'open'

    # We step through the search one node at a time for animation
    step_delay = 30   # milliseconds between steps

    running    = True
    searching  = True
    show_time  = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return found_path, nodes_expanded, fname
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    return found_path, nodes_expanded, fname

        surf.fill(C_BG)

        if searching and open_heap:
            f_cur, h_cur, current = heapq.heappop(open_heap)

            if state.get(current) == 'closed':
                pass
            else:
                state[current] = 'closed'
                nodes_expanded += 1

                if current == GOAL:
                    # Reconstruct path
                    node = GOAL
                    while node in came_from:
                        path_set.add(node)
                        node = came_from[node]
                    path_set.add(START)
                    found_path = list(path_set)
                    searching  = False
                    done       = True
                    show_time  = pygame.time.get_ticks()
                else:
                    r, c = current
                    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < ROWS and 0 <= nc < COLS:
                            if GRID[nr][nc] == 0:
                                nb = (nr, nc)
                                if state.get(nb) != 'closed':
                                    tg = g_score[current] + 1
                                    if tg < g_score.get(nb, float('inf')):
                                        came_from[nb] = current
                                        g_score[nb]   = tg
                                        h_val = manhattan(nb, GOAL)
                                        if use_heuristic:
                                            ec  = h_val
                                            k   = WEIGHT_HIGH if ec > EC_THRESHOLD \
                                                  else WEIGHT_LOW
                                            f   = int(tg + k * h_val)
                                        else:
                                            h_val = 0
                                            f     = tg
                                        heapq.heappush(open_heap, (f, h_val, nb))
                                        state[nb] = 'open'

        elif searching and not open_heap:
            searching = False
            done      = True
            show_time = pygame.time.get_ticks()

        draw_grid(surf, GRID, state, path_set)
        draw_panel(surf, font, title, nodes_expanded, done)
        draw_legend(surf, small_font)
        pygame.display.flip()

        # After search finishes, wait 1.5 seconds then save and close
        if done and pygame.time.get_ticks() - show_time > 1500:
            pygame.image.save(surf, fname)
            print(f"Saved {fname}")
            pygame.quit()
            return found_path, nodes_expanded, fname

        clock.tick(1000 // step_delay)

    pygame.quit()
    return found_path, nodes_expanded, fname


def main():
    print("Running A* visualization...")
    path_a, nodes_a, _ = run_visualization(use_heuristic=True)
    print(f"A* — nodes expanded: {nodes_a}, path length: {len(path_a)}")

    time.sleep(0.5)

    print("Running Dijkstra visualization...")
    path_d, nodes_d, _ = run_visualization(use_heuristic=False)
    print(f"Dijkstra — nodes expanded: {nodes_d}, path length: {len(path_d)}")

    print()
    print("Both figures saved to figures/")
    print("  figures/astar_search.png")
    print("  figures/dijkstra_search.png")


if __name__ == "__main__":
    main()
