"""
test_random_mazes.py - offline (no Webots): does the controller's own route planner explore ANY valid maze correctly?
 1. every generated Arena.proto reads back as exactly the maze that was generated
 2. on N random mazes the planner visits all 36 cells, finds every tile and the end cell, and reaches the end

  python3 tests/test_random_mazes.py [number_of_mazes]
"""

import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import compare_exploration_policies as cp
from gen_random_maze import random_maze, arena_text, same_maze
from maze_ground_truth import load_ground_truth, neighbors_open, COLUMNS, ROWS
from collections import deque

count = int(sys.argv[1]) if len(sys.argv) > 1 else 300

print("1. round trip: generated Arena.proto text -> reader")
bad = 0
with tempfile.TemporaryDirectory() as folder:
    for seed in range(40):
        spec = random_maze(seed)
        path = os.path.join(folder, "Arena.proto")
        open(path, "w").write(arena_text(spec))
        if not same_maze(load_ground_truth(path), spec): bad += 1
print("   %d of 40 differ" % bad)

print("2. planner on %d random mazes" % count)
failures, moves_list, ratios, lower_bounds = [], [], [], []
for seed in range(count):
    spec = random_maze(seed)
    cp.TRUTH = spec
    run = cp.policy_nearest_frontier(0.5)
    problems = []
    if len(run.knowledge.visited_cells) != COLUMNS * ROWS: problems.append("visited %d/36" % len(run.knowledge.visited_cells))
    if dict(run.knowledge.tile_colour_by_cell) != spec["tiles"]: problems.append("tiles %s vs %s" % (dict(run.knowledge.tile_colour_by_cell), spec["tiles"]))
    if run.cell != spec["end"]: problems.append("ended at %s, not %s" % (run.cell, spec["end"]))
    if problems: failures.append((seed, problems))
    moves_list.append(run.moves)
    distance = {(0, 0): 0}; queue = deque([(0, 0)])                         # shortest path start -> end, for the lower bound of a tree maze
    while queue:
        cell = queue.popleft()
        for neighbour in neighbors_open(spec, cell).values():
            if neighbour not in distance: distance[neighbour] = distance[cell] + 1; queue.append(neighbour)
    lower_bounds.append(2 * (COLUMNS * ROWS - 1) - distance[spec["end"]])
    ratios.append(run.moves / lower_bounds[-1])

print("   mazes failing: %d of %d" % (len(failures), count))
for seed, problems in failures[:5]: print("     seed %d: %s" % (seed, "; ".join(problems)))
print("   moves: min %d, mean %.1f, max %d | lower bound (tree) mean %.1f | moves / lower bound: mean %.2f, worst %.2f" % (min(moves_list), sum(moves_list) / count, max(moves_list), sum(lower_bounds) / count, sum(ratios) / count, max(ratios)))
sys.exit(1 if failures or bad else 0)
