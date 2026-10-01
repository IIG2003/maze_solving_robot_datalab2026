"""
gen_random_maze.py - random 9 x 4 mazes (internal walls and blue/green tile positions) for testing the controller on layouts it has never seen.
The official evaluation uses a different maze from the one in the package, so a single test maze is not enough.

What stays fixed (it is part of the locked Arena): the outer walls, the entrance at the bottom of column 0, Red B, and the red end cell (8, 3).
What is random: every internal wall (a perfect maze plus a few extra openings, so loops exist) and 2-6 tiles on dead ends.

  python3 tests/gen_random_maze.py [seed ...]         prints the mazes
  from gen_random_maze import random_maze, build_project
"""

import os
import random
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
from maze_ground_truth import load_ground_truth, ascii_maze, DIRECTION_STEPS, COLUMNS, ROWS, MAZE_LEFT_X, MAZE_BOTTOM_Y, CELL_PITCH

START_CELL = (0, 0)
END_CELL   = (8, 3)


def random_maze(seed, extra_openings=None, tile_count=None):
    """Returns {'walls': set of (cx, cy, direction), 'tiles': {(cx, cy): colour}, 'end': (8, 3)} in the same form maze_ground_truth.load_ground_truth gives."""
    rng        = random.Random(seed)
    cells      = [(x, y) for x in range(COLUMNS) for y in range(ROWS)]
    open_edges = set()                                              # frozenset({cell_a, cell_b})
    visited    = {START_CELL}
    stack      = [START_CELL]
    while stack:                                                    # randomised depth-first carving = a perfect maze
        cell       = stack[-1]
        candidates = [(cell[0] + dx, cell[1] + dy) for dx, dy in DIRECTION_STEPS if 0 <= cell[0] + dx < COLUMNS and 0 <= cell[1] + dy < ROWS and (cell[0] + dx, cell[1] + dy) not in visited]
        if not candidates: stack.pop(); continue
        neighbour = rng.choice(candidates)
        open_edges.add(frozenset((cell, neighbour)))
        visited.add(neighbour)
        stack.append(neighbour)
    all_edges = {frozenset((a, b)) for a in cells for dx, dy in DIRECTION_STEPS for b in [(a[0] + dx, a[1] + dy)] if b in set(cells)}
    closed    = sorted(all_edges - open_edges, key=lambda e: sorted(e))
    if extra_openings is None: extra_openings = rng.randint(0, 3)
    for edge in rng.sample(closed, min(extra_openings, len(closed))): open_edges.add(edge)

    walls = set()
    for a in cells:
        for direction, (dx, dy) in enumerate(DIRECTION_STEPS):
            b = (a[0] + dx, a[1] + dy)
            outside = not (0 <= b[0] < COLUMNS and 0 <= b[1] < ROWS)
            if outside:
                if (a, direction) != (START_CELL, 3): walls.add((a[0], a[1], direction))      # outer walls, except the entrance
            elif frozenset((a, b)) not in open_edges:
                walls.add((a[0], a[1], direction))

    degree     = {cell: sum(1 for edge in open_edges if cell in edge) for cell in cells}
    dead_ends  = [cell for cell in cells if degree[cell] == 1 and cell not in (START_CELL, END_CELL)]
    if tile_count is None: tile_count = rng.randint(2, 6)
    tiles = {cell: rng.choice(("BLUE", "GREEN")) for cell in rng.sample(dead_ends, min(tile_count, len(dead_ends)))}
    return {"walls": walls, "tiles": tiles, "end": END_CELL}


# ------------------------------------------------------------------------------------------------ Arena.proto text
def _solid(name, x, y, z, size, colour=None):
    appearance = "baseColor %s\n              roughness 1" % colour if colour else "roughness 1\n              metalness 0"
    bounding   = "" if colour else "\n        boundingObject Box {\n          size %s\n        }" % size
    return ('      Solid {\n        translation %.4f %.4f %s\n        children [\n          Shape {\n            appearance PBRAppearance {\n              %s\n            }\n'
            '            geometry Box {\n              size %s\n            }\n          }\n        ]\n        name "%s"%s\n      }\n') % (x, y, z, appearance, size, name, bounding)


def arena_text(spec, source_arena_path=None):
    """Arena.proto text with the internal maze walls and tiles replaced by those in `spec`; everything else is copied unchanged."""
    if source_arena_path is None: source_arena_path = os.path.join(PROJECT, "protos", "Arena.proto")
    text   = open(source_arena_path).read()
    kept, position = [], 0
    for match in re.finditer(r"\n      Solid \{", text):                  # cut out the old maze walls and tiles
        start = match.start() + 1
        depth, index = 0, match.end() - 1
        while True:
            if text[index] == "{": depth += 1
            elif text[index] == "}":
                depth -= 1
                if depth == 0: break
            index += 1
        end  = index + 1
        name = re.search(r'name "([^"]+)"', text[start:end]).group(1)
        if name.startswith("maze_wall_") or name.startswith("maze_tile_"):
            kept.append(text[position:start]); position = end + 1
    kept.append(text[position:])
    text = "".join(kept)
    new  = ""
    for (cx, cy, direction) in sorted(spec["walls"]):                        # each internal edge once
        dx, dy = DIRECTION_STEPS[direction]
        nx, ny = cx + dx, cy + dy
        if not (0 <= nx < COLUMNS and 0 <= ny < ROWS): continue
        if (nx, ny, (direction + 2) % 4) in spec["walls"] and (nx, ny) < (cx, cy): continue
        centre_x, centre_y = MAZE_LEFT_X + CELL_PITCH * (cx + 0.5), MAZE_BOTTOM_Y + CELL_PITCH * (cy + 0.5)
        if direction == 0:   new += _solid("maze_wall_v_%d_%d" % (cx, cy), centre_x + CELL_PITCH / 2, centre_y, 0.05, "0.012 0.262 0.1")
        elif direction == 2: new += _solid("maze_wall_v_%d_%d" % (cx - 1, cy), centre_x - CELL_PITCH / 2, centre_y, 0.05, "0.012 0.262 0.1")
        elif direction == 1: new += _solid("maze_wall_h_%d_%d" % (cx, cy), centre_x, centre_y + CELL_PITCH / 2, 0.05, "0.262 0.012 0.1")
        else:                new += _solid("maze_wall_h_%d_%d" % (cx, cy - 1), centre_x, centre_y - CELL_PITCH / 2, 0.05, "0.262 0.012 0.1")
    for (cx, cy), colour in sorted(spec["tiles"].items()):
        new += _solid("maze_tile_%s_%d_%d" % (colour.lower(), cx, cy), MAZE_LEFT_X + CELL_PITCH * (cx + 0.5), MAZE_BOTTOM_Y + CELL_PITCH * (cy + 0.5), 0.001, "0.25 0.25 0.001",
                      "0 0 0.5" if colour == "BLUE" else "0 0.6 0")
    tail = "\n    ]\n  }\n}\n"
    assert text.endswith(tail), "unexpected end of Arena.proto"
    return text[:-len(tail)] + "\n" + new.rstrip("\n") + tail


def build_project(spec, output_dir):
    """Copies this project into output_dir with a generated maze in protos/Arena.proto (for Webots runs). Returns the world path. Your project is not modified."""
    for folder in ("protos", "worlds", "controllers"): shutil.copytree(os.path.join(PROJECT, folder), os.path.join(output_dir, folder))
    open(os.path.join(output_dir, "protos", "Arena.proto"), "w").write(arena_text(spec))
    return os.path.join(output_dir, "worlds", "Arena.wbt")


def same_maze(truth_a, truth_b):
    return truth_a["walls"] == truth_b["walls"] and truth_a["tiles"] == truth_b["tiles"] and truth_a["end"] == truth_b["end"]


if __name__ == "__main__":
    for seed in [int(a) for a in sys.argv[1:]] or [1, 2, 3]:
        maze = random_maze(seed)
        print("seed %d  tiles %s" % (seed, dict(sorted(maze["tiles"].items()))))
        print(ascii_maze(maze), "\n")
