"""
maze_ground_truth.py - reads the real maze (walls, coloured tiles, end cell) out of protos/Arena.proto.

Cell coordinates match the robot's own: cell (0, 0) is the first cell after Red B, x grows east, y grows north.
Directions: 0 east, 1 north, 2 west, 3 south.  Used only by the tests, never by the robot.
"""

import os
import re

CELL_PITCH              = 0.25
MAZE_LEFT_X             = 0.095      # left face of column 0
MAZE_BOTTOM_Y           = 1.33       # bottom face of row 0
COLUMNS                 = 9
ROWS                    = 4
DIRECTION_NAMES         = ("E", "N", "W", "S")
DIRECTION_STEPS         = ((1, 0), (0, 1), (-1, 0), (0, -1))


def _solids(arena_text):
    blocks = []
    for match in re.finditer(r"\n      Solid \{", arena_text):
        start = match.start() + 1
        depth = 0
        index = match.end() - 1
        while True:
            if arena_text[index] == "{": depth += 1
            elif arena_text[index] == "}":
                depth -= 1
                if depth == 0: break
            index += 1
        blocks.append(arena_text[start:index + 1])
    return blocks


def load_ground_truth(arena_proto_path=None):
    if arena_proto_path is None:
        arena_proto_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "protos", "Arena.proto")
    arena_text = open(arena_proto_path).read()
    walls      = set()          # (cx, cy, direction) that are walls
    tiles      = {}             # (cx, cy) -> "BLUE" / "GREEN"
    end_cell   = None

    def cell_index_x(world_x): return int(round((world_x - (MAZE_LEFT_X + CELL_PITCH / 2)) / CELL_PITCH))
    def cell_index_y(world_y): return int(round((world_y - (MAZE_BOTTOM_Y + CELL_PITCH / 2)) / CELL_PITCH))

    for block in _solids(arena_text):
        name        = re.search(r'name "([^"]+)"', block).group(1)
        translation = re.search(r"translation ([\d.\-]+) ([\d.\-]+) ([\d.\-]+)", block)
        size        = re.search(r"geometry Box \{\s*size ([\d.]+) ([\d.]+) ([\d.]+)", block)
        if translation is None or size is None: continue
        x, y, _   = [float(v) for v in translation.groups()]
        sx, sy, sz = [float(v) for v in size.groups()]
        if name.startswith("maze_tile_"):
            tiles[(cell_index_x(x), cell_index_y(y))] = "BLUE" if "blue" in name else "GREEN"
        elif name == "end_marker":
            end_cell = (cell_index_x(x), cell_index_y(y))
        elif name.startswith("maze_wall_") or name.startswith("s2_wall_"):
            if sy < 0.02:        # horizontal wall along x at height y
                boundary = int(round((y - MAZE_BOTTOM_Y) / CELL_PITCH))              # 0 = bottom edge of row 0 ... 4 = top edge of row 3
                for cx in range(COLUMNS):
                    centre_x = MAZE_LEFT_X + CELL_PITCH * (cx + 0.5)
                    if x - sx / 2 - 0.01 <= centre_x <= x + sx / 2 + 0.01:
                        if boundary - 1 >= 0 and boundary - 1 < ROWS: walls.add((cx, boundary - 1, 1))
                        if 0 <= boundary < ROWS: walls.add((cx, boundary, 3))
            elif sx < 0.02:      # vertical wall along y at x
                boundary = int(round((x - MAZE_LEFT_X) / CELL_PITCH))                # 0 = left edge of column 0 ... 9 = right edge of column 8
                for cy in range(ROWS):
                    centre_y = MAZE_BOTTOM_Y + CELL_PITCH * (cy + 0.5)
                    if y - sy / 2 - 0.01 <= centre_y <= y + sy / 2 + 0.01:
                        if boundary - 1 >= 0 and boundary - 1 < COLUMNS: walls.add((boundary - 1, cy, 0))
                        if 0 <= boundary < COLUMNS: walls.add((boundary, cy, 2))
    return {"walls": walls, "tiles": tiles, "end": end_cell}


def is_wall(truth, cx, cy, direction):
    return (cx, cy, direction) in truth["walls"]


def neighbors_open(truth, cell):
    """Cells reachable from `cell` through open edges, as {direction: neighbour}."""
    reachable = {}
    for direction, (step_x, step_y) in enumerate(DIRECTION_STEPS):
        neighbour = (cell[0] + step_x, cell[1] + step_y)
        if 0 <= neighbour[0] < COLUMNS and 0 <= neighbour[1] < ROWS and not is_wall(truth, cell[0], cell[1], direction):
            reachable[direction] = neighbour
    return reachable


def ascii_maze(truth, visited=None, visit_labels=None, width=2):
    """Text picture of the maze: +--+ walls, tiles B/G, E = end cell. `visited` (set) marks never-visited cells with '??'; `visit_labels` {cell: text} prints that text in the cell (set width=4 for 3-digit labels)."""
    lines = []
    for cy in range(ROWS - 1, -1, -1):
        top = "+"
        mid = ""
        for cx in range(COLUMNS):
            top += ("-" * width if is_wall(truth, cx, cy, 1) else " " * width) + "+"
        lines.append(top)
        for cx in range(COLUMNS):
            mid += "|" if is_wall(truth, cx, cy, 2) else " "
            label = " " * width
            if (cx, cy) in truth["tiles"]: label = (truth["tiles"][(cx, cy)][0] + " ").ljust(width)
            if (cx, cy) == truth["end"]: label = "E ".ljust(width)
            if visit_labels and (cx, cy) in visit_labels: label = visit_labels[(cx, cy)].rjust(width)
            elif visited is not None and (cx, cy) not in visited: label = "??".ljust(width) if label.strip() == "" else (label.strip() + "?").ljust(width)
            mid += label
        mid += "|" if is_wall(truth, COLUMNS - 1, cy, 0) else " "
        lines.append(mid)
    bottom = "+"
    for cx in range(COLUMNS):
        bottom += ("-" * width if is_wall(truth, cx, 0, 3) else " " * width) + "+"
    lines.append(bottom)
    return "\n".join(lines)


if __name__ == "__main__":
    ground_truth = load_ground_truth()
    print(ascii_maze(ground_truth))
    print("tiles:", {cell: colour for cell, colour in sorted(ground_truth["tiles"].items())})
    print("end cell:", ground_truth["end"])
    print("wall edges:", len(ground_truth["walls"]))
