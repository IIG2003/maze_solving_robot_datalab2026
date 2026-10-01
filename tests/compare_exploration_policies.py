"""
compare_exploration_policies.py - which way of exploring the maze wastes the least travel?

Runs several exploration policies on the REAL maze (read from protos/Arena.proto) using the robot's own planner
(MazeKnowledge.find_route from main_controller.py), so the numbers describe the actual controller. No Webots needed.

Usage:  python3 tests/compare_exploration_policies.py
"""

import heapq
import os
import sys
import types

sys.modules.setdefault("controller", types.SimpleNamespace(Robot=object))     # main_controller imports Webots' module; a stub is enough here
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers", "main_controller"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import main_controller as mc
from maze_ground_truth import load_ground_truth, is_wall, DIRECTION_STEPS, COLUMNS, ROWS

TRUTH                        = load_ground_truth()
SECONDS_PER_CELL_MOVE        = 0.25 / 0.198 + 0.05       # cruise speed plus a little for speeding up and slowing
SECONDS_PER_QUARTER_TURN     = 0.75                      # stop, pivot 90 degrees, restart


def sense(knowledge, cell, entrance_closed_here):
    """What the robot learns on entering a cell: all four edges (the one behind it is open because it just came from there)."""
    for direction in range(4):
        step_x, step_y = DIRECTION_STEPS[direction]
        neighbour      = (cell[0] + step_x, cell[1] + step_y)
        outside        = not (0 <= neighbour[0] < COLUMNS and 0 <= neighbour[1] < ROWS)
        wall           = is_wall(TRUTH, cell[0], cell[1], direction) or outside
        if entrance_closed_here and direction == 3: wall = True         # the way back to Red B counts as a wall
        knowledge.record_wall(cell, direction, wall)
    knowledge.visited_cells.add(cell)
    if cell in TRUTH["tiles"]: knowledge.tile_colour_by_cell[cell] = TRUTH["tiles"][cell]
    if cell == TRUTH["end"]: knowledge.end_cell = cell


class Run:
    def __init__(self):
        self.cell          = (0, 0)
        self.heading       = 1
        self.moves         = 0
        self.quarter_turns = 0
        self.first_red_move = None
        self.complete_move  = None
        self.path          = [self.cell]
        self.knowledge     = mc.MazeKnowledge()
        sense(self.knowledge, self.cell, True)

    def step(self, direction):
        turn = min((direction - self.heading) % 4, (self.heading - direction) % 4)
        self.quarter_turns += turn
        step_x, step_y = DIRECTION_STEPS[direction]
        self.cell      = (self.cell[0] + step_x, self.cell[1] + step_y)
        self.heading   = direction
        self.moves    += 1
        self.path.append(self.cell)
        if self.cell not in self.knowledge.visited_cells:
            sense(self.knowledge, self.cell, False)
            if self.cell == TRUTH["end"] and self.first_red_move is None: self.first_red_move = self.moves

    def seconds(self):
        return self.moves * SECONDS_PER_CELL_MOVE + self.quarter_turns * SECONDS_PER_QUARTER_TURN


def finish(run):
    """After exploring: route to the end cell with the robot's own planner."""
    run.complete_move = run.moves
    if run.cell != TRUTH["end"]:
        for direction in run.knowledge.find_route(run.cell, run.heading, TRUTH["end"]): run.step(direction)
    return run


def policy_nearest_frontier(turn_penalty):
    """The controller's current policy: always go to the nearest unvisited cell (extra cost per 90-degree turn)."""
    mc.RobotConfiguration.TURN_PENALTY_IN_CELL_STEPS = turn_penalty
    run = Run()
    while True:
        route = run.knowledge.find_route(run.cell, run.heading)
        if route is None: break
        for direction in route: run.step(direction)
    return finish(run)


def policy_depth_first():
    """Classic depth-first: go to an unvisited neighbour (straight first, then left, then right); backtrack one cell at a time when boxed in."""
    run   = Run()
    stack = [run.cell]
    while stack:
        cell       = run.cell
        candidates = []
        for offset in (0, 1, 3):                                   # straight, left (counter-clockwise), right
            direction = (run.heading + offset) % 4
            step_x, step_y = DIRECTION_STEPS[direction]
            neighbour = (cell[0] + step_x, cell[1] + step_y)
            if run.knowledge.is_open(cell, direction) and neighbour not in run.knowledge.visited_cells: candidates.append(direction)
        if candidates:
            run.step(candidates[0])
            stack.append(run.cell)
        else:
            stack.pop()
            if not stack: break
            previous = stack[-1]
            for direction in range(4):
                step_x, step_y = DIRECTION_STEPS[direction]
                if (cell[0] + step_x, cell[1] + step_y) == previous and run.knowledge.is_open(cell, direction): run.step(direction); break
    return finish(run)


def frontier_choices(knowledge, start_cell, start_heading):
    """Dijkstra over (cell, heading) like MazeKnowledge.find_route, but returns the cheapest route to EVERY reachable unvisited cell: {cell: (cost, route)}."""
    heap, settled, counter, choices = [(0.0, 0, start_cell, start_heading, [])], set(), 1, {}
    while heap:
        cost, _, cell, heading, route = heapq.heappop(heap)
        if (cell, heading) in settled: continue
        settled.add((cell, heading))
        if cell not in knowledge.visited_cells:
            if cell not in choices or cost < choices[cell][0]: choices[cell] = (cost, route)
            continue
        for direction in range(4):
            if not knowledge.is_open(cell, direction): continue
            turns = min((direction - heading) % 4, (heading - direction) % 4)
            step_x, step_y = DIRECTION_STEPS[direction]
            heapq.heappush(heap, (cost + 1.0 + mc.RobotConfiguration.TURN_PENALTY_IN_CELL_STEPS * turns, counter, (cell[0] + step_x, cell[1] + step_y), direction, route + [direction]))
            counter += 1
    return choices


def distance_from_start(knowledge):
    distance, queue = {(0, 0): 0}, [(0, 0)]
    for cell in queue:
        for direction in range(4):
            if not knowledge.is_open(cell, direction): continue
            step_x, step_y = DIRECTION_STEPS[direction]
            neighbour = (cell[0] + step_x, cell[1] + step_y)
            if neighbour not in distance:
                distance[neighbour] = distance[cell] + 1
                if neighbour in knowledge.visited_cells: queue.append(neighbour)
    return distance


def policy_start_biased(alpha, turn_penalty=0.5):
    """Nearest unvisited cell, but each candidate also pays alpha x its distance from the START. alpha > 0 finishes the rooms near the start before heading far away."""
    mc.RobotConfiguration.TURN_PENALTY_IN_CELL_STEPS = turn_penalty
    run = Run()
    while True:
        choices = frontier_choices(run.knowledge, run.cell, run.heading)
        if not choices: break
        from_start = distance_from_start(run.knowledge)
        best = min(choices.items(), key=lambda item: item[1][0] + alpha * from_start.get(item[0], 99))
        for direction in best[1][1]: run.step(direction)
    return finish(run)


def policy_stop_when_red_seen():
    """Not allowed for scoring on its own (the majority could be wrong) - shown only to see how much of the maze the robot would skip."""
    mc.RobotConfiguration.TURN_PENALTY_IN_CELL_STEPS = 0.5
    run = Run()
    while run.knowledge.end_cell is None:
        route = run.knowledge.find_route(run.cell, run.heading)
        if route is None: break
        for direction in route: run.step(direction)
    run.complete_move = run.moves
    return run


def report(name, run, note=""):
    knowledge = run.knowledge
    found     = {cell: colour for cell, colour in knowledge.tile_colour_by_cell.items()}
    after_red = "-"
    if run.first_red_move is not None and run.complete_move is not None: after_red = str(run.complete_move - run.first_red_move)
    print("%-30s cells %2d/36 | tiles %d/5 | moves %3d (exploring %3d) | quarter-turns %3d | est. %5.1f s | moves after red first seen %s %s" %
          (name, len(knowledge.visited_cells), len(found), run.moves, run.complete_move or 0, run.quarter_turns, run.seconds(), after_red, note))


if __name__ == "__main__":
    print("Real maze (E = end cell, B/G = blue/green tile):\n")
    from maze_ground_truth import ascii_maze
    print(ascii_maze(TRUTH)); print()
    results = {}
    for label, run in (("nearest, turn penalty 0.0", policy_nearest_frontier(0.0)), ("nearest, turn penalty 0.5 (now)", policy_nearest_frontier(0.5)),
                       ("nearest, turn penalty 1.5", policy_nearest_frontier(1.5)), ("depth-first, straight first", policy_depth_first()),
                       ("start-biased alpha 0.25", policy_start_biased(0.25)), ("start-biased alpha 0.5", policy_start_biased(0.5)), ("start-biased alpha 1.0", policy_start_biased(1.0)),
                       ("start-biased alpha 2.0", policy_start_biased(2.0)), ("start-biased alpha 4.0", policy_start_biased(4.0))):
        report(label, run); results[label] = run
    print()
    report("(stop when red first seen)", policy_stop_when_red_seen(), "<- would skip the rest; unsafe for the majority count")
    print("\nMinimum possible moves to visit all 36 cells of this maze tree and end at E is about 2*35 - (path length to E). Extra moves = revisited cells = 'wandering'.")
    current = results["nearest, turn penalty 0.5 (now)"]
    print("\nVisit order of the current policy (move number each cell was FIRST entered):")
    order = {}
    for move_number, cell in enumerate(current.path):
        order.setdefault(cell, move_number)
    print(ascii_maze(TRUTH, visit_labels={cell: str(n) for cell, n in order.items()}))
