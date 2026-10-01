"""
test_failure_handling.py - offline checks (no Webots) of the safety nets ported from the datalab2026 controller:
unexpected-wall detection, loop guard, banned targets, map forgetting, early stop.  Run:  python3 tests/test_failure_handling.py
"""

import os
import sys
import types

sys.modules.setdefault("controller", types.SimpleNamespace(Robot=object))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers", "main_controller"))
import main_controller as mc

failures = []


def check(label, condition):
    print("  [%s] %s" % ("PASS" if condition else "FAIL", label))
    if not condition: failures.append(label)


print("wall_blocks_leg - must stay silent in normal driving, fire only for a wall the map missed")
facing = mc.RobotConfiguration.FRONT_SENSOR_READING_WHEN_CELL_CENTER_FACES_WALL_METERS
for remaining in (0.25, 0.20, 0.125, 0.06, 0.0, -0.01):
    consistent = remaining + facing                        # wall at the END of the target cell, as in every normal leg
    check("normal leg, wall at the end of the target cell, remaining %.3f" % remaining, not mc.wall_blocks_leg(consistent - facing, remaining, 0.0, consistent))
    beyond = remaining + 0.25 + facing                     # wall one cell further on (the beam sees it from far away)
    check("normal leg, wall one cell beyond the target, remaining %.3f" % remaining, not mc.wall_blocks_leg(beyond - facing, remaining, 0.0, beyond))
check("open corridor, nothing in range", not mc.wall_blocks_leg(0.5 - facing, 0.25, 0.0, 0.5))
check("unexpected wall right in the current cell (front 0.079, target 0.25 away)", mc.wall_blocks_leg(0.0, 0.25, 0.0, 0.079))
check("unexpected wall 0.12 m ahead of a 0.25 m leg", mc.wall_blocks_leg(0.12 + 0.04 - facing - 0.04, 0.25, 0.0, 0.12))
check("not aligned (still turning): never fires", not mc.wall_blocks_leg(0.0, 0.25, 0.3, 0.079))
check("arc entry leg (target is half a cell short of the centre): wall at the cell's front edge is consistent", not mc.wall_blocks_leg(0.125, 0.0 + 0.125 - 0.125 + 0.125, 0.0, 0.125 + facing))

print("majority_is_decided")
check("3 vs 2 with 10 cells unvisited: undecided", not mc.majority_is_decided(2, 3, 10))
check("5 vs 1 with 3 cells unvisited: decided", mc.majority_is_decided(1, 5, 3))
check("tie with nothing unvisited: not decided by lead", not mc.majority_is_decided(2, 2, 0))
check("default: early stop is OFF", mc.RobotConfiguration.EARLY_STOP_WHEN_MAJORITY_DECIDED == 0)

print("LoopGuard")
guard = mc.LoopGuard(limit=3, ban_seconds=30.0)
results = [guard.note((2, 3), t) for t in (1, 2, 3, 4)]
check("banned only on the 4th failure in a row", results == [False, False, False, True])
check("banned target is reported while the ban lasts", (2, 3) in guard.banned_cells(10.0))
check("ban expires after 30 s", (2, 3) not in guard.banned_cells(40.0))
guard = mc.LoopGuard(limit=3, ban_seconds=30.0)
for target in ((1, 1), (1, 1), (1, 1), (2, 2), (1, 1), (1, 1), (1, 1)): guard.note(target, 0.0)
check("a different target in between resets the count", guard.banned_cells(1.0) == set())

print("OccupancyGrid.clear_around")
grid = mc.OccupancyGrid()
near_cell, far_cell = grid.world_to_cell(1.00, 1.00), grid.world_to_cell(2.00, 1.00)
grid.mark_occupied(near_cell); grid.mark_occupied(far_cell)
version_before = grid.change_version
check("both cells start blocked", near_cell in grid.normal_blocked_counts and far_cell in grid.normal_blocked_counts)
grid.clear_around(1.05, 1.0, 0.20)
check("obstacle near the point is forgotten", grid.state_of(near_cell) == mc.OccupancyGrid.UNKNOWN and near_cell not in grid.normal_blocked_counts)
check("obstacle far away is kept, with its inflation", grid.state_of(far_cell) == mc.OccupancyGrid.OCCUPIED and far_cell in grid.normal_blocked_counts and far_cell in grid.clearance_penalty_counts)
check("map version changed (so the route is replanned)", grid.change_version > version_before)
check("inflation counts are exactly what a fresh grid would have", grid.normal_blocked_counts == {(far_cell[0] + dx, far_cell[1] + dy): 1 for dx, dy in grid.normal_offsets})

print("MazeKnowledge")
knowledge = mc.MazeKnowledge()
knowledge.origin_x, knowledge.origin_y = 1.0, 2.0
check("cell_at maps positions to the nearest cell centre", knowledge.cell_at(1.0, 2.0) == (0, 0) and knowledge.cell_at(1.26, 2.49) == (1, 2) and knowledge.cell_at(0.76, 1.74) == (-1, -1))
for x in range(3):
    knowledge.visited_cells.add((x, 0))
    if x < 2: knowledge.record_wall((x, 0), 0, False)
knowledge.record_wall((2, 0), 0, False)
check("unbanned: nearest unvisited cell is found", knowledge.find_route((0, 0), 0) == [0, 0, 0])
check("banned target is skipped (no other target: None)", knowledge.find_route((0, 0), 0, None, {(3, 0)}) is None)
check("conflicting wall reading is logged for diagnosis", (knowledge.record_wall((1, 0), 0, True), len(knowledge.conflicts))[1] == 1)

print("look-ahead camera (offline: pixel vote, beam targets, peek bookkeeping)")
check("look-ahead switch is a plain 0/1 flag", mc.RobotConfiguration.LOOK_AHEAD_ENABLED in (0, 1))
check("blue pixels vote BLUE", mc.vote_floor_colour([(0, 0, 55)] * 48) == "BLUE")
check("green pixels vote GREEN", mc.vote_floor_colour([(0, 66, 0)] * 48) == "GREEN")
check("red pixels vote RED", mc.vote_floor_colour([(108, 0, 0)] * 48) == "RED")
check("black floor votes BLACK", mc.vote_floor_colour([(29, 29, 29)] * 48) == "BLACK")
check("white / bright grey is never a tile", mc.vote_floor_colour([(220, 220, 220)] * 48) == "UNKNOWN")
check("too dark to trust (shaded blue) is UNKNOWN, not guessed", mc.vote_floor_colour([(0, 0, 20)] * 48) == "UNKNOWN")
check("a 50/50 mix is UNKNOWN", mc.vote_floor_colour([(0, 0, 55)] * 24 + [(0, 66, 0)] * 24) == "UNKNOWN")
check("one stray pixel in 48 does not change the vote", mc.vote_floor_colour([(0, 0, 55)] * 47 + [(0, 66, 0)]) == "BLUE")
check("empty sample is UNKNOWN", mc.vote_floor_colour([]) == "UNKNOWN")
front, left, right = mc.closed_dead_end_beam_targets()
print("    beam targets for a closed dead end ahead: front %.3f, left 30deg %.3f, right 30deg %.3f" % (front, left, right))
check("front target is one cell further than a wall at the end of the current cell", abs(front - (mc.RobotConfiguration.FRONT_SENSOR_READING_WHEN_CELL_CENTER_FACES_WALL_METERS + 0.25)) < 1e-9)
check("30 degree side targets are inside the 0.5 m range and symmetric", 0.1 < left < 0.5 and abs(left - right) < 1e-9)
good = {"ds_f0": front, "ds_fl30": left, "ds_fr30": right}
check("centred, aligned, all three beams agree: resolved", mc.look_ahead_resolves_dead_end(good, 0.0, 0.0, 0.0))
check("open corridor ahead (front 0.5): not resolved", not mc.look_ahead_resolves_dead_end(dict(good, ds_f0=0.5), 0.0, 0.0, 0.0))
check("left side open (beam sees far wall): not resolved", not mc.look_ahead_resolves_dead_end(dict(good, ds_fl30=0.5), 0.0, 0.0, 0.0))
check("right side open: not resolved", not mc.look_ahead_resolves_dead_end(dict(good, ds_fr30=0.5), 0.0, 0.0, 0.0))
check("a wall at the end of the CURRENT cell is not a dead end ahead", not mc.look_ahead_resolves_dead_end(dict(good, ds_f0=0.079), 0.0, 0.0, 0.0))
check("robot 3 cm off the centre line: not trusted", not mc.look_ahead_resolves_dead_end(good, 0.0, 0.03, 0.0))
check("robot 3 cm past the cell centre: not trusted", not mc.look_ahead_resolves_dead_end(good, 0.03, 0.0, 0.0))
check("robot rotated 0.1 rad: not trusted", not mc.look_ahead_resolves_dead_end(good, 0.0, 0.0, 0.1))

class FakeRobot:
    look_camera = object()
    odometry_position_x_meters = 0.0
    odometry_position_y_meters = 0.0
    yaw_radians = 0.0
    simulation_time_seconds = 0.0
    distance_readings_meters = dict(good)
    colour = "BLUE"
    pitch_radians = 0.0
    last_look_ahead_mean_rgb = (0, 0, 0)
    roll_radians = 0.0
    def read_look_ahead_colour(self): return self.colour

def fresh_knowledge():
    k = mc.MazeKnowledge()
    k.record_wall((0, 0), 0, False)          # the door between the cells
    k.visited_cells.add((0, 0))
    return k

k = fresh_knowledge(); robot = FakeRobot()
check("peek returns True for a closed BLUE dead end", mc.peek_at_dead_end_ahead(robot, k, (0, 0), 0))
check("  cell (1,0) is now visited and counted BLUE", (1, 0) in k.visited_cells and k.tile_colour_by_cell.get((1, 0)) == "BLUE" and (1, 0) in k.peeked_cells)
check("  its three far walls are known, the door stays open", k.wall_between[((1, 0), 0)] and k.wall_between[((1, 0), 1)] and k.wall_between[((1, 0), 3)] and k.is_open((0, 0), 0))
check("  the planner no longer wants to enter it", k.find_route((0, 0), 0) is None)
k = fresh_knowledge(); robot = FakeRobot(); robot.colour = "RED"
check("a RED reading is never trusted (a red wall looks the same): robot enters, floor camera decides", not mc.peek_at_dead_end_ahead(robot, k, (0, 0), 0) and k.end_cell is None and (1, 0) not in k.visited_cells)
k = fresh_knowledge(); robot = FakeRobot(); robot.pitch_radians = 0.08
check("chassis pitched 0.08 rad: look-ahead ignored", not mc.peek_at_dead_end_ahead(robot, k, (0, 0), 0))
k = fresh_knowledge(); robot = FakeRobot(); robot.colour = "UNKNOWN"
check("unclear colour: nothing recorded, robot will enter", not mc.peek_at_dead_end_ahead(robot, k, (0, 0), 0) and (1, 0) not in k.visited_cells)
k = fresh_knowledge(); robot = FakeRobot(); robot.distance_readings_meters = dict(good, ds_f0=0.5)
check("beams say corridor continues: nothing recorded", not mc.peek_at_dead_end_ahead(robot, k, (0, 0), 0) and (1, 0) not in k.visited_cells)
k = mc.MazeKnowledge(); k.visited_cells.add((0, 0)); robot = FakeRobot()
check("door state unknown: nothing recorded", not mc.peek_at_dead_end_ahead(robot, k, (0, 0), 0))
robot = FakeRobot(); robot.look_camera = None; k = fresh_knowledge()
check("no look camera on the robot: safe no-op", not mc.peek_at_dead_end_ahead(robot, k, (0, 0), 0))
k = fresh_knowledge(); robot = FakeRobot(); robot.yaw_radians = 1.5708; k.record_wall((0, 0), 1, False); robot.distance_readings_meters = dict(good)
check("heading north uses the north neighbour and world axes", mc.peek_at_dead_end_ahead(robot, k, (0, 0), 1) and (0, 1) in k.peeked_cells)

print("\n%s" % ("ALL OFFLINE CHECKS PASSED" if not failures else "FAILED: %s" % failures))
sys.exit(1 if failures else 0)
