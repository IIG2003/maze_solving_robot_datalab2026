"""
test_section1_clearance.py - offline (no Webots): how close does the Section 1 route come to the cubes and walls?

Builds the TRUE Section 1 map from protos/Obstacles.proto and protos/Arena.proto (perfect knowledge, so only the planning/smoothing/tracking is measured),
plans with the robot's own GridPathPlanner, smooths the route, then simulates the pure-pursuit tracking used in run_section_1_obstacle_field
and reports the smallest distance from the axle to any obstacle along the planned route and along the tracked path.

  python3 tests/test_section1_clearance.py            compares smoothing that ignores clearance (old) with smoothing that respects it (new)
"""

import math
import os
import re
import sys
import types

sys.modules.setdefault("controller", types.SimpleNamespace(Robot=object))
HERE    = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(PROJECT, "controllers", "main_controller"))
import main_controller as mc

START_WORLD = (2.315, 0.125)


def load_boxes():
    """Axis-aligned boxes (centre_x, centre_y, half_x, half_y) in WORLD coordinates: the 43 cubes and the Section 1 walls."""
    boxes = []
    text  = open(os.path.join(PROJECT, "protos", "Obstacles.proto")).read()
    for match in re.finditer(r"translation ([-\d.]+) ([-\d.]+) [-\d.]+", text): boxes.append((float(match.group(1)), float(match.group(2)), 0.05, 0.05))
    arena = open(os.path.join(PROJECT, "protos", "Arena.proto")).read()
    for match in re.finditer(r'Solid \{\s*translation ([-\d.]+) ([-\d.]+) ([-\d.]+)(.*?)name "([^"]+)"', arena, re.S):
        if not (match.group(5).startswith("s1_wall") or match.group(5).startswith("chamber_wall")): continue
        size = [float(v) for v in re.search(r"size ([\d. ]+)", match.group(4)).group(1).split()]
        boxes.append((float(match.group(1)), float(match.group(2)), size[0] / 2.0, size[1] / 2.0))
    return boxes


def distance_to_box(x, y, box):
    centre_x, centre_y, half_x, half_y = box
    return math.hypot(max(abs(x - centre_x) - half_x, 0.0), max(abs(y - centre_y) - half_y, 0.0))


def minimum_clearance(points, boxes):
    return min(min(distance_to_box(x, y, box) for box in boxes) for x, y in points)


def build_grid(boxes):
    grid = mc.OccupancyGrid()
    size = grid.cell_size_meters
    for centre_x, centre_y, half_x, half_y in boxes:
        centre_x -= START_WORLD[0]
        centre_y -= START_WORLD[1]
        for cell_x in range(int(math.floor((centre_x - half_x) / size)), int(math.floor((centre_x + half_x) / size)) + 1):
            for cell_y in range(int(math.floor((centre_y - half_y) / size)), int(math.floor((centre_y + half_y) / size)) + 1): grid.mark_occupied((cell_x, cell_y))
    for cell_x in range(-120, 20):                                       # known free floor, so A* does not wander outside the arena
        for cell_y in range(-10, 60):
            if grid.cell_states.get((cell_x, cell_y)) != 2 and 0.0 < cell_x * size + START_WORLD[0] < 2.44 and 0.0 < cell_y * size + START_WORLD[1] < 1.22: grid.cell_states[(cell_x, cell_y)] = 1
    return grid


def track(waypoints, start_x, start_y, start_yaw, boxes):
    """Same law as run_section_1_obstacle_field without the sensors: pick_lookahead_point + turn_rate = 3 x heading error, speed 0.19 m/s. Returns the driven points."""
    class Pose: pass
    pose = Pose()
    pose.odometry_position_x_meters, pose.odometry_position_y_meters, pose.yaw_radians = start_x, start_y, start_yaw
    waypoints = list(waypoints)
    path = [(start_x, start_y)]
    dt   = 0.016
    steps_on_waypoint, remaining_waypoints = 0, len(waypoints)
    for _ in range(int(60 / dt)):
        if len(waypoints) != remaining_waypoints: steps_on_waypoint, remaining_waypoints = 0, len(waypoints)
        steps_on_waypoint += 1
        if steps_on_waypoint * dt > 4.0: break                       # circling one waypoint: a flaw of this simple tracker (the real robot also slows and has stall recovery), stop here
        if math.hypot(waypoints[-1][0] - pose.odometry_position_x_meters, waypoints[-1][1] - pose.odometry_position_y_meters) < 0.05: break
        target_x, target_y = mc.pick_lookahead_point(pose, waypoints)
        error  = mc.MathHelpers.wrap_angle_to_plus_minus_pi(math.atan2(target_y - pose.odometry_position_y_meters, target_x - pose.odometry_position_x_meters) - pose.yaw_radians)
        turn   = mc.MathHelpers.clamp_value(3.0 * error, -2.5, 2.5)
        speed  = 0.19 * max(0.2, 1.0 - abs(error) / 1.2) if abs(error) <= 0.8 else 0.03
        pose.yaw_radians += turn * dt
        pose.odometry_position_x_meters += speed * math.cos(pose.yaw_radians) * dt
        pose.odometry_position_y_meters += speed * math.sin(pose.yaw_radians) * dt
        path.append((pose.odometry_position_x_meters, pose.odometry_position_y_meters))
    return path


def densify(waypoints):
    points = []
    for (x0, y0), (x1, y1) in zip(waypoints, waypoints[1:]):
        steps = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.01))
        points += [(x0 + (x1 - x0) * i / steps, y0 + (y1 - y0) * i / steps) for i in range(steps + 1)]
    return points


def evaluate(respect_clearance, starts):
    mc.RobotConfiguration.SMOOTH_RESPECTS_CLEARANCE = 1 if respect_clearance else 0
    boxes   = [(x - START_WORLD[0], y - START_WORLD[1], hx, hy) for x, y, hx, hy in load_boxes()]
    grid    = build_grid(load_boxes())
    planner = mc.GridPathPlanner(grid)
    gate    = mc.RobotConfiguration.SECTION_1_GATE_OFFSET_FROM_START_METERS
    goal    = grid.world_to_cell(gate[0], gate[1])
    rows    = []
    for start_x, start_y in starts:
        start_cell = grid.world_to_cell(start_x, start_y)
        route      = planner.plan(start_cell, goal, False)
        if route is None: rows.append((start_x, start_y, None, None, None)); continue
        cells      = planner.smooth(route, start_cell, False)
        waypoints  = [grid.cell_to_world(cell) for cell in cells]
        length     = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(waypoints, waypoints[1:]))
        planned    = minimum_clearance(densify(waypoints), boxes)
        driven     = minimum_clearance(track(waypoints, start_x, start_y, math.pi, boxes), boxes)
        rows.append((start_x, start_y, length, planned, driven))
    return rows


if __name__ == "__main__":
    starts = [(0.0, 0.0), (0.04, 0.03), (-0.05, 0.05), (0.05, -0.03), (-0.04, -0.04), (0.0, 0.06)]       # inside Red A (25 cm square) around the start point
    print("minimum distance from the AXLE to any cube or wall (robot half-width 0.025 m, rear corner reach 0.056 m; below ~0.06 m the body is touching):")
    results = {}
    for label, flag in (("old smoothing (ignores clearance)", False), ("new smoothing (respects clearance)", True)):
        rows = evaluate(flag, starts)
        results[label] = rows
        print("\n" + label)
        for start_x, start_y, length, planned, driven in rows:
            if length is None: print("  start (%+.2f, %+.2f): NO ROUTE" % (start_x, start_y)); continue
            print("  start (%+.2f, %+.2f): route %.2f m | clearance on planned lines %.3f m | clearance while tracking %.3f m" % (start_x, start_y, length, planned, driven))
        ok = [row for row in rows if row[2] is not None]
        print("  mean route %.2f m | worst planned %.3f m | worst tracked %.3f m" % (sum(r[2] for r in ok) / len(ok), min(r[3] for r in ok), min(r[4] for r in ok)))
