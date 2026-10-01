"""
run_maze_test.py - runs the real controller in Webots (headless, on a temporary copy of the project) and compares what the robot
LEARNED about the maze with the TRUE maze from protos/Arena.proto.  Your project files are never modified.

What it checks
  1. every wall the robot believes in (per visited cell and direction) against the true wall
  2. blue/green tiles found vs the true tiles, and the red end cell
  3. coverage: how many of the 36 cells were visited
  4. "matrix cleared" suspicion: did any wall reading CONTRADICT an earlier reading of the same edge (conflicts)
  5. visit order and move count vs what the robot's planner does on the true maze offline (should be identical)

Usage:  python3 tests/run_maze_test.py [scenario ...]        scenarios: red_b_north  red_b_south  full_course
"""

import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

HERE        = os.path.dirname(os.path.abspath(__file__))
PROJECT     = os.path.abspath(os.path.join(HERE, ".."))
WEBOTS      = os.environ.get("WEBOTS", "/Applications/Webots.app/Contents/MacOS/webots")
sys.path.insert(0, HERE)
from maze_ground_truth import load_ground_truth, ascii_maze, DIRECTION_NAMES, COLUMNS, ROWS

SCENARIOS = {
    "red_b_north": (0.22, 1.275, math.pi / 2,  "Section 2 only: placed on Red B facing the maze"),
    "red_b_south": (0.22, 1.275, -math.pi / 2, "Section 2 only: placed on Red B facing AWAY from the maze (must turn around)"),
    "full_course": (2.315, 0.125, math.pi,     "whole course: Red A, through Section 1, into the maze"),
}


def build_world(scenario, directory):
    x, y, yaw, _ = SCENARIOS[scenario]
    for folder in ("protos", "worlds", "controllers"): shutil.copytree(os.path.join(PROJECT, folder), os.path.join(directory, folder))
    shutil.rmtree(os.path.join(directory, "controllers", "main_controller", "__pycache__"), ignore_errors=True)
    world_path = os.path.join(directory, "worlds", "Arena.wbt")
    world      = open(world_path).read()
    world      = re.sub(r'EXTERNPROTO "https[^\n]*\n', "", world)                       # no network needed
    world      = re.sub(r"TexturedBackground(Light)? \{[^}]*\}\n", "", world)
    world      = re.sub(r"TwoWheelRobot \{.*\}\s*$", 'TwoWheelRobot {\n  translation %f %f 0.0\n  rotation 0 0 1 %f\n  name "team_robot_template"\n  controller "main_controller"\n  bodyColor 1 0.55 0\n}\n' % (x, y, yaw), world, flags=re.S)
    open(world_path, "w").write(world)
    controller_path = os.path.join(directory, "controllers", "main_controller", "main_controller.py")
    source          = open(controller_path).read()
    assert "PRINT_PROGRESS = False" in source
    open(controller_path, "w").write(source.replace("PRINT_PROGRESS = False", "PRINT_PROGRESS = True"))
    return world_path


def run_webots(world_path, wall_clock_limit_seconds):
    """Starts Webots, collects the controller's printed lines, stops as soon as the 'done' knowledge dump arrives."""
    process = subprocess.Popen([WEBOTS, "--batch", "--mode=fast", "--no-rendering", "--stdout", "--stderr", world_path], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    lines, deadline, finished = [], time.time() + wall_clock_limit_seconds, False
    try:
        for line in process.stdout:
            lines.append(line.rstrip("\n"))
            if line.startswith("KNOWLEDGE") and '"label": "done"' in line: finished = True; break
            if time.time() > deadline: break
    finally:
        process.terminate()
        try: process.wait(timeout=10)
        except subprocess.TimeoutExpired: process.kill()
    return lines, finished


def parse_knowledge(lines):
    dumps = {}
    for line in lines:
        if line.startswith("KNOWLEDGE "):
            payload = json.loads(line[len("KNOWLEDGE "):])
            dumps[payload["label"]] = payload
    return dumps


def compare(dump, truth):
    """Returns a dict of findings comparing one knowledge dump with the true maze."""
    findings = {"wall_mismatches": [], "walls_checked": 0, "tile_problems": [], "conflicts": dump["conflicts"]}
    for key, believed in dump["walls"].items():
        cx, cy, direction = [int(v) for v in key.split(",")]
        if not (0 <= cx < COLUMNS and 0 <= cy < ROWS): continue
        step_x, step_y = ((1, 0), (0, 1), (-1, 0), (0, -1))[direction]
        outside         = not (0 <= cx + step_x < COLUMNS and 0 <= cy + step_y < ROWS)
        real            = ((cx, cy, direction) in truth["walls"]) or outside
        entrance        = (cx, cy, direction) == (0, 0, 3)            # the way back to Red B is deliberately treated as a wall
        findings["walls_checked"] += 1
        if believed != real and not (entrance and believed):
            findings["wall_mismatches"].append(((cx, cy, DIRECTION_NAMES[direction]), "robot says %s, truth says %s" % ("wall" if believed else "open", "wall" if real else "open")))
    found = {tuple(int(v) for v in key.split(",")): colour for key, colour in dump["tiles"].items()}
    for cell, colour in truth["tiles"].items():
        if cell not in found: findings["tile_problems"].append("missed %s tile at %s" % (colour, cell))
        elif found[cell] != colour: findings["tile_problems"].append("tile at %s is %s but the robot saw %s" % (cell, colour, found[cell]))
    for cell in found:
        if cell not in truth["tiles"]: findings["tile_problems"].append("false tile %s at %s" % (found[cell], cell))
    findings["end_ok"] = tuple(dump["end"]) == truth["end"] if dump["end"] else False
    return findings


def offline_prediction():
    """What the robot's planner should do on the true maze (same code as the controller)."""
    import compare_exploration_policies as policies
    run = policies.policy_nearest_frontier(0.5)
    order, seen = [], set()
    for cell in run.path:
        if cell not in seen: seen.add(cell); order.append(cell)
    return order, run


def run_scenario(name, truth, wall_clock_limit_seconds=420):
    print("=" * 100)
    print("SCENARIO %s - %s" % (name, SCENARIOS[name][3]))
    directory = tempfile.mkdtemp(prefix="perabots_test_")
    try:
        world_path = build_world(name, directory)
        started    = time.time()
        lines, finished = run_webots(world_path, wall_clock_limit_seconds)
        dumps      = parse_knowledge(lines)
        print("wall-clock %.0f s | finished: %s" % (time.time() - started, finished))
        for line in lines:
            if re.search(r"Traceback|Error|start mode|Section 1 complete|stalled|stuck", line): print("  log:", line.strip())
        if "done" not in dumps:
            print("FAIL: the robot did not finish. Last log lines:"); [print("   ", l) for l in lines[-8:]]; return False
        dump, complete = dumps["done"], dumps.get("exploration_complete")
        findings = compare(dump, truth)
        order, predicted_run = offline_prediction()
        robot_order = [tuple(cell) for _, cell in dump["visit_order"]]
        first_difference = next((i for i, (a, b) in enumerate(zip(robot_order, order)) if a != b), None)
        visited = set(tuple(c) for c in dump["visited"])

        print("\nRobot's own picture of the maze (?? = never visited):")
        print(ascii_maze(truth, visited=visited))
        print("\nFirst-visit order in the simulator (time in seconds the cell was first entered):")
        print(ascii_maze(truth, visit_labels={tuple(cell): "%d" % t for t, cell in dump["visit_order"]}, width=4))

        ok = True
        def check(label, good, detail=""):
            nonlocal ok
            ok = ok and good
            print("  [%s] %s %s" % ("PASS" if good else "FAIL", label, detail))
        print("\nRESULTS")
        check("coverage", len(visited) == COLUMNS * ROWS, "(%d of 36 cells visited)" % len(visited))
        check("walls", not findings["wall_mismatches"], "(%d edges checked, %d wrong)" % (findings["walls_checked"], len(findings["wall_mismatches"])))
        for cell, message in findings["wall_mismatches"][:12]: print("        wall at %s: %s" % (cell, message))
        check("tiles", not findings["tile_problems"], "(found %s)" % dict(sorted(dump["tiles"].items())))
        for message in findings["tile_problems"]: print("        " + message)
        check("end cell", findings["end_ok"], "(robot %s, truth %s)" % (dump["end"], truth["end"]))
        check("no contradicting wall readings ('matrix cleared' test)", not findings["conflicts"], "(%d conflicts)" % len(findings["conflicts"]))
        for conflict in findings["conflicts"][:8]: print("        conflict:", conflict)
        check("visit order equals the offline planner's order", first_difference is None and len(robot_order) == len(order),
              "(robot %d cells, offline %d cells%s)" % (len(robot_order), len(order), "" if first_difference is None else ", first difference at visit %d: robot %s vs offline %s" % (first_difference, robot_order[first_difference], order[first_difference])))
        check("cell moves equal the offline plan", dump["moves"] == predicted_run.moves, "(robot %d, offline %d)" % (dump["moves"], predicted_run.moves))
        print("  timing: exploration finished at %.1f s, LED lit at %.1f s" % (complete["time"] if complete else -1, dump["time"]))
        print("  => %s" % ("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED"))
        return ok
    finally:
        shutil.rmtree(directory, ignore_errors=True)


if __name__ == "__main__":
    wanted = sys.argv[1:] or ["red_b_north"]
    truth  = load_ground_truth()
    print("TRUE maze from protos/Arena.proto:\n" + ascii_maze(truth) + "\n")
    results = {name: run_scenario(name, truth) for name in wanted}
    print("\nSUMMARY:", {name: ("pass" if ok else "FAIL") for name, ok in results.items()})
    sys.exit(0 if all(results.values()) else 1)
