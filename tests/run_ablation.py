"""
run_ablation.py - full optimisation tests for the real controller in Webots (headless, on temporary copies; your project is never modified).

  python3 tests/run_ablation.py sensors      which laser sensors can be removed without hurting (each single sensor, groups, and candidate minimal sets)
  python3 tests/run_ablation.py speed        which speed settings are faster without collisions, tipping or failures
  python3 tests/run_ablation.py baseline     just the current settings on all start poses

How a sensor is "removed": inside the temporary copy its reading is forced to "nothing in range" (0.5 m), which is what an absent sensor gives.
How a run is judged: the robot must finish (LED lit), visit all 36 cells, find all 5 tiles and the end cell. Collisions are counted by a separate
supervisor that looks at real contact points; pitch is the robot's real tilt (a tip-over shows as 35+ degrees).
"""

import concurrent.futures
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time

HERE    = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(HERE, ".."))
WEBOTS  = os.environ.get("WEBOTS", "/Applications/Webots.app/Contents/MacOS/webots")
WORKERS = int(os.environ.get("WORKERS", "6"))
START_LOCK = threading.Lock()    # Webots instances started at the same moment race for the same network port; start them one at a time
sys.path.insert(0, HERE)
from maze_ground_truth import load_ground_truth

TRUTH = load_ground_truth()
POSES = [(2.315, 0.125, math.pi, "west"), (2.315, 0.125, math.pi / 2, "north"), (2.315, 0.125, -math.pi / 2, "south"), (2.30, 0.14, 2.4, "north-west"), (2.269, 0.16, 2.32, "your-world-pose"), (2.33, 0.11, -2.0, "south-west")]

ALL_SENSORS = ["ds_f0", "ds_fl15", "ds_fl30", "ds_fr15", "ds_fr30", "ds_cl", "ds_cr", "ds_l", "ds_r", "ds_rear", "ds_rear_left", "ds_rear_right",
               "ds_left_front", "ds_left_rear", "ds_right_front", "ds_right_rear"]

MONITOR_SOURCE = '''
import math
from controller import Supervisor
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
bot = robot.getFromDef("BOT")
episodes, last_contact, max_pitch, last_print = 0, -9.0, 0.0, -9.0
previous_position, travelled = None, 0.0
min_clear = 9.0
FOOTPRINT = [(-0.045 + 0.09 * i / 8.0, sy * 0.038) for i in range(9) for sy in (-1, 1)] + [(sx * 0.045, -0.038 + 0.076 * j / 4.0) for j in range(5) for sx in (-1, 1)]
def box_distance(px, py, box):
    return math.hypot(max(abs(px - box[0]) - box[2], 0.0), max(abs(py - box[1]) - box[3], 0.0))
while robot.step(timestep) != -1:
    now = robot.getTime()
    o = bot.getOrientation()
    max_pitch = max(max_pitch, abs(math.degrees(math.asin(max(-1.0, min(1.0, o[6]))))))
    position = bot.getPosition()
    if previous_position is not None: travelled += math.hypot(position[0] - previous_position[0], position[1] - previous_position[1])
    previous_position = position
    if position[1] < 1.3 and position[0] > 0.0:
        yaw = math.atan2(o[3], o[0]); c, s = math.cos(yaw), math.sin(yaw)
        near = [b for b in BOXES if abs(b[0] - position[0]) < 0.3 and abs(b[1] - position[1]) < 0.3]
        for b in near:
            for fx, fy in FOOTPRINT:
                d = box_distance(position[0] + c * fx - s * fy, position[1] + s * fx + c * fy, b)
                if d < min_clear: min_clear = d
    if any(cp.point[2] > 0.012 for cp in bot.getContactPoints(True)):
        if now - last_contact > 0.4: episodes += 1
        last_contact = now
    if now - last_print >= 1.0:
        print("MON t=%.1f collisions=%d pitch=%.1f x=%.3f y=%.3f dist=%.2f clear=%.4f" % (now, episodes, max_pitch, position[0], position[1], travelled, min_clear), flush=True)
        last_print = now
'''


# ---------------------------------------------------------------------------------------------------------------- variants
def disable_sensors(names):
    def patch(source):
        source = source.replace("PRINT_PROGRESS = False", "PRINT_PROGRESS = True")
        source = source.replace("import heapq\n", "import heapq\nDISABLED_SENSORS = %r\n" % (tuple(names),), 1)
        old    = "self.distance_readings_meters[sensor_name] = distance_sensor.getValue()"
        assert old in source
        return source.replace(old, "self.distance_readings_meters[sensor_name] = 0.5 if sensor_name in DISABLED_SENSORS else distance_sensor.getValue()")
    return patch


def set_constants(**constants):
    def patch(source):
        source = source.replace("PRINT_PROGRESS = False", "PRINT_PROGRESS = True")
        for name, value in constants.items():
            pattern = r"(\n\s+%s\s*=\s*)[\d.]+" % name
            assert re.search(pattern, source), name
            source = re.sub(pattern, lambda m: m.group(1) + repr(value), source, count=1)
        return source
    return patch


def sensor_variants():
    keep = lambda kept: [name for name in ALL_SENSORS if name not in kept]
    variants = {"baseline (all 16)": disable_sensors([])}
    for name in ALL_SENSORS: variants["remove " + name] = disable_sensors([name])
    groups = {"remove front +-30 pair": ["ds_fl30", "ds_fr30"], "remove front +-15 pair": ["ds_fl15", "ds_fr15"], "remove front corners +-45": ["ds_cl", "ds_cr"],
              "remove sides +-90": ["ds_l", "ds_r"], "remove side-front +-67": ["ds_left_front", "ds_right_front"], "remove side-rear +-113": ["ds_left_rear", "ds_right_rear"],
              "remove all 4 extra side beams": ["ds_left_front", "ds_left_rear", "ds_right_front", "ds_right_rear"], "remove rear corners +-135": ["ds_rear_left", "ds_rear_right"],
              "remove rear centre": ["ds_rear"], "remove all 3 rear beams": ["ds_rear", "ds_rear_left", "ds_rear_right"]}
    for label, names in groups.items(): variants[label] = disable_sensors(names)
    sets = {"KEEP 7: f0 fl15 fr15 cl cr l r": ["ds_f0", "ds_fl15", "ds_fr15", "ds_cl", "ds_cr", "ds_l", "ds_r"],
            "KEEP 11 (no rear / side-rear)": ["ds_f0", "ds_fl15", "ds_fr15", "ds_fl30", "ds_fr30", "ds_cl", "ds_cr", "ds_l", "ds_r", "ds_left_front", "ds_right_front"]}
    for label, kept in sets.items(): variants[label] = disable_sensors(keep(kept))
    return variants


def speed_variants():
    variants = {"baseline": set_constants()}
    for value in (0.6, 2.0, 3.0, 4.5): variants["acceleration %.1f m/s^2" % value] = set_constants(ACCELERATION_LIMIT_METERS_PER_SECOND_SQUARED=value)
    for value in (2.0, 3.0, 4.5, 6.0): variants["braking %.1f m/s^2 (limit and profile)" % value] = set_constants(DECELERATION_LIMIT_METERS_PER_SECOND_SQUARED=value, MAZE_BRAKING_METERS_PER_SECOND_SQUARED=value)
    for value in (3.0, 6.0, 8.0, 10.0): variants["pivot max turn rate %.0f rad/s" % value] = set_constants(PIVOT_MAXIMUM_TURN_RATE_RADIANS_PER_SECOND=value)
    variants["pivot gain 8"] = set_constants(PIVOT_PROPORTIONAL_GAIN=8.0)
    return variants


def margin_variants():
    """Safe margins: the wider the margins, the less often the robot has to slow down near walls."""
    variants = {"baseline": set_constants()}
    for value in (0.088, 0.098): variants["obstacle inflation %.3f m" % value] = set_constants(NORMAL_INFLATION_RADIUS_METERS=value)
    for value in (0.13, 0.15): variants["clearance-penalty radius %.2f m" % value] = set_constants(CLEARANCE_PENALTY_RADIUS_METERS=value)
    for value in (6.0, 8.0): variants["clearance-penalty weight %.0f" % value] = set_constants(CLEARANCE_PENALTY_WEIGHT=value)
    for value in (0.7, 1.0): variants["side-slowdown minimum factor %.1f" % value] = set_constants(SIDE_CLEARANCE_MINIMUM_SPEED_FACTOR=value)
    for value in (0.20, 0.15): variants["front slow-down starts at %.2f m" % value] = set_constants(SLOW_DOWN_START_DISTANCE_METERS=value)
    for value in (0.10, 0.25): variants["section-1 lookahead %.2f m" % value] = set_constants(SECTION_1_LOOKAHEAD_METERS=value)
    return variants


def combo_variants():
    """Second round: the best turning settings together, with the safe-margin setting that tested neutral."""
    combo = lambda gain, rate, minimum, tolerance: set_constants(PIVOT_PROPORTIONAL_GAIN=gain, PIVOT_MAXIMUM_TURN_RATE_RADIANS_PER_SECOND=rate, PIVOT_MINIMUM_TURN_RATE_RADIANS_PER_SECOND=minimum, PIVOT_TOLERANCE_RADIANS=tolerance)
    return {"baseline (gain 5)": set_constants(), "A: gain 12, rate 6, min 0.8, tol 0.02": combo(12.0, 6.0, 0.8, 0.02), "B: gain 16, rate 6, min 0.8, tol 0.02": combo(16.0, 6.0, 0.8, 0.02),
            "C: gain 24, rate 8, min 0.8, tol 0.02": combo(24.0, 8.0, 0.8, 0.02), "D: gain 24, rate 8, min 1.2, tol 0.03": combo(24.0, 8.0, 1.2, 0.03), "E: gain 32, rate 8, min 1.0, tol 0.02": combo(32.0, 8.0, 1.0, 0.02),
            "F: C + side-slowdown factor 0.7": set_constants(PIVOT_PROPORTIONAL_GAIN=24.0, PIVOT_MAXIMUM_TURN_RATE_RADIANS_PER_SECOND=8.0, PIVOT_MINIMUM_TURN_RATE_RADIANS_PER_SECOND=0.8, PIVOT_TOLERANCE_RADIANS=0.02, SIDE_CLEARANCE_MINIMUM_SPEED_FACTOR=0.7)}


def arc_variants():
    """Arc corners on (the default) against off, and different arc speeds. The outer wheel must stay under 0.2 m/s, so 0.159 m/s is the ceiling."""
    variants = {"arcs ON, 0.15 m/s (default)": set_constants(), "arcs OFF (stop and pivot)": set_constants(ARC_CORNERS_ENABLED=0)}
    for value in (0.10, 0.12, 0.158): variants["arcs ON, %.3f m/s" % value] = set_constants(ARC_SPEED_METERS_PER_SECOND=value)
    variants["arcs ON, 0.15 m/s, radial gain 25"] = set_constants(ARC_RADIAL_GAIN=25.0)
    variants["arcs ON, 0.15 m/s, heading gain 8"] = set_constants(ARC_HEADING_GAIN=8.0)
    return variants


def stress_variants():
    """Settings that used to trap the robot (turn-around loops, false 'Section 1 complete'): they must now all finish."""
    return {"baseline": set_constants(), "obstacle inflation 0.098 m": set_constants(NORMAL_INFLATION_RADIUS_METERS=0.098), "clearance-penalty radius 0.13 m": set_constants(CLEARANCE_PENALTY_RADIUS_METERS=0.13),
            "clearance-penalty radius 0.15 m": set_constants(CLEARANCE_PENALTY_RADIUS_METERS=0.15), "section-1 lookahead 0.25 m": set_constants(SECTION_1_LOOKAHEAD_METERS=0.25),
            "front slow-down starts at 0.20 m": set_constants(SLOW_DOWN_START_DISTANCE_METERS=0.20), "braking 6.0 m/s^2": set_constants(DECELERATION_LIMIT_METERS_PER_SECOND_SQUARED=6.0, MAZE_BRAKING_METERS_PER_SECOND_SQUARED=6.0),
            "side-slowdown factor 1.0": set_constants(SIDE_CLEARANCE_MINIMUM_SPEED_FACTOR=1.0)}


def turning_variants():
    """Turning is where time is lost: gain, minimum rate near the target, and tolerance."""
    variants = {"baseline (gain 5)": set_constants()}
    for value in (8.0, 10.0, 12.0, 16.0, 24.0): variants["pivot gain %.0f" % value] = set_constants(PIVOT_PROPORTIONAL_GAIN=value)
    for value in (0.8, 1.2): variants["gain 10, minimum rate %.1f rad/s" % value] = set_constants(PIVOT_PROPORTIONAL_GAIN=10.0, PIVOT_MINIMUM_TURN_RATE_RADIANS_PER_SECOND=value)
    for value in (0.02, 0.03): variants["gain 10, tolerance %.2f rad" % value] = set_constants(PIVOT_PROPORTIONAL_GAIN=10.0, PIVOT_TOLERANCE_RADIANS=value)
    variants["gain 12, rate 6, min 0.8, tol 0.02"] = set_constants(PIVOT_PROPORTIONAL_GAIN=12.0, PIVOT_MAXIMUM_TURN_RATE_RADIANS_PER_SECOND=6.0, PIVOT_MINIMUM_TURN_RATE_RADIANS_PER_SECOND=0.8, PIVOT_TOLERANCE_RADIANS=0.02)
    return variants


def section_1_boxes():
    """(centre_x, centre_y, half_x, half_y) in WORLD coordinates of the 43 cubes and the Section 1 walls, for the monitor's closest-approach measurement."""
    boxes = [(float(m.group(1)), float(m.group(2)), 0.05, 0.05) for m in re.finditer(r"translation ([-\d.]+) ([-\d.]+) [-\d.]+", open(os.path.join(PROJECT, "protos", "Obstacles.proto")).read())]
    arena = open(os.path.join(PROJECT, "protos", "Arena.proto")).read()
    for m in re.finditer(r'Solid \{\s*translation ([-\d.]+) ([-\d.]+) ([-\d.]+)(.*?)name "([^"]+)"', arena, re.S):
        if not (m.group(5).startswith("s1_wall") or m.group(5).startswith("chamber_wall")): continue
        size = [float(v) for v in re.search(r"size ([\d. ]+)", m.group(4)).group(1).split()]
        boxes.append((float(m.group(1)), float(m.group(2)), size[0] / 2.0, size[1] / 2.0))
    return boxes


# ---------------------------------------------------------------------------------------------------------------- one run
def prepare_copy(directory, patch, x, y, yaw, project=PROJECT):
    """Builds the temporary project copy (patched controller, monitor, world with the robot at the start pose); returns the world path."""
    for folder in ("protos", "worlds", "controllers"): shutil.copytree(os.path.join(project, folder), os.path.join(directory, folder))
    shutil.rmtree(os.path.join(directory, "controllers", "main_controller", "__pycache__"), ignore_errors=True)
    controller_path = os.path.join(directory, "controllers", "main_controller", "main_controller.py")
    source = open(controller_path).read()            # read BEFORE opening for writing (opening with "w" empties the file)
    open(controller_path, "w").write(patch(source))
    os.makedirs(os.path.join(directory, "controllers", "monitor"))
    open(os.path.join(directory, "controllers", "monitor", "monitor.py"), "w").write("BOXES = %r\n" % (section_1_boxes(),) + MONITOR_SOURCE)
    world_path = os.path.join(directory, "worlds", "Arena.wbt")
    world      = open(world_path).read()
    world      = re.sub(r'EXTERNPROTO "https[^\n]*\n', "", world)
    world      = re.sub(r"TexturedBackground(Light)? \{[^}]*\}\n", "", world)
    world      = re.sub(r"TwoWheelRobot \{.*\}\s*$", 'DEF BOT TwoWheelRobot {\n  translation %f %f 0.0\n  rotation 0 0 1 %f\n  name "team_robot_template"\n  controller "main_controller"\n  bodyColor 1 0.55 0\n}\nRobot {\n  name "monitor"\n  controller "monitor"\n  supervisor TRUE\n}\n' % (x, y, yaw), world, flags=re.S)
    world      = re.sub(r"basicTimeStep \d+", "basicTimeStep %s" % os.environ.get("TIMESTEP", "16"), world)
    open(world_path, "w").write(world)
    return world_path


def run_one(job):
    label, patch, pose, port = job
    x, y, yaw, pose_name = pose
    directory = tempfile.mkdtemp(prefix="perabots_ablation_")
    result    = {"variant": label, "pose": pose_name, "ok": False, "reason": "", "total": None, "section1": None, "collisions": None, "pitch": None, "stalls": 0, "clearance": None}
    try:
        world_path = prepare_copy(directory, patch, x, y, yaw)
        with START_LOCK:
            process = subprocess.Popen([WEBOTS, "--batch", "--mode=fast", "--no-rendering", "--stdout", "--stderr", world_path], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            time.sleep(4)
        deadline, dump, lines_seen, last_monitor = time.time() + float(os.environ.get("RUN_TIMEOUT", "330")), None, [], None
        try:
            for line in process.stdout:
                if line.startswith("MON "): last_monitor = line
                elif line.startswith("KNOWLEDGE ") and '"label": "done"' in line: dump = json.loads(line[len("KNOWLEDGE "):]); break
                else:
                    lines_seen.append(line)
                    if "Section 1 complete" in line: result["section1"] = float(re.search(r"\[\s*([\d.]+) s\]", line).group(1))
                    if "stalled" in line or "stuck" in line: result["stalls"] += 1
                if time.time() > deadline: break
        finally:
            process.terminate()
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired: process.kill()
        if last_monitor:
            match = re.search(r"collisions=(\d+) pitch=([\d.]+).*clear=([\d.]+)", last_monitor)
            result["collisions"], result["pitch"], result["clearance"] = int(match.group(1)), float(match.group(2)), float(match.group(3))
        if dump is None:
            result["reason"] = "did not finish; last output: " + " | ".join(l.strip() for l in lines_seen[-4:])
            return result
        tiles_ok = {tuple(int(v) for v in key.split(",")): colour for key, colour in dump["tiles"].items()} == TRUTH["tiles"]
        result["total"] = dump["time"]
        if len(dump["visited"]) != 36: result["reason"] = "visited %d/36 cells" % len(dump["visited"])
        elif not tiles_ok: result["reason"] = "wrong tiles"
        elif tuple(dump["end"] or ()) != TRUTH["end"]: result["reason"] = "wrong end cell"
        else: result["ok"] = True
        return result
    except Exception as error:
        result["reason"] = "error: %r" % error
        return result
    finally:
        shutil.rmtree(directory, ignore_errors=True)


# ---------------------------------------------------------------------------------------------------------------- reporting
def run_all(variants, poses=POSES):
    jobs, port = [], 4000
    for label, patch in variants.items():
        for pose in poses:
            jobs.append((label, patch, pose, port))
            port += 1
    results = {label: [] for label in variants}
    started = time.time()
    with concurrent.futures.ThreadPoolExecutor(max_workers=WORKERS) as pool:
        for index, result in enumerate(pool.map(run_one, jobs)):
            results[result["variant"]].append(result)
            if (index + 1) % len(poses) == 0: print("  finished %d/%d runs (%.0f s elapsed): %s" % (index + 1, len(jobs), time.time() - started, result["variant"]), flush=True)
    return results


def summarise(results):
    rows = []
    for label, runs in results.items():
        ok      = [r for r in runs if r["ok"]]
        rows.append({"variant": label, "ok": len(ok), "runs": len(runs),
                     "total": sum(r["total"] for r in ok) / len(ok) if ok else None, "section1": sum(r["section1"] for r in ok if r["section1"]) / max(1, len([r for r in ok if r["section1"]])) if ok else None,
                     "collisions": sum(r["collisions"] or 0 for r in runs), "stalls": sum(r["stalls"] for r in runs), "pitch": max((r["pitch"] or 0) for r in runs), "clearance": min((r["clearance"] for r in runs if r.get("clearance") is not None), default=None),
                     "failures": sorted({r["pose"] + ": " + r["reason"] for r in runs if not r["ok"]})})
    return rows


def print_table(rows, baseline_label):
    baseline = next(row for row in rows if row["variant"] == baseline_label)
    print("\n%-46s %5s %9s %9s %10s %6s %6s %9s   verdict" % ("variant", "ok", "total s", "sect.1 s", "collisions", "stalls", "pitch", "min gap m"))
    for row in rows:
        total    = "%.1f" % row["total"] if row["total"] else "-"
        section1 = "%.1f" % row["section1"] if row["section1"] else "-"
        verdict  = ""
        if row is not baseline:
            if row["ok"] < row["runs"]: verdict = "FAILS: " + "; ".join(row["failures"][:2])
            elif row["pitch"] > 15: verdict = "TIPS (pitch %.0f)" % row["pitch"]
            elif row["collisions"] > baseline["collisions"] or row["stalls"] > baseline["stalls"]: verdict = "WORSE (more collisions/stalls)"
            elif baseline["total"] and row["total"] and row["total"] < baseline["total"] - 1.0: verdict = "FASTER by %.1f s, same safety" % (baseline["total"] - row["total"])
            elif baseline["total"] and row["total"] and row["total"] > baseline["total"] + 1.0: verdict = "slower by %.1f s" % (row["total"] - baseline["total"])
            else: verdict = "NO DIFFERENCE (safe to remove / change)"
        gap = "%.3f" % row["clearance"] if row.get("clearance") is not None else "-"
        print("%-46s %2d/%d %9s %9s %10d %6d %6.1f %9s   %s" % (row["variant"], row["ok"], row["runs"], total, section1, row["collisions"], row["stalls"], row["pitch"], gap, verdict))


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "baseline"
    if mode == "sensors":   variants = sensor_variants()
    elif mode == "speed":   variants = speed_variants()
    elif mode == "margins": variants = margin_variants()
    elif mode == "stress":  variants = stress_variants()
    elif mode == "arcs":    variants = arc_variants()
    elif mode == "combo":   variants = combo_variants()
    elif mode == "turning": variants = turning_variants()
    else:                   variants = {"baseline": set_constants()}
    print("%d variants x %d start poses = %d runs, %d in parallel" % (len(variants), len(POSES), len(variants) * len(POSES), WORKERS), flush=True)
    results = run_all(variants)
    rows    = summarise(results)
    print_table(rows, next(iter(variants)))
    json.dump(results, open(os.path.join(tempfile.gettempdir(), "perabots_ablation_%s.json" % mode), "w"), indent=1)
