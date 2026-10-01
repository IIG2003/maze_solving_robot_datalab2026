"""
run_random_sim.py - Webots batch on random mazes (never seen by the controller): for each seed, runs the robot with look-ahead OFF and ON
(on temporary copies, your project is not modified) and checks what it learned against the true generated maze, plus the LED and the time.

  python3 tests/run_random_sim.py [seed ...]        default seeds 1..10
"""

import os
import re
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run_maze_test as runner
from gen_random_maze import random_maze, build_project

START_X, START_Y, START_YAW = 0.22, 1.275, 1.5708          # on Red B facing the maze (Section 2 only)


def prepare(directory, spec, look_ahead):
    world_path = build_project(spec, directory)
    world = open(world_path).read()
    world = re.sub(r'EXTERNPROTO "https[^\n]*\n', "", world)
    world = re.sub(r"TexturedBackground(Light)? \{[^}]*\}\n", "", world)
    world = re.sub(r"TwoWheelRobot \{.*\}\s*$", 'TwoWheelRobot {\n  translation %f %f 0.0\n  rotation 0 0 1 %f\n  name "team_robot_template"\n  controller "main_controller"\n  bodyColor 1 0.55 0\n}\n' % (START_X, START_Y, START_YAW), world, flags=re.S)
    open(world_path, "w").write(world)
    controller = os.path.join(directory, "controllers", "main_controller", "main_controller.py")
    source = open(controller).read().replace("PRINT_PROGRESS = False", "PRINT_PROGRESS = True")
    source = re.sub(r"(LOOK_AHEAD_ENABLED\s+= )\d", r"\g<1>%d" % (1 if look_ahead else 0), source)
    open(controller, "w").write(source)
    shutil.rmtree(os.path.join(directory, "controllers", "main_controller", "__pycache__"), ignore_errors=True)
    return world_path


def run_one(seed, look_ahead):
    spec = random_maze(seed)
    directory = tempfile.mkdtemp(prefix="perabots_rand_")
    try:
        lines, finished = runner.run_webots(prepare(directory, spec, look_ahead), 300)
    finally:
        shutil.rmtree(directory, ignore_errors=True)
    dumps = runner.parse_knowledge(lines)
    peeks = [line for line in lines if "look-ahead" in line]
    if "done" not in dumps: return {"ok": False, "why": "did not finish", "peeks": len(peeks)}
    findings = runner.compare(dumps["done"], spec)
    blue = sum(1 for colour in spec["tiles"].values() if colour == "BLUE")
    green = sum(1 for colour in spec["tiles"].values() if colour == "GREEN")
    true_majority = "BLUE" if blue > green else "GREEN" if green > blue else "TIE"
    seen_blue = sum(1 for colour in dumps["done"]["tiles"].values() if colour == "BLUE")
    seen_green = sum(1 for colour in dumps["done"]["tiles"].values() if colour == "GREEN")
    robot_majority = "BLUE" if seen_blue > seen_green else "GREEN" if seen_green > seen_blue else "TIE"
    problems = findings["wall_mismatches"][:3] + findings["tile_problems"] + ([] if findings["end_ok"] else ["wrong end cell"])
    return {"ok": not problems and robot_majority == true_majority.replace("TIE", "TIE"), "problems": problems, "time": dumps["done"]["time"], "moves": dumps["done"]["moves"],
            "visited": len(dumps["done"]["visited"]), "peeks": len(peeks), "majority": (robot_majority, true_majority), "blue_green": (blue, green)}


if __name__ == "__main__":
    seeds = [int(a) for a in sys.argv[1:]] or list(range(1, 11))
    rows = []
    for seed in seeds:
        off = run_one(seed, False)
        on = run_one(seed, True)
        rows.append((seed, off, on))
        print("seed %2d | OFF ok=%s %s | ON ok=%s %s" % (seed, off["ok"], ("%.1fs %dmoves" % (off["time"], off["moves"])) if "time" in off else off.get("why"),
                                                         on["ok"], ("%.1fs %dmoves %dpeeks visited %d" % (on["time"], on["moves"], on["peeks"], on["visited"])) if "time" in on else on.get("why")), flush=True)
        for label, result in (("OFF", off), ("ON", on)):
            for problem in result.get("problems", []): print("      %s problem: %s" % (label, problem), flush=True)
            if result.get("majority") and result["majority"][0] != result["majority"][1]: print("      %s wrong majority %s (blue/green %s)" % (label, result["majority"], result["blue_green"]), flush=True)
    both = [(s, a, b) for s, a, b in rows if "time" in a and "time" in b]
    if both:
        print("\nOFF all ok: %s | ON all ok: %s" % (all(a["ok"] for _, a, _ in rows), all(b["ok"] for _, _, b in rows)))
        print("mean LED time: OFF %.1f s, ON %.1f s (saving %.1f s); mean moves OFF %.1f, ON %.1f" % (sum(a["time"] for _, a, _ in both) / len(both), sum(b["time"] for _, _, b in both) / len(both),
              (sum(a["time"] for _, a, _ in both) - sum(b["time"] for _, _, b in both)) / len(both), sum(a["moves"] for _, a, _ in both) / len(both), sum(b["moves"] for _, _, b in both) / len(both)))
