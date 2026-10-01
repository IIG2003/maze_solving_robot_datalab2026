"""
run_compare_projects.py - runs two Webots projects (this one and another controller for the same competition) in the SAME harness:
same Webots, same 16 ms step, same start poses, same supervisor that counts real contacts, real tilt and real path length.
Both projects are copied to temporary folders and are never modified.

  python3 tests/run_compare_projects.py /path/to/other/project
"""

import concurrent.futures
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run_ablation as ra

END_CENTRE = (2.21, 2.205)
POSES = [("west",) + (2.315, 0.125, math.pi), ("north",) + (2.315, 0.125, math.pi / 2), ("south",) + (2.315, 0.125, -math.pi / 2), ("north-west",) + (2.30, 0.14, 2.4),
         ("your-world-pose",) + (2.269, 0.16, 2.32), ("south-west",) + (2.33, 0.11, -2.0),
         ("Red B facing maze",) + (0.22, 1.275, math.pi / 2), ("Red B facing away",) + (0.22, 1.275, -math.pi / 2)]


def run_job(job):
    label, project, patch, finished_pattern, pose = job
    name, x, y, yaw = pose
    directory = tempfile.mkdtemp(prefix="perabots_compare_")
    result = {"project": label, "pose": name, "ok": False, "reason": "", "total": None, "section1": None, "collisions": None, "pitch": None, "distance": None, "end_error": None, "blue": None, "green": None, "led": None, "stalls": 0}
    try:
        world_path = ra.prepare_copy(directory, patch, x, y, yaw, project)
        with ra.START_LOCK:
            process = subprocess.Popen([ra.WEBOTS, "--batch", "--mode=fast", "--no-rendering", "--stdout", "--stderr", world_path], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            time.sleep(4)
        deadline, last_monitor, done_line, lines = time.time() + 330, None, None, []
        try:
            for line in process.stdout:
                if line.startswith("MON "): last_monitor = line
                else:
                    lines.append(line)
                    if re.search(finished_pattern, line): done_line = line; break
                if time.time() > deadline: break
        finally:
            process.terminate()
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired: process.kill()
        text = "".join(lines)
        if last_monitor:
            m = re.search(r"collisions=(\d+) pitch=([\d.]+) x=([\-\d.]+) y=([\-\d.]+) dist=([\d.]+)", last_monitor)
            result["collisions"], result["pitch"], result["distance"] = int(m.group(1)), float(m.group(2)), float(m.group(5))
            result["end_error"] = math.hypot(float(m.group(3)) - END_CENTRE[0], float(m.group(4)) - END_CENTRE[1])
        result["stalls"] = len(re.findall(r"stalled|stuck", text))
        if done_line is None:
            result["reason"] = "did not finish"
            return result
        if "KNOWLEDGE" in done_line:                                   # this project
            import json
            dump = json.loads(done_line[len("KNOWLEDGE "):])
            result["total"], result["blue"], result["green"] = dump["time"], sum(1 for c in dump["tiles"].values() if c == "BLUE"), sum(1 for c in dump["tiles"].values() if c == "GREEN")
            result["led"] = "GREEN" if result["green"] > result["blue"] else "BLUE" if result["blue"] > result["green"] else "TIE"
            s1 = re.search(r"\[\s*([\d.]+) s\] Section 1 complete", text)
            if s1: result["section1"] = float(s1.group(1))
        else:                                                           # the other project
            m = re.search(r"majority=(\w+)\s+t=([\d.]+)s", done_line)
            result["led"], result["total"] = m.group(1).upper(), float(m.group(2))
            m2 = re.search(r"exploration done in [\d.]+s: blue=(\d+) green=(\d+)", text)
            if m2: result["blue"], result["green"] = int(m2.group(1)), int(m2.group(2))
            s1 = re.search(r"Section 1 done in ([\d.]+)s", text)
            if s1:
                start = re.search(r"\[bot\] start zone", text)
                result["section1"] = float(s1.group(1))
        result["ok"] = (result["blue"], result["green"], result["led"]) == (2, 3, "GREEN")
        if not result["ok"]: result["reason"] = "blue %s green %s led %s" % (result["blue"], result["green"], result["led"])
        return result
    except Exception as error:
        result["reason"] = "error: %r" % error
        return result
    finally:
        shutil.rmtree(directory, ignore_errors=True)


if __name__ == "__main__":
    other = os.path.abspath(sys.argv[1])
    ours  = os.path.abspath(os.path.join(HERE, ".."))
    identity = lambda source: source
    jobs  = []
    for pose in POSES:
        jobs.append(("this project", ours, ra.set_constants(), r"KNOWLEDGE .*\"label\": \"done\"", pose))
        jobs.append(("other project", other, identity, r"\[bot\] FINISH", pose))
    started = time.time()
    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=ra.WORKERS) as pool:
        for index, result in enumerate(pool.map(run_job, jobs)):
            results.append(result)
            print("  finished %d/%d (%.0f s): %s / %s" % (index + 1, len(jobs), time.time() - started, result["project"], result["pose"]), flush=True)
    print("\n%-18s %-14s %-3s %8s %9s %10s %9s %7s %6s %9s %s" % ("pose", "project", "ok", "total s", "sect.1 s", "collisions", "distance", "pitch", "stalls", "end err", "tiles/LED"))
    for pose in POSES:
        for r in [r for r in results if r["pose"] == pose[0]]:
            f = lambda v, fmt: (fmt % v) if v is not None else "-"
            print("%-18s %-14s %-3s %8s %9s %10s %9s %7s %6d %9s B%s G%s %s %s" % (r["pose"], r["project"], "yes" if r["ok"] else "NO", f(r["total"], "%.1f"), f(r["section1"], "%.1f"), f(r["collisions"], "%d"), f(r["distance"], "%.2f m"), f(r["pitch"], "%.1f"),
                  r["stalls"], f(r["end_error"], "%.3f m"), r["blue"], r["green"], r["led"], r["reason"]))
    print("\nMEANS over the runs where the project finished correctly:")
    for label in ("this project", "other project"):
        good = [r for r in results if r["project"] == label and r["ok"]]
        allr = [r for r in results if r["project"] == label]
        if good:
            mean = lambda key: sum(r[key] for r in good if r[key] is not None) / max(1, len([r for r in good if r[key] is not None]))
            print("  %-14s %d/%d ok | total %.1f s | distance %.2f m | collisions %d total | max pitch %.1f | end-cell error %.3f m" % (label, len(good), len(allr), mean("total"), mean("distance"), sum(r["collisions"] or 0 for r in allr), max((r["pitch"] or 0) for r in allr), mean("end_error")))
        else: print("  %-14s 0/%d ok" % (label, len(allr)))
