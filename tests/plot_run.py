"""
plot_run.py - turns the robot's run_log.csv into one self-contained HTML report (no libraries, works offline):
speed, acceleration, turn rate, wheel speeds, clearance and tilt over time, the path coloured by speed, summary tiles, a phase filter, and a table view.

How to get a log (nothing here starts Webots):
  1. in controllers/main_controller/main_controller.py set  ENABLE_RUN_MONITOR = True
  2. run the course once in Webots; run_log.csv appears in controllers/main_controller/
  3. python3 tests/plot_run.py controllers/main_controller/run_log.csv          (writes run_report.html next to the CSV; open it in a browser)
  4. set ENABLE_RUN_MONITOR back to False before submitting
"""

import csv
import json
import math
import os
import sys

MAX_POINTS = 3000
NUMERIC    = ["time_s", "x_m", "y_m", "yaw_rad", "distance_m", "commanded_speed_m_s", "measured_speed_m_s", "commanded_turn_rate_rad_s", "measured_turn_rate_rad_s", "left_command_rad_s",
              "right_command_rad_s", "front_clearance_m", "closest_beam_m", "pitch_deg", "roll_deg", "stalled", "stall_recoveries"]


def moving_average(values, window):
    if window <= 1: return list(values)
    half, out, running, count = window // 2, [], 0.0, 0
    prefix = [0.0]
    for value in values: prefix.append(prefix[-1] + value)
    for index in range(len(values)):
        low, high = max(0, index - half), min(len(values), index + half + 1)
        out.append((prefix[high] - prefix[low]) / (high - low))
    return out


def derivative(values, times, span=2):
    out = []
    for index in range(len(values)):
        low, high = max(0, index - span), min(len(values) - 1, index + span)
        out.append((values[high] - values[low]) / (times[high] - times[low]) if times[high] > times[low] else 0.0)
    return out


def load(csv_path):
    if not os.path.exists(csv_path):
        raise SystemExit("%s does not exist yet. It is written by the robot during a Webots run:\n  1. set ENABLE_RUN_MONITOR = True in controllers/main_controller/main_controller.py\n  2. Webots: File > Reload World, press play, let the run finish\n  3. then run this command again" % csv_path)
    rows = []
    with open(csv_path, newline="") as handle:
        reader  = csv.DictReader(handle)
        missing = [name for name in NUMERIC if name not in (reader.fieldnames or [])]
        if missing: raise SystemExit("%s was written by an older run monitor (missing columns: %s). Run the course once more with ENABLE_RUN_MONITOR = True." % (csv_path, ", ".join(missing[:4])))
        for record in reader:
            try:
                row = {name: float(record[name]) for name in NUMERIC}
            except (KeyError, ValueError):
                continue
            row["phase"] = record.get("phase", "RUN")
            rows.append(row)
    if len(rows) < 10: raise SystemExit("not enough rows in %s" % csv_path)
    return rows


def build_series(rows):
    times     = [row["time_s"] for row in rows]
    timestep  = (times[-1] - times[0]) / (len(times) - 1)
    window    = max(1, int(round(0.10 / timestep)))                      # 0.1 s smoothing for speed and turn rate
    measured  = moving_average([row["measured_speed_m_s"] for row in rows], window)
    commanded = [row["commanded_speed_m_s"] for row in rows]
    acceleration_measured  = moving_average(derivative(measured, times), window)
    acceleration_commanded = derivative(commanded, times)
    jerk                   = moving_average(derivative(acceleration_measured, times), window)
    stride = max(1, int(math.ceil(len(rows) / MAX_POINTS)))
    pick   = lambda values: [round(value, 4) for value in values[::stride]]
    data = {"t": pick(times), "phase": [row["phase"] for row in rows][::stride], "x": pick([row["x_m"] for row in rows]), "y": pick([row["y_m"] for row in rows]),
            "distance": pick([row["distance_m"] for row in rows]), "speed_measured": pick(measured), "speed_commanded": pick(commanded),
            "accel_measured": pick(acceleration_measured), "accel_commanded": pick(acceleration_commanded), "jerk": pick(jerk),
            "turn_measured": pick(moving_average([row["measured_turn_rate_rad_s"] for row in rows], window)), "turn_commanded": pick([row["commanded_turn_rate_rad_s"] for row in rows]),
            "wheel_left": pick([row["left_command_rad_s"] for row in rows]), "wheel_right": pick([row["right_command_rad_s"] for row in rows]),
            "front": pick([row["front_clearance_m"] for row in rows]), "closest": pick([row["closest_beam_m"] for row in rows]),
            "pitch": pick([row["pitch_deg"] for row in rows]), "roll": pick([row["roll_deg"] for row in rows]),
            "stalled": [int(row["stalled"]) for row in rows][::stride], "recoveries": [int(row["stall_recoveries"]) for row in rows][::stride]}
    return data


def write_report(csv_path, html_path):
    rows = load(csv_path)
    data = build_series(rows)
    template_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plot_run_template.html")
    safe_data  = json.dumps(data, separators=(",", ":")).replace("<", "\\u003c")             # no "</script>" can end the data block early
    safe_title = json.dumps(os.path.basename(csv_path))[1:-1]                                 # escaped for use inside a JavaScript string
    html = open(template_path).read().replace("__DATA__", safe_data).replace("__TITLE__", safe_title)
    open(html_path, "w").write(html)
    return html_path, len(rows), data


if __name__ == "__main__":
    arguments = [a for a in sys.argv[1:] if not a.startswith("-")]
    csv_path  = arguments[0] if arguments else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers", "main_controller", "run_log.csv")
    html_path = os.path.join(os.path.dirname(os.path.abspath(csv_path)), "run_report.html")
    if "-o" in sys.argv: html_path = sys.argv[sys.argv.index("-o") + 1]
    path, count, data = write_report(csv_path, html_path)
    print("wrote %s from %d rows (%d points plotted). Open it in a browser." % (path, count, len(data["t"])))
