# TEST ONLY - not part of the submission.
# Supervisor "judge" for automated test runs. Records the true robot pose and
# contacts, watches the robot's log (PERABOT_LOG) for the FINISH line, keeps
# running 3 s after it, then writes a JSON verdict (PERABOT_RESULT) and quits.
#
# env: PERABOT_LOG     robot log file (written by hardware.log)
#      PERABOT_RESULT  JSON result file
#      PERABOT_TMAX    simulated-time limit in s (default 600)
#      PERABOT_MODE    'realtime' / 'fast': force the simulation speed (the
#                      --mode command-line option is not always honoured)
#      PERABOT_MOVIE   optional .mp4 path: record the 3D view (needs rendering)
#      PERABOT_MOVIE_X playback speed of the movie (default 8 = 8x real time)
import json
import math
import os
import time

from controller import Supervisor

LOG = os.environ.get('PERABOT_LOG', 'bot.log')
OUT = os.environ.get('PERABOT_RESULT', 'result.json')
TMAX = float(os.environ.get('PERABOT_TMAX', '600'))
HOLD = 3.0                       # s to keep watching after FINISH (LED rule)
IDLE_ABORT = 90.0                # s without moving 2 cm and no FINISH -> give up early

ARENA = 2.44
S1_TOP = 1.22                    # y of the Section 1 / maze boundary
MAZE_Y = 1.33                    # y where maze row 1 starts
END_C, END_H = (2.22, 2.205), 0.125           # end cell centre, half size
FOOT = [(x, y) for x in (-0.042, 0.042) for y in (-0.038, 0.038)]   # body footprint
CONTACT_Z = 0.004                # contacts above the floor = hits

sup = Supervisor()
WALL0 = time.time()
dt_ms = int(sup.getBasicTimeStep())
dt = dt_ms / 1000.0
bot = sup.getFromDef('BOT')
bot.enableContactPointsTracking(dt_ms, True)
MOVIE = os.environ.get('PERABOT_MOVIE')
MOVIE_X = int(os.environ.get('PERABOT_MOVIE_X', '8'))
CAPTION = os.environ.get('PERABOT_CAPTION', '')
WANT_MODE = {'realtime': Supervisor.SIMULATION_MODE_REAL_TIME,
             'fast': Supervisor.SIMULATION_MODE_FAST}.get(os.environ.get('PERABOT_MODE'))

track = []                       # (t, x, y, th) every 0.1 s
hits = []                        # (t, x, y, z) of every contact step above the floor
max_tilt = 0.0
outside_since, outside_max = None, 0.0
t_gap = t_maze = t_finish = None
finish_line = None
log_pos = 0
k = 0
result = None


def pose():
    x, y, _ = bot.getPosition()
    R = bot.getOrientation()
    th = math.atan2(R[3], R[0])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[8]))))
    return x, y, th, tilt


def corners(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in FOOT]


def read_log():
    global log_pos
    try:
        with open(LOG, encoding='utf8', errors='replace') as f:
            f.seek(log_pos)
            text = f.read()
            log_pos = f.tell()
        return text
    except OSError:
        return ''


def events(samples, gap=0.25):
    """Group contact samples into collision events (gap s apart)."""
    ev = []
    for t, x, y, z in samples:
        if ev and t - ev[-1]['t1'] <= gap:
            ev[-1]['t1'] = t
        else:
            ev.append({'t0': t, 't1': t, 'x': x, 'y': y, 'z': z,
                       'where': 'S1' if y < S1_TOP else 'maze'})
    return ev


def longest_no_progress(tr, t_end, radius=0.05):
    """Longest span (s) in which the robot stayed within `radius` of one point."""
    best, best_at = 0.0, None
    pts = [p for p in tr if p[0] <= t_end]
    for i in range(len(pts)):
        lo = i                      # walk back while still within radius of pts[i]
        while lo > 0 and math.hypot(pts[lo - 1][1] - pts[i][1], pts[lo - 1][2] - pts[i][2]) <= radius:
            lo -= 1
        span = pts[i][0] - pts[lo][0]
        if span > best:
            best, best_at = span, pts[lo][0]
    return best, best_at


def verdict(reason):
    t = sup.getTime()
    x, y, th, _ = pose()
    cs = corners(x, y, th)
    margin = min(min(END_H - abs(px - END_C[0]), END_H - abs(py - END_C[1])) for px, py in cs)
    t_end = t_finish if t_finish is not None else t
    ev = events(hits)
    nop, nop_at = longest_no_progress(track, t_end)
    dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(track, track[1:])
               if b[0] <= t_end)
    colour = None
    if finish_line:
        colour = finish_line.split('majority=')[1].split()[0]
    # did the robot move after FINISH?
    after = [p for p in track if t_finish is not None and p[0] >= t_finish]
    drift = max((math.hypot(p[1] - after[0][1], p[2] - after[0][2]) for p in after), default=0.0)
    return {
        'reason': reason, 'sim_time': round(t, 2), 'wall_s': round(time.time() - WALL0, 1),
        'mode': sup.simulationGetMode(),
        'finished': t_finish is not None, 't_finish': t_finish,
        't_gap': t_gap, 't_maze': t_maze,
        'led': colour,
        'final_pose': [round(x, 4), round(y, 4), round(math.degrees(th), 1)],
        'in_end_cell': margin >= 0, 'end_margin_cm': round(100 * margin, 1),
        'end_centre_err_cm': round(100 * math.hypot(x - END_C[0], y - END_C[1]), 1),
        'moved_after_finish_cm': round(100 * drift, 1),
        'distance_m': round(dist, 2),
        'collisions_s1': sum(e['where'] == 'S1' for e in ev),
        'collisions_maze': sum(e['where'] == 'maze' for e in ev),
        'collision_events': [{k_: (round(v, 3) if isinstance(v, float) else v) for k_, v in e.items()}
                             for e in ev],
        'longest_no_progress_s': round(nop, 1), 'no_progress_at': nop_at,
        'max_tilt_deg': round(max_tilt, 1), 'max_outside_s': round(outside_max, 2),
        'track': [[round(v, 3) for v in p] for p in track[::3]],
    }


while sup.step(dt_ms) != -1:
    k += 1
    t = sup.getTime()
    if WANT_MODE is not None and k % 30 == 1 and sup.simulationGetMode() != WANT_MODE:
        sup.simulationSetMode(WANT_MODE)          # Webots may apply its startup mode late
    x, y, th, tilt = pose()
    max_tilt = max(max_tilt, tilt)
    if k % 6 == 0:
        track.append((t, x, y, th))
    for cp in bot.getContactPoints(True):
        px, py, pz = cp.getPoint()
        if pz > CONTACT_Z:
            hits.append((t, px, py, pz))
            break
    if t_gap is None and y > S1_TOP:
        t_gap = round(t, 2)
    if t_maze is None and y > MAZE_Y:
        t_maze = round(t, 2)
    out = not any(0 <= px <= ARENA and 0 <= py <= ARENA for px, py in corners(x, y, th))
    if out:
        outside_since = t if outside_since is None else outside_since
        outside_max = max(outside_max, t - outside_since)
    else:
        outside_since = None

    if k % 30 == 0 and t_finish is None:
        text = read_log()
        for line in text.splitlines():
            if 'FINISH' in line:
                t_finish, finish_line = round(t, 2), line.strip()
        recent = [p for p in track if p[0] >= t - IDLE_ABORT]
        if t > IDLE_ABORT + 5 and recent and \
                max(math.hypot(p[1] - x, p[2] - y) for p in recent) < 0.02:
            result = verdict('idle %.0fs without FINISH' % IDLE_ABORT)
    if result is None and t_finish is not None and t >= t_finish + HOLD:
        result = verdict('finished')
    if result is None and t >= TMAX:
        result = verdict('time limit %.0fs' % TMAX)
    if MOVIE and k == 1:
        sup.movieStartRecording(MOVIE, 800, 800, 0, 85, MOVIE_X, False)
    if result is not None:
        with open(OUT, 'w') as f:
            json.dump(result, f)
        if MOVIE:
            sup.movieStopRecording()
            for _ in range(20000):                # let Webots finish encoding
                if sup.movieIsReady() or sup.movieFailed() or sup.step(dt_ms) == -1:
                    break
            result['movie'] = 'failed' if sup.movieFailed() else MOVIE
            with open(OUT, 'w') as f:
                json.dump(result, f)
        sup.simulationQuit(0)
        break
