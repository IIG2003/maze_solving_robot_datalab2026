# Run the advanced test worlds headless in Webots (TEST ONLY).
#
# Each world runs with the test_judge supervisor, which writes
#   adv_results/<world>.json   verdict + true trajectory
# and the robot writes adv_results/<world>.log (PERABOT_LOG).
#
# usage: python run_advanced.py [--mode realtime|fast] [--tmax S] [--project DIR] [--out DIR]
#                               [--headless [-j JOBS]] [world ...]
#   default     the Webots window is shown with 3D rendering, one run at a time,
#               in real time (--mode fast: rendered but as fast as possible)
#   --headless  no window, no rendering, fast mode, JOBS runs in parallel
#        (default: every test_adv*.wbt in ../test_run/worlds -> adv_results/)
import argparse
import glob
import os
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'adv_results')
WEBOTS = os.environ.get('WEBOTS_EXE', r'C:\Campus\Webots\msys64\mingw64\bin\webots.exe')


def run(world, port, tmax, out, show=False, mode='fast', movie=None):
    name = os.path.splitext(os.path.basename(world))[0]
    log, res = os.path.join(out, name + '.log'), os.path.join(out, name + '.json')
    for f in (log, res):
        if os.path.exists(f):
            os.remove(f)
    env = dict(os.environ, PERABOT_LOG=log, PERABOT_RESULT=res, PERABOT_TMAX=str(tmax), PERABOT_MODE=mode)
    if movie:
        os.makedirs(movie, exist_ok=True)
        env['PERABOT_MOVIE'] = os.path.join(os.path.abspath(movie), name + '.mp4')
    t0 = time.time()
    try:
        view = [] if show else ['--no-rendering', '--minimize']
        subprocess.run([WEBOTS, '--batch', '--mode=%s' % mode] + view +
                       ['--port=%d' % port, os.path.abspath(world)],
                       env=env, timeout=tmax * 2 + 300,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        status = 'ok' if os.path.exists(res) else 'no result'
    except subprocess.TimeoutExpired:
        status = 'wall-clock timeout'
    print('%-16s %-10s %4.0fs' % (name, status, time.time() - t0), flush=True)
    return name, status


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-j', '--jobs', type=int, default=4, help='parallel runs (--headless only)')
    ap.add_argument('--tmax', type=float, default=600)
    ap.add_argument('--project', default=os.path.join(HERE, '..', 'test_run'))
    ap.add_argument('--out', default=OUT)
    ap.add_argument('--headless', action='store_true',
                    help='no window / rendering, fast mode, parallel runs')
    ap.add_argument('--show', action='store_true', help=argparse.SUPPRESS)   # old flag: now default
    ap.add_argument('--mode', choices=('fast', 'realtime'),
                    help='simulation speed (default: realtime when shown, fast when headless)')
    ap.add_argument('--movie', help='record an .mp4 of every run into this directory (shown runs only)')
    ap.add_argument('worlds', nargs='*')
    a = ap.parse_args()
    worlds = a.worlds or sorted(glob.glob(os.path.join(a.project, 'worlds', 'test_adv*.wbt')))
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    show = not a.headless
    mode = a.mode or ('realtime' if show else 'fast')
    jobs = 1 if show else a.jobs            # one visible window at a time
    with ThreadPoolExecutor(jobs) as ex:
        list(ex.map(lambda iw: run(iw[1], 1240 + iw[0], a.tmax, out, show, mode,
                                     a.movie if show else None),
                    enumerate(worlds)))


if __name__ == '__main__':
    main()
