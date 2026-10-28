# Offline replay of the advanced designs (--suite: every suite arena) (TEST ONLY): real section2 planner,
# perfect sensing / driving from maze_sim.py. usage: python offline_designs.py [--suite]
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
_argv, sys.argv = sys.argv, [sys.argv[0], os.path.join(HERE, '..', 'PeraBots_UC', 'controllers', 'main_controller'), '0']
import types                                                   # noqa: E402
ms = types.ModuleType('maze_sim')    # maze_sim.py minus its batch run at the bottom
src = open(os.path.join(HERE, 'maze_sim.py')).read()
exec(src[:src.index('res = [simulate')], ms.__dict__)

SUITE = '--suite' in _argv[1:]
designs = json.load(open(os.path.join(HERE, 'suite.json' if SUITE else 'advanced_designs.json')))
out = {}
for name, d in designs.items():
    open_e = {tuple(tuple(c) for c in e) for e in d['open']}
    tiles = {tuple(c): v for c, v in d['tiles']}
    ms.gen_maze = lambda seed, o=open_e, t=tiles: (o, t)
    r = ms.simulate(0)
    out[name] = r
    print('%s %-15s cells %3d turns %3d stops %3d est %5.1fs  LED ok %s  at end %s  end early %s'
          % (name, d['name'], r['cells'], r['turns'], r['stops'], r['time'], r['ok'],
             r['at_end'], r['end_early']))
res_dir = os.path.join(HERE, 'suite_results' if SUITE else 'adv_results')
os.makedirs(res_dir, exist_ok=True)
json.dump(out, open(os.path.join(res_dir, 'offline.json'), 'w'), indent=1)
