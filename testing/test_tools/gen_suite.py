# Build the full Webots test suite with the test_judge supervisor (TEST ONLY).
#
# For every test arena (official Arena, designs D1-D5, random R1-R3, advanced
# A1-A6) writes two worlds into the test project (default ../test_run):
#   worlds/test_j_<key>.wbt      full run from the Section 1 start zone
#   worlds/test_j_<key>_s2.wbt   dedicated Section 2 attempt (transition tile)
# and suite.json: maze walls and tiles read back from the proto geometry (so
# the report checks against exactly what Webots simulates).
#
# usage: python gen_suite.py [project_dir]
import json
import os
import re
import shutil
import sys

import gen_design_worlds as gd
from gen_design_worlds import R, C, X0, Y0, CELL, START, END, key, all_edges, cells, deg, \
    ascii_map

HERE = os.path.dirname(os.path.abspath(__file__))
OFFICIAL = os.path.join(HERE, '..', 'PeraBots_UC', 'protos')


def solids(path):
    text = open(path, encoding='utf8').read()
    out = []
    for blk in text.split('Solid {')[1:]:
        t = re.search(r'translation ([-\d.e]+) ([-\d.e]+) ([-\d.e]+)', blk)
        s = re.search(r'size ([-\d.e]+) ([-\d.e]+) ([-\d.e]+)', blk)
        n = re.search(r'name "([^"]+)"', blk)
        if t and s and n:
            out.append((n.group(1), [float(v) for v in t.groups()], [float(v) for v in s.groups()]))
    return out


def maze_of(proto_path):
    """Open edges + tiles of the maze, from wall boxes and tile solids."""
    items = solids(proto_path)
    walls = [(p, s) for n, p, s in items if s[2] >= 0.05]

    def blocked(x, y):
        return any(abs(x - p[0]) <= s[0] / 2 + 0.003 and abs(y - p[1]) <= s[1] / 2 + 0.003
                   for p, s in walls)
    open_e = set()
    for a, b in all_edges():
        xa, ya = X0 + CELL * (a[1] - 1), Y0 + CELL * (a[0] - 1)
        xb, yb = X0 + CELL * (b[1] - 1), Y0 + CELL * (b[0] - 1)
        if not blocked((xa + xb) / 2, (ya + yb) / 2):
            open_e.add((a, b))
    tiles = {}
    for n, p, s in items:
        if 'tile' in n and ('blue' in n or 'green' in n) and p[1] > 1.33:
            cell = (int(round((p[1] - Y0) / CELL)) + 1, int(round((p[0] - X0) / CELL)) + 1)
            tiles[cell] = 'blue' if 'blue' in n else 'green'
    return open_e, tiles


def world_protos(path):
    w = open(path, encoding='utf8').read()
    arena = re.search(r'EXTERNPROTO "\.\./protos/((?:Test)?Arena\w*)\.proto"', w).group(1)
    obst = re.search(r'EXTERNPROTO "\.\./protos/(\w*Obstacles\w*)\.proto"', w).group(1)
    return arena, obst


def main():
    proj = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', 'test_run')
    wdir, pdir = os.path.join(proj, 'worlds'), os.path.join(proj, 'protos')
    for p in ('Arena.proto', 'Obstacles.proto'):          # test copies of the official ones
        shutil.copy(os.path.join(OFFICIAL, p), os.path.join(pdir, p))
    template = open(os.path.join(wdir, 'test_rand1.wbt'), encoding='utf8').read()
    d_about = {'D%d' % i: f()[1] for i, f in enumerate(gd.DESIGNS, 1)}
    d_name = {'D%d' % i: f()[0] for i, f in enumerate(gd.DESIGNS, 1)}
    adv = json.load(open(os.path.join(HERE, 'advanced_designs.json')))

    entries = [('OFF', 'Official arena', 'the provided Arena.wbt layout (Arena.proto + '
                'Obstacles.proto)', 'Arena', 'Obstacles')]
    for i in range(1, 6):
        a, o = world_protos(os.path.join(wdir, 'test_design%d.wbt' % i))
        entries.append(('D%d' % i, d_name['D%d' % i], d_about['D%d' % i], a, o))
    for i in range(1, 4):
        a, o = world_protos(os.path.join(wdir, 'test_rand%d.wbt' % i))
        entries.append(('R%d' % i, 'Random %d' % i, 'random DFS maze + 3 loops, random tiles '
                        'and obstacle clusters (gen_random_world.py seed %d)' % i, a, o))
    for i in range(1, len(adv) + 1):
        d = adv['A%d' % i]
        entries.append(('A%d' % i, d['name'], d['about'], 'TestArenaA%d' % i, d['obstacles']))

    suite = {}
    for k, name, about, arena, obst in entries:
        open_e, tiles = maze_of(os.path.join(pdir, arena + '.proto'))
        b = sum(v == 'blue' for v in tiles.values())
        g = len(tiles) - b
        worlds = {}
        for mode, suffix in (('full', ''), ('s2', '_s2')):
            w = template.replace('TestArena1', arena).replace('TestObstacles1', obst)
            w = w.replace('controller "test_truth"', 'controller "test_judge"')
            if mode == 's2':
                w = w.replace('translation 2.315 0.125 0\n  rotation 0 0 1 3.14159',
                              'translation 0.22 1.275 0\n  rotation 0 0 1 1.5708')
                assert '0.22 1.275' in w
            worlds[mode] = 'test_j_%s%s' % (k, suffix)
            open(os.path.join(wdir, worlds[mode] + '.wbt'), 'w', encoding='utf8',
                 newline='\n').write(w)
        dead = [x for x in cells() if x not in (START, END) and deg(open_e, x) == 1]
        suite[k] = {'name': name, 'about': about, 'arena': arena, 'obstacles': obst,
                    'worlds': worlds, 'open': sorted(open_e), 'walls': len(all_edges()) - len(open_e),
                    'dead_ends': len(dead), 'tiles': [[list(c), v] for c, v in sorted(tiles.items())],
                    'blue': b, 'green': g,
                    'expect': 'blue' if b > g else 'green' if g > b else 'tie',
                    'map': ascii_map(open_e, tiles)}
        print('%-4s %-16s %-14s %-15s tiles B%d G%d -> %-5s dead ends %2d walls %2d' % (
            k, name, arena, obst, b, g, suite[k]['expect'], len(dead), suite[k]['walls']))
    json.dump(suite, open(os.path.join(HERE, 'suite.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
