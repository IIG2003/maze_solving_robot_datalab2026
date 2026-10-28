# Hand-designed hard maze worlds (TEST ONLY, not part of the submission).
#
# Every world uses the same locked arena outline (2.44 x 2.44 m, Section 1
# floor, 4 x 9 maze of 25 cm cells, start / transition / end markers) as the
# official Arena.proto; only the internal maze walls, tiles and Section 1
# obstacles change. Each design is checked before it is written:
#   * every cell is reachable from the entrance cell (1, 1)
#   * tiles sit in dead ends that can be reached WITHOUT entering the end
#     cell (4, 9), which the robot may not drive through
#   * an odd number of tiles (no tie)
#
# Writes protos/TestArenaD<n>.proto and worlds/test_design<n>.wbt into every
# Webots test project given on the command line (default: ../test_run and
# ../test_run_old), plus designs.json for the offline simulator.
#
# usage: python gen_design_worlds.py [project_dir ...]
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ARENA = os.path.join(HERE, '..', 'PeraBots_UC', 'protos', 'Arena.proto')
R, C = 4, 9
X0, Y0, CELL = 0.22, 1.455, 0.25
START, END = (1, 1), (4, 9)


def key(a, b):
    return tuple(sorted((a, b)))


def adjacent(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


def all_edges():
    return [key((r, c), (r + dr, c + dc)) for r in range(1, R + 1) for c in range(1, C + 1)
            for dr, dc in ((1, 0), (0, 1)) if r + dr <= R and c + dc <= C]


def paths(*seqs):
    """Open edges along each sequence of adjacent cells."""
    out = set()
    for seq in seqs:
        for a, b in zip(seq, seq[1:]):
            assert adjacent(a, b), (a, b)
            out.add(key(a, b))
    return out


def run(r0, c0, r1, c1):
    """Straight run of cells from (r0, c0) to (r1, c1) inclusive."""
    if r0 == r1:
        step = 1 if c1 >= c0 else -1
        return [(r0, c) for c in range(c0, c1 + step, step)]
    step = 1 if r1 >= r0 else -1
    return [(r, c0) for r in range(r0, r1 + step, step)]


# --------------------------------------------------------------------- designs
def d1_snake():
    """Serpentine through every column: longest possible route, pockets near the end."""
    main = (run(1, 1, 4, 1) + run(4, 2, 2, 2) + run(2, 3, 4, 3) + run(4, 4, 1, 4)
            + run(1, 5, 4, 5) + run(4, 6, 1, 6) + run(1, 7, 3, 7) + run(3, 8, 1, 8)
            + run(1, 9, 4, 9))
    open_e = paths(main, [(2, 2), (1, 2)], [(2, 3), (1, 3)], [(3, 7), (4, 7)], [(3, 8), (4, 8)])
    tiles = {(1, 2): 'blue', (4, 7): 'green', (4, 8): 'green'}
    return 'Snake', 'one long serpentine path; pockets right beside the end cell', open_e, tiles


def d2_open_hall():
    """Almost no internal walls: big open rooms, alcoves on the edges."""
    walls = {key((4, 1), (4, 2)),                                   # alcove (4,1)
             key((1, 4), (1, 5)), key((1, 5), (1, 6)),              # alcove (1,5)
             key((4, 4), (4, 5)), key((4, 5), (4, 6)),              # alcove (4,5)
             key((1, 8), (1, 9)),                                   # alcove (1,9)
             key((2, 2), (2, 3)), key((2, 3), (2, 4)), key((1, 3), (2, 3))}  # alcove (2,3)
    open_e = set(all_edges()) - walls
    tiles = {(4, 1): 'blue', (1, 5): 'blue', (1, 9): 'green', (4, 5): 'green', (2, 3): 'green'}
    return 'Open hall', 'few walls, wide open areas (few walls to localise on), 5 alcove tiles', \
        open_e, tiles


def d3_comb():
    """One corridor with 16 dead-end teeth: the most dead ends to check, 4:3 majority."""
    spine = run(2, 1, 2, 9) + run(3, 9, 4, 9)
    teeth = [[(1, 1), (2, 1)]] + [[(2, c), (1, c)] for c in range(2, 10)] \
        + [[(2, c), (3, c), (4, c)] for c in range(1, 9)]
    open_e = paths(spine, *teeth)
    tiles = {(1, 3): 'blue', (4, 2): 'blue', (1, 6): 'blue', (4, 7): 'blue',
             (1, 8): 'green', (4, 4): 'green', (4, 5): 'green'}
    return 'Comb', '16 dead ends off one corridor, 7 tiles, close 4:3 majority', open_e, tiles


def d4_deep_branches():
    """Long dead-end branches: tiles at the far end of 5-9 cell corridors."""
    open_e = paths(run(1, 1, 1, 9) + run(2, 9, 4, 9),
                   [(1, 2)] + run(2, 2, 4, 2) + run(4, 3, 4, 8),           # to (4,8)
                   run(1, 1, 4, 1),                                       # to (4,1)
                   [(1, 6)] + run(2, 6, 2, 3) + run(3, 3, 3, 8),           # to (3,8)
                   [(1, 7), (2, 7), (2, 8)])                              # to (2,8)
    tiles = {(4, 8): 'green', (4, 1): 'blue', (3, 8): 'green'}
    return 'Deep branches', 'tiles at the far ends of long corridors (camera sees 1 cell ahead)', \
        open_e, tiles


def d5_loops():
    """Many loops: two long wall bars with gaps, every area reachable several ways."""
    walls = {key((2, c), (3, c)) for c in (2, 3, 4, 6, 7, 8)}             # two bars
    walls |= {key((1, 4), (2, 4)), key((3, 6), (4, 6)), key((3, 2), (4, 2))}
    walls |= {key((4, 2), (4, 3)), key((4, 3), (4, 4))}                  # alcove (4,3)
    walls |= {key((1, 6), (1, 7)), key((1, 7), (1, 8))}                  # alcove (1,7)
    walls |= {key((3, 4), (3, 5)), key((3, 5), (3, 6)), key((3, 5), (4, 5))}  # alcove (3,5)
    open_e = set(all_edges()) - walls
    tiles = {(4, 3): 'green', (1, 7): 'blue', (3, 5): 'green'}
    return 'Loops', 'many loops and equal-length routes; 3 alcove tiles', open_e, tiles


DESIGNS = [d1_snake, d2_open_hall, d3_comb, d4_deep_branches, d5_loops]
OBSTACLES = ['TestObstacles1', 'TestObstacles2', 'TestObstacles3', 'TestObstacles1', 'TestObstacles2']


# ------------------------------------------------------------------ checking
def cells():
    return [(r, c) for r in range(1, R + 1) for c in range(1, C + 1)]


def neighbours(cell):
    r, c = cell
    return [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
            if 1 <= r + dr <= R and 1 <= c + dc <= C]


def deg(open_e, cell):
    return sum(1 for n in neighbours(cell) if key(cell, n) in open_e)


def reach(open_e, avoid_end):
    seen, todo = {START}, [START]
    while todo:
        cur = todo.pop()
        for n in neighbours(cur):
            if key(cur, n) in open_e and n not in seen and not (avoid_end and n == END):
                seen.add(n)
                todo.append(n)
    return seen


def check(name, open_e, tiles):
    errs = []
    if len(reach(open_e, False)) != R * C:
        errs.append('not every cell is reachable')
    legal = reach(open_e, True)
    for t in tiles:
        if t in (START, END) or deg(open_e, t) != 1:
            errs.append('tile %s is not in a dead end' % (t,))
        if t not in legal:
            errs.append('tile %s only reachable through the end cell' % (t,))
    if len(tiles) % 2 == 0:
        errs.append('even number of tiles (tie)')
    if errs:
        raise SystemExit('%s: %s' % (name, '; '.join(errs)))


def ascii_map(open_e, tiles):
    lines = []
    for r in range(R, 0, -1):
        top, mid = '', ''
        for c in range(1, C + 1):
            north = r < R and key((r, c), (r + 1, c)) in open_e
            top += '+' + ('   ' if north else '---')
            west = c > 1 and key((r, c - 1), (r, c)) in open_e
            mark = {'blue': 'B', 'green': 'G'}.get(tiles.get((r, c)), ' ')
            if (r, c) == END:
                mark = 'E'
            mid += (' ' if west else '|') + ' %s ' % mark
        lines.append(top + '+')
        lines.append(mid + '|')
    lines.append('+---' * C + '+')
    return '\n'.join(lines)


# ------------------------------------------------------------------ writing
def solid(name, x, y, z, sx, sy, sz, color=None, bound=True):
    """Same node layout as gen_random_world.py."""
    col = 'baseColor %s\n              ' % color if color else ''
    b = '\n        boundingObject Box {\n          size %g %g %g\n        }' % (sx, sy, sz) if bound else ''
    return '''      Solid {
        translation %.4f %.4f %g
        children [
          Shape {
            appearance PBRAppearance {
              %sroughness 1
              metalness 0
            }
            geometry Box {
              size %g %g %g
            }
          }
        ]
        name "%s"%s
      }
''' % (x, y, z, col, sx, sy, sz, name, b)


def arena_proto(n, open_e, tiles):
    def cx(c): return X0 + CELL * (c - 1)
    def cy(r): return Y0 + CELL * (r - 1)
    nodes = ''
    for e in all_edges():
        if e in open_e:
            continue
        (r1, c1), (r2, c2) = e
        if r1 == r2:
            nodes += solid('w_v_%d_%d' % (r1, c1), cx(c1) + 0.125, cy(r1), 0.05, 0.012, 0.262, 0.1)
        else:
            nodes += solid('w_h_%d_%d' % (r1, c1), cx(c1), cy(r1) + 0.125, 0.05, 0.262, 0.012, 0.1)
    for (r, c), col in tiles.items():
        nodes += solid('tile_%s_%d_%d' % (col, r, c), cx(c), cy(r), 0.001, 0.25, 0.25, 0.001,
                       '0 0 0.5' if col == 'blue' else '0 0.6 0', bound=False)
    a = open(ARENA, encoding='utf8').read()
    mark = 'name "end_marker"\n      }\n'
    cut = a.index(mark) + len(mark)
    return a[:cut].replace('PROTO Arena [', 'PROTO TestArenaD%d [' % n) + nodes + '    ]\n  }\n}\n'


def main():
    projects = sys.argv[1:] or [os.path.join(HERE, '..', 'test_run'),
                                os.path.join(HERE, '..', 'test_run_old')]
    out = {}
    for n, (design, obst) in enumerate(zip(DESIGNS, OBSTACLES), 1):
        name, about, open_e, tiles = design()
        check(name, open_e, tiles)
        b = sum(v == 'blue' for v in tiles.values())
        g = len(tiles) - b
        dead = [x for x in cells() if x not in (START, END) and deg(open_e, x) == 1]
        print('D%d %s - %s' % (n, name, about))
        print('   dead ends %d, tiles %d (blue %d, green %d) -> %s, obstacles %s' % (
            len(dead), len(tiles), b, g, 'blue' if b > g else 'green', obst))
        print(ascii_map(open_e, tiles))
        print()
        out['D%d' % n] = {'name': name, 'open': sorted(open_e),
                          'tiles': [[list(k), v] for k, v in tiles.items()]}
        proto = arena_proto(n, open_e, tiles)
        for proj in projects:
            open(os.path.join(proj, 'protos', 'TestArenaD%d.proto' % n), 'w',
                 encoding='utf8', newline='\n').write(proto)
            w = open(os.path.join(proj, 'worlds', 'test_rand1.wbt'), encoding='utf8').read()
            w = w.replace('TestArena1', 'TestArenaD%d' % n).replace('TestObstacles1', obst)
            open(os.path.join(proj, 'worlds', 'test_design%d.wbt' % n), 'w',
                 encoding='utf8', newline='\n').write(w)
    json.dump(out, open(os.path.join(HERE, 'designs.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
