# Advanced hand-designed maze worlds (TEST ONLY, not part of the submission).
#
# Same locked arena as the official Arena.proto (2.44 x 2.44 m, 4 x 9 maze of
# 25 cm cells, 1.2 cm walls, start / transition / end markers); only the
# internal maze walls, tiles and Section 1 obstacles change. Each design is
# checked with gen_design_worlds.check() before it is written (all cells
# reachable, tiles only in dead ends reachable without the end cell, odd tile
# count) - except where a design deliberately breaks "all cells reachable
# without the end cell" (A5), which check() still allows.
#
# Writes protos/TestArenaA<n>.proto plus two worlds per design into the test
# project (default ../test_run):
#   worlds/test_adv<n>.wbt      full run from the Section 1 start zone
#   worlds/test_adv<n>_s2.wbt   dedicated Section 2 attempt (robot placed on
#                               the transition tile, facing north)
# and advanced_designs.json for the report.
#
# usage: python gen_advanced_worlds.py [project_dir]
import json
import os
import sys

from gen_design_worlds import (R, C, START, END, key, all_edges, paths, run, cells,
                               deg, check, ascii_map, arena_proto)

HERE = os.path.dirname(os.path.abspath(__file__))


def bar(r, c0, c1):
    """Horizontal wall along the north side of row r, columns c0..c1."""
    return {key((r, c), (r + 1, c)) for c in range(c0, c1 + 1)}


def post(r0, c, r1):
    """Vertical wall along the east side of column c, rows r0..r1."""
    return {key((r, c), (r, c + 1)) for r in range(r0, r1 + 1)}


# --------------------------------------------------------------------- designs
def a1_staircase():
    """Diagonal staircase: a 90-degree turn after every single cell."""
    main = [(1, 1), (1, 2), (2, 2), (2, 3), (3, 3), (3, 4), (4, 4), (4, 5), (3, 5), (3, 6),
            (2, 6), (2, 7), (1, 7), (1, 8), (2, 8), (3, 8), (3, 9), (4, 9)]
    open_e = paths(main,
                   [(1, 1), (2, 1), (3, 1), (3, 2), (4, 2), (4, 3)],   # stair branch -> (4,3)
                   [(3, 1), (4, 1)],
                   [(1, 2), (1, 3), (1, 4)],
                   [(3, 4), (2, 4), (2, 5), (1, 5), (1, 6)],          # stair branch -> (1,6)
                   [(4, 5), (4, 6), (4, 7), (4, 8)],
                   [(2, 7), (3, 7)],
                   [(1, 8), (1, 9), (2, 9)])
    tiles = {(4, 1): 'blue', (4, 3): 'green', (1, 6): 'green', (3, 7): 'blue', (2, 9): 'green'}
    return 'Staircase', 'zig-zag staircase: a 90-degree turn after nearly every cell; ' \
        'stair-shaped side branches', open_e, tiles


def a2_nine_tiles():
    """Forked teeth: 12 dead ends, 9 tiles, the closest possible majority (5:4)."""
    spine = run(1, 1, 1, 9) + run(2, 9, 4, 9)
    teeth = []
    for c in (1, 3, 5, 7):
        teeth += [[(1, c), (2, c), (3, c), (4, c)],               # dead end (4,c)
                  [(3, c), (3, c + 1), (4, c + 1)],               # dead end (4,c+1)
                  [(3, c + 1), (2, c + 1)]]                       # dead end (2,c+1), walled off
    open_e = paths(spine, *teeth)                                 # from the spine at (1,c+1)
    tiles = {(4, 1): 'green', (4, 2): 'blue', (2, 2): 'green', (4, 3): 'blue',
             (2, 4): 'green', (4, 6): 'blue', (2, 6): 'green', (4, 8): 'blue',
             (2, 8): 'green'}
    return 'Nine tiles', '12 dead ends, 9 tiles, 5:4 majority; dead ends hidden behind ' \
        'walls next to the main corridor', open_e, tiles


def a3_long_corridors():
    """Four 8-9 cell corridors: dead ends beyond the 1 m sensor range."""
    open_e = paths(run(1, 1, 4, 1),
                   run(1, 1, 1, 9) + run(2, 9, 3, 9),             # -> (3,9), beside the end
                   run(2, 1, 2, 8),                                # -> (2,8)
                   run(3, 1, 3, 8),                                # -> (3,8)
                   run(4, 1, 4, 9))                                # -> end cell
    tiles = {(3, 9): 'green', (2, 8): 'blue', (3, 8): 'green'}
    return 'Long corridors', 'four parallel 2 m corridors with featureless walls; dead ends ' \
        'out of sensor range, one right beside the end cell', open_e, tiles


def parse(art):
    """Open edges + tiles from an ASCII map in ascii_map() layout (row 4 on top)."""
    lines = art.strip('\n').split('\n')
    assert len(lines) == 2 * R + 1
    open_e, tiles = set(), {}
    for i, r in enumerate(range(R, 0, -1)):
        mid, below = lines[2 * i + 1], lines[2 * i + 2]
        for c in range(1, C + 1):
            x = 4 * (c - 1)
            if c > 1 and mid[x] == ' ':
                open_e.add(key((r, c - 1), (r, c)))
            if r > 1 and below[x + 1:x + 4] == '   ':
                open_e.add(key((r - 1, c), (r, c)))
            if mid[x + 2] in 'BG':
                tiles[(r, c)] = 'blue' if mid[x + 2] == 'B' else 'green'
    return open_e, tiles


def a4_islands():
    """Free-standing wall islands: loops everywhere, few walls to localise on."""
    open_e, tiles = parse(r"""
+---+---+---+---+---+---+---+---+---+
| B |                   | G |     E |
+   +   +---+---+   +   +   +   +   +
|       |           |               |
+   +   +   +   +---+   +---+---+   +
|                   |       |       |
+---+   +---+   +   +   +   +   +---+
|                                 B |
+---+---+---+---+---+---+---+---+---+
""")
    return 'Islands', 'free-standing wall islands, loops everywhere, open areas with no side ' \
        'walls; only 3 dead ends (boundary alcoves)', open_e, tiles


def a5_behind_end():
    """Part of the maze is only reachable THROUGH the end cell (must be left alone)."""
    open_e = paths(run(1, 1, 1, 7) + run(2, 7, 4, 7) + [(4, 8), (4, 9)],
                   run(4, 9, 1, 9) + [(1, 8)], [(3, 9), (3, 8), (2, 8)],   # behind the end
                   run(1, 1, 4, 1), [(4, 1), (4, 2), (3, 2), (2, 2)],
                   [(1, 3), (2, 3), (3, 3), (4, 3), (4, 4)], [(3, 3), (3, 4), (2, 4)],
                   [(1, 5), (2, 5), (3, 5), (4, 5), (4, 6)], [(2, 5), (2, 6), (3, 6)])
    tiles = {(2, 2): 'blue', (4, 4): 'blue', (2, 4): 'green', (4, 6): 'green', (3, 6): 'blue'}
    return 'Behind the end', '6 cells (2 empty dead ends) reachable only THROUGH the end ' \
        'cell; exploration must give up on them and still finish', open_e, tiles


def a6_rooms():
    """Open 2x2 / 2x3 rooms joined by single doorways, tiles in closets."""
    rooms = [{(r, c) for r in rs for c in cs} for rs, cs in
             (((1, 2), (1, 2)), ((3, 4), (1, 2, 3)), ((1, 2), (3, 4, 5)), ((3, 4), (4, 5, 6)),
              ((1, 2), (6, 7, 8)), ((3, 4), (7, 8, 9)))]
    open_e = {e for e in all_edges() if any(e[0] in rm and e[1] in rm for rm in rooms)}
    open_e |= {key((2, 1), (3, 1)), key((2, 2), (2, 3)), key((2, 5), (3, 5)),     # doors
               key((2, 6), (3, 6)), key((1, 5), (1, 6)), key((2, 8), (3, 8)),
               key((1, 8), (1, 9)), key((2, 8), (2, 9))}                          # closets
    open_e -= {key((4, 1), (4, 2)), key((4, 4), (4, 5)), key((1, 3), (1, 4)),     # closets
               key((3, 7), (4, 7))}                                               # inside rooms
    tiles = {(4, 1): 'green', (4, 4): 'blue', (1, 3): 'green', (1, 9): 'blue', (2, 9): 'green'}
    return 'Rooms', 'open 2x2 / 2x3 rooms (no walls inside) joined by single doorways, ' \
        'tiles in closets, two empty closets in the end room', open_e, tiles


def a7_ladder():
    """Two full-length rails joined by rungs; every other cell is a dead-end pocket."""
    rails = [run(2, 1, 2, 9), run(3, 1, 3, 9)]
    rungs = [[(2, c), (3, c)] for c in (1, 3, 5, 7, 9)]
    pockets = [[(1, c), (2, c)] for c in range(1, 10)] + [[(3, c), (4, c)] for c in range(1, 10)]
    open_e = paths(*rails, *rungs, *pockets)
    tiles = {(1, 3): 'green', (4, 2): 'blue', (1, 6): 'blue', (4, 5): 'green', (1, 8): 'green',
             (4, 7): 'blue', (4, 8): 'green'}
    return 'Ladder', 'two 9-cell rails joined by 5 rungs (many loops), 16 dead-end pockets ' \
        'beside the rails, 7 tiles 4:3', open_e, tiles


def a8_spiral():
    """Outer ring to the end; a 24-cell spiral dead end with the only tile at its centre."""
    ring = run(1, 1, 4, 1) + run(4, 2, 4, 9)
    spiral = [(1, 1)] + run(1, 2, 1, 9) + run(2, 9, 3, 9) + run(3, 8, 3, 2) + run(2, 2, 2, 8)
    open_e = paths(ring, spiral)
    tiles = {(2, 8): 'blue'}
    return 'Spiral', '24-cell spiral dead end, the single tile at its centre right beside ' \
        'the end cell (behind a wall)', open_e, tiles


def _perfect(seed):
    import random
    rnd = random.Random(seed)
    open_e, seen, stack = set(), {START}, [START]
    while stack:
        r, c = stack[-1]
        nb = [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
              if 1 <= r + dr <= R and 1 <= c + dc <= C and (r + dr, c + dc) not in seen]
        if not nb:
            stack.pop()
            continue
        n = rnd.choice(nb)
        open_e.add(key((r, c), n))
        seen.add(n)
        stack.append(n)
    return open_e, rnd


def _legal_dead(open_e):
    from gen_design_worlds import reach
    legal = reach(open_e, True)
    return [x for x in cells() if x not in (START, END) and deg(open_e, x) == 1 and x in legal]


def _depth(open_e):
    """Total BFS distance from the entrance to every cell (long winding = hard)."""
    dist, todo = {START: 0}, [START]
    for cur in todo:
        for n in [(cur[0] + 1, cur[1]), (cur[0] - 1, cur[1]), (cur[0], cur[1] + 1),
                  (cur[0], cur[1] - 1)]:
            if key(cur, n) in open_e and n not in dist:
                dist[n] = dist[cur] + 1
                todo.append(n)
    return sum(dist.values())


def _tiles(rnd, dead, k):
    k = min(k, len(dead) - (1 - len(dead) % 2))
    k -= 1 - k % 2                                             # odd
    pick = rnd.sample(sorted(dead), k)
    cols = ['blue'] * (k // 2) + ['green'] * (k // 2) + [rnd.choice(['blue', 'green'])]
    rnd.shuffle(cols)
    return dict(zip(pick, cols))


def hard_perfect(rank):
    """Perfect maze (no loops) with the most dead ends + deepest layout among 3000 seeds."""
    scored = []
    for seed in range(3000):
        o, _ = _perfect(seed)
        scored.append((len(_legal_dead(o)) * 40 + _depth(o), seed))
    seed = sorted(scored, reverse=True)[rank][1]
    open_e, rnd = _perfect(seed)
    tiles = _tiles(rnd, _legal_dead(open_e), 7)
    return 'Hard perfect %d' % (rank + 1), 'no loops; the deepest / most branched of 3000 ' \
        'random perfect mazes (seed %d), %d dead ends, 7 tiles' % (seed, len(_legal_dead(open_e))), \
        open_e, tiles


def braided(rank):
    """Heavily looped maze: every dead end but 3 opened, plus 6 extra walls removed."""
    import random
    from gen_design_worlds import reach

    def braid(seed):
        open_e, _ = _perfect(seed)
        rnd = random.Random(seed + 7)
        dead = _legal_dead(open_e)
        rnd.shuffle(dead)
        keep = dead[:3]                         # tiled dead ends
        for cell in dead[3:] + [x for x in cells() if deg(open_e, x) == 1]:
            if cell in keep or cell in (START, END) or deg(open_e, cell) != 1:
                continue
            nb = [(cell[0] + dr, cell[1] + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))]
            nb = [n for n in nb if 1 <= n[0] <= R and 1 <= n[1] <= C and n != END
                  and n not in keep and key(cell, n) not in open_e]
            if nb:
                open_e.add(key(cell, rnd.choice(nb)))
        spare = [e for e in all_edges() if e not in open_e and not set(e) & set(keep + [END])]
        for e in rnd.sample(spare, min(6, len(spare))):
            open_e.add(e)
        return open_e, rnd, keep
    good = []
    for sd in range(400):
        o, _, keep = braid(sd)
        legal = reach(o, True)
        if len(keep) == 3 and all(k in legal and deg(o, k) == 1 for k in keep):
            good.append(sd)
    seed = good[rank * 7]
    open_e, rnd, keep = braid(seed)
    cols = ['blue', 'green', rnd.choice(['blue', 'green'])]
    rnd.shuffle(cols)
    tiles = dict(zip(keep, cols))
    loops = len(open_e) - (R * C - 1)
    return 'Braided %d' % (rank + 1), 'random maze (seed %d) braided into %d loops: open '         'areas, many equal-cost routes (the A4 livelock pattern), only 3 dead ends' % (seed, loops),         open_e, tiles


DESIGNS = [a1_staircase, a2_nine_tiles, a3_long_corridors, a4_islands, a5_behind_end, a6_rooms,
           a7_ladder, a8_spiral, lambda: hard_perfect(0), lambda: hard_perfect(1),
           lambda: braided(0), lambda: braided(1)]
OBSTACLES = ['TestObstacles1', 'TestObstacles2', 'TestObstacles3',
             'TestObstacles2', 'TestObstacles3', 'TestObstacles1',
             'TestObstacles3', 'TestObstacles1', 'TestObstacles2',
             'TestObstacles3', 'TestObstacles1', 'TestObstacles2']



def world(template, n, obst, s2_only):
    w = template.replace('TestArena1', 'TestArenaA%d' % n).replace('TestObstacles1', obst)
    w = w.replace('controller "test_truth"', 'controller "test_judge"')
    if s2_only:      # judges place the robot on the transition tile, facing north
        w = w.replace('translation 2.315 0.125 0\n  rotation 0 0 1 3.14159',
                      'translation 0.22 1.275 0\n  rotation 0 0 1 1.5708')
        assert '0.22 1.275' in w
    return w


def main():
    proj = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', 'test_run')
    template = open(os.path.join(proj, 'worlds', 'test_rand1.wbt'), encoding='utf8').read()
    out = {}
    for n, (design, obst) in enumerate(zip(DESIGNS, OBSTACLES), 1):
        name, about, open_e, tiles = design()
        check(name, open_e, tiles)
        b = sum(v == 'blue' for v in tiles.values())
        g = len(tiles) - b
        dead = [x for x in cells() if x not in (START, END) and deg(open_e, x) == 1]
        n_walls = len(all_edges()) - len(open_e)
        print('A%d %s - %s' % (n, name, about))
        print('   dead ends %d, tiles %d (blue %d, green %d) -> %s, internal walls %d, '
              'obstacles %s' % (len(dead), len(tiles), b, g, 'blue' if b > g else 'green',
                                n_walls, obst))
        amap = ascii_map(open_e, tiles)
        print(amap)
        print()
        out['A%d' % n] = {'name': name, 'about': about, 'obstacles': obst,
                          'open': sorted(open_e), 'walls': n_walls, 'dead_ends': len(dead),
                          'tiles': [[list(k), v] for k, v in tiles.items()],
                          'blue': b, 'green': g, 'expect': 'blue' if b > g else 'green',
                          'map': amap}
        open(os.path.join(proj, 'protos', 'TestArenaA%d.proto' % n), 'w',
             encoding='utf8', newline='\n').write(arena_proto(n, open_e, tiles)
                                                  .replace('TestArenaD%d' % n, 'TestArenaA%d' % n))
        for s2_only, suffix in ((False, ''), (True, '_s2')):
            open(os.path.join(proj, 'worlds', 'test_adv%d%s.wbt' % (n, suffix)), 'w',
                 encoding='utf8', newline='\n').write(world(template, n, obst, s2_only))
    json.dump(out, open(os.path.join(HERE, 'advanced_designs.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
