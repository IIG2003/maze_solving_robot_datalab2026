# Offline Section 2 simulator (TEST ONLY, not part of the submission).
#
# Runs the real section2.run() exploration / planning code against random
# 4 x 9 mazes, replacing sensing and driving with perfect fakes:
#   sense()      -> walls up to 3 cells away along each axis (like peek with a
#                   1.0 m range sensor), floor of the current cell
#   look_ahead() -> floor of the next cell if the edge is open
#   Navigator    -> moves cell by cell; stops at walls it did not know about
# It reports cells driven, 90-degree turns, stops, an estimated time, and
# whether the tile count / majority LED is correct.
#
# usage: python maze_sim.py <controllers/main_controller dir> [n_mazes] [seed0]
import os
import random
import sys
import types

ctrl = os.path.abspath(sys.argv[1])
n_mazes = int(sys.argv[2]) if len(sys.argv) > 2 else 200
seed0 = int(sys.argv[3]) if len(sys.argv) > 3 else 1
sys.path.insert(0, ctrl)
stub = types.ModuleType('controller')
stub.Robot = object
sys.modules['controller'] = stub

import config as cfg            # noqa: E402
cfg.DEBUG = False
import section2 as s2           # noqa: E402

R, C = cfg.ROWS, cfg.COLS
# rough time model (s), from the Webots logs: 0.25 m at 0.16 m/s plus ramps,
# IMU turn + settle, halt + sensing at every stop
T_CELL, T_TURN, T_STOP = 1.7, 0.6, 0.4


def gen_maze(seed):
    """
    Same recipe as gen_random_world.py: DFS perfect maze + 3 loops, odd tiles
    in dead ends, but only in dead ends reachable without the end cell.
    """
    rnd = random.Random(seed)
    def key(a, b): return tuple(sorted((a, b)))
    open_e, seen, stack = set(), {(1, 1)}, [(1, 1)]
    while stack:
        r, c = stack[-1]
        nb = [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
              if 1 <= r + dr <= R and 1 <= c + dc <= C and (r + dr, c + dc) not in seen]
        if not nb:
            stack.pop()
            continue
        n = rnd.choice(nb)
        open_e.add(key((r, c), n)); seen.add(n); stack.append(n)
    edges = [key((r, c), (r + dr, c + dc)) for r in range(1, R + 1) for c in range(1, C + 1)
             for dr, dc in ((1, 0), (0, 1)) if r + dr <= R and c + dc <= C]
    closed = [e for e in edges if e not in open_e]
    for e in rnd.sample(closed, 3):
        open_e.add(e)
    def deg(cell): return sum(1 for e in open_e if cell in e)
    # cells reachable without entering the end cell (it may not be driven
    # through, so a tile only reachable through it could never be seen)
    legal, todo = {(1, 1)}, [(1, 1)]
    while todo:
        cur = todo.pop()
        for e in open_e:
            if cur in e:
                n = e[0] if e[1] == cur else e[1]
                if n != (4, 9) and n not in legal:
                    legal.add(n)
                    todo.append(n)
    dead = [(r, c) for r in range(1, R + 1) for c in range(1, C + 1)
            if deg((r, c)) == 1 and (r, c) in legal and (r, c) != (1, 1)]
    if not dead:
        return gen_maze(seed + 100000)         # no dead end for a tile: re-roll
    k = min(len(dead), rnd.choice([1, 3, 5, 7]))
    if k % 2 == 0:
        k -= 1                                  # odd count: no ties
    tiles = {cell: rnd.choice(['blue', 'green']) for cell in rnd.sample(dead, k)}
    return open_e, tiles


class Done(Exception):
    pass


def simulate(seed):
    open_e, tiles = gen_maze(seed)

    def is_open(cell, d):
        nxt = (cell[0] + s2.DR[d], cell[1] + s2.DC[d])
        return s2.inside(*nxt) and tuple(sorted((cell, nxt))) in open_e

    st = {'cells': 0, 'turns': 0, 'stops': 0, 'nav': None, 'end_entries': 0}

    class Bot:
        t = 0.0
        listeners = []
        def halt(self, n=3): pass
        def finish(self, colour): raise Done(colour)

    bot = Bot()

    def fake_sense(bot_, maze, cell, d):
        st['stops'] += 1
        bot.t += T_STOP
        for dd in range(4):
            cc = cell
            for _ in range(4):                       # walls up to 3 cells away
                if not s2.inside(cc[0] + s2.DR[dd], cc[1] + s2.DC[dd]):
                    break
                if is_open(cc, dd):
                    maze.set(cc, dd, False)
                    cc = (cc[0] + s2.DR[dd], cc[1] + s2.DC[dd])
                else:
                    maze.set(cc, dd, True)
                    break
        maze.visited.add(cell)
        maze.tile[cell] = tiles.get(cell, 'none')
        fake_look(bot_, maze, cell, d)

    def fake_look(bot_, maze, cell, d):
        if maze.get(cell, d) is not False:
            return
        nxt = (cell[0] + s2.DR[d], cell[1] + s2.DC[d])
        maze.tile[nxt] = tiles.get(nxt, 'none')

    class Nav:
        def __init__(self, bot_, maze, cell, d):
            self.maze, self.cell, self.d = maze, cell, d
            st['nav'] = self
        def turn(self, d):
            q = (d - self.d) % 4
            st['turns'] += 2 if q == 2 else (1 if q else 0)
            bot.t += T_TURN * (2 if q == 2 else (1 if q else 0))
            self.d = d
        def run(self, d, end):
            self.turn(d)
            while self.cell != end:
                if not is_open(self.cell, d):          # wall we did not know about
                    self.maze.set(self.cell, d, True)
                    return False
                self.cell = (self.cell[0] + s2.DR[d], self.cell[1] + s2.DC[d])
                self.maze.visited.add(self.cell)
                st['cells'] += 1
                st['end_entries'] += self.cell == cfg.END_CELL
                bot.t += T_CELL
            return True

    s2.sense, s2.look_ahead, s2.Navigator = fake_sense, fake_look, Nav
    try:
        s2.run(bot, cfg.ENTRY_CELL, s2.N)
        colour = None
    except Done as e:
        colour = e.args[0]
    b = sum(v == 'blue' for v in tiles.values())
    g = len(tiles) - b
    truth = 'blue' if b > g else 'green'
    return dict(cells=st['cells'], turns=st['turns'], stops=st['stops'], time=bot.t,
                ok=(colour == truth), at_end=(st['nav'].cell == cfg.END_CELL),
                end_early=st['end_entries'] > 1)


res = [simulate(seed0 + i) for i in range(n_mazes)]
n = len(res)
avg = lambda k: sum(r[k] for r in res) / n
print('%s  mazes=%d' % (ctrl, n))
print('  cells %.1f  turns %.1f  stops %.1f  est.time %.1fs' % (
    avg('cells'), avg('turns'), avg('stops'), avg('time')))
print('  correct LED %d/%d   reached end %d/%d   entered end cell early %d' % (
    sum(r['ok'] for r in res), n, sum(r['at_end'] for r in res), n,
    sum(r['end_early'] for r in res)))
