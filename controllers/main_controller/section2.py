import heapq
import math

import config as cfg
import vision
from hardware import log, wrap
from watchdog import LoopGuard

N, E, S, W = 0, 1, 2, 3
DR = (1, 0, -1, 0)
DC = (0, 1, 0, -1)
HEADING = (math.pi / 2, 0.0, -math.pi / 2, math.pi)
NAMES = 'NESW'


def left_of(d):
    return (d + 3) % 4


def right_of(d):
    return (d + 1) % 4


def back_of(d):
    return (d + 2) % 4


def inside(r, c):
    return 1 <= r <= cfg.ROWS and 1 <= c <= cfg.COLS


def centre(cell):
    r, c = cell
    return cfg.MAZE_X0 + cfg.CELL * (c - 1), cfg.MAZE_Y0 + cfg.CELL * (r - 1)


def cell_of(x, y):
    c = int(round((x - cfg.MAZE_X0) / cfg.CELL)) + 1
    r = int(round((y - cfg.MAZE_Y0) / cfg.CELL)) + 1
    return r, c


def nearest_dir(th):
    return min(range(4), key=lambda d: abs(wrap(th - HEADING[d])))


class Maze:
    def __init__(self):
        self.walls = {}
        self.visited = set()
        self.tile = {}

    @staticmethod
    def _key(r, c, d):
        if d == S:
            return (r - 1, c, N)
        if d == W:
            return (r, c - 1, E)
        return (r, c, d)

    def get(self, cell, d):
        r, c = cell
        if not inside(r + DR[d], c + DC[d]):
            return True
        return self.walls.get(self._key(r, c, d))

    def set(self, cell, d, wall):
        r, c = cell
        if inside(r, c) and inside(r + DR[d], c + DC[d]):
            self.walls[self._key(r, c, d)] = wall

    def n_walls(self, cell):
        known = [self.get(cell, d) for d in range(4)]
        return sum(1 for w in known if w is True), sum(1 for w in known if w is None)

    def resolved(self, cell):
        if cell in self.visited:
            return True
        if self.tile.get(cell) in ('blue', 'green'):
            return True
        walls, unknown = self.n_walls(cell)
        if unknown:
            return False
        return walls < 3 or cell in self.tile

    def counts(self):
        b = sum(1 for v in self.tile.values() if v == 'blue')
        g = sum(1 for v in self.tile.values() if v == 'green')
        return b, g

    def dump(self):
        lines = []
        for r in range(cfg.ROWS, 0, -1):
            top, mid = '', ''
            for c in range(1, cfg.COLS + 1):
                n = self.get((r, c), N)
                top += '+' + ('---' if n else '   ' if n is False else ' ? ')
                w = self.get((r, c), W)
                t = self.tile.get((r, c), '')
                mark = {'blue': 'B', 'green': 'G'}.get(t, '.' if (r, c) in self.visited else ' ')
                mid += ('|' if w else ' ' if w is False else '?') + ' %s ' % mark
            e = self.get((r, cfg.COLS), E)
            lines.append(top + '+')
            lines.append(mid + ('|' if e else '?'))
        lines.append('+---' * cfg.COLS + '+')
        return '\n'.join(lines)


def lateral_listener(bot):
    def f():
        if bot.y < cfg.S1_H + 0.02:
            return
        d = nearest_dir(bot.th)
        h = HEADING[d]
        if abs(wrap(bot.th - h)) > math.radians(6):
            return
        lx, ly = -math.sin(h), math.cos(h)
        est = []
        side = cfg.FACE - cfg.OFF_SIDE
        if bot.d['ds_l'] < 0.16:
            est.append(side - bot.d['ds_l'])
        if bot.d['ds_r'] < 0.16:
            est.append(bot.d['ds_r'] - side)
        if not est:
            return
        off = sum(est) / len(est)
        if abs(off) > 0.045:
            return
        if d in (N, S):
            line = cfg.MAZE_X0 + cfg.CELL * round((bot.x - cfg.MAZE_X0) / cfg.CELL)
            want = line + off * lx
            bot.x += 0.25 * (want - bot.x)
        else:
            line = cfg.MAZE_Y0 + cfg.CELL * round((bot.y - cfg.MAZE_Y0) / cfg.CELL)
            want = line + off * ly
            bot.y += 0.25 * (want - bot.y)
    return f


def snap(bot, cell, d):
    cx, cy = centre(cell)
    h = HEADING[d]
    ux, uy = round(math.cos(h)), round(math.sin(h))
    lx, ly = -uy, ux
    along, lat = [], []
    if bot.d['ds_f'] < cfg.WALL_THRESH_F:
        along.append(cfg.FACE - cfg.OFF_F - bot.d['ds_f'])
    if bot.d['ds_b'] < cfg.WALL_THRESH_F:
        along.append(-(cfg.FACE - cfg.OFF_B - bot.d['ds_b']))
    if bot.d['ds_l'] < cfg.WALL_THRESH_S:
        lat.append(cfg.FACE - cfg.OFF_SIDE - bot.d['ds_l'])
    if bot.d['ds_r'] < cfg.WALL_THRESH_S:
        lat.append(bot.d['ds_r'] - (cfg.FACE - cfg.OFF_SIDE))
    x, y = bot.x, bot.y
    if along:
        a = sum(along) / len(along)
        if abs(a) < 0.07:
            if ux:
                x = cx + a * ux
            else:
                y = cy + a * uy
    if lat:
        l = sum(lat) / len(lat)
        if abs(l) < 0.07:
            if lx:
                x = cx + l * lx
            else:
                y = cy + l * ly
    bot.set_pose(x, y)


def peek(maze, cell, d, reading, offset):
    base = cfg.FACE - offset
    r, c = cell
    if reading >= cfg.DS_HIT:
        k_wall = None
        k_open = 3
    else:
        k = (reading - base) / cfg.CELL
        kr = int(round(k))
        if kr < 0 or abs(k - kr) > 0.2 or kr > 3:
            return
        k_wall, k_open = kr, kr
    for k in range(k_open):
        cc = (r + DR[d] * k, c + DC[d] * k)
        if not inside(*cc) or not inside(cc[0] + DR[d], cc[1] + DC[d]):
            return
        maze.set(cc, d, False)
    if k_wall is not None:
        cc = (r + DR[d] * k_wall, c + DC[d] * k_wall)
        if inside(*cc):
            maze.set(cc, d, True)


def look_ahead(bot, maze, cell, d):
    if maze.get(cell, d) is not False:
        return
    nxt = (cell[0] + DR[d], cell[1] + DC[d])
    if maze.tile.get(nxt) in ('blue', 'green'):
        return
    col = vision.stable(bot, vision.ahead, 2)
    if col in ('blue', 'green'):
        if maze.tile.get(nxt) != col:
            log('  tile %s seen ahead in cell %s' % (col, nxt))
        maze.tile[nxt] = col
        for k in range(4):
            if k != back_of(d) and maze.get(nxt, k) is None:
                maze.set(nxt, k, True)
    elif col == 'black' and nxt not in maze.tile:
        maze.tile[nxt] = 'none'


def sense(bot, maze, cell, d):
    bot.halt(2)
    snap(bot, cell, d)
    peek(maze, cell, d, bot.d['ds_f'], cfg.OFF_F)
    peek(maze, cell, back_of(d), bot.d['ds_b'], cfg.OFF_B)
    peek(maze, cell, left_of(d), bot.d['ds_l'], cfg.OFF_SIDE)
    peek(maze, cell, right_of(d), bot.d['ds_r'], cfg.OFF_SIDE)
    maze.visited.add(cell)
    here = vision.stable(bot, vision.near, 2)
    if here in ('blue', 'green'):
        if maze.tile.get(cell) != here:
            log('  tile %s under robot in cell %s' % (here, cell))
        maze.tile[cell] = here
    elif cell not in maze.tile:
        maze.tile[cell] = 'none'
    look_ahead(bot, maze, cell, d)


def search(maze, start, d0, goal_fn=None, allow_unknown=False, allow_end=False):
    s = (start, d0)
    dist = {s: 0.0}
    parent = {s: None}
    heap = [(0.0, 0, s)]
    tie = 1
    done = set()
    while heap:
        g, _, st = heapq.heappop(heap)
        if st in done:
            continue
        done.add(st)
        cell, d = st
        if goal_fn is not None and cell != start and goal_fn(cell):
            return dist, parent, st
        moves = []
        for turn, cost in ((1, cfg.TURN_COST), (3, cfg.TURN_COST), (2, 1.7 * cfg.TURN_COST)):
            moves.append(((cell, (d + turn) % 4), cost))
        w = maze.get(cell, d)
        nxt = (cell[0] + DR[d], cell[1] + DC[d])
        if inside(*nxt) and (w is False or (w is None and allow_unknown)) \
                and (allow_end or nxt != cfg.END_CELL):
            moves.append(((nxt, d), 1.0 if w is False else cfg.UNKNOWN_EDGE_COST))
        for ns, cost in moves:
            ng = g + cost
            if ng < dist.get(ns, 1e18):
                dist[ns] = ng
                parent[ns] = st
                heapq.heappush(heap, (ng, tie, ns))
                tie += 1
    return dist, parent, None


def cells_to(parent, st):
    path = []
    while st is not None:
        if not path or path[-1] != st[0]:
            path.append(st[0])
        st = parent[st]
    return path[::-1]


def plan(maze, start, d0, goal_fn, allow_unknown=False, allow_end=False):
    _, parent, goal = search(maze, start, d0, goal_fn, allow_unknown, allow_end)
    return None if goal is None else cells_to(parent, goal)


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def hides_cell(maze, c, reachable):
    for d in range(4):
        nb = (c[0] + DR[d], c[1] + DC[d])
        if maze.get(c, d) is None and nb != cfg.END_CELL and nb not in reachable \
                and not maze.resolved(nb):
            return True
    return False


def candidates(maze, start, dist, banned):
    best, hidden = {}, {}
    reachable = {st[0] for st in dist}
    for st, g in dist.items():
        c, d = st
        cands = []
        if c != start and not maze.resolved(c):
            cands.append(('visit', c))
        nxt = (c[0] + DR[d], c[1] + DC[d])
        if inside(*nxt) and nxt != cfg.END_CELL and maze.get(c, d) is False \
                and not maze.resolved(nxt) and maze.n_walls(nxt) == (3, 0):
            cands.append(('view', nxt))
        for kind, cell in cands:
            if not banned(cell) and (cell not in best or g < best[cell][0]):
                best[cell] = (g, kind, st)
        if c != start and not banned(c) and hides_cell(maze, c, reachable) \
                and (c not in hidden or g < hidden[c][0]):
            hidden[c] = (g, 'visit', st)
    return best or hidden


def tour_order(first_cost, pair, end_cost):
    n = len(first_cost)
    if n == 1:
        return 0
    if n <= cfg.TOUR_EXACT:
        best = {(1 << j, j): first_cost[j] for j in range(n)}
        for mask in range(1, 1 << n):
            for j in range(n):
                g = best.get((mask, j))
                if g is None:
                    continue
                for k in range(n):
                    if mask & (1 << k):
                        continue
                    key = (mask | (1 << k), k)
                    ng = g + pair[j][k]
                    if ng < best.get(key, 1e18):
                        best[key] = ng
        full = (1 << n) - 1
        last = min(range(n), key=lambda j: best[(full, j)] + end_cost[j])
        mask, j = full, last
        while mask != 1 << j:
            prev = mask & ~(1 << j)
            j = min((i for i in range(n) if prev & (1 << i)),
                    key=lambda i: best[(prev, i)] + pair[i][j])
            mask = prev
        return j

    def nn_tour(i):
        left = set(range(n)) - {i}
        g, cur = first_cost[i], i
        while left:
            nxt = min(left, key=lambda k: pair[cur][k])
            g += pair[cur][nxt]
            left.discard(nxt)
            cur = nxt
        return g + end_cost[cur]
    return min(range(n), key=nn_tour)


def pick_target(maze, start, d0, banned, keep=None):
    dist, parent, _ = search(maze, start, d0)
    best = candidates(maze, start, dist, banned)
    if not best:
        return None
    glance = [c for c, (g, kind, st) in best.items() if kind == 'view' and st[0] == start]
    cells = list(best)
    if glance:
        cell = min(glance, key=lambda c: best[c][0])
    elif keep in best:
        cell = keep
    elif cfg.TARGET_ORDER == 'tour':
        first_cost = [best[c][0] for c in cells]
        pair, end_cost = [], []
        for c in cells:
            st = best[c][2]
            dmap, _, _ = search(maze, st[0], st[1])
            pair.append([min((g for s, g in dmap.items()
                              if s == best[k][2] or (best[k][1] == 'visit' and s[0] == k)),
                             default=1e6) for k in cells])
            emap, _, _ = search(maze, st[0], st[1], allow_unknown=True, allow_end=True)
            end_cost.append(min((g for s, g in emap.items() if s[0] == cfg.END_CELL),
                                default=1e6))
        cell = cells[tour_order(first_cost, pair, end_cost)]
    else:
        cell = min(cells, key=lambda c: best[c][0]
                   - cfg.GOAL_DEFER * manhattan(c, cfg.END_CELL))
    _, kind, st = best[cell]
    return kind, st, cells_to(parent, st)


def view(bot, maze, nav, d):
    nb = (nav.cell[0] + DR[d], nav.cell[1] + DC[d])
    nav.turn(d)
    bot.halt(1)
    look_ahead(bot, maze, nav.cell, d)
    if nb not in maze.tile:
        bot.halt(2)
        look_ahead(bot, maze, nav.cell, d)
    maze.tile.setdefault(nb, 'none')


def first_run(path):
    a, b = path[0], path[1]
    d = next(k for k in range(4) if (a[0] + DR[k], a[1] + DC[k]) == b)
    end = b
    for nxt in path[2:]:
        if (end[0] + DR[d], end[1] + DC[d]) != nxt:
            break
        end = nxt
    return d, end


class Navigator:
    def __init__(self, bot, maze, cell, d):
        self.bot, self.maze = bot, maze
        self.cell, self.d = cell, d
        self.exploring = True

    def turn(self, d):
        if d != self.d or abs(wrap(self.bot.th - HEADING[d])) > cfg.TURN_TOL * 2:
            self.bot.turn_to(HEADING[d])
        self.d = d

    def run(self, d, end):
        bot, maze = self.bot, self.maze
        self.turn(d)
        bot.halt(1)
        if bot.d['ds_f'] < cfg.WALL_THRESH_F:
            maze.set(self.cell, d, True)
            log('  unexpected wall %s of %s' % (NAMES[d], self.cell))
            return False
        f = bot.d['ds_f']
        n = abs(end[0] - self.cell[0]) + abs(end[1] - self.cell[1])
        if f < cfg.DS_HIT:
            k = (f - (cfg.FACE - cfg.OFF_F)) / cfg.CELL
            kr = int(round(k))
            if abs(k - kr) < 0.25 and 1 <= kr < n:
                stop = (self.cell[0] + DR[d] * kr, self.cell[1] + DC[d] * kr)
                maze.set(stop, d, True)
                log('  wall %s of %s seen ahead - run shortened' % (NAMES[d], stop))
                end = stop
        look_ahead(bot, maze, self.cell, d)
        nxt = (self.cell[0] + DR[d], self.cell[1] + DC[d])
        if self.exploring and end == nxt and maze.tile.get(nxt) in ('blue', 'green'):
            return True
        c, known = self.cell, True
        while c != end:
            c = (c[0] + DR[d], c[1] + DC[d])
            known = known and c in maze.visited
        speed = cfg.S2_KNOWN if known and n >= 2 else cfg.S2_CRUISE
        tx, ty = centre(end)
        res = bot.drive_axis(tx, ty, HEADING[d], speed)
        c = self.cell
        while c != end:
            c = (c[0] + DR[d], c[1] + DC[d])
            if res == 'ok':
                maze.visited.add(c)
        if res == 'ok':
            self.cell = end
            return True
        cell = cell_of(bot.x, bot.y)
        if not inside(*cell):
            cell = self.cell
        log('  run %s -> %s ended "%s" at %s' % (self.cell, end, res, cell))
        if res == 'blocked':
            maze.set(cell, d, True)
        cx, cy = centre(cell)
        bot.set_pose(cx if d in (N, S) else bot.x, cy if d in (E, W) else bot.y)
        ahead = (cx - bot.x) * math.cos(HEADING[d]) + (cy - bot.y) * math.sin(HEADING[d])
        if ahead > 0.005:
            bot.drive_axis(cx, cy, HEADING[d], 3.0, timeout=2.0)
        elif ahead < -0.005:
            bot.back_off(-ahead / (3.0 * cfg.WHEEL_RADIUS), 3.0)
        self.cell = cell
        return False


def majority(maze):
    b, g = maze.counts()
    if b == g:
        return cfg.TIE_COLOUR
    return 'blue' if b > g else 'green'


def run(bot, cell=cfg.ENTRY_CELL, d=N):
    log('Section 2 start in cell %s' % (cell,))
    t0 = bot.t
    maze = Maze()
    nav = Navigator(bot, maze, cell, d)
    lat = lateral_listener(bot)
    bot.listeners.append(lat)
    guard = LoopGuard()
    committed = None

    steps = 0
    viewed = False
    while steps < 200:
        steps += 1
        if not viewed:
            sense(bot, maze, nav.cell, nav.d)
        viewed = False

        b, g = maze.counts()
        if cfg.EARLY_STOP:
            open_cells = sum(1 for r in range(1, cfg.ROWS + 1) for c in range(1, cfg.COLS + 1)
                             if not maze.resolved((r, c)) and (r, c) != cfg.END_CELL)
            if abs(b - g) > open_cells:
                log('majority decided early (blue %d, green %d)' % (b, g))
                break

        target = pick_target(maze, nav.cell, nav.d, lambda c: guard.banned(c, bot.t),
                             committed)
        if target is None:
            break
        kind, (tcell, td), path = target
        if kind == 'view' and tcell == nav.cell:
            view(bot, maze, nav, td)
            viewed = True
            continue
        committed = tcell if kind == 'visit' else (tcell[0] + DR[td], tcell[1] + DC[td])
        d_run, end = first_run(path)
        goal = path[-1] if kind == 'visit' else (tcell[0] + DR[td], tcell[1] + DC[td])
        if not nav.run(d_run, end) and guard.note(goal, bot.t):
            log('  target %s keeps failing - skipping it for now' % (goal,))

    b, g = maze.counts()
    log('exploration done in %.1fs: blue=%d green=%d' % (bot.t - t0, b, g))
    log('maze map:\n' + maze.dump())
    colour = majority(maze)

    nav.exploring = False
    for _ in range(80):
        if nav.cell == cfg.END_CELL:
            break
        path = plan(maze, nav.cell, nav.d, lambda c: c == cfg.END_CELL,
                    allow_unknown=True, allow_end=True)
        if path is None:
            log('no route to the end cell!')
            break
        d_run, end = first_run(path)
        if nav.run(d_run, end) and nav.cell == cfg.END_CELL:
            break
        sense(bot, maze, nav.cell, nav.d)

    bot.listeners.remove(lat)
    log('Section 2 done in %.1fs' % (bot.t - t0))
    bot.finish(colour)
