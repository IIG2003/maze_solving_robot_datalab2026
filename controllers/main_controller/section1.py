"""
section1.py - obstacle field navigation.

The robot builds an occupancy grid of Section 1 from its distance sensors while
driving, and continuously re-plans an A* path to the gap in the top wall.
Unknown cells are treated as free, so the robot heads straight for the goal and
bends its route around cubes as soon as it sees them.
"""
import heapq
import math

import config as cfg
from hardware import clamp, log, wrap
from watchdog import Watchdog

SQRT2 = math.sqrt(2.0)


class Grid:
    def __init__(self):
        self.res = cfg.S1_RES
        self.nx = int(round(cfg.ARENA_W / self.res))
        self.ny = int(round(cfg.S1_H / self.res))
        n = self.nx * self.ny
        self.occ = [0] * n              # hit score, occupied when >= 1
        self.fixed = bytearray(n)       # arena boundary (never cleared)
        self.dist = [1.0] * n           # distance (m) from cell centre to nearest obstacle
        self.need_full = False
        self.dirty = False
        self.k = int(math.ceil(cfg.R_SOFT / self.res))

        for i in range(self.nx):
            for j in range(self.ny):
                edge = i == 0 or j == 0 or i == self.nx - 1 or j == self.ny - 1
                if j == self.ny - 1:
                    cx = (i + 0.5) * self.res
                    if cfg.S1_OPENING_X[0] + 0.01 < cx < cfg.S1_OPENING_X[1] - 0.01:
                        edge = False
                if edge:
                    self.fixed[self.idx(i, j)] = 1
                    self.occ[self.idx(i, j)] = 99
        self._recompute()

    # ------------------------------------------------------------ helpers
    def idx(self, i, j):
        return j * self.nx + i

    def cell_of(self, x, y):
        return int(x / self.res), int(y / self.res)

    def inside(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny

    def centre(self, i, j):
        return (i + 0.5) * self.res, (j + 0.5) * self.res

    def occupied(self, k):
        return self.occ[k] >= 1

    # ------------------------------------------------------ distance field
    def _stamp(self, i0, j0):
        k = self.k
        res = self.res
        for dj in range(-k, k + 1):
            j = j0 + dj
            if j < 0 or j >= self.ny:
                continue
            row = j * self.nx
            for di in range(-k, k + 1):
                i = i0 + di
                if i < 0 or i >= self.nx:
                    continue
                d = res * math.hypot(di, dj)
                if d < self.dist[row + i]:
                    self.dist[row + i] = d

    def _recompute(self):
        self.dist = [1.0] * (self.nx * self.ny)
        for j in range(self.ny):
            for i in range(self.nx):
                if self.occupied(self.idx(i, j)):
                    self._stamp(i, j)
        self.need_full = False

    # ------------------------------------------------------------ mapping
    def update(self, bot):
        res = self.res
        step = res * 0.5
        for name in cfg.DISTANCE_SENSORS:
            r = bot.d[name]
            sx, sy, a = bot.sensor_pose(name)
            ca, sa = math.cos(a), math.sin(a)
            hit = r < cfg.DS_HIT and r < cfg.MAP_RANGE
            free_len = (r - 0.02) if r < cfg.DS_HIT else cfg.FREE_RANGE
            free_len = min(free_len, cfg.FREE_RANGE)
            # clear free space along the ray
            last = -1
            n = int(free_len / step)
            for s in range(1, n + 1):
                px, py = sx + ca * s * step, sy + sa * s * step
                i, j = int(px / res), int(py / res)
                if not (0 <= i < self.nx and 0 <= j < self.ny):
                    break
                kk = j * self.nx + i
                if kk == last:
                    continue
                last = kk
                if not self.fixed[kk] and self.occ[kk] > 0:
                    self.occ[kk] -= 1
                    if self.occ[kk] == 0:
                        self.need_full = True
                        self.dirty = True
            # mark the hit (pushed slightly into the obstacle)
            if hit:
                px, py = sx + ca * (r + 0.008), sy + sa * (r + 0.008)
                i, j = int(px / res), int(py / res)
                if self.inside(i, j):
                    kk = self.idx(i, j)
                    if not self.fixed[kk]:
                        was = self.occupied(kk)
                        self.occ[kk] = min(self.occ[kk] + 2, 6)
                        if not was:
                            self._stamp(i, j)
                            self.dirty = True

    def clear_around(self, x, y, radius):
        """Forget mapped obstacles near (x, y) (used when drift corrupted the map)."""
        ci, cj = self.cell_of(x, y)
        k = int(math.ceil(radius / self.res))
        for j in range(cj - k, cj + k + 1):
            for i in range(ci - k, ci + k + 1):
                if self.inside(i, j) and not self.fixed[self.idx(i, j)]:
                    self.occ[self.idx(i, j)] = 0
        self.need_full = True
        self.dirty = True

    # ------------------------------------------------------------ planning
    def cost(self, k):
        d = self.dist[k]
        if d < cfg.R_FORBID:
            return None
        if d < cfg.R_HARD:
            return 1.0 + cfg.HARD_COST
        if d < cfg.R_SOFT:
            return 1.0 + cfg.SOFT_COST * (cfg.R_SOFT - d) / (cfg.R_SOFT - cfg.R_HARD)
        return 1.0

    def plan(self, start, goal):
        if self.need_full:
            self._recompute()
        self.dirty = False
        si, sj = self.cell_of(*start)
        gi, gj = self.cell_of(*goal)
        si = clamp(si, 0, self.nx - 1)
        sj = clamp(sj, 0, self.ny - 1)
        s = self.idx(si, sj)
        g = self.idx(gi, gj)
        nx = self.nx
        res = self.res
        # the start cell and its close surroundings are always allowed (escape)
        g_cost = {s: 0.0}
        parent = {s: None}
        heap = [(0.0, s)]
        closed = set()
        neigh = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]
        while heap:
            _, cur = heapq.heappop(heap)
            if cur in closed:
                continue
            if cur == g:
                break
            closed.add(cur)
            ci, cj = cur % nx, cur // nx
            for di, dj, step in neigh:
                i, j = ci + di, cj + dj
                if not (0 <= i < nx and 0 <= j < self.ny):
                    continue
                k = j * nx + i
                if k in closed or self.fixed[k]:
                    continue
                c = self.cost(k)
                if c is None:
                    if abs(i - si) + abs(j - sj) > 3:
                        continue
                    c = 1.0 + cfg.HARD_COST * 2
                ng = g_cost[cur] + step * c
                if ng < g_cost.get(k, 1e18):
                    g_cost[k] = ng
                    parent[k] = cur
                    h = math.hypot(i - gi, j - gj)
                    heapq.heappush(heap, (ng + h, k))
        if g not in parent:
            return None
        path = []
        k = g
        while k is not None:
            path.append(self.centre(k % nx, k // nx))
            k = parent[k]
        path.reverse()
        path[-1] = goal
        return path


def _lookahead(path, x, y, start_idx):
    """Index of the closest path point, and the pursuit target beyond it."""
    best, bi = 1e9, start_idx
    for i in range(start_idx, min(len(path), start_idx + 40)):
        d = (path[i][0] - x) ** 2 + (path[i][1] - y) ** 2
        if d < best:
            best, bi = d, i
    ti = bi
    while ti < len(path) - 1 and math.hypot(path[ti][0] - x, path[ti][1] - y) < cfg.LOOKAHEAD:
        ti += 1
    return bi, path[ti]


def reactive(bot, goal, seconds):
    """
    Bug-style fallback when A* has no path: head for the goal, and when the
    front is blocked follow the obstacle on the side that is more open.
    """
    gx, gy = goal
    end = bot.t + seconds
    side = 1.0 if bot.d['ds_l'] > bot.d['ds_r'] else -1.0     # +1: turn left
    while bot.t < end:
        if math.hypot(gx - bot.x, gy - bot.y) < cfg.S1_GOAL_TOL:
            break
        f = bot.front_min()
        if f < cfg.EMERGENCY_DIST:
            bot.back_off(0.2)
            continue
        if f < 0.12 or min(bot.d['ds_fl45'], bot.d['ds_fr45']) < 0.08:
            w = 3.0 * side                                  # turn away from the obstacle
            bot.wheels(-w, w)
        else:
            err = wrap(math.atan2(gy - bot.y, gx - bot.x) - bot.th)
            # keep a little clearance from the followed side
            near = bot.d['ds_fr70'] if side > 0 else bot.d['ds_fl70']
            if near < 0.07:
                err = 0.3 * side
            v = cfg.S1_CRUISE * 0.6
            w = clamp(cfg.HEAD_K * err, -4.0, 4.0)
            bot.wheels(v - w, v + w)
        bot.step()
    bot.halt(1)


def run(bot, goal=cfg.S1_GOAL):
    log('Section 1 start at (%.3f, %.3f)' % (bot.x, bot.y))
    grid = Grid()
    listener = lambda: grid.update(bot)
    bot.listeners.append(listener)
    t0 = bot.t

    path, pidx, last_plan = None, 0, -1e9
    dog = Watchdog(bot)
    fails = 0                                   # consecutive failed plans
    try:
        while True:
            gx, gy = goal
            dgoal = math.hypot(gx - bot.x, gy - bot.y)
            if dgoal < cfg.S1_GOAL_TOL:
                break

            if path is None or (grid.dirty and bot.t - last_plan > cfg.REPLAN_PERIOD) \
                    or bot.t - last_plan > 1.5:
                path = grid.plan((bot.x, bot.y), goal)
                pidx, last_plan = 0, bot.t
                if path is None:
                    fails += 1
                    if fails == 1:
                        log('S1: no path - clearing the map around the robot')
                        grid.clear_around(bot.x, bot.y, 0.20)
                    else:
                        log('S1: still no path - reactive mode')
                        reactive(bot, goal, cfg.REACTIVE_TIME)
                        grid.clear_around(bot.x, bot.y, 0.20)
                        dog.reset()
                    continue
                fails = 0

            # emergency: something right in front
            if bot.front_min() < cfg.EMERGENCY_DIST:
                bot.back_off(0.25)
                path = None
                continue

            pidx, (tx, ty) = _lookahead(path, bot.x, bot.y, pidx)
            err = wrap(math.atan2(ty - bot.y, tx - bot.x) - bot.th)
            if abs(err) > 0.9:
                w = clamp(cfg.TURN_K * abs(err), cfg.TURN_MIN, cfg.TURN_MAX)
                bot.wheels(-math.copysign(w, err), math.copysign(w, err))
            else:
                slow = clamp(dgoal / 0.10, 0.3, 1.0)
                v = cfg.S1_CRUISE * slow * (1.0 - 0.7 * abs(err) / 0.9)
                w = cfg.HEAD_K * err
                bot.wheels(v - w, v + w)
            bot.step()

            if dog.stuck():
                log('S1: stuck - backing off and re-planning')
                bot.back_off(0.4)
                path = None
    finally:
        bot.listeners.remove(listener)
    bot.halt()
    log('Section 1 done in %.1fs, travelled %.2fm' % (bot.t - t0, bot.travelled))
