"""
watchdog.py - anti-termination helpers.

The rulebook ends an attempt after 15 s without meaningful progress. We react
much earlier:

  Watchdog  : pose history over a sliding window; "stuck" when the robot has
              moved less than STUCK_MOVE in the last STUCK_WINDOW seconds.
              Turning on the spot does NOT count as progress.
  LoopGuard : counts how often the same target is chosen in a row without being
              reached and blacklists it for a while.
"""
import math

import config as cfg


class Watchdog:
    def __init__(self, bot, window=cfg.STUCK_WINDOW, move=cfg.STUCK_MOVE):
        self.bot = bot
        self.window = window
        self.move = move
        self.hist = []
        self.t0 = bot.t

    def reset(self):
        self.hist = []
        self.t0 = self.bot.t

    def stuck(self):
        """Call once per control step. True when no meaningful progress was made."""
        b = self.bot
        self.hist.append((b.t, b.x, b.y))
        while self.hist and b.t - self.hist[0][0] > self.window:
            self.hist.pop(0)
        if b.t - self.t0 < self.window or not self.hist:
            return False
        _, hx, hy = self.hist[0]
        if math.hypot(b.x - hx, b.y - hy) < self.move:
            self.reset()
            return True
        return False


class LoopGuard:
    def __init__(self, limit=cfg.LOOP_LIMIT, ban_time=cfg.LOOP_BAN):
        self.limit = limit
        self.ban_time = ban_time
        self.last = None
        self.count = 0
        self.bans = {}                  # target -> time the ban ends

    def note(self, target, t):
        """Record that target was chosen again. Returns True if it just got banned."""
        if target == self.last:
            self.count += 1
        else:
            self.last, self.count = target, 1
        if self.count > self.limit:
            self.bans[target] = t + self.ban_time
            self.last, self.count = None, 0
            return True
        return False

    def banned(self, target, t):
        end = self.bans.get(target)
        if end is None:
            return False
        if t >= end:
            del self.bans[target]
            return False
        return True
