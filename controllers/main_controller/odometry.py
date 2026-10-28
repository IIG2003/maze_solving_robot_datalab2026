import math

import config as cfg


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Odometry:
    def __init__(self):
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.travelled = 0.0
        self._enc = None

    def update(self, el, er, yaw):
        if math.isnan(yaw) or math.isnan(el) or math.isnan(er):
            return False
        if self._enc is None:
            self._enc = (el, er)
            self.th = yaw
            return True
        dl = (el - self._enc[0]) * cfg.WHEEL_RADIUS
        dr = (er - self._enc[1]) * cfg.WHEEL_RADIUS
        self._enc = (el, er)
        dist = 0.5 * (dl + dr)
        mid = self.th + 0.5 * wrap(yaw - self.th)
        a = cfg.AXLE_X
        self.x += dist * math.cos(mid) + a * (math.cos(self.th) - math.cos(yaw))
        self.y += dist * math.sin(mid) + a * (math.sin(self.th) - math.sin(yaw))
        self.th = yaw
        self.travelled += abs(dist)
        return True

    def set_pose(self, x, y):
        self.x, self.y = x, y

    def shift(self, dx, dy):
        self.x += dx
        self.y += dy
