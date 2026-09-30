"""
odometry.py - pose estimate (x, y, theta) without GPS.

  * heading  : InertialUnit yaw (absolute, no drift)
  * distance : mean of the two wheel-encoder deltas
  * position : integrated along the mid-step heading (robot origin = chassis
               centre; the wheel axle is AXLE_X ahead of it)

Absolute fixes (walls seen at known distances, cell centres) are applied by
the section modules through set_pose() / shift().
"""
import math

import config as cfg


def wrap(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class Odometry:
    def __init__(self):
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.travelled = 0.0            # total distance driven (m), for logging
        self._enc = None

    def update(self, el, er, yaw):
        """Integrate one control step. Returns False while readings are invalid."""
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
        # the wheel axle sits AXLE_X ahead of the robot origin: move the axle
        # point, then express the pose at the origin again
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
