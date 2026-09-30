"""
motion.py - motion primitives for the differential-drive robot.

Motion is a mixin used by hardware.Bot. It only relies on:
    self.step(), self.wheels(l, r), self.t, self.x / self.y / self.th, self.d
so every primitive keeps the sensors, odometry and listeners running.

  turn_to(theta)          spin in place to an absolute IMU heading
  drive_to(x, y)          turn toward a point, then drive to it
  drive_axis(x, y, h)     drive along a grid axis to a point (maze / transition)
  drive_cells(n, h)       drive n maze cells along heading h
  back_off(s)             reverse for s seconds
  stop() / halt()         zero velocity (halt also lets the robot settle)
"""
import math

import config as cfg
from odometry import wrap


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Motion:
    # ---------------------------------------------------------------- basics
    def stop(self):
        self.wheels(0.0, 0.0)

    def halt(self, settle_steps=3):
        self.stop()
        for _ in range(settle_steps):
            self.step()

    def wait(self, seconds):
        end = self.t + seconds
        while self.t < end:
            self.step()

    def front_min(self):
        return min(self.d[n] for n in cfg.FRONT_SENSORS)

    # ---------------------------------------------------------------- turning
    def turn_to(self, target, timeout=4.0):
        """Spin in place to an absolute heading (IMU yaw). P control with a floor."""
        start = self.t
        for _ in range(3):                      # re-check after settling
            while self.t - start < timeout:
                err = wrap(target - self.th)
                if abs(err) < cfg.TURN_TOL:
                    break
                w = clamp(cfg.TURN_K * abs(err), cfg.TURN_MIN, cfg.TURN_MAX)
                w = math.copysign(w, err)
                self.wheels(-w, w)
                self.step()
            self.halt(2)
            if abs(wrap(target - self.th)) < cfg.TURN_TOL * 1.5:
                return True
        return False

    # ---------------------------------------------------------------- driving
    def drive_to(self, tx, ty, speed=None, tol=0.01, timeout=None):
        """
        Turn toward (tx, ty), then drive there with heading correction, slowing
        down near the target. Stops early if something appears right in front.
        Returns 'ok', 'blocked' or 'timeout'.
        """
        speed = cfg.S1_CRUISE if speed is None else speed
        dist0 = math.hypot(tx - self.x, ty - self.y)
        if dist0 < tol:
            return 'ok'
        if timeout is None:
            timeout = 3.0 + dist0 / 0.05
        self.turn_to(math.atan2(ty - self.y, tx - self.x))
        start = self.t
        result = 'ok'
        while True:
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, dy)
            if dist < tol:
                break
            if self.front_min() < cfg.EMERGENCY_DIST:
                result = 'blocked'
                break
            if self.t - start > timeout:
                result = 'timeout'
                break
            err = wrap(math.atan2(dy, dx) - self.th)
            if abs(err) > math.pi / 2:          # overshot: target is behind us
                break
            ramp_up = min(1.0, (self.t - start) / cfg.ACCEL_TIME + 0.2)
            ramp_down = clamp(dist / 0.08, 0.0, 1.0)
            v = max(cfg.MIN_DRIVE, speed * min(ramp_up, ramp_down)) * (1.0 - 0.6 * abs(err))
            w = cfg.HEAD_K * err
            self.wheels(v - w, v + w)
            self.step()
        self.halt(2)
        return result

    def drive_axis(self, tx, ty, heading, speed, timeout=None, front_stop=True):
        """
        Drive forward along a grid axis (heading) to the point (tx, ty), keeping on
        the line through the target. When a wall is ahead in the final cell the
        front sensor is used to stop exactly at the cell centre.

        Returns 'ok', 'blocked' (unexpected wall ahead) or 'timeout'.
        """
        ux, uy = math.cos(heading), math.sin(heading)
        dist0 = abs((tx - self.x) * ux + (ty - self.y) * uy)
        if timeout is None:
            timeout = 2.0 + dist0 / 0.06
        start = self.t
        result = 'ok'
        while True:
            ex, ey = tx - self.x, ty - self.y
            along = ex * ux + ey * uy
            cross = -ex * uy + ey * ux
            f = self.d['ds_f']
            if front_stop:
                # front wall inside the target cell: measure 'along' directly
                meas = f - (cfg.FACE - cfg.OFF_F)
                if f < 0.45 and abs(meas - along) < 0.05 and along < 0.3:
                    self.shift((along - meas) * ux, (along - meas) * uy)
                    along = meas
                if along > 0.12 and f < 0.045:
                    result = 'blocked'
                    break
            if along < 0.003:
                break
            if self.t - start > timeout:
                result = 'timeout'
                break
            desired = heading + clamp(cfg.CROSS_K * cross, -cfg.CROSS_MAX, cfg.CROSS_MAX)
            err = wrap(desired - self.th)
            ramp_up = min(1.0, (self.t - start) / cfg.ACCEL_TIME + 0.2)
            ramp_down = clamp(along / 0.08, 0.0, 1.0)
            v = max(cfg.MIN_DRIVE, speed * min(ramp_up, ramp_down))
            w = cfg.HEAD_K * err
            self.wheels(v - w, v + w)
            self.step()
        self.halt(2)
        return result

    def drive_cells(self, n, heading, speed=None):
        """Maze: drive n cells (n * CELL) along a grid heading from the current pose."""
        speed = cfg.S2_CRUISE if speed is None else speed
        ux, uy = math.cos(heading), math.sin(heading)
        return self.drive_axis(self.x + n * cfg.CELL * ux, self.y + n * cfg.CELL * uy,
                               heading, speed)

    def back_off(self, seconds=0.35, speed=4.0):
        """Reverse straight back (unless something is right behind)."""
        self.wheels(-speed, -speed)
        end = self.t + seconds
        while self.t < end:
            if self.d['ds_b'] < 0.03:
                break
            self.step()
        self.halt(2)
