import math

import config as cfg
from odometry import wrap


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Motion:
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

    def turn_to(self, target, timeout=4.0):
        start = self.t
        for _ in range(3):
            while self.t - start < timeout:
                err = wrap(target - self.th)
                if abs(err) < cfg.TURN_TOL:
                    break
                brake = math.sqrt(2 * cfg.TURN_DECEL * abs(err)) * cfg.AXLE_TRACK / (2 * cfg.WHEEL_RADIUS)
                w = clamp(min(cfg.TURN_K_END * abs(err), brake), cfg.TURN_MIN, cfg.TURN_MAX)
                w = math.copysign(w, err)
                self.wheels(-w, w)
                self.step()
            self.halt(2)
            if abs(wrap(target - self.th)) < cfg.TURN_TOL * 1.5:
                return True
        return False

    def drive_to(self, tx, ty, speed=None, tol=0.01, timeout=None):
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
            if abs(err) > math.pi / 2:
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
            brake = math.sqrt(2 * cfg.DRIVE_DECEL * max(along, 0.0)) / cfg.WHEEL_RADIUS
            v = max(cfg.MIN_DRIVE, min(speed * ramp_up, brake))
            w = cfg.HEAD_K * err
            self.wheels(v - w, v + w)
            self.step()
        self.halt(2)
        return result

    def drive_cells(self, n, heading, speed=None):
        speed = cfg.S2_CRUISE if speed is None else speed
        ux, uy = math.cos(heading), math.sin(heading)
        return self.drive_axis(self.x + n * cfg.CELL * ux, self.y + n * cfg.CELL * uy,
                               heading, speed)

    def back_off(self, seconds=0.35, speed=4.0):
        self.wheels(-speed, -speed)
        end = self.t + seconds
        while self.t < end:
            if self.d['ds_b'] < 0.03:
                break
            self.step()
        self.halt(2)
