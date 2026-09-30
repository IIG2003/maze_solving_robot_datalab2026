"""
hardware.py - device setup and the per-step update loop.

All code steps the simulation through Bot.step(), which also:
  * refreshes every sensor reading (self.d[name])
  * updates the pose estimate (odometry.Odometry -> self.x, self.y, self.th)
  * calls registered per-step listeners (mapping, wall corrections, ...)

Motion primitives (turn_to, drive_to, drive_axis, ...) come from motion.Motion.
"""
import math
import os
from controller import Robot

import config as cfg
from motion import Motion, clamp
from odometry import Odometry, wrap




_LOG_FILE = os.environ.get('PERABOT_LOG')      # optional file log for test runs


def log(*args):
    if cfg.DEBUG:
        print('[bot]', *args, flush=True)
    if _LOG_FILE:
        with open(_LOG_FILE, 'a') as f:
            print('[bot]', *args, file=f)


class Bot(Motion):
    def __init__(self):
        self.robot = Robot()
        self.dt_ms = int(self.robot.getBasicTimeStep())
        self.dt = self.dt_ms / 1000.0
        dev = self.robot.getDevice

        self.lm = dev('left_wheel_motor')
        self.rm = dev('right_wheel_motor')
        for m in (self.lm, self.rm):
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
            # limited wheel acceleration: hard braking tips the robot onto its nose
            m.setAcceleration(cfg.WHEEL_ACCEL)
        self.le = dev('left_wheel_sensor')
        self.re = dev('right_wheel_sensor')
        self.le.enable(self.dt_ms)
        self.re.enable(self.dt_ms)

        self.ds = {}
        for name in cfg.DISTANCE_SENSORS:
            s = dev(name)
            s.enable(self.dt_ms)
            self.ds[name] = s
        self.imu = dev('imu')
        self.imu.enable(self.dt_ms)
        self.cam = dev('cam')
        self.cam.enable(self.dt_ms)
        self.cam_w = self.cam.getWidth()
        self.cam_h = self.cam.getHeight()

        # required LEDs: both OFF until the finish
        self.led_blue = dev('led_blue')
        self.led_green = dev('led_green')
        self.led_blue.set(0)
        self.led_green.set(0)

        # state
        self.t = 0.0
        self.d = {name: cfg.DS_MAX for name in cfg.DISTANCE_SENSORS}
        self.odo = Odometry()
        self._cmd = (0.0, 0.0)
        self.listeners = []

        # prime all sensors (first readings are NaN / zero)
        for _ in range(3):
            self.step()

    # ------------------------------------------------------------ pose access
    @property
    def x(self):
        return self.odo.x

    @x.setter
    def x(self, v):
        self.odo.x = v

    @property
    def y(self):
        return self.odo.y

    @y.setter
    def y(self, v):
        self.odo.y = v

    @property
    def th(self):
        return self.odo.th

    @property
    def travelled(self):
        return self.odo.travelled

    def set_pose(self, x, y):
        self.odo.set_pose(x, y)

    def shift(self, dx, dy):
        self.odo.shift(dx, dy)

    # ------------------------------------------------------------------ core
    def step(self):
        if self.robot.step(self.dt_ms) == -1:
            raise SystemExit(0)
        self.t += self.dt
        for name, s in self.ds.items():
            v = s.getValue()
            self.d[name] = cfg.DS_MAX if math.isnan(v) else v
        ok = self.odo.update(self.le.getValue(), self.re.getValue(),
                             self.imu.getRollPitchYaw()[2])
        if not ok:
            return
        for f in list(self.listeners):
            f()

    def wheels(self, left, right):
        left = clamp(left, -cfg.MAX_WHEEL_SPEED, cfg.MAX_WHEEL_SPEED)
        right = clamp(right, -cfg.MAX_WHEEL_SPEED, cfg.MAX_WHEEL_SPEED)
        self._cmd = (left, right)
        self.lm.setVelocity(left)
        self.rm.setVelocity(right)

    def sensor_pose(self, name):
        """World position and direction of a distance sensor."""
        ox, oy, oa = cfg.DISTANCE_SENSORS[name]
        c, s = math.cos(self.th), math.sin(self.th)
        return (self.x + c * ox - s * oy, self.y + s * ox + c * oy, self.th + oa)

    # ------------------------------------------------------------- finishing
    def finish(self, colour):
        """Stop and show the majority colour, then idle forever."""
        self.wheels(0.0, 0.0)
        self.led_blue.set(1 if colour == 'blue' else 0)
        self.led_green.set(1 if colour == 'green' else 0)
        log('FINISH  majority=%s  t=%.1fs  distance=%.2fm' % (colour, self.t, self.travelled))
        while self.robot.step(self.dt_ms) != -1:
            pass
        raise SystemExit(0)
