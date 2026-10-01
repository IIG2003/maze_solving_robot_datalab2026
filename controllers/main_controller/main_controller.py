"""
main_controller.py  -  PeraBots 2026 Phase 1 scaffold

Everything lives in this file except the optional analytics (run_monitor.py, safe to delete for the submission).
Flow: detect the start mode -> Section 1 (map + A* + pure pursuit) -> transition corridor -> Section 2 (frontier exploration, tile counting, shortest route to Red C, LED).

Frame conventions (measured in Webots R2025a, ENU):
  * robot: +x forward, +y left, +z up
  * yaw   : counter-clockwise positive, radians, wraps at +-pi
  * wheels: positive motor velocity = forward, max 10 rad/s (~0.2 m/s)
  * distance sensors return METRES, 0.5 means "nothing in range"

File placement (do not rename/move):
  controllers/main_controller/main_controller.py
"""

import heapq
import math
from collections import deque
from controller import Robot


# ----------------------------------------------------------------------------
# Static configuration (single source of truth - never instantiated)
# ----------------------------------------------------------------------------
class RobotConfiguration:
    """All fixed numbers in one place. Use them as RobotConfiguration.NAME.
    Robot values come from TwoWheelRobot.proto (do not change the robot itself)."""

    # ---- drivetrain ----
    WHEEL_RADIUS_METERS                    = 0.02
    WHEEL_TRACK_WIDTH_METERS               = 0.064      # distance between the two wheels
    AXLE_FORWARD_OFFSET_METERS             = 0.010      # wheels sit 10 mm ahead of the robot origin (balance); odometry follows the axle
    MAXIMUM_WHEEL_SPEED_RADIANS_PER_SECOND = 10.0

    # ---- distance sensors ----
    DISTANCE_SENSOR_MAXIMUM_RANGE_METERS         = 0.5  # reading when nothing is hit
    DISTANCE_SENSOR_NO_OBSTACLE_THRESHOLD_METERS = 0.49

    # name -> (x_meters, y_meters, angle_radians) in the robot frame (+x forward, +y left, angle counter-clockwise)
    DISTANCE_SENSOR_MOUNTING_POSES = {
        "ds_f0":         (  0.04,  0.000,     0.0),
        "ds_fl15":       (  0.04,  0.010,  0.2618),
        "ds_fl30":       (  0.04,  0.020,  0.5236),
        "ds_fr15":       (  0.04, -0.010, -0.2618),
        "ds_fr30":       (  0.04, -0.020, -0.5236),
        "ds_cl":         (  0.04,  0.025,  0.7854),
        "ds_cr":         (  0.04, -0.025, -0.7854),
        "ds_l":          (   0.0,  0.025,  1.5708),
        "ds_r":          (   0.0, -0.025, -1.5708),
        "ds_rear":       ( -0.04,  0.000,  3.1416),
        "ds_rear_left":  ( -0.04,  0.025,  2.3562),
        "ds_rear_right": ( -0.04, -0.025, -2.3562),
        "ds_left_front":  (  0.02,  0.025,  1.1781),
        "ds_right_front": (  0.02, -0.025, -1.1781),
        "ds_left_rear":   ( -0.02,  0.025,  1.9635),
        "ds_right_rear":  ( -0.02, -0.025, -1.9635),
    }

    # ---- maze facts from the rulebook / measured ----
    MAZE_CELL_PITCH_METERS                                  = 0.25
    SIDE_SENSOR_READING_WHEN_CENTERED_AGAINST_WALL_METERS   = 0.094
    FRONT_SENSOR_READING_WHEN_CELL_CENTER_FACES_WALL_METERS = 0.079

    # ---- mapping and planning ----
    GRID_CELL_SIZE_METERS                = 0.025
    NORMAL_INFLATION_RADIUS_METERS       = 0.078    # robot reaches 0.056 m from the axle (rear corners); the rest covers the 2.5 cm cell quantisation
    TIGHT_INFLATION_RADIUS_METERS        = 0.062    # used when the normal radius blocks every route
    CLEARANCE_PENALTY_RADIUS_METERS      = 0.115    # cells this close to an obstacle cost more; a 25 cm gap centre (12.5 cm from both faces) stays unpenalised
    SMOOTH_RESPECTS_CLEARANCE            = 1      # 1 = a shortcut between waypoints is only taken if it passes no closer to obstacles than the A* route did (A* is what centres the robot in gaps)
    CLEARANCE_PENALTY_WEIGHT             = 4.0    # narrow diagonal gaps cost 5x, so wide gaps win unless the detour is very long

    # ---- section 1 ----
    # Soft prior from the locked Arena (structure, not the obstacle layout): Red B relative to Red A, in start-relative world axes.
    SECTION_1_GOAL_OFFSET_FROM_START_METERS           = (-2.095, 1.150)    # centre of Red B
    SECTION_1_GATE_OFFSET_FROM_START_METERS           = (-2.095, 0.925)    # 10 cm below the opening: lining up here makes the robot enter Red B straight, not at an angle
    SECTION_1_GATE_REACHED_DISTANCE_METERS            = 0.10
    SECTION_1_MINIMUM_DISTANCE_BEFORE_RED_IS_GOAL_METERS = 0.6
    SECTION_1_STUCK_CHECK_INTERVAL_SECONDS            = 3.0
    SECTION_1_STUCK_MINIMUM_PROGRESS_METERS           = 0.04

    # ---- maze ----
    WALL_DETECTION_THRESHOLD_METERS                = 0.16
    CAMERA_FORWARD_OFFSET_METERS                   = 0.045   # floor_cam lead ahead of the robot centre
    TRANSITION_TILE_END_TO_FIRST_CELL_CENTER_METERS = 0.12   # from the locked Arena geometry
    MAZE_CRUISE_SPEED_METERS_PER_SECOND            = 0.198   # just under the 0.2 m/s motor limit
    TURN_PENALTY_IN_CELL_STEPS                     = 0.5
    START_BIAS_WEIGHT                              = 2.0     # unexplored cells close to the entrance (by route) are explored before cells far from it. 0 = plain nearest-first, 4+ = too much backtracking (measured +54 s)
    MAZE_BRAKING_METERS_PER_SECOND_SQUARED         = 1.2     # braking is safe: the wheel shift needs > 6 m/s^2 to tip the robot
    SECTION_1_REPLAN_INTERVAL_SECONDS              = 1.5     # replan sooner only if the route becomes blocked
    TIE_DEFAULT_COLOR                              = "GREEN" # the rulebook defines no tie rule

    # --- look-ahead camera (reads the colour of a dead-end cell from the cell before it, so the robot does not have to enter it) ---
    LOOK_AHEAD_ENABLED                             = 0       # 1 = read dead-end tiles from the cell before instead of entering them. OFF: every dead end is physically visited (the rules rank "sufficient exploration" first). See PLAN.md section 0.49
    LOOK_CAMERA_WIDTH_PIXELS                       = 32
    LOOK_CAMERA_HEIGHT_PIXELS                      = 24
    LOOK_CAMERA_SAMPLE_ROWS                        = range(0, 6)     # top rows = floor 17-30 cm ahead of the axle = the middle of the next cell
    LOOK_CAMERA_SAMPLE_COLUMNS                     = range(12, 20)   # centre columns only: the side walls never enter the sample
    LOOK_AHEAD_REQUIRED_AGREEMENT                  = 0.7     # share of sampled pixels that must show the same colour
    LOOK_AHEAD_BEAM_TOLERANCE_METERS               = 0.03    # how far a beam may differ from "next cell is a closed dead end" and still count
    LOOK_AHEAD_MAXIMUM_OFFSET_METERS               = 0.02    # robot must be this close to the cell centre line (along and across) for the beams to mean anything
    LOOK_AHEAD_MAXIMUM_YAW_ERROR_RADIANS           = 0.05
    LOOK_AHEAD_MAXIMUM_TILT_RADIANS                = 0.04    # the camera view moves a lot when the chassis pitches (braking, accelerating): then it can see the wall instead of the floor
    MAZE_WALL_HALF_THICKNESS_METERS                = 0.006

    # ---- motion (tuned by tests/run_ablation.py) ----
    MAXIMUM_SPEED_METERS_PER_SECOND              = 0.198   # just under the 0.2 m/s wheel limit (10 rad/s x 0.02 m)
    ACCELERATION_LIMIT_METERS_PER_SECOND_SQUARED = 1.2
    DECELERATION_LIMIT_METERS_PER_SECOND_SQUARED = 1.2
    SLOW_DOWN_START_DISTANCE_METERS              = 0.30   # front clearance below this starts slowing the robot (Section 1)
    STOP_DISTANCE_METERS                         = 0.05
    PIVOT_MAXIMUM_TURN_RATE_RADIANS_PER_SECOND   = 8.0
    PIVOT_PROPORTIONAL_GAIN                      = 32.0
    PIVOT_MINIMUM_TURN_RATE_RADIANS_PER_SECOND   = 1.0
    PIVOT_TOLERANCE_RADIANS                      = 0.02
    SIDE_CLEARANCE_MINIMUM_SPEED_FACTOR          = 1.0
    SECTION_1_LOOKAHEAD_METERS                   = 0.15

    # ---- arc corners in the maze (through already-visited cells) ----
    ARC_CORNERS_ENABLED                          = 1       # 1 = take 90-degree turns as a quarter circle through the cell corner; 0 = stop and pivot
    ARC_SPEED_METERS_PER_SECOND                  = 0.155    # radius 0.125 m: the outer wheel must stay under 0.2 m/s, so v <= 0.2 / (1 + 0.032 / 0.125) = 0.159
    ARC_HEADING_GAIN                             = 4.0
    ARC_RADIAL_GAIN                              = 12.0

    # ---- safety nets ported from the datalab2026 controller (inert unless something goes wrong) ----
    MAZE_COLUMNS                                 = 9       # fixed by the rulebook
    MAZE_ROWS                                    = 4
    UNEXPECTED_WALL_MARGIN_METERS                = 0.15    # a wall this much nearer than the target cell, where the map says open, aborts the leg
    LEG_TIMEOUT_SECONDS                          = 8.0     # a normal 25 cm leg takes 2-3 s; the rulebook ends an attempt after 15 s without progress
    LOOP_GUARD_LIMIT                             = 3       # the same maze target aborted this many times in a row gets banned
    LOOP_GUARD_BAN_SECONDS                       = 30.0
    SECTION_1_FORGET_RADIUS_METERS               = 0.20    # when no route exists, forget mapped obstacles this close to the robot (drift can leave phantom ones)
    SECTION_1_REACTIVE_SECONDS                   = 2.0     # then drive reactively toward the goal for this long
    EARLY_STOP_WHEN_MAJORITY_DECIDED             = 0       # 1 = stop exploring once the blue/green lead exceeds the cells still unvisited (ask the committee what counts as sufficient exploration)

    # ---- stall fail-safe ----
    STALL_COMMANDED_WHEEL_SPEED_RADIANS_PER_SECOND = 2.0    # only judge a stall when a wheel is commanded at least this fast
    STALL_MEASURED_FRACTION                        = 0.35   # measured wheel speed below this fraction of the command = blocked wheel
    STALL_CONTACT_DISTANCE_METERS                  = 0.03   # a beam this short while driving into it = the body is touching (tires can keep spinning, so wheel speed alone is not enough)
    STALL_DURATION_SECONDS                         = 0.6
    STALL_BACK_OFF_DISTANCE_METERS                 = 0.07
    STALL_SCAN_HALF_ANGLE_RADIANS                  = 0.45   # rotate one way then the other so the side and corner beams sweep the surroundings

    # ---- debugging ----
    PRINT_PROGRESS = False

    def __init__(self):
        raise TypeError("RobotConfiguration is static; use RobotConfiguration.NAME")


class MathHelpers:
    """Static math helpers. Use them as MathHelpers.name(...). Never instantiated."""

    def __init__(self):
        raise TypeError("MathHelpers is static; use MathHelpers.name(...)")

    @staticmethod
    def wrap_angle_to_plus_minus_pi(angle_radians):
        """Wrap an angle to the range [-pi, pi]."""
        return math.atan2(math.sin(angle_radians), math.cos(angle_radians))

    @staticmethod
    def clamp_value(value, lowest_allowed, highest_allowed):
        return max(lowest_allowed, min(highest_allowed, value))

    @staticmethod
    def nearest_axis_index(yaw_radians):
        """0 = +x (east), 1 = +y (north), 2 = -x (west), 3 = -y (south): the axis closest to this yaw."""
        return int(round(yaw_radians / (math.pi / 2.0))) % 4

    @staticmethod
    def axis_yaw(axis_index):
        return MathHelpers.wrap_angle_to_plus_minus_pi(axis_index * math.pi / 2.0)

    @staticmethod
    def axis_step(axis_index):
        return ((1, 0), (0, 1), (-1, 0), (0, -1))[axis_index % 4]


# ----------------------------------------------------------------------------
# Motion profile (how smoothly the robot is allowed to move)
# ----------------------------------------------------------------------------
class MotionProfile:
    """Immutable speed rules. Create it with MotionProfileBuilder."""

    def __init__(self, maximum_speed_meters_per_second, maximum_acceleration_meters_per_second_squared, slow_down_start_distance_meters, stop_distance_meters, maximum_deceleration_meters_per_second_squared):
        self.maximum_speed_meters_per_second                = maximum_speed_meters_per_second
        self.maximum_acceleration_meters_per_second_squared = maximum_acceleration_meters_per_second_squared
        self.maximum_deceleration_meters_per_second_squared = maximum_deceleration_meters_per_second_squared
        self.slow_down_start_distance_meters                = slow_down_start_distance_meters
        self.stop_distance_meters                           = stop_distance_meters

    def speed_for_clearance(self, clearance_meters):
        """Smooth speed from the free distance ahead: full speed beyond the
        slow-down distance, falling linearly to 0 at the stop distance."""
        usable_range = self.slow_down_start_distance_meters - self.stop_distance_meters
        fraction     = (clearance_meters - self.stop_distance_meters) / usable_range
        return self.maximum_speed_meters_per_second * MathHelpers.clamp_value(fraction, 0.0, 1.0)

    def limit_acceleration(self, current_speed_meters_per_second, desired_speed_meters_per_second, timestep_seconds):
        """Slew-rate limiter: move the speed toward the desired value without
        changing faster than the acceleration limit."""
        largest_increase = self.maximum_acceleration_meters_per_second_squared * timestep_seconds
        largest_decrease = self.maximum_deceleration_meters_per_second_squared * timestep_seconds
        change           = MathHelpers.clamp_value(desired_speed_meters_per_second - current_speed_meters_per_second, -largest_decrease, largest_increase)
        return current_speed_meters_per_second + change


class MotionProfileBuilder:
    """Fluent builder for MotionProfile.  Example:

        profile = MotionProfileBuilder().with_maximum_speed(0.18).with_acceleration_limit(0.6).with_clearance_slow_down(0.25, 0.06).build()
    """

    def __init__(self):
        self.maximum_speed_meters_per_second                = 0.15
        self.maximum_acceleration_meters_per_second_squared = 0.5
        self.maximum_deceleration_meters_per_second_squared = None    # None = same as the acceleration limit
        self.slow_down_start_distance_meters                = 0.25
        self.stop_distance_meters                           = 0.06

    def with_maximum_speed(self, meters_per_second):
        self.maximum_speed_meters_per_second = meters_per_second
        return self

    def with_acceleration_limit(self, meters_per_second_squared):
        self.maximum_acceleration_meters_per_second_squared = meters_per_second_squared
        return self

    def with_deceleration_limit(self, meters_per_second_squared):
        self.maximum_deceleration_meters_per_second_squared = meters_per_second_squared
        return self

    def with_clearance_slow_down(self, slow_down_start_distance_meters, stop_distance_meters):
        self.slow_down_start_distance_meters = slow_down_start_distance_meters
        self.stop_distance_meters            = stop_distance_meters
        return self

    def build(self):
        maximum_possible_speed = RobotConfiguration.MAXIMUM_WHEEL_SPEED_RADIANS_PER_SECOND * RobotConfiguration.WHEEL_RADIUS_METERS
        if not 0.0 < self.maximum_speed_meters_per_second <= maximum_possible_speed:
            raise ValueError("maximum speed must be between 0 and %.3f m/s (motor limit)" % maximum_possible_speed)
        if self.maximum_acceleration_meters_per_second_squared <= 0.0:
            raise ValueError("acceleration limit must be positive")
        if self.stop_distance_meters >= self.slow_down_start_distance_meters:
            raise ValueError("stop distance must be smaller than the slow-down start distance")
        deceleration = self.maximum_deceleration_meters_per_second_squared
        if deceleration is None: deceleration = self.maximum_acceleration_meters_per_second_squared
        if deceleration <= 0.0: raise ValueError("deceleration limit must be positive")
        return MotionProfile(self.maximum_speed_meters_per_second, self.maximum_acceleration_meters_per_second_squared, self.slow_down_start_distance_meters, self.stop_distance_meters, deceleration)


# ----------------------------------------------------------------------------
# Robot builder (hardware setup)
# ----------------------------------------------------------------------------
class RobotBuilder:
    """Fluent builder for DifferentialDriveRobot.  Example:

        robot = RobotBuilder().with_distance_sensors(["ds_f0", "ds_l", "ds_r"]).with_timestep_multiplier(2).with_step_observer(monitor).build()

    Defaults: all ten distance sensors, the world's basic timestep.
    """

    def __init__(self):
        self.distance_sensor_names = list(RobotConfiguration.DISTANCE_SENSOR_MOUNTING_POSES)
        self.timestep_multiplier   = 1
        self.step_observers        = []

    def with_distance_sensors(self, distance_sensor_names):
        unknown_names = [name for name in distance_sensor_names if name not in RobotConfiguration.DISTANCE_SENSOR_MOUNTING_POSES]
        if unknown_names:
            raise ValueError("unknown distance sensors: %s" % unknown_names)
        self.distance_sensor_names = list(distance_sensor_names)
        return self

    def with_timestep_multiplier(self, multiplier):
        """Run the control loop every `multiplier` world timesteps."""
        if multiplier < 1:
            raise ValueError("timestep multiplier must be at least 1")
        self.timestep_multiplier = int(multiplier)
        return self

    def with_step_observer(self, step_observer):
        """Attach an observer (for example a RunMonitor). It needs on_step(robot) and on_simulation_end(robot)."""
        self.step_observers.append(step_observer)
        return self

    def build(self):
        return DifferentialDriveRobot(self.distance_sensor_names, self.timestep_multiplier, self.step_observers)


# ----------------------------------------------------------------------------
# Hardware layer
# ----------------------------------------------------------------------------
class DifferentialDriveRobot:
    """Create with RobotBuilder().build()."""

    def __init__(self, distance_sensor_names, timestep_multiplier, step_observers):
        self.step_observers        = list(step_observers)
        self.webots_robot          = Robot()
        self.timestep_milliseconds = int(self.webots_robot.getBasicTimeStep()) * timestep_multiplier   # never hard-code: read the world's step

        # --- motors (velocity mode) ---
        self.left_wheel_motor  = self.get_required_device("left_wheel_motor")
        self.right_wheel_motor = self.get_required_device("right_wheel_motor")
        for wheel_motor in (self.left_wheel_motor, self.right_wheel_motor):
            wheel_motor.setPosition(float("inf"))
            wheel_motor.setVelocity(0.0)

        # --- wheel encoders ---
        self.left_wheel_encoder  = self.get_required_device("left_wheel_sensor")
        self.right_wheel_encoder = self.get_required_device("right_wheel_sensor")
        self.left_wheel_encoder.enable(self.timestep_milliseconds)
        self.right_wheel_encoder.enable(self.timestep_milliseconds)

        # --- distance sensors ---
        self.distance_sensors = {}
        for sensor_name in distance_sensor_names:
            distance_sensor = self.get_required_device(sensor_name)
            distance_sensor.enable(self.timestep_milliseconds)
            self.distance_sensors[sensor_name] = distance_sensor

        # --- inertial unit (heading) and floor colour camera ---
        self.inertial_unit = self.get_required_device("imu")
        self.floor_camera  = self.get_required_device("floor_cam")
        self.inertial_unit.enable(self.timestep_milliseconds)
        self.floor_camera.enable(self.timestep_milliseconds)
        self.look_camera   = None
        self.last_look_ahead_mean_rgb = (0, 0, 0)
        if RobotConfiguration.LOOK_AHEAD_ENABLED:
            self.look_camera = self.webots_robot.getDevice("look_cam")      # optional: without it the robot simply enters every cell as before
            if self.look_camera is not None: self.look_camera.enable(self.timestep_milliseconds)

        # --- required LEDs ---
        self.blue_led  = self.get_required_device("led_blue")
        self.green_led = self.get_required_device("led_green")

        # --- state, refreshed on every call of step_simulation() ---
        self.distance_readings_meters         = {sensor_name: RobotConfiguration.DISTANCE_SENSOR_MAXIMUM_RANGE_METERS for sensor_name in distance_sensor_names}
        self.yaw_radians                      = 0.0      # world yaw from the IMU
        self.odometry_position_x_meters       = 0.0      # relative to start
        self.odometry_position_y_meters       = 0.0      # relative to start
        self.total_distance_travelled_meters  = 0.0
        self.simulation_time_seconds          = 0.0
        self.previous_left_encoder_radians    = None
        self.previous_right_encoder_radians   = None
        self.commanded_left_wheel_radians_per_second  = 0.0
        self.commanded_right_wheel_radians_per_second = 0.0
        self.roll_radians                             = 0.0
        self.pitch_radians                            = 0.0
        self.front_reference_distance                 = 9.0     # front distance when the no-progress timer last restarted
        self.front_reference_time                     = 0.0
        self.measured_forward_speed_meters_per_second = 0.0     # from the wheel encoders, every step (read by the optional run monitor)
        self.phase_label                              = "START" # which part of the course the robot is in (read by the optional run monitor)
        self.stalled_seconds                          = 0.0     # how long a commanded wheel has been (nearly) motionless
        self.stall_recoveries                         = 0

        self.step_simulation()   # first step: sensors become valid

    def get_required_device(self, device_name):
        """getDevice that fails loudly when a device is missing (usually: the world was not reloaded after editing the PROTO)."""
        device = self.webots_robot.getDevice(device_name)
        if device is None:
            raise RuntimeError("Device '%s' not found on the robot. If you just edited TwoWheelRobot.proto, reload the world (File > Reload World)." % device_name)
        return device

    # ---- simulation step -------------------------------------------------
    def step_simulation(self):
        """Advance one timestep and refresh sensors and odometry. Returns False when the simulation is ending."""
        if self.webots_robot.step(self.timestep_milliseconds) == -1:
            for step_observer in self.step_observers:
                step_observer.on_simulation_end(self)
            return False
        self.simulation_time_seconds += self.timestep_milliseconds / 1000.0

        for sensor_name, distance_sensor in self.distance_sensors.items():
            self.distance_readings_meters[sensor_name] = distance_sensor.getValue()
        roll_pitch_yaw      = self.inertial_unit.getRollPitchYaw()
        self.roll_radians   = roll_pitch_yaw[0]
        self.pitch_radians  = roll_pitch_yaw[1]
        self.yaw_radians    = roll_pitch_yaw[2]

        left_encoder_radians  = self.left_wheel_encoder.getValue()
        right_encoder_radians = self.right_wheel_encoder.getValue()
        encoders_are_valid    = not (math.isnan(left_encoder_radians) or math.isnan(right_encoder_radians))
        if self.previous_left_encoder_radians is not None and encoders_are_valid:
            left_wheel_distance_meters   = (left_encoder_radians  - self.previous_left_encoder_radians)  * RobotConfiguration.WHEEL_RADIUS_METERS
            right_wheel_distance_meters  = (right_encoder_radians - self.previous_right_encoder_radians) * RobotConfiguration.WHEEL_RADIUS_METERS
            timestep_seconds             = self.timestep_milliseconds / 1000.0
            limit                        = RobotConfiguration.STALL_COMMANDED_WHEEL_SPEED_RADIANS_PER_SECOND
            fraction                     = RobotConfiguration.STALL_MEASURED_FRACTION
            left_measured                = abs(left_wheel_distance_meters / RobotConfiguration.WHEEL_RADIUS_METERS / timestep_seconds)
            right_measured               = abs(right_wheel_distance_meters / RobotConfiguration.WHEEL_RADIUS_METERS / timestep_seconds)
            left_blocked                 = abs(self.commanded_left_wheel_radians_per_second) >= limit and left_measured < fraction * abs(self.commanded_left_wheel_radians_per_second)
            right_blocked                = abs(self.commanded_right_wheel_radians_per_second) >= limit and right_measured < fraction * abs(self.commanded_right_wheel_radians_per_second)
            readings                     = self.distance_readings_meters
            contact_distance             = RobotConfiguration.STALL_CONTACT_DISTANCE_METERS
            forward_command              = 0.5 * (self.commanded_left_wheel_radians_per_second + self.commanded_right_wheel_radians_per_second) * RobotConfiguration.WHEEL_RADIUS_METERS
            pushing_into_front           = forward_command > 0.03 and min(readings["ds_f0"], readings["ds_fl15"], readings["ds_fr15"], readings["ds_cl"], readings["ds_cr"]) < contact_distance
            pushing_into_rear            = forward_command < -0.03 and min(readings["ds_rear"], readings["ds_rear_left"], readings["ds_rear_right"]) < contact_distance
            front_distance               = min(readings["ds_f0"], readings["ds_fl15"], readings["ds_fr15"])
            driving_toward_something     = forward_command > 0.05 and front_distance < 0.25
            if not driving_toward_something or abs(front_distance - self.front_reference_distance) > 0.012:
                self.front_reference_distance = front_distance
                self.front_reference_time     = self.simulation_time_seconds
            no_progress_toward_front     = driving_toward_something and self.simulation_time_seconds - self.front_reference_time >= 1.2      # driving at something yet the distance is not shrinking: wheels slipping against it
            if left_blocked or right_blocked or pushing_into_front or pushing_into_rear or no_progress_toward_front:
                self.stalled_seconds += timestep_seconds
            else:
                self.stalled_seconds = 0.0
            forward_distance_meters      = 0.5 * (left_wheel_distance_meters + right_wheel_distance_meters)
            self.measured_forward_speed_meters_per_second = forward_distance_meters / timestep_seconds
            self.odometry_position_x_meters      += forward_distance_meters * math.cos(self.yaw_radians)
            self.odometry_position_y_meters      += forward_distance_meters * math.sin(self.yaw_radians)
            self.total_distance_travelled_meters += abs(forward_distance_meters)
        self.previous_left_encoder_radians  = left_encoder_radians
        self.previous_right_encoder_radians = right_encoder_radians
        for step_observer in self.step_observers:
            step_observer.on_step(self)
        return True

    # ---- drive controls --------------------------------------------------
    def set_wheel_speeds(self, left_wheel_radians_per_second, right_wheel_radians_per_second):
        """Raw wheel speeds in rad/s (clipped to the motor limit)."""
        limit = RobotConfiguration.MAXIMUM_WHEEL_SPEED_RADIANS_PER_SECOND
        self.commanded_left_wheel_radians_per_second  = MathHelpers.clamp_value(left_wheel_radians_per_second, -limit, limit)
        self.commanded_right_wheel_radians_per_second = MathHelpers.clamp_value(right_wheel_radians_per_second, -limit, limit)
        self.left_wheel_motor.setVelocity(self.commanded_left_wheel_radians_per_second)
        self.right_wheel_motor.setVelocity(self.commanded_right_wheel_radians_per_second)

    def drive_with_forward_speed_and_turn_rate(self, forward_speed_meters_per_second, turn_rate_radians_per_second):
        """Body command: forward speed in m/s, turn rate in rad/s (counter-clockwise positive). An arc has turn_rate = forward_speed / arc_radius."""
        half_track                    = RobotConfiguration.WHEEL_TRACK_WIDTH_METERS / 2.0
        left_wheel_radians_per_second  = (forward_speed_meters_per_second - turn_rate_radians_per_second * half_track) / RobotConfiguration.WHEEL_RADIUS_METERS
        right_wheel_radians_per_second = (forward_speed_meters_per_second + turn_rate_radians_per_second * half_track) / RobotConfiguration.WHEEL_RADIUS_METERS
        self.set_wheel_speeds(left_wheel_radians_per_second, right_wheel_radians_per_second)

    def stop_motors(self):
        self.set_wheel_speeds(0.0, 0.0)

    def is_level(self):
        """False while the robot is tilted (camera readings are unreliable then)."""
        return abs(self.pitch_radians) < 0.14 and abs(self.roll_radians) < 0.14

    def is_stalled(self):
        return self.stalled_seconds >= RobotConfiguration.STALL_DURATION_SECONDS

    def back_off_and_scan(self):
        """Fail-safe for an unplanned stop: stop, back away from the obstacle (reverse if the rear is clear, otherwise forward; switches once if the wheels cannot move), rotate one way then the other
        so the beams sweep the surroundings (the mapper records it), and return to the original heading. False if the simulation ended."""
        self.stall_recoveries += 1
        self.stop_motors()
        self.stalled_seconds = 0.0
        readings       = self.distance_readings_meters
        direction      = -1.0                      # reverse if the rear beams see room; if the wheels still cannot move, try the other way once
        if min(readings["ds_rear"], readings["ds_rear_left"], readings["ds_rear_right"]) <= 0.06: direction = 1.0
        switched       = False
        start_x        = self.odometry_position_x_meters
        start_y        = self.odometry_position_y_meters
        start_yaw      = self.yaw_radians
        while math.hypot(self.odometry_position_x_meters - start_x, self.odometry_position_y_meters - start_y) < RobotConfiguration.STALL_BACK_OFF_DISTANCE_METERS:
            self.drive_with_forward_speed_and_turn_rate(direction * 0.06, 0.0)
            if not self.step_simulation(): return False
            if self.stalled_seconds >= RobotConfiguration.STALL_DURATION_SECONDS:
                if switched: break                         # blocked both ways: stop trying and let the caller decide
                switched, direction = True, 1.0
                start_x, start_y    = self.odometry_position_x_meters, self.odometry_position_y_meters
                self.stalled_seconds = 0.0
        self.stop_motors()
        half_angle = RobotConfiguration.STALL_SCAN_HALF_ANGLE_RADIANS
        for target in (start_yaw + half_angle, start_yaw - half_angle, start_yaw):
            if not self.turn_in_place_to_absolute_yaw(MathHelpers.wrap_angle_to_plus_minus_pi(target), 0.03, 2.5, 4.0, False): return False
        self.stalled_seconds = 0.0
        return True

    def pivot_creep_speed(self):
        """Tiny forward/backward speed that moves the robot away from a close obstacle while it pivots (the body swings out to 0.056 m behind the axle)."""
        readings    = self.distance_readings_meters
        front_close = min(readings["ds_f0"], readings["ds_cl"], readings["ds_cr"], readings["ds_fl15"], readings["ds_fr15"], readings["ds_left_front"], readings["ds_right_front"])
        rear_close  = min(readings["ds_rear"], readings["ds_rear_left"], readings["ds_rear_right"], readings["ds_left_rear"], readings["ds_right_rear"])
        if rear_close < 0.045 and front_close > rear_close + 0.03: return 0.04
        if front_close < 0.045 and rear_close > front_close + 0.03: return -0.04
        return 0.0

    def turn_in_place_to_absolute_yaw(self, target_yaw_radians, tolerance_radians=RobotConfiguration.PIVOT_TOLERANCE_RADIANS, maximum_turn_rate_radians_per_second=RobotConfiguration.PIVOT_MAXIMUM_TURN_RATE_RADIANS_PER_SECOND, proportional_gain=RobotConfiguration.PIVOT_PROPORTIONAL_GAIN, allow_recovery=True):
        """Blocking in-place turn to an absolute yaw (rad). Uses the IMU."""
        timestep_seconds                     = self.timestep_milliseconds / 1000.0
        safe_gain                            = min(proportional_gain, 0.8 / timestep_seconds)      # gain x timestep must stay below 1 or the turn oscillates
        minimum_turn_rate_radians_per_second = min(RobotConfiguration.PIVOT_MINIMUM_TURN_RATE_RADIANS_PER_SECOND, 1.5 * tolerance_radians / timestep_seconds)    # never steps over the tolerance band, whatever the timestep
        while True:
            yaw_error_radians = MathHelpers.wrap_angle_to_plus_minus_pi(target_yaw_radians - self.yaw_radians)
            if abs(yaw_error_radians) < tolerance_radians:
                break
            turn_rate = MathHelpers.clamp_value(safe_gain * yaw_error_radians, -maximum_turn_rate_radians_per_second, maximum_turn_rate_radians_per_second)
            if abs(turn_rate) < minimum_turn_rate_radians_per_second:
                turn_rate = math.copysign(minimum_turn_rate_radians_per_second, yaw_error_radians)
            self.drive_with_forward_speed_and_turn_rate(self.pivot_creep_speed(), turn_rate)
            if not self.step_simulation():
                return False
            if allow_recovery and self.is_stalled():
                if not self.back_off_and_scan(): return False
        self.stop_motors()
        return True

    def turn_in_place_by_relative_angle(self, angle_radians):
        """Blocking relative turn (rad, counter-clockwise positive)."""
        return self.turn_in_place_to_absolute_yaw(MathHelpers.wrap_angle_to_plus_minus_pi(self.yaw_radians + angle_radians))

    # ---- distance sensor helpers ----------------------------------------
    def is_obstacle_closer_than(self, sensor_name, limit_meters=RobotConfiguration.DISTANCE_SENSOR_NO_OBSTACLE_THRESHOLD_METERS):
        """True if that sensor sees something closer than the limit."""
        return self.distance_readings_meters[sensor_name] < limit_meters

    def front_minimum_distance_meters(self):
        front_sensor_names = ("ds_f0", "ds_fl15", "ds_fr15")
        return min((self.distance_readings_meters[name] for name in front_sensor_names if name in self.distance_readings_meters), default=RobotConfiguration.DISTANCE_SENSOR_MAXIMUM_RANGE_METERS)

    def world_hit_points(self):
        """World-frame hit points [(x_meters, y_meters, distance_meters), ...] for mapping. Uses the odometry position and the IMU yaw."""
        hit_points      = []
        cosine_of_yaw   = math.cos(self.yaw_radians)
        sine_of_yaw     = math.sin(self.yaw_radians)
        for sensor_name, distance_meters in self.distance_readings_meters.items():
            if distance_meters >= RobotConfiguration.DISTANCE_SENSOR_NO_OBSTACLE_THRESHOLD_METERS:
                continue
            mount_x, mount_y, mount_angle = RobotConfiguration.DISTANCE_SENSOR_MOUNTING_POSES[sensor_name]
            mount_x         -= RobotConfiguration.AXLE_FORWARD_OFFSET_METERS    # odometry point is the axle
            sensor_world_x   = self.odometry_position_x_meters + cosine_of_yaw * mount_x - sine_of_yaw * mount_y
            sensor_world_y   = self.odometry_position_y_meters + sine_of_yaw * mount_x + cosine_of_yaw * mount_y
            beam_world_angle = self.yaw_radians + mount_angle
            hit_points.append((sensor_world_x + distance_meters * math.cos(beam_world_angle), sensor_world_y + distance_meters * math.sin(beam_world_angle), distance_meters))
        return hit_points

    # ---- floor colour ----------------------------------------------------
    def read_floor_rgb(self):
        camera_image = self.floor_camera.getImage()
        return (self.floor_camera.imageGetRed(camera_image, 1, 0, 0), self.floor_camera.imageGetGreen(camera_image, 1, 0, 0), self.floor_camera.imageGetBlue(camera_image, 1, 0, 0))

    def classify_floor_color(self):
        """Classify the floor under the camera: 'RED' | 'BLUE' | 'GREEN' | 'WHITE' | 'BLACK' | 'UNKNOWN'.
        Measured (R, G, B) in the dev arena: white (220,220,220), black floor (29,29,29), red (108,0,0), green (0,66,0), blue (0,0,55).
        Lighting in the evaluation world should match, but re-check if you change the light."""
        red, green, blue  = self.read_floor_rgb()
        brightest_channel = max(red, green, blue)
        if brightest_channel < 42:
            return "BLACK"
        if min(red, green, blue) > 0.7 * brightest_channel and brightest_channel > 120:
            return "WHITE"
        if red > 60 and green < 30 and blue < 30:
            return "RED"
        if blue > 45 and red < 30 and green < 30:
            return "BLUE"
        if green > 45 and red < 30 and blue < 30:
            return "GREEN"
        return "UNKNOWN"

    def read_look_ahead_colour(self):
        """Colour of the floor in the middle of the cell ahead, from the tilted look-ahead camera: 'RED' | 'BLUE' | 'GREEN' | 'BLACK' | 'UNKNOWN'."""
        if self.look_camera is None: return "UNKNOWN"
        camera_image = self.look_camera.getImage()
        if camera_image is None: return "UNKNOWN"
        pixels = []
        for row in RobotConfiguration.LOOK_CAMERA_SAMPLE_ROWS:
            for column in RobotConfiguration.LOOK_CAMERA_SAMPLE_COLUMNS:
                pixels.append((self.look_camera.imageGetRed(camera_image, RobotConfiguration.LOOK_CAMERA_WIDTH_PIXELS, column, row), self.look_camera.imageGetGreen(camera_image, RobotConfiguration.LOOK_CAMERA_WIDTH_PIXELS, column, row),
                               self.look_camera.imageGetBlue(camera_image, RobotConfiguration.LOOK_CAMERA_WIDTH_PIXELS, column, row)))
        self.last_look_ahead_mean_rgb = tuple(int(sum(pixel[channel] for pixel in pixels) / len(pixels)) for channel in range(3))     # for the log line
        return vote_floor_colour(pixels)

    # ---- LEDs ------------------------------------------------------------
    def light_majority_led(self, majority_color):
        """majority_color: 'BLUE' or 'GREEN' (turns the other LED off)."""
        self.blue_led.set(1 if majority_color == "BLUE" else 0)
        self.green_led.set(1 if majority_color == "GREEN" else 0)


# ----------------------------------------------------------------------------
# Occupancy grid (map built while driving, in the start-relative frame)
# ----------------------------------------------------------------------------
class OccupancyGrid:
    """Sparse grid of square cells. Every cell is UNKNOWN until a beam passes through it (FREE) or ends in it (OCCUPIED).
    Occupied cells are sticky. Inflated copies are kept for planning so the robot can be treated as a point."""

    UNKNOWN  = 0
    FREE     = 1
    OCCUPIED = 2

    def __init__(self):
        self.cell_size_meters         = RobotConfiguration.GRID_CELL_SIZE_METERS
        self.cell_states              = {}
        self.normal_blocked_counts    = {}
        self.tight_blocked_counts     = {}
        self.clearance_penalty_counts = {}
        self.change_version           = 0
        self.normal_offsets           = self.build_disk_offsets(RobotConfiguration.NORMAL_INFLATION_RADIUS_METERS)
        self.tight_offsets            = self.build_disk_offsets(RobotConfiguration.TIGHT_INFLATION_RADIUS_METERS)
        self.penalty_offsets          = self.build_disk_offsets(RobotConfiguration.CLEARANCE_PENALTY_RADIUS_METERS)

    def build_disk_offsets(self, radius_meters):
        radius_cells = radius_meters / self.cell_size_meters
        reach        = int(math.ceil(radius_cells))
        return [(offset_x, offset_y) for offset_x in range(-reach, reach + 1) for offset_y in range(-reach, reach + 1) if offset_x * offset_x + offset_y * offset_y <= radius_cells * radius_cells]

    def world_to_cell(self, world_x, world_y):
        return (int(math.floor(world_x / self.cell_size_meters)), int(math.floor(world_y / self.cell_size_meters)))

    def cell_to_world(self, cell):
        return ((cell[0] + 0.5) * self.cell_size_meters, (cell[1] + 0.5) * self.cell_size_meters)

    def state_of(self, cell):
        return self.cell_states.get(cell, 0)

    @staticmethod
    def cells_on_line(start_cell, end_cell):
        """Bresenham line, both ends included."""
        x, y         = start_cell
        end_x, end_y = end_cell
        delta_x      = abs(end_x - x)
        delta_y      = -abs(end_y - y)
        step_x       = 1
        step_y       = 1
        if x > end_x: step_x = -1
        if y > end_y: step_y = -1
        error        = delta_x + delta_y
        cells        = []
        while True:
            cells.append((x, y))
            if x == end_x and y == end_y: break
            double_error = 2 * error
            if double_error >= delta_y:
                error += delta_y
                x     += step_x
            if double_error <= delta_x:
                error += delta_x
                y     += step_y
        return cells

    def mark_ray(self, start_x, start_y, end_x, end_y, ray_hit_obstacle):
        """Hot path: Bresenham inlined, no list building, no helper calls."""
        inverse_size = 1.0 / self.cell_size_meters
        x            = int(math.floor(start_x * inverse_size))
        y            = int(math.floor(start_y * inverse_size))
        end_cell_x   = int(math.floor(end_x * inverse_size))
        end_cell_y   = int(math.floor(end_y * inverse_size))
        states       = self.cell_states
        delta_x      = abs(end_cell_x - x)
        delta_y      = -abs(end_cell_y - y)
        step_x       = 1
        step_y       = 1
        if x > end_cell_x: step_x = -1
        if y > end_cell_y: step_y = -1
        error        = delta_x + delta_y
        while x != end_cell_x or y != end_cell_y:
            if states.get((x, y)) != 2: states[(x, y)] = 1
            double_error = 2 * error
            if double_error >= delta_y:
                error += delta_y
                x     += step_x
            if double_error <= delta_x:
                error += delta_x
                y     += step_y
        if ray_hit_obstacle:
            self.mark_occupied((x, y))
        elif states.get((x, y)) != 2:
            states[(x, y)] = 1

    def mark_occupied(self, cell):
        if self.cell_states.get(cell) == 2: return
        self.cell_states[cell] = 2
        self.change_version   += 1
        self.stamp_inflation(cell)

    def stamp_inflation(self, cell):
        cell_x, cell_y = cell
        for offsets, counts in ((self.normal_offsets, self.normal_blocked_counts), (self.tight_offsets, self.tight_blocked_counts), (self.penalty_offsets, self.clearance_penalty_counts)):
            for offset_x, offset_y in offsets:
                key         = (cell_x + offset_x, cell_y + offset_y)
                counts[key] = counts.get(key, 0) + 1

    def clear_around(self, world_x, world_y, radius_meters):
        """Forgets every mapped cell within radius_meters of a point and rebuilds the inflated layers. Used when no route exists: drift or a mis-read can
        leave phantom obstacles that block everything, and the beams will simply re-map the real ones."""
        centre_x, centre_y = self.world_to_cell(world_x, world_y)
        radius_cells       = radius_meters / self.cell_size_meters
        for cell in [cell for cell in self.cell_states if math.hypot(cell[0] - centre_x, cell[1] - centre_y) <= radius_cells]: del self.cell_states[cell]
        self.normal_blocked_counts, self.tight_blocked_counts, self.clearance_penalty_counts = {}, {}, {}
        for cell, state in self.cell_states.items():
            if state == 2: self.stamp_inflation(cell)
        self.change_version += 1

    def blocked_cells(self, use_tight_inflation):
        if use_tight_inflation: return self.tight_blocked_counts
        return self.normal_blocked_counts

    def to_ascii(self, robot_cell=None, goal_cell=None, route_cells=()):
        """Text picture for debugging: # occupied, . free, blank unknown, R robot, G goal, * route."""
        if not self.cell_states: return ""
        xs        = [cell[0] for cell in self.cell_states]
        ys        = [cell[1] for cell in self.cell_states]
        route_set = set(route_cells)
        rows      = []
        for y in range(max(ys), min(ys) - 1, -1):
            row = ""
            for x in range(min(xs), max(xs) + 1):
                cell   = (x, y)
                state  = self.state_of(cell)
                symbol = " "
                if state == OccupancyGrid.FREE: symbol = "."
                if state == OccupancyGrid.OCCUPIED: symbol = "#"
                if cell in route_set: symbol = "*"
                if cell == goal_cell: symbol = "G"
                if cell == robot_cell: symbol = "R"
                row += symbol
            rows.append(row)
        return "\n".join(rows)


class OccupancyMapper:
    """Step observer: writes the distance-sensor beams into the grid about every 30 ms of simulated time. Set enabled = False to stop mapping (the maze does not use the map)."""

    UPDATE_INTERVAL_SECONDS = 0.03

    def __init__(self, occupancy_grid):
        self.occupancy_grid    = occupancy_grid
        self.enabled           = True
        self.last_update_time  = -1.0
        axle_offset            = RobotConfiguration.AXLE_FORWARD_OFFSET_METERS
        self.beam_constants    = [(name, pose[0] - axle_offset, pose[1], pose[2], math.cos(pose[2]), math.sin(pose[2])) for name, pose in RobotConfiguration.DISTANCE_SENSOR_MOUNTING_POSES.items()]

    def on_step(self, robot):
        if not self.enabled: return
        now = robot.simulation_time_seconds
        if now - self.last_update_time < self.UPDATE_INTERVAL_SECONDS: return
        self.last_update_time = now
        readings              = robot.distance_readings_meters
        cosine_yaw            = math.cos(robot.yaw_radians)
        sine_yaw              = math.sin(robot.yaw_radians)
        position_x            = robot.odometry_position_x_meters
        position_y            = robot.odometry_position_y_meters
        no_hit_threshold      = RobotConfiguration.DISTANCE_SENSOR_NO_OBSTACLE_THRESHOLD_METERS
        mark_ray              = self.occupancy_grid.mark_ray
        for name, mount_x, mount_y, _, cosine_mount, sine_mount in self.beam_constants:
            distance     = readings[name]
            sensor_x     = position_x + cosine_yaw * mount_x - sine_yaw * mount_y
            sensor_y     = position_y + sine_yaw * mount_x + cosine_yaw * mount_y
            beam_cosine  = cosine_yaw * cosine_mount - sine_yaw * sine_mount
            beam_sine    = sine_yaw * cosine_mount + cosine_yaw * sine_mount
            mark_ray(sensor_x, sensor_y, sensor_x + distance * beam_cosine, sensor_y + distance * beam_sine, distance < no_hit_threshold)

    def on_simulation_end(self, robot):
        pass


# ----------------------------------------------------------------------------
# Path planning on the grid
# ----------------------------------------------------------------------------
class GridPathPlanner:
    NEIGHBOR_STEPS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142))
    HEURISTIC_WEIGHT = 1.1    # slightly greedy A*: far fewer expansions, path within about 10 percent of optimal

    def __init__(self, occupancy_grid):
        self.occupancy_grid = occupancy_grid

    def exempt_cells(self, start_cell):
        """Free-or-unknown cells right around the robot stay passable so it can always drive out of a tight spot."""
        states = self.occupancy_grid.cell_states
        return {(start_cell[0] + offset_x, start_cell[1] + offset_y) for offset_x in range(-3, 4) for offset_y in range(-3, 4) if states.get((start_cell[0] + offset_x, start_cell[1] + offset_y)) != 2}

    def nearest_passable_cell(self, cell, start_cell, use_tight_inflation, search_radius_cells=12):
        blocked = self.occupancy_grid.blocked_cells(use_tight_inflation)
        exempt  = self.exempt_cells(start_cell)
        if cell not in blocked or cell in exempt: return cell
        for radius in range(1, search_radius_cells + 1):
            for offset_x in range(-radius, radius + 1):
                for offset_y in range(-radius, radius + 1):
                    if max(abs(offset_x), abs(offset_y)) != radius: continue
                    candidate = (cell[0] + offset_x, cell[1] + offset_y)
                    if candidate not in blocked or candidate in exempt: return candidate
        return None

    def plan(self, start_cell, goal_cell, use_tight_inflation, maximum_expansions=60000):
        """A* over free and unknown cells (unknown is treated as free, so the robot replans as it learns). Returns [cell, ...] or None."""
        goal_cell = self.nearest_passable_cell(goal_cell, start_cell, use_tight_inflation)
        if goal_cell is None: return None
        grid        = self.occupancy_grid
        blocked     = grid.blocked_cells(use_tight_inflation)
        penalty     = grid.clearance_penalty_counts
        exempt      = self.exempt_cells(start_cell)
        known_cells = list(grid.cell_states.keys()) + [start_cell, goal_cell]
        low_x       = min(cell[0] for cell in known_cells) - 20
        high_x      = max(cell[0] for cell in known_cells) + 20
        low_y       = min(cell[1] for cell in known_cells) - 20
        high_y      = max(cell[1] for cell in known_cells) + 20
        penalty_factor = 1.0 + RobotConfiguration.CLEARANCE_PENALTY_WEIGHT
        weight      = self.HEURISTIC_WEIGHT
        goal_x, goal_y = goal_cell
        open_heap   = [(0.0, 0, start_cell)]
        best_cost   = {start_cell: 0.0}
        came_from   = {}
        counter     = 1
        expansions  = 0
        heappush    = heapq.heappush
        heappop     = heapq.heappop
        while open_heap and expansions < maximum_expansions:
            _, _, cell = heappop(open_heap)
            expansions += 1
            if cell == goal_cell:
                route = [cell]
                while cell in came_from:
                    cell = came_from[cell]
                    route.append(cell)
                route.reverse()
                return route
            cell_x, cell_y = cell
            cell_cost      = best_cost[cell]
            for step_x, step_y, step_length in self.NEIGHBOR_STEPS:
                neighbor_x = cell_x + step_x
                neighbor_y = cell_y + step_y
                if neighbor_x < low_x or neighbor_x > high_x or neighbor_y < low_y or neighbor_y > high_y: continue
                neighbor = (neighbor_x, neighbor_y)
                if neighbor in blocked and neighbor not in exempt: continue
                if step_x != 0 and step_y != 0:
                    side_a = (neighbor_x, cell_y)
                    side_b = (cell_x, neighbor_y)
                    if (side_a in blocked and side_a not in exempt) or (side_b in blocked and side_b not in exempt): continue
                new_cost = cell_cost + step_length
                if neighbor in penalty: new_cost += step_length * penalty_factor - step_length
                if new_cost < best_cost.get(neighbor, 1e18):
                    best_cost[neighbor] = new_cost
                    came_from[neighbor] = cell
                    heappush(open_heap, (new_cost + weight * math.hypot(goal_x - neighbor_x, goal_y - neighbor_y), counter, neighbor))
                    counter += 1
        return None

    def line_is_clear(self, first_cell, second_cell, blocked, exempt):
        for cell in OccupancyGrid.cells_on_line(first_cell, second_cell):
            if cell in blocked and cell not in exempt: return False
        return True

    def route_is_clear(self, waypoint_cells, start_cell, use_tight_inflation):
        """True while every straight segment of the current route is still free of (inflated) obstacles."""
        blocked = self.occupancy_grid.blocked_cells(use_tight_inflation)
        exempt  = self.exempt_cells(start_cell)
        previous = start_cell
        for waypoint in waypoint_cells:
            if not self.line_is_clear(previous, waypoint, blocked, exempt): return False
            previous = waypoint
        return True

    @staticmethod
    def shortcut_keeps_clearance(route, first_index, last_index, penalty):
        """A straight shortcut replaces route[first_index..last_index]. A* already pays extra to stay away from obstacles, so the shortcut may not touch more
        near-obstacle (penalised) cells than the part of the route it replaces; otherwise smoothing would undo the centring in a gap."""
        line_cells  = sum(1 for cell in OccupancyGrid.cells_on_line(route[first_index], route[last_index]) if cell in penalty)
        route_cells = sum(1 for cell in route[first_index:last_index + 1] if cell in penalty)
        return line_cells <= route_cells

    def smooth(self, route, start_cell, use_tight_inflation, maximum_look_ahead_cells=40):
        """Drops zig-zag points: keeps only the cells needed to stay on clear straight lines."""
        if len(route) <= 2: return route
        blocked   = self.occupancy_grid.blocked_cells(use_tight_inflation)
        exempt    = self.exempt_cells(start_cell)
        penalty   = self.occupancy_grid.clearance_penalty_counts
        respect_clearance = bool(RobotConfiguration.SMOOTH_RESPECTS_CLEARANCE)
        waypoints = [route[0]]
        index     = 0
        while index < len(route) - 1:
            furthest = index + 1
            for candidate in range(min(len(route) - 1, index + maximum_look_ahead_cells), index, -1):
                if self.line_is_clear(route[index], route[candidate], blocked, exempt) and (not respect_clearance or self.shortcut_keeps_clearance(route, index, candidate, penalty)):
                    furthest = candidate
                    break
            waypoints.append(route[furthest])
            index = furthest
        return waypoints

    def nearest_frontier(self, start_cell, use_tight_inflation, minimum_distance_cells=4, maximum_visited=8000):
        """Breadth-first search over free, passable cells for a free cell that touches unknown space."""
        states  = self.occupancy_grid.cell_states
        blocked = self.occupancy_grid.blocked_cells(use_tight_inflation)
        exempt  = self.exempt_cells(start_cell)
        queue   = deque([start_cell])
        visited = {start_cell}
        while queue and len(visited) < maximum_visited:
            cell = queue.popleft()
            if max(abs(cell[0] - start_cell[0]), abs(cell[1] - start_cell[1])) >= minimum_distance_cells:
                for step_x, step_y in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    if (cell[0] + step_x, cell[1] + step_y) not in states: return cell
            for step_x, step_y in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (cell[0] + step_x, cell[1] + step_y)
                if neighbor in visited: continue
                if states.get(neighbor) != 1: continue
                if neighbor in blocked and neighbor not in exempt: continue
                visited.add(neighbor)
                queue.append(neighbor)
        return None


# ----------------------------------------------------------------------------
# Shared driving helpers
# ----------------------------------------------------------------------------
def log_progress(robot, message):
    if RobotConfiguration.PRINT_PROGRESS: print("[%7.2f s] %s" % (robot.simulation_time_seconds, message))


def corridor_turn_rate(robot, target_yaw_radians):
    """Turn rate that holds a heading and centres the robot between walls (or keeps a fixed gap to one wall)."""
    threshold     = RobotConfiguration.WALL_DETECTION_THRESHOLD_METERS
    nominal       = RobotConfiguration.SIDE_SENSOR_READING_WHEN_CENTERED_AGAINST_WALL_METERS
    left_distance = robot.distance_readings_meters["ds_l"]
    right_distance = robot.distance_readings_meters["ds_r"]
    lateral_error = 0.0
    if left_distance < threshold and right_distance < threshold:
        lateral_error = (left_distance - right_distance) / 2.0
    elif left_distance < threshold:
        lateral_error = left_distance - nominal
    elif right_distance < threshold:
        lateral_error = nominal - right_distance
    heading_error      = MathHelpers.wrap_angle_to_plus_minus_pi(target_yaw_radians - robot.yaw_radians)
    heading_correction = MathHelpers.clamp_value(6.0 * lateral_error, -0.2, 0.2)
    return MathHelpers.clamp_value(5.0 * (heading_error + heading_correction), -2.0, 2.0)


def reactive_avoidance_turn_rate(robot):
    """Steers away from anything very close at the front corners or beside the wheels (works alongside the planner)."""
    readings       = robot.distance_readings_meters
    left_distance  = min(readings["ds_cl"], readings["ds_fl30"], readings["ds_fl15"], readings["ds_l"] - 0.02, readings["ds_left_front"] - 0.02)
    right_distance = min(readings["ds_cr"], readings["ds_fr30"], readings["ds_fr15"], readings["ds_r"] - 0.02, readings["ds_right_front"] - 0.02)
    left_push      = max(0.0, 0.07 - left_distance)
    right_push     = max(0.0, 0.07 - right_distance)
    return 20.0 * (right_push - left_push)


def side_clearance_speed_factor(robot):
    """0.45 to 1.0: slows down when an obstacle is close beside or ahead-diagonal, so corners are not clipped."""
    readings  = robot.distance_readings_meters
    closest   = min(readings["ds_l"], readings["ds_r"], readings["ds_cl"], readings["ds_cr"])
    return MathHelpers.clamp_value((closest - 0.03) / 0.08, RobotConfiguration.SIDE_CLEARANCE_MINIMUM_SPEED_FACTOR, 1.0)


def drive_straight_distance(robot, profile, axis_index, distance_meters, cruise_speed_meters_per_second):
    """Drives distance_meters along a world axis, holding the heading, then stops. False if the simulation ended."""
    target_yaw     = MathHelpers.axis_yaw(axis_index)
    step_x, step_y = MathHelpers.axis_step(axis_index)
    start_x        = robot.odometry_position_x_meters
    start_y        = robot.odometry_position_y_meters
    current_speed  = 0.0
    while True:
        travelled = (robot.odometry_position_x_meters - start_x) * step_x + (robot.odometry_position_y_meters - start_y) * step_y
        remaining = distance_meters - travelled
        if remaining <= 0.004: break
        desired_speed = min(cruise_speed_meters_per_second, max(0.03, 2.5 * remaining))
        current_speed = profile.limit_acceleration(current_speed, desired_speed, robot.timestep_milliseconds / 1000.0)
        robot.drive_with_forward_speed_and_turn_rate(current_speed, corridor_turn_rate(robot, target_yaw))
        if not robot.step_simulation(): return False
        if robot.is_stalled():
            if not robot.back_off_and_scan(): return False
            current_speed = 0.0
    robot.stop_motors()
    return True


# ----------------------------------------------------------------------------
# Section 1: obstacle field (map + A* + pure pursuit)
# ----------------------------------------------------------------------------
def choose_section_1_waypoints(robot, occupancy_grid, planner, goal_cell, use_tight_inflation, allow_exploration):
    """Returns (world_waypoints, cell_waypoints) toward the goal; falls back to exploring the nearest unknown edge if the goal cannot be reached."""
    start_cell = occupancy_grid.world_to_cell(robot.odometry_position_x_meters, robot.odometry_position_y_meters)
    route      = planner.plan(start_cell, goal_cell, use_tight_inflation)
    if route is None and not use_tight_inflation: route = planner.plan(start_cell, goal_cell, True)
    arrived_near_goal = allow_exploration and max(abs(start_cell[0] - goal_cell[0]), abs(start_cell[1] - goal_cell[1])) <= 5
    if allow_exploration and (route is None or arrived_near_goal):
        frontier = planner.nearest_frontier(start_cell, use_tight_inflation)
        if frontier is not None:
            frontier_route = planner.plan(start_cell, frontier, use_tight_inflation)
            if frontier_route is not None: route = frontier_route
    if route is None: return ([], [])
    cell_waypoints = planner.smooth(route, start_cell, use_tight_inflation)
    return ([occupancy_grid.cell_to_world(cell) for cell in cell_waypoints], cell_waypoints)


def pick_lookahead_point(robot, waypoints, lookahead_meters=RobotConfiguration.SECTION_1_LOOKAHEAD_METERS):
    while len(waypoints) > 1 and math.hypot(waypoints[0][0] - robot.odometry_position_x_meters, waypoints[0][1] - robot.odometry_position_y_meters) < 0.06:
        waypoints.pop(0)
    for waypoint in waypoints:
        if math.hypot(waypoint[0] - robot.odometry_position_x_meters, waypoint[1] - robot.odometry_position_y_meters) >= lookahead_meters: return waypoint
    return waypoints[-1]


def reactive_drive_toward(robot, goal_x, goal_y, seconds):
    """Bug-style fallback when A* finds no route (ported from the datalab2026 controller): head for the goal; when the front is blocked, turn away on the
    more open side; keep a little clearance from the followed side. Returns False if the simulation ended."""
    readings = robot.distance_readings_meters
    end_time = robot.simulation_time_seconds + seconds
    side     = 1.0
    if readings["ds_l"] <= readings["ds_r"]: side = -1.0           # +1: turn left, -1: turn right (toward the more open side)
    while robot.simulation_time_seconds < end_time:
        if math.hypot(goal_x - robot.odometry_position_x_meters, goal_y - robot.odometry_position_y_meters) < 0.035: break
        front = robot.front_minimum_distance_meters()
        if front < 0.12 or min(readings["ds_cl"], readings["ds_cr"]) < 0.08:
            robot.drive_with_forward_speed_and_turn_rate(robot.pivot_creep_speed(), 3.0 * side)       # turn away from the obstacle
        else:
            error    = MathHelpers.wrap_angle_to_plus_minus_pi(math.atan2(goal_y - robot.odometry_position_y_meters, goal_x - robot.odometry_position_x_meters) - robot.yaw_radians)
            followed = readings["ds_right_front"]
            if side > 0: followed = readings["ds_left_front"]
            if followed < 0.07: error = 0.3 * side                                                        # keep clear of the side being followed
            robot.drive_with_forward_speed_and_turn_rate(RobotConfiguration.MAXIMUM_SPEED_METERS_PER_SECOND * 0.5, MathHelpers.clamp_value(2.0 * error, -2.5, 2.5))
        if not robot.step_simulation(): return False
        if robot.is_stalled():
            if not robot.back_off_and_scan(): return False
    robot.stop_motors()
    return True


def run_section_1_obstacle_field(robot, occupancy_grid, planner, profile, start_point=None):
    """Drives from Red A to Red B. Returns True when the red transition tile is reached, False if the simulation ended."""
    start_x               = robot.odometry_position_x_meters
    start_y               = robot.odometry_position_y_meters
    if start_point is not None: start_x, start_y = start_point      # a retry keeps the ORIGINAL start, so the goal stays where it was
    red_confirmations     = 0
    goal_offset           = RobotConfiguration.SECTION_1_GOAL_OFFSET_FROM_START_METERS
    goal_cell             = occupancy_grid.world_to_cell(start_x + goal_offset[0], start_y + goal_offset[1])
    gate_offset           = RobotConfiguration.SECTION_1_GATE_OFFSET_FROM_START_METERS
    gate_x                = start_x + gate_offset[0]
    gate_y                = start_y + gate_offset[1]
    gate_cell             = occupancy_grid.world_to_cell(gate_x, gate_y)
    goal_world_x          = start_x + goal_offset[0]
    goal_world_y          = start_y + goal_offset[1]
    plan_failures         = 0
    going_to_gate         = True
    minimum_red_distance  = RobotConfiguration.SECTION_1_MINIMUM_DISTANCE_BEFORE_RED_IS_GOAL_METERS
    replan_interval       = RobotConfiguration.SECTION_1_REPLAN_INTERVAL_SECONDS
    timestep_seconds      = robot.timestep_milliseconds / 1000.0
    waypoints             = []
    cell_waypoints        = []
    last_plan_time        = -10.0
    last_check_time       = -10.0
    current_speed         = 0.0
    progress_time         = robot.simulation_time_seconds
    progress_x            = start_x
    progress_y            = start_y
    recovery_until_time   = 0.0
    recovery_turn_sign    = 1.0
    tight_until_time      = 0.0
    readings              = robot.distance_readings_meters
    while True:
        now        = robot.simulation_time_seconds
        position_x = robot.odometry_position_x_meters
        position_y = robot.odometry_position_y_meters
        if robot.is_stalled():
            log_progress(robot, "wheel stalled at odometry (%.3f, %.3f): backing off, rotating and scanning" % (position_x, position_y))
            if not robot.back_off_and_scan(): return False
            waypoints        = []
            tight_until_time = robot.simulation_time_seconds + 4.0
            progress_time    = robot.simulation_time_seconds
            progress_x       = robot.odometry_position_x_meters
            progress_y       = robot.odometry_position_y_meters
            current_speed    = 0.0
            continue
        if math.hypot(position_x - start_x, position_y - start_y) > minimum_red_distance and robot.is_level() and robot.classify_floor_color() == "RED":
            red_confirmations += 1
        else:
            red_confirmations = 0
        if red_confirmations >= 4:
            robot.stop_motors()
            log_progress(robot, "Section 1 complete: red transition tile reached")
            return True
        if now - progress_time >= RobotConfiguration.SECTION_1_STUCK_CHECK_INTERVAL_SECONDS:
            if math.hypot(position_x - progress_x, position_y - progress_y) < RobotConfiguration.SECTION_1_STUCK_MINIMUM_PROGRESS_METERS:
                recovery_until_time = now + 0.8
                recovery_turn_sign  = -recovery_turn_sign
                tight_until_time    = now + 4.0
                waypoints           = []
                log_progress(robot, "stuck at odometry (%.3f, %.3f), yaw %.2f: backing out and replanning" % (position_x, position_y, robot.yaw_radians))
                if RobotConfiguration.PRINT_PROGRESS: print(occupancy_grid.to_ascii(occupancy_grid.world_to_cell(position_x, position_y), goal_cell, [occupancy_grid.world_to_cell(wx, wy) for wx, wy in waypoints]))
            progress_time = now
            progress_x    = position_x
            progress_y    = position_y
        if now < recovery_until_time:
            current_speed = -0.06
            robot.drive_with_forward_speed_and_turn_rate(current_speed, 0.9 * recovery_turn_sign)
            if not robot.step_simulation(): return False
            continue
        if going_to_gate and math.hypot(position_x - gate_x, position_y - gate_y) < RobotConfiguration.SECTION_1_GATE_REACHED_DISTANCE_METERS:
            going_to_gate = False
            waypoints     = []
            log_progress(robot, "gate reached: heading straight into Red B")
        use_tight_inflation = now < tight_until_time
        replan_needed       = not waypoints or now - last_plan_time > replan_interval
        if not replan_needed and now - last_check_time > 0.1:
            last_check_time = now
            replan_needed   = not planner.route_is_clear(cell_waypoints, occupancy_grid.world_to_cell(position_x, position_y), use_tight_inflation)
        if replan_needed:
            current_goal_cell         = goal_cell
            if going_to_gate: current_goal_cell = gate_cell
            waypoints, cell_waypoints = choose_section_1_waypoints(robot, occupancy_grid, planner, current_goal_cell, use_tight_inflation, not going_to_gate)
            last_plan_time            = now
        if not waypoints:
            plan_failures += 1
            aim_x, aim_y   = goal_world_x, goal_world_y
            if going_to_gate: aim_x, aim_y = gate_x, gate_y
            robot.drive_with_forward_speed_and_turn_rate(0.0, 0.8 * recovery_turn_sign)
            if plan_failures == 1:
                log_progress(robot, "no route: forgetting mapped obstacles near the robot and replanning")
                occupancy_grid.clear_around(position_x, position_y, RobotConfiguration.SECTION_1_FORGET_RADIUS_METERS)
            else:
                log_progress(robot, "still no route: driving reactively toward the goal")
                if not reactive_drive_toward(robot, aim_x, aim_y, RobotConfiguration.SECTION_1_REACTIVE_SECONDS): return False
                occupancy_grid.clear_around(robot.odometry_position_x_meters, robot.odometry_position_y_meters, RobotConfiguration.SECTION_1_FORGET_RADIUS_METERS)
            last_plan_time = -10.0                                  # plan again at once
            if not robot.step_simulation(): return False
            continue
        plan_failures = 0
        target_x, target_y = pick_lookahead_point(robot, waypoints)
        heading_error      = MathHelpers.wrap_angle_to_plus_minus_pi(math.atan2(target_y - position_y, target_x - position_x) - robot.yaw_radians)
        front_clearance    = min(readings["ds_f0"], readings["ds_fl15"], readings["ds_fr15"])
        turn_rate          = MathHelpers.clamp_value(3.0 * heading_error, -2.5, 2.5)
        desired_speed      = 0.0
        if abs(heading_error) <= 0.8: desired_speed = max(0.03, profile.speed_for_clearance(front_clearance)) * max(0.2, 1.0 - abs(heading_error) / 1.2) * side_clearance_speed_factor(robot)
        turn_rate         += reactive_avoidance_turn_rate(robot)
        if abs(heading_error) > 0.8: desired_speed = robot.pivot_creep_speed()
        if front_clearance < 0.045:
            desired_speed = -0.05
            turn_rate     = 1.0
            if readings["ds_fr30"] > readings["ds_fl30"]: turn_rate = -1.0
        current_speed = profile.limit_acceleration(current_speed, desired_speed, timestep_seconds)
        robot.drive_with_forward_speed_and_turn_rate(current_speed, turn_rate)
        if not robot.step_simulation(): return False


# ----------------------------------------------------------------------------
# Transition corridor (Red B) into the maze
# ----------------------------------------------------------------------------
def enter_maze_through_transition(robot, profile):
    """Aligns with the corridor, leaves the red tile toward the black floor (turning round if it faces the white field),
    then stops on the centre of the first maze cell. Returns False if the simulation ended."""
    threshold    = RobotConfiguration.WALL_DETECTION_THRESHOLD_METERS
    readings     = robot.distance_readings_meters
    side_walls   = readings["ds_l"] < threshold and readings["ds_r"] < threshold
    end_walls    = readings["ds_f0"] < threshold and readings["ds_rear"] < threshold
    axis_index   = MathHelpers.nearest_axis_index(robot.yaw_radians)
    if not side_walls and end_walls: axis_index = (axis_index + 1) % 4
    if abs(MathHelpers.wrap_angle_to_plus_minus_pi(MathHelpers.axis_yaw(axis_index) - robot.yaw_radians)) > 0.35:
        if not robot.turn_in_place_to_absolute_yaw(MathHelpers.axis_yaw(axis_index)): return False
    start_x       = robot.odometry_position_x_meters
    start_y       = robot.odometry_position_y_meters
    current_speed = 0.0
    turn_arounds  = 0
    while True:
        colour   = robot.classify_floor_color()
        distance = math.hypot(robot.odometry_position_x_meters - start_x, robot.odometry_position_y_meters - start_y)
        if colour == "BLACK": break
        if colour == "WHITE" and distance > 0.03:
            turn_arounds += 1
            if turn_arounds > 1:
                log_progress(robot, "white floor in both directions: this is not Red B, going back to Section 1")
                return "NOT_IN_CORRIDOR"
            axis_index = (axis_index + 2) % 4
            log_progress(robot, "facing the white field: turning around")
            if not robot.turn_in_place_to_absolute_yaw(MathHelpers.axis_yaw(axis_index)): return False
            start_x = robot.odometry_position_x_meters
            start_y = robot.odometry_position_y_meters
            continue
        if distance > 0.8: break
        current_speed = profile.limit_acceleration(current_speed, 0.10, robot.timestep_milliseconds / 1000.0)
        robot.drive_with_forward_speed_and_turn_rate(current_speed, corridor_turn_rate(robot, MathHelpers.axis_yaw(axis_index)))
        if not robot.step_simulation(): return False
        if robot.is_stalled():
            if not robot.back_off_and_scan(): return False
            current_speed = 0.0
    distance_to_first_cell = RobotConfiguration.CAMERA_FORWARD_OFFSET_METERS + RobotConfiguration.TRANSITION_TILE_END_TO_FIRST_CELL_CENTER_METERS
    log_progress(robot, "entering maze: advancing %.3f m to the first cell centre" % distance_to_first_cell)
    return drive_straight_distance(robot, profile, axis_index, distance_to_first_cell, 0.10)


# ----------------------------------------------------------------------------
# Section 2: maze (cell graph, nearest-frontier exploration, shortest route to Red C)
# ----------------------------------------------------------------------------
class LegAborted(Exception):
    """Raised inside a maze leg that cannot finish as planned: a wall where the map said open ("wall") or a leg that took too long ("timeout")."""
    def __init__(self, reason):
        Exception.__init__(self, reason)
        self.reason = reason


def wall_blocks_leg(sensor_remaining_meters, remaining_meters, heading_error_radians, front_distance_meters):
    """True when the front beam sees a wall much nearer than the cell the leg is heading for, i.e. the map was wrong about this edge.
    A wall at the END of the target cell gives sensor_remaining close to remaining, and a wall beyond it gives a larger value, so neither triggers this."""
    if heading_error_radians >= 0.1: return False
    if front_distance_meters >= 0.25: return False
    return sensor_remaining_meters < remaining_meters - RobotConfiguration.UNEXPECTED_WALL_MARGIN_METERS


def majority_is_decided(blue_count, green_count, unvisited_cell_count):
    """Each unvisited cell can hold at most one tile, so a lead bigger than the unvisited cells can no longer change."""
    return abs(blue_count - green_count) > unvisited_cell_count


class LoopGuard:
    """Ported from the datalab2026 controller: bans a maze target that keeps being chosen but keeps failing."""

    def __init__(self, limit=RobotConfiguration.LOOP_GUARD_LIMIT, ban_seconds=RobotConfiguration.LOOP_GUARD_BAN_SECONDS):
        self.limit       = limit
        self.ban_seconds = ban_seconds
        self.last_target = None
        self.count       = 0
        self.ban_ends    = {}

    def note(self, target, now):
        """Record that `target` was chosen and its route aborted. Returns True if it has just been banned."""
        if target == self.last_target: self.count += 1
        else: self.last_target, self.count = target, 1
        if self.count > self.limit:
            self.ban_ends[target] = now + self.ban_seconds
            self.last_target, self.count = None, 0
            return True
        return False

    def banned_cells(self, now):
        for target in [target for target, ends in self.ban_ends.items() if now >= ends]: del self.ban_ends[target]
        return set(self.ban_ends)


class MazeKnowledge:
    """What the robot knows about the maze. Directions are world axes: 0 east, 1 north, 2 west, 3 south."""

    def __init__(self):
        self.visited_cells       = set()
        self.wall_between        = {}
        self.tile_colour_by_cell = {}
        self.end_cell            = None
        self.origin_x            = 0.0    # odometry position of the centre of cell (0, 0)
        self.origin_y            = 0.0
        self.conflicts           = []     # wall readings that disagreed with an earlier reading of the same edge (for diagnosis)
        self.visit_order         = []     # (time, cell) each time a cell is sensed for the first time
        self.move_count          = 0
        self.aborted_legs        = 0     # legs stopped by an unexpected wall or a timeout
        self.peeked_cells        = set()  # dead ends read from the cell before them with the look-ahead camera (never entered unless the route to the end cell passes through)

    def cell_center(self, cell):
        pitch = RobotConfiguration.MAZE_CELL_PITCH_METERS
        return (self.origin_x + cell[0] * pitch, self.origin_y + cell[1] * pitch)

    def cell_at(self, world_x, world_y):
        """The maze cell whose centre is nearest to an odometry position."""
        pitch = RobotConfiguration.MAZE_CELL_PITCH_METERS
        return (int(round((world_x - self.origin_x) / pitch)), int(round((world_y - self.origin_y) / pitch)))

    @staticmethod
    def neighbor(cell, direction_index):
        step_x, step_y = MathHelpers.axis_step(direction_index)
        return (cell[0] + step_x, cell[1] + step_y)

    def record_wall(self, cell, direction_index, is_wall):
        previous = self.wall_between.get((cell, direction_index))
        if previous is not None and previous != is_wall: self.conflicts.append((cell, direction_index, previous, is_wall))
        self.wall_between[(cell, direction_index)] = is_wall
        self.wall_between[(self.neighbor(cell, direction_index), (direction_index + 2) % 4)] = is_wall

    def close_edge(self, cell, direction_index):
        """Deliberately marks an edge as a wall (the way back to Red B). Not counted as a contradiction."""
        self.wall_between[(cell, direction_index)] = True
        self.wall_between[(self.neighbor(cell, direction_index), (direction_index + 2) % 4)] = True

    def is_open(self, cell, direction_index):
        return self.wall_between.get((cell, direction_index)) is False

    def tile_count(self, colour):
        return sum(1 for tile_colour in self.tile_colour_by_cell.values() if tile_colour == colour)

    def steps_from_entrance(self):
        """Route length (in cells) from cell (0, 0) through known open edges to every known cell, including unvisited cells next to visited ones."""
        steps, queue = {(0, 0): 0}, [(0, 0)]
        for cell in queue:
            for direction_index in range(4):
                if not self.is_open(cell, direction_index): continue
                neighbour = self.neighbor(cell, direction_index)
                if neighbour in steps: continue
                steps[neighbour] = steps[cell] + 1
                if neighbour in self.visited_cells: queue.append(neighbour)
        return steps

    def find_route(self, start_cell, start_heading_index, target_cell=None, banned_cells=()):
        """Dijkstra over (cell, heading). With no target it returns the directions to the best unvisited cell: the cheapest to reach, plus START_BIAS_WEIGHT x its
        distance from the entrance (so the area near the entrance is finished before heading far away); otherwise to target_cell.
        Every 90 degrees of turning costs extra, so straight routes win ties."""
        heap       = [(0.0, 0, start_cell, start_heading_index, [])]
        settled    = set()
        candidates = {}
        counter    = 1
        while heap:
            cost, _, cell, heading, route = heapq.heappop(heap)
            if (cell, heading) in settled: continue
            settled.add((cell, heading))
            if target_cell is None and cell not in self.visited_cells:
                if cell not in banned_cells and cell not in candidates: candidates[cell] = (cost, route)
                continue
            if target_cell is not None and cell == target_cell: return route
            if cell not in self.visited_cells: continue
            for direction_index in range(4):
                if not self.is_open(cell, direction_index): continue
                turns = min((direction_index - heading) % 4, (heading - direction_index) % 4)
                heapq.heappush(heap, (cost + 1.0 + RobotConfiguration.TURN_PENALTY_IN_CELL_STEPS * turns, counter, self.neighbor(cell, direction_index), direction_index, route + [direction_index]))
                counter += 1
        if target_cell is not None or not candidates: return None
        bias = RobotConfiguration.START_BIAS_WEIGHT
        if bias <= 0.0: return min(candidates.values(), key=lambda item: item[0])[1]
        from_entrance = self.steps_from_entrance()
        best_cell     = min(candidates, key=lambda cell: candidates[cell][0] + bias * from_entrance.get(cell, 99))
        return candidates[best_cell][1]


def classify_pixel(red, green, blue):
    """One pixel of the tilted camera: the tile colours are each one strong channel with the other two near zero; the black floor is dark grey."""
    brightest = max(red, green, blue)
    others    = red + green + blue - brightest - min(red, green, blue)       # the middle channel
    if brightest < 42 and brightest - min(red, green, blue) < 15: return "BLACK"
    if brightest < 30 or others > 0.35 * brightest or min(red, green, blue) > 0.35 * brightest: return "UNKNOWN"
    if red == brightest: return "RED"
    if green == brightest: return "GREEN"
    return "BLUE"


def vote_floor_colour(pixels):
    """The colour most sampled pixels agree on, or UNKNOWN when they do not agree strongly enough (then the robot just enters the cell)."""
    if not pixels: return "UNKNOWN"
    votes = {}
    for red, green, blue in pixels:
        colour        = classify_pixel(red, green, blue)
        votes[colour] = votes.get(colour, 0) + 1
    colour = max(votes, key=votes.get)
    if colour != "UNKNOWN" and votes[colour] >= RobotConfiguration.LOOK_AHEAD_REQUIRED_AGREEMENT * len(pixels): return colour
    return "UNKNOWN"


def closed_dead_end_beam_targets():
    """Beam readings (ds_f0, ds_fl30, ds_fr30) that mean: the cell ahead has a wall in front and a wall on each side, i.e. it is a dead end whose only door is the one we look through."""
    pitch      = RobotConfiguration.MAZE_CELL_PITCH_METERS
    half_wall  = RobotConfiguration.MAZE_WALL_HALF_THICKNESS_METERS
    front      = RobotConfiguration.FRONT_SENSOR_READING_WHEN_CELL_CENTER_FACES_WALL_METERS + pitch
    left_pose  = RobotConfiguration.DISTANCE_SENSOR_MOUNTING_POSES["ds_fl30"]
    right_pose = RobotConfiguration.DISTANCE_SENSOR_MOUNTING_POSES["ds_fr30"]
    side_reach = pitch / 2.0 - half_wall
    return front, (side_reach - abs(left_pose[1])) / abs(math.sin(left_pose[2])), (side_reach - abs(right_pose[1])) / abs(math.sin(right_pose[2]))


def look_ahead_resolves_dead_end(readings, forward_offset_meters, side_offset_meters, yaw_error_radians):
    """True when the robot stands on the cell centre line, facing along the corridor, and three beams agree that the next cell is closed on three sides."""
    if abs(forward_offset_meters) > RobotConfiguration.LOOK_AHEAD_MAXIMUM_OFFSET_METERS or abs(side_offset_meters) > RobotConfiguration.LOOK_AHEAD_MAXIMUM_OFFSET_METERS: return False
    if abs(yaw_error_radians) > RobotConfiguration.LOOK_AHEAD_MAXIMUM_YAW_ERROR_RADIANS: return False
    front, left, right = closed_dead_end_beam_targets()
    tolerance          = RobotConfiguration.LOOK_AHEAD_BEAM_TOLERANCE_METERS
    return abs(readings["ds_f0"] - front) < tolerance and abs(readings["ds_fl30"] - left) < tolerance and abs(readings["ds_fr30"] - right) < tolerance


def peek_at_dead_end_ahead(robot, knowledge, cell, heading_index):
    """If the cell ahead is provably a closed dead end (see look_ahead_resolves_dead_end) and the tilted camera shows a clear colour, records that cell as visited
    without entering it. Returns True when it did. Anything uncertain returns False and the robot enters the cell as usual."""
    if robot.look_camera is None: return False
    ahead = MazeKnowledge.neighbor(cell, heading_index)
    if ahead in knowledge.visited_cells or knowledge.wall_between.get((cell, heading_index)) is not False: return False
    step_x, step_y     = MathHelpers.axis_step(heading_index)
    center_x, center_y = knowledge.cell_center(cell)
    offset_x           = robot.odometry_position_x_meters - center_x
    offset_y           = robot.odometry_position_y_meters - center_y
    forward_offset     = offset_x * step_x + offset_y * step_y
    side_offset        = -offset_x * step_y + offset_y * step_x
    yaw_error          = MathHelpers.wrap_angle_to_plus_minus_pi(MathHelpers.axis_yaw(heading_index) - robot.yaw_radians)
    if not look_ahead_resolves_dead_end(robot.distance_readings_meters, forward_offset, side_offset, yaw_error): return False
    if abs(robot.pitch_radians) > RobotConfiguration.LOOK_AHEAD_MAXIMUM_TILT_RADIANS or abs(robot.roll_radians) > RobotConfiguration.LOOK_AHEAD_MAXIMUM_TILT_RADIANS: return False
    colour = robot.read_look_ahead_colour()
    if colour in ("UNKNOWN", "RED"): return False       # red is the end cell: a red wall looks the same from here, so the floor camera decides that one after entering
    knowledge.record_wall(ahead, heading_index, True)
    knowledge.record_wall(ahead, (heading_index + 1) % 4, True)
    knowledge.record_wall(ahead, (heading_index - 1) % 4, True)
    knowledge.visited_cells.add(ahead)
    knowledge.peeked_cells.add(ahead)
    if colour in ("BLUE", "GREEN"): knowledge.tile_colour_by_cell[ahead] = colour
    log_progress(robot, "look-ahead: dead end at %s is %s, mean rgb %s, pitch %.3f (blue %d, green %d)" % (ahead, colour, robot.last_look_ahead_mean_rgb, robot.pitch_radians, knowledge.tile_count("BLUE"), knowledge.tile_count("GREEN")))
    return True


def brake_to_stop(robot, profile, knowledge, speed):
    """Slows to a standstill at the usual deceleration while holding the corridor heading. Returns False when the simulation ended."""
    axis_index       = MathHelpers.nearest_axis_index(robot.yaw_radians)
    target_yaw       = MathHelpers.axis_yaw(axis_index)
    timestep_seconds = robot.timestep_milliseconds / 1000.0
    while speed > 0.005:
        speed = profile.limit_acceleration(speed, 0.0, timestep_seconds)
        robot.drive_with_forward_speed_and_turn_rate(speed, corridor_turn_rate(robot, target_yaw))
        if not robot.step_simulation(): return False
    robot.stop_motors()
    return True


def sense_new_cell(robot, knowledge, cell, heading_index, is_first_cell):
    """At a cell centre: record the three walls around the heading (and the rear on the first cell), then read the floor colour."""
    threshold = RobotConfiguration.WALL_DETECTION_THRESHOLD_METERS
    readings  = robot.distance_readings_meters
    knowledge.record_wall(cell, heading_index, readings["ds_f0"] < threshold)
    knowledge.record_wall(cell, (heading_index + 1) % 4, readings["ds_l"] < threshold)
    knowledge.record_wall(cell, (heading_index - 1) % 4, readings["ds_r"] < threshold)
    if is_first_cell: knowledge.record_wall(cell, (heading_index + 2) % 4, readings["ds_rear"] < threshold)
    knowledge.visited_cells.add(cell)
    knowledge.visit_order.append((round(robot.simulation_time_seconds, 1), cell))
    colour = robot.classify_floor_color()
    if colour in ("BLUE", "GREEN") and cell not in knowledge.tile_colour_by_cell:
        knowledge.tile_colour_by_cell[cell] = colour
        log_progress(robot, "%s tile at cell %s (blue %d, green %d)" % (colour, cell, knowledge.tile_count("BLUE"), knowledge.tile_count("GREEN")))
    if colour == "RED" and knowledge.end_cell is None:
        knowledge.end_cell = cell
        log_progress(robot, "red end cell at %s (rgb %s)" % (cell, robot.read_floor_rgb()))
    peek_at_dead_end_ahead(robot, knowledge, cell, heading_index)


def back_up_to_cell_center(robot, knowledge, cell):
    """If the robot stopped past the centre of the cell it is in, reverse to the centre before pivoting (keeps the swinging corners away from wall ends)."""
    axis_index     = MathHelpers.nearest_axis_index(robot.yaw_radians)
    step_x, step_y = MathHelpers.axis_step(axis_index)
    center_x, center_y = knowledge.cell_center(cell)
    overshoot      = (robot.odometry_position_x_meters - center_x) * step_x + (robot.odometry_position_y_meters - center_y) * step_y
    if overshoot < 0.012 or overshoot > 0.08: return True
    start_x = robot.odometry_position_x_meters
    start_y = robot.odometry_position_y_meters
    while math.hypot(robot.odometry_position_x_meters - start_x, robot.odometry_position_y_meters - start_y) < overshoot - 0.003:
        robot.drive_with_forward_speed_and_turn_rate(-0.05, corridor_turn_rate(robot, MathHelpers.axis_yaw(axis_index)))
        if not robot.step_simulation(): return False
    robot.stop_motors()
    return True


def drive_to_next_cell(robot, profile, knowledge, cell, direction_index, stop_at_end, starting_speed, arc_entry=False, skip_alignment_turn=False):
    """Drives to the centre of the neighbouring cell. Targets are absolute (no error builds up from cell to cell), braking starts from the
    distance left (v = sqrt(2 a d)), and with stop_at_end False the robot carries its speed through the centre. A wall ahead always stops it.
    With arc_entry the leg ends half a cell early, at the edge where the arc starts, already slowed to the arc speed. skip_alignment_turn: the robot has just left an arc, so small heading error is fine.
    Returns (still_running, speed_on_leaving)."""
    target_yaw = MathHelpers.axis_yaw(direction_index)
    alignment_tolerance = 0.03
    if skip_alignment_turn: alignment_tolerance = 0.25
    if abs(MathHelpers.wrap_angle_to_plus_minus_pi(target_yaw - robot.yaw_radians)) > alignment_tolerance:
        robot.stop_motors()
        if not back_up_to_cell_center(robot, knowledge, cell): return (False, 0.0)
        if not robot.turn_in_place_to_absolute_yaw(target_yaw): return (False, 0.0)
        starting_speed = 0.0
    step_x, step_y   = MathHelpers.axis_step(direction_index)
    target_x, target_y = knowledge.cell_center(MazeKnowledge.neighbor(cell, direction_index))
    if arc_entry:
        target_x -= step_x * RobotConfiguration.MAZE_CELL_PITCH_METERS / 2.0
        target_y -= step_y * RobotConfiguration.MAZE_CELL_PITCH_METERS / 2.0
    arc_speed        = RobotConfiguration.ARC_SPEED_METERS_PER_SECOND
    facing_wall      = RobotConfiguration.FRONT_SENSOR_READING_WHEN_CELL_CENTER_FACES_WALL_METERS
    cruise           = RobotConfiguration.MAZE_CRUISE_SPEED_METERS_PER_SECOND
    braking          = RobotConfiguration.MAZE_BRAKING_METERS_PER_SECOND_SQUARED
    timestep_seconds = robot.timestep_milliseconds / 1000.0
    readings         = robot.distance_readings_meters
    current_speed    = starting_speed
    wall_at_end      = False
    leg_start_time   = robot.simulation_time_seconds
    while True:
        remaining        = (target_x - robot.odometry_position_x_meters) * step_x + (target_y - robot.odometry_position_y_meters) * step_y
        sensor_remaining = readings["ds_f0"] - facing_wall
        wall_at_end      = False
        if wall_blocks_leg(sensor_remaining, remaining, abs(MathHelpers.wrap_angle_to_plus_minus_pi(target_yaw - robot.yaw_radians)), readings["ds_f0"]):
            robot.stop_motors()
            raise LegAborted("wall")
        if robot.simulation_time_seconds - leg_start_time > RobotConfiguration.LEG_TIMEOUT_SECONDS:
            robot.stop_motors()
            raise LegAborted("timeout")
        if not arc_entry and abs(sensor_remaining - remaining) < 0.05 and abs(MathHelpers.wrap_angle_to_plus_minus_pi(target_yaw - robot.yaw_radians)) < 0.1:
            remaining   = sensor_remaining    # only trust the wall if it agrees with odometry (else it belongs to the next cell)
            wall_at_end = True
        if remaining <= 0.004: break
        desired_speed = cruise
        if stop_at_end or wall_at_end: desired_speed = min(cruise, max(0.03, math.sqrt(2.0 * braking * remaining)))
        if arc_entry: desired_speed = min(cruise, math.sqrt(arc_speed * arc_speed + 2.0 * braking * max(remaining, 0.0)))     # slow down to the arc speed exactly where the arc starts
        current_speed = profile.limit_acceleration(current_speed, desired_speed, timestep_seconds)
        robot.drive_with_forward_speed_and_turn_rate(current_speed, corridor_turn_rate(robot, target_yaw))
        if not robot.step_simulation(): return (False, current_speed)
        if robot.is_stalled():
            log_progress(robot, "maze move stalled: backing off, rotating and scanning")
            if not robot.back_off_and_scan(): return (False, 0.0)
            current_speed = 0.0
    if stop_at_end or wall_at_end:
        robot.stop_motors()
        return (True, 0.0)
    return (True, current_speed)


def drive_arc_through_cell(robot, profile, knowledge, cell, direction_in, direction_out, starting_speed):
    """Quarter-circle turn through `cell`: enters at the middle of the entry edge heading direction_in, leaves at the middle of the exit edge heading
    direction_out. The circle (radius half a cell) is centred on the cell corner on the inside of the turn, so the path stays 12.5 cm from that corner.
    Feed-forward turn rate v / R plus feedback on heading and distance from the circle. Returns (still_running, speed_on_leaving)."""
    radius              = RobotConfiguration.MAZE_CELL_PITCH_METERS / 2.0
    in_x, in_y          = MathHelpers.axis_step(direction_in)
    out_x, out_y        = MathHelpers.axis_step(direction_out)
    center_x, center_y  = knowledge.cell_center(cell)
    arc_center_x        = center_x - radius * in_x + radius * out_x
    arc_center_y        = center_y - radius * in_y + radius * out_y
    turn_sign           = 1.0
    if in_x * out_y - in_y * out_x < 0.0: turn_sign = -1.0         # right turn
    yaw_in              = MathHelpers.axis_yaw(direction_in)
    arc_speed           = RobotConfiguration.ARC_SPEED_METERS_PER_SECOND
    timestep_seconds    = robot.timestep_milliseconds / 1000.0
    current_speed       = starting_speed
    while True:
        progress = turn_sign * MathHelpers.wrap_angle_to_plus_minus_pi(robot.yaw_radians - yaw_in)
        if progress >= math.pi / 2.0 - 0.03: break
        offset_x      = robot.odometry_position_x_meters - arc_center_x
        offset_y      = robot.odometry_position_y_meters - arc_center_y
        radial_error  = math.hypot(offset_x, offset_y) - radius                                   # positive = outside the circle
        tangent_yaw   = math.atan2(offset_y, offset_x) + turn_sign * math.pi / 2.0
        heading_error = MathHelpers.wrap_angle_to_plus_minus_pi(tangent_yaw - robot.yaw_radians)
        current_speed = profile.limit_acceleration(current_speed, arc_speed, timestep_seconds)
        turn_rate     = turn_sign * current_speed / radius + RobotConfiguration.ARC_HEADING_GAIN * heading_error + turn_sign * RobotConfiguration.ARC_RADIAL_GAIN * radial_error
        robot.drive_with_forward_speed_and_turn_rate(current_speed, MathHelpers.clamp_value(turn_rate, -3.0, 3.0))
        if not robot.step_simulation(): return (False, current_speed)
        if robot.is_stalled():
            log_progress(robot, "arc stalled: backing off, rotating and scanning")
            if not robot.back_off_and_scan(): return (False, 0.0)
            return (True, 0.0)
    return (True, current_speed)


def recover_from_aborted_leg(robot, knowledge, direction_index, reason):
    """A leg stopped early. Work out which cell the robot is in, learn the wall if that was the cause, return to the cell centre and let the planner
    choose again. Returns what follow_route returns."""
    here = knowledge.cell_at(robot.odometry_position_x_meters, robot.odometry_position_y_meters)
    knowledge.aborted_legs += 1
    if reason == "wall": knowledge.record_wall(here, direction_index, True)
    log_progress(robot, "leg aborted (%s) in cell %s heading %s" % (reason, here, "ENWS"[direction_index]))
    if not back_up_to_cell_center(robot, knowledge, here): return (False, here, direction_index, 0.0)
    heading_index = MathHelpers.nearest_axis_index(robot.yaw_radians)
    if here not in knowledge.visited_cells: sense_new_cell(robot, knowledge, here, heading_index, False)     # a cell must be sensed before the planner can leave it
    return (True, here, heading_index, 0.0)


def follow_route(robot, profile, knowledge, cell, route, speed, stop_at_last_cell):
    """Drives the directions in order without stopping on straight stretches; 90-degree turns through already-visited cells are taken as arcs.
    Returns (still_running, new_cell, new_heading_index, speed)."""
    heading_index    = MathHelpers.nearest_axis_index(robot.yaw_radians)
    just_left_an_arc = False
    for index, direction_index in enumerate(route):
        is_last     = index == len(route) - 1
        stop_at_end = stop_at_last_cell
        take_arc    = False
        if not is_last:
            stop_at_end = route[index + 1] != direction_index
            take_arc    = bool(RobotConfiguration.ARC_CORNERS_ENABLED) and (route[index + 1] - direction_index) % 4 in (1, 3)
        if take_arc: stop_at_end = False
        try:
            still_running, speed = drive_to_next_cell(robot, profile, knowledge, cell, direction_index, stop_at_end, speed, take_arc, just_left_an_arc)
        except LegAborted as aborted:
            return recover_from_aborted_leg(robot, knowledge, direction_index, aborted.reason)
        if not still_running: return (False, cell, direction_index, speed)
        cell          = MazeKnowledge.neighbor(cell, direction_index)
        heading_index = direction_index
        knowledge.move_count += 1
        if cell not in knowledge.visited_cells: sense_new_cell(robot, knowledge, cell, heading_index, False)
        just_left_an_arc = False
        if not take_arc and index == len(route) - 2 and MazeKnowledge.neighbor(cell, route[index + 1]) in knowledge.peeked_cells and not stop_at_last_cell:     # the cell this route was heading for was just read from here
            if not brake_to_stop(robot, profile, knowledge, speed): return (False, cell, heading_index, 0.0)
            return (True, cell, heading_index, 0.0)
        if take_arc:
            still_running, speed = drive_arc_through_cell(robot, profile, knowledge, cell, direction_index, route[index + 1], speed)
            if not still_running: return (False, cell, route[index + 1], speed)
            just_left_an_arc = True
    return (True, cell, heading_index, speed)


def dump_knowledge(robot, knowledge, label):
    """Debug only (PRINT_PROGRESS): prints everything the robot has learned as one JSON line, for the comparison tests."""
    if not RobotConfiguration.PRINT_PROGRESS: return
    import json
    print("KNOWLEDGE " + json.dumps({"label": label, "time": round(robot.simulation_time_seconds, 1), "moves": knowledge.move_count,
        "visited": sorted(knowledge.visited_cells), "walls": {"%d,%d,%d" % (cell[0], cell[1], direction): is_wall for (cell, direction), is_wall in knowledge.wall_between.items()},
        "tiles": {"%d,%d" % cell: colour for cell, colour in knowledge.tile_colour_by_cell.items()}, "end": knowledge.end_cell,
        "conflicts": [[list(cell), direction, old, new] for cell, direction, old, new in knowledge.conflicts], "visit_order": [[t, list(cell)] for t, cell in knowledge.visit_order]}), flush=True)


def run_section_2_maze(robot, profile, entrance_is_closed):
    """Explores the maze, counts blue and green tiles, goes to the red end cell and lights the majority LED.
    entrance_is_closed: True when the robot came in through the transition corridor, so the way back out is treated as a wall."""
    knowledge          = MazeKnowledge()
    knowledge.origin_x = robot.odometry_position_x_meters
    knowledge.origin_y = robot.odometry_position_y_meters
    cell               = (0, 0)
    heading_index      = MathHelpers.nearest_axis_index(robot.yaw_radians)
    speed              = 0.0
    sense_new_cell(robot, knowledge, cell, heading_index, True)
    if entrance_is_closed: knowledge.close_edge(cell, (heading_index + 2) % 4)
    guard = LoopGuard()
    while True:
        if RobotConfiguration.EARLY_STOP_WHEN_MAJORITY_DECIDED and knowledge.end_cell is not None:
            unvisited = RobotConfiguration.MAZE_COLUMNS * RobotConfiguration.MAZE_ROWS - len(knowledge.visited_cells)
            if majority_is_decided(knowledge.tile_count("BLUE"), knowledge.tile_count("GREEN"), unvisited) and knowledge.find_route(cell, heading_index, knowledge.end_cell) is not None:
                log_progress(robot, "majority decided early (blue %d, green %d, %d cells unvisited)" % (knowledge.tile_count("BLUE"), knowledge.tile_count("GREEN"), unvisited))
                break
        route = knowledge.find_route(cell, heading_index, None, guard.banned_cells(robot.simulation_time_seconds))
        if route is None: break
        if not route:                                                   # already standing on an unvisited cell: sense it
            sense_new_cell(robot, knowledge, cell, heading_index, False)
            continue
        target         = cell
        for direction_index in route: target = MazeKnowledge.neighbor(target, direction_index)
        aborted_before = knowledge.aborted_legs
        still_running, cell, heading_index, speed = follow_route(robot, profile, knowledge, cell, route, speed, False)
        if not still_running: return False
        if knowledge.aborted_legs != aborted_before and guard.note(target, robot.simulation_time_seconds): log_progress(robot, "target %s keeps failing: skipping it for %.0f s" % (target, RobotConfiguration.LOOP_GUARD_BAN_SECONDS))
    robot.stop_motors()
    speed = 0.0
    dump_knowledge(robot, knowledge, "exploration_complete")
    log_progress(robot, "exploration complete: %d cells, blue %d, green %d, end cell %s" % (len(knowledge.visited_cells), knowledge.tile_count("BLUE"), knowledge.tile_count("GREEN"), knowledge.end_cell))
    if knowledge.end_cell is not None and cell != knowledge.end_cell:
        route = knowledge.find_route(cell, heading_index, knowledge.end_cell)
        if route is not None:
            still_running, cell, heading_index, speed = follow_route(robot, profile, knowledge, cell, route, speed, True)
            if not still_running: return False
    robot.stop_motors()
    blue_count  = knowledge.tile_count("BLUE")
    green_count = knowledge.tile_count("GREEN")
    majority    = RobotConfiguration.TIE_DEFAULT_COLOR
    if blue_count > green_count: majority = "BLUE"
    if green_count > blue_count: majority = "GREEN"
    robot.light_majority_led(majority)
    dump_knowledge(robot, knowledge, "done")
    log_progress(robot, "DONE at cell %s: LED %s (blue %d, green %d)" % (cell, majority, blue_count, green_count))
    return True


# ----------------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------------
ENABLE_RUN_MONITOR = False    # True = write run_log.csv (needs run_monitor.py). Keep False for the submission, then delete run_monitor.py


def detect_start_mode(robot):
    """'SECTION_1' (white field or Red A), 'TRANSITION' (Red B: red with walls on both sides) or 'MAZE' (black floor)."""
    for _ in range(3):
        if not robot.step_simulation(): return "SECTION_1"
    colour    = robot.classify_floor_color()
    threshold = RobotConfiguration.WALL_DETECTION_THRESHOLD_METERS
    readings  = robot.distance_readings_meters
    if colour == "BLACK": return "MAZE"
    if colour == "RED" and readings["ds_l"] < threshold and readings["ds_r"] < threshold: return "TRANSITION"
    return "SECTION_1"


def main():
    occupancy_grid = OccupancyGrid()
    mapper         = OccupancyMapper(occupancy_grid)
    robot_builder  = RobotBuilder().with_step_observer(mapper)
    if ENABLE_RUN_MONITOR:
        from run_monitor import RunMonitor    # imported only when enabled: delete run_monitor.py for the submission
        robot_builder.with_step_observer(RunMonitor("run_log.csv"))
    robot   = robot_builder.build()
    profile = MotionProfileBuilder().with_maximum_speed(RobotConfiguration.MAXIMUM_SPEED_METERS_PER_SECOND).with_acceleration_limit(RobotConfiguration.ACCELERATION_LIMIT_METERS_PER_SECOND_SQUARED).with_deceleration_limit(RobotConfiguration.DECELERATION_LIMIT_METERS_PER_SECOND_SQUARED).with_clearance_slow_down(RobotConfiguration.SLOW_DOWN_START_DISTANCE_METERS, RobotConfiguration.STOP_DISTANCE_METERS).build()
    planner = GridPathPlanner(occupancy_grid)
    mode    = detect_start_mode(robot)
    entered_through_transition = mode != "MAZE"
    section_1_start            = (robot.odometry_position_x_meters, robot.odometry_position_y_meters)
    log_progress(robot, "start mode: %s" % mode)
    for _ in range(4):                                # a false red or a bad approach sends the robot back to Section 1 instead of trapping it
        if mode == "SECTION_1":
            robot.phase_label = "SECTION_1"
            if run_section_1_obstacle_field(robot, occupancy_grid, planner, profile, section_1_start): mode = "TRANSITION"
            else: break
        if mode == "TRANSITION":
            robot.phase_label = "CORRIDOR"
            entered = enter_maze_through_transition(robot, profile)
            if entered == "NOT_IN_CORRIDOR": mode = "SECTION_1"
            elif entered: mode = "MAZE"
            else: break
        if mode == "MAZE": break
    if mode == "MAZE":
        mapper.enabled = False    # the maze uses its own cell graph; skip the per-step mapping
        robot.phase_label = "MAZE"
        run_section_2_maze(robot, profile, entered_through_transition)
        robot.phase_label = "DONE"
    while robot.step_simulation():     # keep the simulation alive, robot stopped
        robot.stop_motors()


if __name__ == "__main__":
    main()
