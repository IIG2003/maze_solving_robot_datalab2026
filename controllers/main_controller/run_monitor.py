"""
run_monitor.py - analytics only. Watches the robot each step and writes run_log.csv for tests/plot_run.py (speed, acceleration, tilt, clearance, ...).
It never commands the robot. Used only when ENABLE_RUN_MONITOR = True in main_controller.py.
Safe to delete for the submission (keep ENABLE_RUN_MONITOR = False).
"""

import csv
import math


class RunMonitor:
    CSV_COLUMN_NAMES = ["time_s", "phase", "x_m", "y_m", "yaw_rad", "distance_m", "commanded_speed_m_s", "measured_speed_m_s", "commanded_turn_rate_rad_s", "measured_turn_rate_rad_s",
                        "left_command_rad_s", "right_command_rad_s", "front_clearance_m", "closest_beam_m", "pitch_deg", "roll_deg", "stalled", "stall_recoveries"]

    def __init__(self, csv_file_path="run_log.csv", rows_between_flushes=100):
        self.csv_file_path        = csv_file_path
        self.rows_between_flushes = rows_between_flushes
        self.csv_file             = open(csv_file_path, "w", newline="")
        self.csv_writer           = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.CSV_COLUMN_NAMES)
        self.rows_written         = 0
        self.previous_yaw         = None
        self.peak_speed           = 0.0
        self.peak_tilt_degrees    = 0.0
        self.last_time_seconds    = 0.0
        self.last_distance        = 0.0
        self.last_stall_recoveries = 0

    @staticmethod
    def wheel_radius_and_track():
        from main_controller import RobotConfiguration              # imported lazily: this module is only used from inside main_controller
        return RobotConfiguration.WHEEL_RADIUS_METERS, RobotConfiguration.WHEEL_TRACK_WIDTH_METERS

    def on_step(self, robot):
        radius, track = self.wheel_radius_and_track()
        timestep_seconds = robot.timestep_milliseconds / 1000.0
        left_command     = robot.commanded_left_wheel_radians_per_second
        right_command    = robot.commanded_right_wheel_radians_per_second
        turn_rate        = 0.0
        if self.previous_yaw is not None: turn_rate = math.atan2(math.sin(robot.yaw_radians - self.previous_yaw), math.cos(robot.yaw_radians - self.previous_yaw)) / timestep_seconds
        self.previous_yaw = robot.yaw_radians
        pitch_degrees    = math.degrees(robot.pitch_radians)
        roll_degrees     = math.degrees(robot.roll_radians)
        stalled          = 0
        if robot.stalled_seconds > 0.0: stalled = 1
        self.csv_writer.writerow(["%.3f" % robot.simulation_time_seconds, robot.phase_label, "%.4f" % robot.odometry_position_x_meters, "%.4f" % robot.odometry_position_y_meters, "%.4f" % robot.yaw_radians,
                                  "%.4f" % robot.total_distance_travelled_meters, "%.4f" % (0.5 * (left_command + right_command) * radius), "%.4f" % robot.measured_forward_speed_meters_per_second,
                                  "%.4f" % ((right_command - left_command) * radius / track), "%.4f" % turn_rate, "%.3f" % left_command, "%.3f" % right_command, "%.4f" % robot.front_minimum_distance_meters(),
                                  "%.4f" % min(robot.distance_readings_meters.values()), "%.2f" % pitch_degrees, "%.2f" % roll_degrees, stalled, robot.stall_recoveries])
        self.rows_written += 1
        if self.rows_written % self.rows_between_flushes == 0: self.csv_file.flush()
        self.peak_speed            = max(self.peak_speed, abs(robot.measured_forward_speed_meters_per_second))
        self.peak_tilt_degrees     = max(self.peak_tilt_degrees, abs(pitch_degrees), abs(roll_degrees))
        self.last_time_seconds     = robot.simulation_time_seconds
        self.last_distance         = robot.total_distance_travelled_meters
        self.last_stall_recoveries = robot.stall_recoveries

    def on_simulation_end(self, robot):
        self.csv_file.flush()
        self.csv_file.close()
        print("RunMonitor summary: time %.1f s, distance %.2f m, peak speed %.3f m/s, peak tilt %.1f deg, stall recoveries %d, log: %s (chart it with: python3 tests/plot_run.py %s)" %
              (self.last_time_seconds, self.last_distance, self.peak_speed, self.peak_tilt_degrees, self.last_stall_recoveries, self.csv_file_path, self.csv_file_path))
