import math

DEBUG = True
CAM_DEBUG = False

WHEEL_RADIUS = 0.02
AXLE_TRACK = 0.064
AXLE_X = 0.008
MAX_WHEEL_SPEED = 10.0
WHEEL_ACCEL = 40.0
ROBOT_RADIUS = 0.05

DS_MAX = 1.0
DS_HIT = 0.97
DISTANCE_SENSORS = {
    'ds_f':    (0.041,  0.000, math.radians(0)),
    'ds_fl20': (0.041,  0.008, math.radians(20)),
    'ds_fr20': (0.041, -0.008, math.radians(-20)),
    'ds_fl45': (0.041,  0.018, math.radians(45)),
    'ds_fr45': (0.041, -0.018, math.radians(-45)),
    'ds_fl70': (0.030,  0.026, math.radians(70)),
    'ds_fr70': (0.030, -0.026, math.radians(-70)),
    'ds_l':    (0.000,  0.026, math.radians(90)),
    'ds_r':    (0.000, -0.026, math.radians(-90)),
    'ds_bl':   (-0.035, 0.026, math.radians(135)),
    'ds_br':   (-0.035, -0.026, math.radians(-135)),
    'ds_b':    (-0.041, 0.000, math.radians(180)),
}
FRONT_SENSORS = ('ds_f', 'ds_fl20', 'ds_fr20')

OFF_F = 0.041
OFF_B = 0.041
OFF_SIDE = 0.026

CAM_AHEAD_ROWS = range(1, 9)
CAM_NEAR_ROWS = range(16, 24)
CAM_COL_FRAC = (0.25, 0.75)

S1_CRUISE = 8.0
S2_CRUISE = 8.0
S2_KNOWN = 10.0
MIN_DRIVE = 1.5
ACCEL_TIME = 0.25
DRIVE_DECEL = 0.5
TURN_MAX = 6.0
TURN_MIN = 0.5
TURN_K = 9.0
TURN_K_END = 20.0
TURN_DECEL = 15.0
TURN_TOL = math.radians(1.2)
HEAD_K = 7.0
CROSS_K = 5.0
CROSS_MAX = 0.35

ARENA_W = 2.44
S1_H = 1.22
WALL_HALF = 0.006
START_CENTRE = (2.315, 0.125)
S1_GOAL = (0.22, 1.12)
S1_GOAL_TOL = 0.035
CHAMBER_CENTRE = (0.22, 1.275)
S1_OPENING_X = (0.101, 0.339)

ROWS, COLS = 4, 9
CELL = 0.25
MAZE_X0 = 0.22
MAZE_Y0 = 1.455
ENTRY_CELL = (1, 1)
END_CELL = (4, 9)

FACE = CELL / 2 - WALL_HALF
WALL_THRESH_F = 0.20
WALL_THRESH_S = 0.20

S1_RES = 0.025
FREE_RANGE = 0.6
MAP_RANGE = 0.9
R_FORBID = 0.045
R_HARD = 0.07
R_SOFT = 0.13
HARD_COST = 25.0
SOFT_COST = 2.5
LOOKAHEAD = 0.10
REPLAN_PERIOD = 0.3
EMERGENCY_DIST = 0.035

EARLY_STOP = False
TIE_COLOUR = 'blue'
TURN_COST = 0.35
UNKNOWN_EDGE_COST = 1.5
TARGET_ORDER = 'tour'
TOUR_EXACT = 8
GOAL_DEFER = 0.4

STUCK_WINDOW = 3.0
STUCK_MOVE = 0.02
LOOP_LIMIT = 3
LOOP_BAN = 30.0
REACTIVE_TIME = 2.0
