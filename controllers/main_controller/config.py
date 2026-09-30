"""
config.py - every tunable constant for the PeraBots 2026 Phase 1 controller.

Only fixed, rulebook/Arena-defined facts live here (robot geometry, arena
outline, start / transition / end locations, maze grid). Nothing about the
obstacle layout, maze walls or tile positions is hard-coded - those are all
discovered at runtime.
"""
import math

DEBUG = True                # print progress messages to the Webots console
CAM_DEBUG = False           # print raw camera region colours (for tuning)

# ---------------------------------------------------------------------------
# Robot geometry (locked by TwoWheelRobot.proto)
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.02
AXLE_TRACK = 0.064
AXLE_X = 0.008              # wheels moved 0.8 cm forward in the proto (balance: CoM
                            # behind the axle so the robot rests on its caster)
MAX_WHEEL_SPEED = 10.0      # rad/s (RotationalMotor default maxVelocity)
WHEEL_ACCEL = 40.0          # rad/s^2 motor acceleration limit (anti nose-dive)
ROBOT_RADIUS = 0.05         # circle that encloses the robot footprint

# ---------------------------------------------------------------------------
# Sensors - name: (x, y, yaw) in the robot frame. Must match the proto.
# All distance sensors are mounted at z = 0.045 (above the wheel tops).
# ---------------------------------------------------------------------------
DS_MAX = 1.0                # lookupTable [0 0 0, 1 1 0] -> metres, 1.0 = no hit
DS_HIT = 0.97               # readings below this are real hits
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

# Offsets of the four axis sensors from the robot centre (used in the maze)
OFF_F = 0.041
OFF_B = 0.041
OFF_SIDE = 0.026

# Camera: 32 x 24, fieldOfView 1.0, mounted at (0.042, 0, 0.06), pitched 0.6 rad down.
# Rows 0..10 look at the NEXT cell (0.13-0.30 m ahead of the robot centre),
# rows 15..23 look at the floor of the CURRENT cell (0.08-0.11 m ahead).
CAM_AHEAD_ROWS = range(1, 9)
CAM_NEAR_ROWS = range(16, 24)
CAM_COL_FRAC = (0.25, 0.75)       # use only the centre half of the columns

# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------
S1_CRUISE = 8.0             # wheel rad/s in the obstacle field
S2_CRUISE = 8.0             # wheel rad/s in the maze
MIN_DRIVE = 1.5             # slowest forward wheel speed while approaching a target
ACCEL_TIME = 0.25           # s to ramp from 0 to cruise
TURN_MAX = 6.0
TURN_MIN = 0.5
TURN_K = 9.0
TURN_TOL = math.radians(1.2)
HEAD_K = 7.0                # heading-error gain while driving
CROSS_K = 5.0               # cross-track gain (rad per metre)
CROSS_MAX = 0.35            # max heading correction from cross-track (rad)

# ---------------------------------------------------------------------------
# Arena (fixed, locked Arena node) - world frame, metres, Z up
# ---------------------------------------------------------------------------
ARENA_W = 2.44              # x extent
S1_H = 1.22                 # Section 1 y extent (y 0 .. 1.22)
WALL_HALF = 0.006           # half wall thickness
START_CENTRE = (2.315, 0.125)
S1_GOAL = (0.22, 1.12)      # just below the gap in the Section 1 top wall
S1_GOAL_TOL = 0.035
CHAMBER_CENTRE = (0.22, 1.275)
S1_OPENING_X = (0.101, 0.339)

# Maze: row r = 1..4 (south -> north, +y), column c = 1..9 (west -> east, +x)
ROWS, COLS = 4, 9
CELL = 0.25
MAZE_X0 = 0.22              # x of column 1 centre
MAZE_Y0 = 1.455             # y of row 1 centre
ENTRY_CELL = (1, 1)         # entered from the south (transition chamber)
END_CELL = (4, 9)           # red end marker (2.21, 2.205)

# Wall face distance from a cell centre = CELL/2 - WALL_HALF
FACE = CELL / 2 - WALL_HALF          # 0.119
WALL_THRESH_F = 0.20        # axis sensor reading below this => wall in this cell
WALL_THRESH_S = 0.20

# ---------------------------------------------------------------------------
# Section 1 planner
# ---------------------------------------------------------------------------
S1_RES = 0.025              # occupancy grid cell size
FREE_RANGE = 0.6            # rays clear free space up to this distance
MAP_RANGE = 0.9             # only map hits closer than this
R_FORBID = 0.045            # centre closer than this to an obstacle = forbidden
R_HARD = 0.07               # closer than this = very expensive
R_SOFT = 0.13               # closer than this = mild penalty (keeps to passage centre)
HARD_COST = 25.0
SOFT_COST = 2.5
LOOKAHEAD = 0.10
REPLAN_PERIOD = 0.3
EMERGENCY_DIST = 0.035      # front sensor reading that triggers a back-off

# ---------------------------------------------------------------------------
# Section 2 logic
# ---------------------------------------------------------------------------
EARLY_STOP = False          # stop exploring once the majority can no longer change
TIE_COLOUR = 'blue'         # used if blue == green (not defined in the rulebook)
TURN_COST = 0.35            # planner cost of a 90 degree turn (in cells)
UNKNOWN_EDGE_COST = 1.5     # planner cost multiplier for an unknown edge

# ---------------------------------------------------------------------------
# Watchdog
# ---------------------------------------------------------------------------
STUCK_WINDOW = 3.0          # s
STUCK_MOVE = 0.02           # m
LOOP_LIMIT = 3              # same maze target failed this many times in a row -> ban it
LOOP_BAN = 30.0             # s a banned target stays banned
REACTIVE_TIME = 2.0         # s of reactive (Bug-style) driving when A* finds no path
