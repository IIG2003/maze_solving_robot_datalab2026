# PeraBots 2026 – Phase 1 (Simulation) Full Process

> Source: *PeraBots 2026 Rulebook – University Category* §4, plus the provided package `PeraBots_UC/`.
> Webots version: **R2025a**. Submission deadline: **October 28**.

---

## 0. Timeline (today → deadline)

| Dates | Goal |
|---|---|
| Sep 29 – Oct 3 | Robot sensors added, odometry + motion primitives working, camera colour classification tuned |
| Oct 4 – Oct 10 | Section 1 (obstacle field) working reliably |
| Oct 11 – Oct 18 | Section 2 (maze exploration, tile counting, end cell, LED) working |
| Oct 19 – Oct 24 | Continuous run S1 → S2, random test worlds, robustness, stuck watchdog |
| Oct 25 – Oct 27 | Freeze code, final checks, build the zip |
| **Oct 28** | **Submit** |

---

## 1. What the task is

1. **Section 1 – Obstacle navigation** (white floor, 2.44 m × 1.22 m)
   Start in the 25 × 25 cm red start zone → drive through a field of 10 cm cubes (passages ≥ 25 cm) → reach the red transition zone into the maze.
   Scored on: success, time, distance, collisions.
2. **Section 2 – Maze** (black floor, 9 × 4 cells, 25 cm pitch, 1.2 cm walls, 23.8 cm clear per cell)
   Explore → find blue/green tiles at dead ends → count them → decide majority → drive to the end cell (whole body inside) → **within 3 s** light the correct LED (`led_blue` / `led_green`) and stop.
   Scored on: exploration, time, distance, wall collisions, correct tile detection, correct majority, reaching end cell, LED timing.
3. **Bonus marks**: autonomous S1 → S2 transition, and a full continuous run start → finish.
4. **Evaluation**: 3 attempts for S1, 3 dedicated attempts for S2 (robot may be placed at the maze entrance by the judges). Best run, consistency and overall performance all count.
5. **Attempt terminated (= failed)** if:
   - no meaningful progress for **15 consecutive seconds** (stuck, looping, spinning), or
   - whole body **outside the track for > 2 s**.

The official evaluation world has a **different obstacle layout, different maze walls and different tile positions**. The arena (floors, outer walls, start/transition/end markers) and dimensions stay the same.

---

## 2. Rules checklist

### Must
- [ ] Use the provided `TwoWheelRobot.proto` base robot.
- [ ] Keep robot within **20 × 20 × 20 cm**.
- [ ] Keep `led_blue` and `led_green` exactly as provided and use them **only** for the majority colour.
- [ ] Fully autonomous once started.
- [ ] Keep names: `TwoWheelRobot.proto`, `controllers/main_controller/`, `main_controller.py`.
- [ ] All extra code/files inside `controllers/main_controller/`.
- [ ] Submit as `TeamName_Submission.zip` (team name only on the outer zip).

### Must NOT
- [ ] Modify, unlock, or replace `Arena.proto` / `Obstacles.proto`.
- [ ] Change chassis, motors, wheel size, physics, collision geometry (only small wheel *position* tweaks for balance).
- [ ] Use **GPS, Emitter, Receiver**, or enable Supervisor.
- [ ] Hard-code obstacle positions, maze walls, tile locations or a fixed route.

### Allowed devices
DistanceSensor, Camera, TouchSensor, Accelerometer, Gyroscope, InertialUnit, LightSensor, PositionSensor (for added joints), Speaker, extra LEDs (debug/cosmetic).

---

## 3. Facts extracted from the package

### Robot (`protos/TwoWheelRobot.proto`)
| Item | Value |
|---|---|
| Chassis box | 0.08 × 0.05 × 0.03 m, centred at z = 0.04 (front face x = +0.04, top z = 0.055) |
| Wheels | radius **0.02 m**, width 0.01 m, at y = ±0.032 → track width **0.064 m** |
| Caster | sphere r = 0.008 at x = −0.03 |
| Motors | `left_wheel_motor`, `right_wheel_motor` |
| Encoders | `left_wheel_sensor`, `right_wheel_sensor` |
| LEDs | `led_blue` (x 0.02, y +0.012), `led_green` (x 0.02, y −0.012) |
| Forward direction | robot local **+x** |
| Sensor mount point | `extensionSlot` |

The robot is small (8 × 6.4 cm footprint), which leaves a lot of room in 25 cm passages.

### Arena (`protos/Arena.proto`, world frame, Z up)
| Item | Position (approx.) |
|---|---|
| Section 1 floor (white) | x 0 → 2.44, y 0 → 1.22 |
| Section 2 floor (black 0.02) | x 0 → 2.44, y 1.22 → 2.44 |
| **Start marker** (red) | centre (2.315, 0.125), bottom-right corner |
| Opening in S1 top wall | x ≈ 0.10 → 0.345 at y = 1.22 |
| **Transition tile** (red) | centre (0.22, 1.275) |
| Maze cells | 9 columns × 4 rows; column centre x ≈ 0.22 + 0.25·(c−1), row centre y ≈ 1.455 + 0.25·(r−1) |
| Maze entrance | into cell r1 c1 from below |
| **End marker** (red) | centre (2.21, 2.205) → cell r4 c9 (top-right) |
| Tile colours | blue = (0, 0, 0.5), green = (0, 0.6, 0), red = (1, 0, 0) |
| Maze walls | white, 0.1 m high |
| Obstacles | black 10 cm cubes |

These are fixed **Arena** features, so using them is fine. Obstacles, internal maze walls and tiles must be discovered at runtime.
*(Confirm with coordinators that using the fixed start / transition / end locations is allowed – see §13.)*

---

## 4. Robot configuration (sensors)

Add everything inside the robot's `extensionSlot`. Recommended set:

| Device | Name | Pose (robot frame) | Purpose |
|---|---|---|---|
| DistanceSensor | `ds_front` | (0.04, 0, 0.04), yaw 0 | wall/obstacle ahead |
| DistanceSensor | `ds_fl` | (0.04, 0.02, 0.04), yaw +45° | front-left diagonal |
| DistanceSensor | `ds_fr` | (0.04, −0.02, 0.04), yaw −45° | front-right diagonal |
| DistanceSensor | `ds_left` | (0, 0.025, 0.04), yaw +90° | left wall |
| DistanceSensor | `ds_right` | (0, −0.025, 0.04), yaw −90° | right wall |
| DistanceSensor | `ds_back` (optional) | (−0.04, 0, 0.04), yaw 180° | reversing safety |
| InertialUnit | `imu` | (0, 0, 0.04) | absolute heading (yaw) |
| Gyroscope | `gyro` (optional) | (0, 0, 0.04) | smooth turning control |
| Camera | `cam` | (0.04, 0, 0.06), pitched down ~40° | floor colour + next-cell tile |

### Example `extensionSlot` content

```vrml
extensionSlot [
  DistanceSensor {
    translation 0.04 0 0.04
    name "ds_front"
    lookupTable [ 0 0 0, 0.6 0.6 0 ]      # returns metres, max 0.6 m
    numberOfRays 1
  }
  DistanceSensor {
    translation 0.04 0.02 0.04
    rotation 0 0 1 0.7854
    name "ds_fl"
    lookupTable [ 0 0 0, 0.6 0.6 0 ]
  }
  DistanceSensor {
    translation 0.04 -0.02 0.04
    rotation 0 0 1 -0.7854
    name "ds_fr"
    lookupTable [ 0 0 0, 0.6 0.6 0 ]
  }
  DistanceSensor {
    translation 0 0.025 0.04
    rotation 0 0 1 1.5708
    name "ds_left"
    lookupTable [ 0 0 0, 0.6 0.6 0 ]
  }
  DistanceSensor {
    translation 0 -0.025 0.04
    rotation 0 0 1 -1.5708
    name "ds_right"
    lookupTable [ 0 0 0, 0.6 0.6 0 ]
  }
  InertialUnit {
    translation 0 0 0.04
    name "imu"
  }
  Gyro {
    translation 0 0 0.04
    name "gyro"
  }
  Camera {
    translation 0.04 0 0.06
    rotation 0 1 0 0.7                    # tilt ~40° down
    name "cam"
    fieldOfView 1.0
    width 32
    height 24
  }
]
```

Notes:
- `lookupTable [0 0 0, 0.6 0.6 0]` makes `getValue()` return distance in metres. At max range it returns 0.6, meaning nothing was detected.
- Sensors at z = 0.04 hit both the 10 cm cubes and the 10 cm maze walls.
- After adding sensors, check that the robot still sits level. If it tips, shift the wheels slightly (the only mechanical change allowed).
- The height must stay ≤ 20 cm. This setup is about 7 cm tall.

---

## 5. Software architecture

Everything lives in `controllers/main_controller/`:

```
controllers/main_controller/
├── main_controller.py   # entry point + top-level state machine
├── config.py            # all tunable constants (speeds, thresholds, gains)
├── hardware.py          # device setup, sensor reading helpers
├── odometry.py          # pose (x, y, θ) from encoders + IMU
├── motion.py            # drive_to_point, turn_to_heading, drive_distance, stop
├── vision.py            # camera colour classification (near floor / next cell)
├── section1.py          # occupancy grid + A* planner for the obstacle field
├── section2.py          # maze map, exploration, tile counting, BFS to end
└── watchdog.py          # stuck / no-progress detection and recovery
```

### Top-level state machine

```
INIT
 ├─ floor colour = white  → S1_NAVIGATE
 └─ floor colour = black/red at maze entrance → S2_ENTER   (judges placed robot at maze entrance)

S1_NAVIGATE  → (red transition tile seen near (0.22, 1.275)) → TRANSITION
TRANSITION   → drive straight into cell r1c1 centre → S2_EXPLORE
S2_EXPLORE   → (exploration finished / majority decided) → S2_GO_END
S2_GO_END    → (inside end cell, centred) → FINISH
FINISH       → motors 0, correct LED on, loop forever
```

**At startup, turn both LEDs OFF.** The sample controller turns both on.

---

## 6. Localization (no GPS)

- **Heading θ:** use `imu.getRollPitchYaw()[2]`. It is absolute in the world frame and doesn't drift like pure encoder heading.
- **Distance:** encoder deltas. `d_left = Δφ_left · 0.02`, `d_right = Δφ_right · 0.02`, `d = (d_left + d_right)/2`.
- **Update:** `x += d·cos θ`, `y += d·sin θ`.
- **Initial pose:**
  - Starting in S1: start zone centre (2.315, 0.125) with θ read from the IMU.
  - Starting in S2 (judge placement): maze entrance, the transition tile (0.22, 1.275) or cell r1c1.
- **Drift correction (important in the maze):**
  - When stopped at a cell centre with a wall in front, set the position along that axis from `ds_front`. The wall centreline is at the cell edge, so the wall face is 0.119 m from the cell centre.
  - When there are walls on both sides, centre laterally from `ds_left` − `ds_right`.
  - Snap θ to the nearest 90° grid direction when you're sure you're aligned.
  - Red markers seen by the camera give absolute position fixes.

---

## 7. Motion primitives (`motion.py`)

| Function | Behaviour |
|---|---|
| `turn_to(θ_target)` | P or PD control on the IMU yaw error, spinning in place; stop when \|error\| < ~1.5° |
| `drive_to(x, y)` | turn toward the target, then drive forward with heading correction; slow down near the target |
| `drive_cells(n)` | maze: drive n × 0.25 m while keeping centred between walls (wall-following correction) |
| `stop()` | both velocities 0 |

Tips:
- Max wheel speed about 6 rad/s ≈ 0.12 m/s. Start around 4–5 rad/s, then raise it once things are stable.
- Ramp speed up and down to prevent wheel slip, which ruins odometry.
- Emergency stop: if `ds_front` < 0.03 m, stop and re-plan. Never touch a cube or wall (every touch counts as a collision).

---

## 8. Vision – one camera for both jobs (`vision.py`)

A forward camera tilted down about 40° sees:
- **Bottom rows** → floor just ahead: white / black / red (start, transition, end).
- **Upper rows** → floor of the **next cell** (about 8–33 cm ahead): detects blue or green tiles without entering the cell.

```python
def classify(r, g, b):
    if r > 150 and g < 80 and b < 80:   return 'red'
    if b > 80  and r < 60 and g < 60:   return 'blue'
    if g > 100 and r < 60 and b < 60:   return 'green'
    if r > 180 and g > 180 and b > 180: return 'white'
    if r < 40 and g < 40 and b < 40:    return 'black'
    return None

def region_colour(cam, row0, row1):
    img = cam.getImage(); w = cam.getWidth()
    votes = {}
    for y in range(row0, row1):
        for x in range(w // 4, 3 * w // 4):     # centre columns only
            c = classify(cam.imageGetRed(img, w, x, y),
                         cam.imageGetGreen(img, w, x, y),
                         cam.imageGetBlue(img, w, x, y))
            if c: votes[c] = votes.get(c, 0) + 1
    return max(votes, key=votes.get) if votes else None

near  = region_colour(cam, 18, 24)   # floor right in front
ahead = region_colour(cam, 6, 12)    # next cell's floor
```

Rules:
- Tune the row ranges and thresholds by printing RGB values in the sample world.
- Only trust `ahead` when **stopped at a cell centre and aligned to the grid**.
- In the maze, ignore `white` (it's the walls).
- Store tiles **by cell**, e.g. `tiles[(r, c)] = 'blue'`, so the same tile is never counted twice.
- Confirm a detection over 2–3 frames before accepting it.

---

## 9. Section 1 – Obstacle navigation (`section1.py`)

### Approach: occupancy grid + A* with continuous re-planning

1. **Grid:** 5 cm cells covering 2.44 × 1.22 m → 49 × 25 grid. Mark the outer walls as occupied.
2. **Mapping:** each control step, for every distance sensor with reading < max range, project the hit point into world coordinates using the robot pose and the sensor's mounting pose, and mark that cell as occupied. Optionally mark free cells along the ray.
3. **Inflation:** inflate obstacles by the robot radius (about 5 cm) plus a margin (about 2–3 cm). Passages are ≥ 25 cm, so an inflated path still fits.
4. **Goal:** point just below the opening, e.g. (0.22, 1.15), then (0.22, 1.275) on the transition tile.
5. **Plan:** A* (8-connected, diagonal cost √2) from the current cell to the goal. Unknown cells count as free (optimistic), so the robot heads straight to the goal and re-plans when it discovers obstacles.
6. **Follow:** simplify the path into waypoints (drop collinear points, use line-of-sight smoothing) and call `drive_to` for each waypoint.
7. **Re-plan** whenever a newly marked obstacle blocks the current path, or every N steps.
8. **Done** when the camera `near` region sees red near the transition coordinates, or the pose is within about 5 cm of the tile.

Why not simple wall-following: it works, but produces long paths. Distance and time are scored, so A* toward the known goal gives much shorter runs.

Fallback: if A* finds no path (e.g. the map is corrupted by drift), clear the unknown/obstacle memory around the robot and re-plan. If that fails too, switch to a Bug2 reactive mode toward the goal.

---

## 10. Transition S1 → S2

1. From the transition tile, face +y (θ = 90°).
2. Drive forward into the centre of maze cell **r1 c1**, about (0.22, 1.455).
3. Reset or refine the pose using the side walls (the cell is 23.8 cm wide).
4. Switch to cell-based navigation.

If the judges place the robot at the maze entrance, INIT detects black/red floor and starts here directly.

---

## 11. Section 2 – Maze exploration (`section2.py`)

### Data structures

```python
ROWS, COLS = 4, 9
walls = {}        # walls[(r, c, dir)] = True / False / None(unknown); dir in N,E,S,W
visited = set()
tiles = {}        # (r, c) -> 'blue' / 'green'
END = (4, 9)      # top-right cell (from Arena end marker)
```

Walls are shared: `walls[(r,c,'N')] == walls[(r+1,c,'S')]`. Always set both sides.
The outer boundary walls are known from the start.

### At every cell centre (robot stopped and aligned)

1. Read `ds_front`, `ds_left`, `ds_right`. A reading < ~0.18 m means a wall on that side, otherwise open. Record in `walls`.
2. (Optional) Read `ds_front` further: if the next cell's far wall is visible (≈ 0.37 m), you learn the next cell's front wall too.
3. Camera `ahead` → if blue/green and there's no wall in front, `tiles[next_cell] = colour`.
4. Camera `near`/floor under the robot → if blue/green, `tiles[current_cell] = colour`.
5. Mark the current cell visited.

### Exploration strategy

```
while True:
    targets = cells that are reachable through known-open walls
              and are not yet "resolved"
    if no targets: break
    go to the nearest target by BFS (on known-open edges)
```

A cell is **resolved** when all four of its walls are known **and** its tile status is known (seen directly or through the camera from a neighbour).

Distance savers:
- Don't enter a dead end if the camera already saw its tile colour (or no tile) from the neighbouring cell. You only need to know its walls, which you usually do from the neighbour's sensors plus a look down the corridor.
- Choose the target with the **shortest BFS path**, and on ties prefer continuing straight (turns cost time).
- **Early stop:** let `U` = number of possible dead ends not yet checked. If `|blue − green| > U`, the majority is decided and you can go to the end straight away. Only do this if full exploration isn't scored higher; the rulebook scores "successful and sufficient exploration", so test both options.

### Majority decision

```python
blue  = sum(1 for v in tiles.values() if v == 'blue')
green = sum(1 for v in tiles.values() if v == 'green')
majority = 'blue' if blue > green else 'green'
```

(A tie is not defined in the rulebook. Ask the organizers, see §13.)

### Go to the end cell

1. BFS on known-open edges from the current cell to `END` (4, 9).
2. Drive the path cell by cell (merge straight runs into one `drive_cells(n)`).
3. Centre precisely inside the end cell. The **whole body** must be inside the 23.8 × 23.8 cm clear area, and the robot footprint is about 8 × 6.4 cm, so there's lots of margin as long as you're centred.
4. Optional check: the camera `near` region sees **red** (end marker).

---

## 12. Finish (≤ 3 s after entering the end cell)

```python
left_motor.setVelocity(0); right_motor.setVelocity(0)
led_blue.set(1 if majority == 'blue' else 0)
led_green.set(1 if majority == 'green' else 0)
while robot.step(TIME_STEP) != -1:
    pass   # stay stopped, LED on
```

Switch the LED on as soon as the robot is inside the cell. Don't spend the 3 s on fine centring.

---

## 13. Watchdog / anti-termination (`watchdog.py`)

- Store the pose every 1 s. If the robot moved < 2 cm **and** turned < 10° in the last **4 s**, it's stuck:
  1. stop, reverse 5 cm,
  2. mark the blocking cell as an obstacle,
  3. re-plan.
- Detect loops: if the same waypoint/cell has been the target more than 3 times in a row, blacklist it temporarily.
- Never spin in place for long. Turning in place may count as "no meaningful progress" (limit 15 s).
- Stay inside the track: never plan outside the arena bounds.

---

## 14. Testing plan

### Unit tests (in the sample world)
- [ ] Encoder odometry: drive 1 m straight and compare with the robot's real position (read from the scene tree during development only).
- [ ] `turn_to` accuracy over 20 turns.
- [ ] Distance sensor readings vs. known wall distances.
- [ ] Camera colour classification on each floor and tile colour.

### Randomised worlds (important, since the official world is different)
1. Copy `Arena.wbt` → `worlds/test_*.wbt`.
2. Make **copies** of the protos under new names, e.g. `protos/TestObstacles1.proto` and `protos/TestArena1.proto`, and change cube positions, internal maze walls and tile positions.
3. Point the test worlds at the copies (`EXTERNPROTO "../protos/TestObstacles1.proto"`).
4. Build at least **5 obstacle layouts** and **5 mazes**, including: narrow 25 cm passages, dead-end pockets in S1, mazes with 1 tile, many tiles, and tiles only in far corners.
5. **Do not include** the test worlds or test protos in the submission (or at least make sure the main `Arena.wbt` still uses the originals).

### Metrics to log per run
Time for S1, time for S2, total distance (sum of encoder distance), collisions (sensor reading < ~1 cm or velocity drop), detected tiles vs. truth, correct LED.

### Run modes to test
- [ ] Full continuous run: start zone → end cell.
- [ ] S2 only: robot placed at the maze entrance.
- [ ] Different start headings in the start zone.
- [ ] Fast mode (`Ctrl+3`) many times to catch rare failures.

---

## 15. Tuning list (`config.py`)

```python
TIME_STEP        = 16          # match world basicTimeStep
MAX_SPEED        = 6.0         # rad/s
CRUISE_SPEED     = 5.0
TURN_SPEED       = 3.0
WHEEL_RADIUS     = 0.02
AXLE_TRACK       = 0.064
CELL             = 0.25
WALL_THRESH      = 0.18        # m, wall present if closer than this (from cell centre)
S1_GRID_RES      = 0.05
INFLATE_RADIUS   = 0.07
STOP_DIST        = 0.03
STUCK_WINDOW_S   = 4.0
CAM_NEAR_ROWS    = (18, 24)
CAM_AHEAD_ROWS   = (6, 12)
```

---

## 16. Questions to ask the coordinators (via WhatsApp)

1. Can we use the **fixed arena positions** (start zone, transition tile, end cell) in the controller? They're part of the locked Arena.
2. What if blue and green counts are **equal**?
3. Is the robot's **starting orientation** in the start zone fixed or random?
4. In a S2-only attempt, exactly **where and how** is the robot placed (on the transition tile, or inside r1c1)?
5. Does early termination of exploration (majority mathematically decided) count as "sufficient exploration"?

---

## 17. Submission checklist

- [ ] `protos/Arena.proto` and `protos/Obstacles.proto` **byte-identical** to the originals (compare against `PeraBots_UC.zip`).
- [ ] `protos/TwoWheelRobot.proto`: only `extensionSlot` content, `bodyColor`, and at most small wheel position changes are different.
- [ ] `led_blue` / `led_green` unchanged (name, colour, position).
- [ ] No GPS / Emitter / Receiver anywhere; `supervisor FALSE` untouched.
- [ ] Robot ≤ 20 × 20 × 20 cm.
- [ ] `controllers/main_controller/main_controller.py` present and runs with a **fresh** Webots R2025a (no external pip packages that the judges won't have; stick to the standard library + `controller`).
- [ ] All helper `.py` files inside `controllers/main_controller/`.
- [ ] No debug keyboard control, no hard-coded routes or maze maps.
- [ ] `worlds/Arena.wbt` opens and the robot runs from the start zone (reset the robot pose in the world file to the start zone before saving).
- [ ] Zip the **whole project folder** → `TeamName_Submission.zip`.
- [ ] Unzip it into a fresh folder, open in Webots, and run once more before submitting.
- [ ] Follow any final submission instructions posted on the WhatsApp channel.
