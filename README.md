# How the PeraBots 2026 Phase 1 controller works

This explains the code in `controllers/main_controller/`: what each file does, how a run flows from start to finish, and which settings in `config.py` you can tune.

---

## 1. Files

| File | Job |
|---|---|
| `main_controller.py` | Entry point. Works out where the robot was placed, then runs Section 1, the transition and Section 2 in order. |
| `config.py` | Every constant: robot geometry, sensor poses, speeds, gains, arena coordinates, thresholds. |
| `hardware.py` | `Bot` class. Sets up all devices and runs `step()`, which advances the simulation, reads the sensors, updates the pose and calls listeners. |
| `odometry.py` | Pose estimate (x, y, θ) from the wheel encoders plus the IMU heading. |
| `motion.py` | Motion primitives: `turn_to`, `drive_to`, `drive_axis`, `drive_cells`, `back_off`, `stop`, `halt`. |
| `vision.py` | Camera floor-colour classification (`near` and `ahead` regions). |
| `section1.py` | Obstacle field: occupancy grid, A* planner, path following, fallbacks. |
| `section2.py` | Maze: wall map, exploration, tile counting, majority vote, drive to the end cell, LED. |
| `watchdog.py` | Anti-termination: stuck detection (`Watchdog`) and repeated-failure detection (`LoopGuard`). |

How the modules depend on each other:

```
main_controller ──► section1 ──┐
       │         └► section2 ──┼──► hardware.Bot ──► motion.Motion ──► odometry
       └──────────► vision ◄───┘         │
                                          └──► config (everything reads config)
section1/section2 ──► watchdog
```

---

## 2. The robot

- Base: the provided `TwoWheelRobot.proto`. Wheel radius 0.02 m, track width 0.064 m, caster at the back.
- **Wheels moved 0.8 cm forward** (`AXLE_X = 0.008`). This puts the centre of mass slightly behind the axle, so the robot rests on its caster and doesn't tip onto its nose when braking.
- Sensors in `extensionSlot`:
  - **12 distance sensors** at z = 0.045 m. They return metres, and 1.0 means nothing was detected.
    - `ds_f` (0°), `ds_fl20` / `ds_fr20` (±20°), `ds_fl45` / `ds_fr45` (±45°), `ds_fl70` / `ds_fr70` (±70°)
    - `ds_l` / `ds_r` (±90°), `ds_bl` / `ds_br` (±135°), `ds_b` (180°)
  - **InertialUnit** `imu`: absolute yaw.
  - **Camera** `cam`: 32 × 24 pixels, pitched 0.6 rad down, sees the floor ahead.
- LEDs `led_blue` and `led_green` are switched **off** at startup and only ever used for the final answer.

---

## 3. The control loop: `Bot.step()`

Everything moves the simulation forward through `bot.step()`. There is no other `robot.step()` call except inside `finish()`. Each step:

1. Calls `robot.step(16 ms)`. If Webots ends the simulation, the controller exits.
2. Reads all distance sensors into `bot.d[name]`.
3. Updates odometry from the encoders and the IMU yaw.
4. Calls every function in `bot.listeners`. Examples: the Section 1 map update, and the Section 2 correction that keeps the robot centred between walls.

This means every motion primitive also keeps mapping and localisation running.

Motors use **velocity control** (`setPosition(inf)`) with an **acceleration limit** of 40 rad/s² (`WHEEL_ACCEL`). The limit keeps the wheels from slipping and the robot from tipping.

---

## 4. Localisation (no GPS)

`odometry.py`

- **Heading θ** = IMU yaw (absolute, no drift).
- **Distance** = mean of the left and right encoder changes × wheel radius.
- **Position** = integrated along the average heading over each step. Because the axle sits `AXLE_X` ahead of the chassis centre, the code moves the axle point and then converts back to the chassis centre. The pose is always the chassis centre.

Absolute corrections, using only **fixed arena features and walls**:

| Where | Correction |
|---|---|
| Start zone | Face −x. The back sensor measures the right arena wall to fix x; the left sensor measures the bottom wall to fix y. |
| Below the gap (end of Section 1) | Face −x. The front sensor measures the left arena wall to fix x (`fix_x_at_gap`). |
| Driving in the maze (listener) | Side walls at a known distance correct the position across the corridor (`lateral_listener`). |
| Stopped at a cell centre | Front, back and side walls snap x and y to the cell centre (`snap`). |
| Approaching a front wall | `drive_axis` measures the remaining distance with `ds_f` and stops exactly at the cell centre. |

In tests, odometry stayed within about 1–3 cm of the true position.

---

## 5. Program flow

```
main()
 ├─ Bot()                      set up devices, LEDs off, prime sensors
 ├─ detect_start()             where am I?
 │    ├─ open space around     → 'S1'  (start zone, set pose from the corner walls)
 │    └─ narrow corridor       → 'S2'  (judges placed the robot at the maze entrance)
 ├─ if S1:
 │    ├─ section1.run()        obstacle field → point below the gap (0.22, 1.12)
 │    └─ fix_x_at_gap()        re-measure x against the left wall, line up again
 ├─ enter_maze()               face north, drive through the red chamber into cell (1,1)
 └─ section2.run()             explore → count tiles → go to end cell → LED → stop forever
```

### 5.1 `detect_start`
1. Turn to the nearest grid direction.
2. If the left + right readings and the front + back readings are both ≥ 0.30 m, the robot is in open space. That means the **start zone**, so it faces −x and measures its pose against the bottom-right corner walls.
3. Otherwise it's in a narrow corridor, so this is a **Section 2 attempt**. It faces south and looks at the floor behind with the camera:
   - white or red floor → it's on the transition tile, pose (0.22, 1.275)
   - otherwise → it's already in cell (1,1)

---

## 6. Section 1: obstacle field (`section1.py`)

### Occupancy grid
- 2.5 cm cells covering 2.44 × 1.22 m (98 × 49 cells). The outer walls are marked as fixed, except for the gap in the top wall (x 0.101–0.339).
- **Every step**, each distance sensor ray:
  - lowers the "occupied" score of the cells it passes through (clears free space, up to 0.6 m), and
  - raises the score of the cell where it hits something (hits within 0.9 m only).
- A **distance field** stores how far each cell is from the nearest obstacle. It's updated incrementally for new hits and fully recomputed when a cell gets cleared.

### Cost of a cell (keeps the robot in the middle of passages)
| Distance to obstacle | Cost |
|---|---|
| < 4.5 cm (`R_FORBID`) | forbidden |
| < 7 cm (`R_HARD`) | +25 |
| < 13 cm (`R_SOFT`) | +0 to 2.5, sliding scale |
| otherwise | 1 |

### Planning and following
- **A*** with 8-connected moves (diagonals cost √2) from the robot to the goal (0.22, 1.12). Unknown cells count as free, so the robot heads straight for the goal and bends around cubes as it discovers them.
- Re-plans when the map changes (at most every 0.3 s), and at least every 1.5 s.
- **Pure-pursuit following:** aim at the path point 10 cm ahead. If the heading error is more than about 50°, spin in place; otherwise drive with a proportional heading correction and slow down near the goal.
- Finishes when within 3.5 cm of the goal.

### Safety and fallbacks
| Situation | Reaction |
|---|---|
| A front sensor reads < 3.5 cm | Back off and re-plan |
| No A* path (1st time) | Clear map memory within 20 cm of the robot, then re-plan |
| No A* path again | `reactive()`: head for the goal and turn away from obstacles for 2 s, then re-plan |
| Moved < 2 cm in 3 s (`Watchdog`) | Back off and re-plan |

---

## 7. Transition to the maze

`enter_maze()` faces north (+y) and uses `drive_axis` to reach the centre of cell (1,1) at (0.22, 1.455). As soon as the robot is inside the chamber, the side-wall correction keeps it centred.

---

## 8. Section 2: maze (`section2.py`)

### Grid and map
- 4 rows × 9 columns, 25 cm cells. Row r grows with +y, column c grows with +x.
  Cell centre = (0.22 + 0.25·(c−1), 1.455 + 0.25·(r−1)).
- Directions `N E S W` = headings 90°, 0°, −90°, 180°.
- `Maze.walls`: each wall between two cells is stored **once** (shared by both cells). Values are `True` (wall), `False` (open) or missing (unknown). The outer boundary always counts as a wall.
- `Maze.tile[cell]` = `'blue'`, `'green'` or `'none'` (floor seen, no tile).
- `Maze.visited` = cells the robot has stood in.

### At every stop: `sense()`
1. Stop and settle, then `snap()` the pose to the cell centre using the walls it can see.
2. **Look along all four directions** (`peek`) with the front, back, left and right sensors. A reading tells how many cells away the next wall is:
   `k = (reading − (0.119 − sensor_offset)) / 0.25`, rounded. Edges before cell k are marked open, and the wall at k is marked as a wall. So one stop can map walls several cells away.
3. Camera `near` → tile under or just ahead of the robot, stored for the current cell.
4. Camera `ahead` (only if the edge in front is open) → tile in the **next** cell.
5. **Dead-end peeking:** if a neighbouring cell is a known dead end (3 walls) that hasn't been checked, turn toward it and read its tile with the camera instead of driving in.

### Exploration loop
```
repeat (max 200 stops):
    sense()
    peek into known dead ends next to us
    (optional EARLY_STOP: quit if |blue − green| > cells still unresolved)
    plan to the cheapest cell that is not resolved (and not banned)
    if there is none: exploration finished
    drive the FIRST straight run of that path
```
- A cell is **resolved** when it has been visited, or all its walls are known and either it isn't a dead end or its tile is known. Only dead ends can hold tiles.
- **Planner:** Dijkstra over (cell, heading) states, with a cost of 1 per cell move and 0.35 per 90° turn. So it prefers straight runs. The end cell is avoided during exploration.
- **Driving:** `Navigator.run()` turns to the run's direction, checks for an unexpected wall, looks ahead with the camera, then `drive_axis` to the end of the run. If the run is blocked or times out, the robot re-localises to the nearest cell, records the wall and returns to that cell's centre.
- **LoopGuard:** if runs toward the same target fail more than 3 times in a row, that target is skipped for 30 s.

### Majority decision
```
blue  = number of 'blue' tiles
green = number of 'green' tiles
majority = 'blue' if blue > green, 'green' if green > blue, TIE_COLOUR if equal
```

### Going to the end cell
- Plan to `END_CELL` (4,9), allowing unknown edges (they cost 1.5 instead of 1), and drive run by run, re-sensing after each run.
- When the robot is inside the end cell, `bot.finish(colour)` runs.

### Finish
`finish()` sets both wheels to 0, switches on **only** the majority LED, and then keeps stepping forever. The LED comes on immediately on arrival, well inside the 3 s limit.

---

## 9. Vision (`vision.py`)

- `classify(r, g, b)` works on ratios, so it's robust to lighting:
  - black: max < 45
  - white: min > 150
  - red: r > 80 and r > 2g and r > 2b
  - green: g > 45 and g > 1.6r and g > 1.3b
  - blue: b > 35 and b > 1.6r and b > 1.3g
- `region_colour` votes over a band of rows, using only the centre half of the columns. It returns the winner only if it has at least 40% of the pixels.
  - `near` = rows 16–23 → floor of the current cell, just ahead
  - `ahead` = rows 1–8 → floor of the next cell (about 13–30 cm ahead)
- `stable(fn, frames)` reads over 2–3 frames and accepts a colour only if every frame agrees.
- Tiles are stored per cell, so the same tile is never counted twice.

---

## 10. Watchdog and anti-termination (`watchdog.py`)

The rulebook ends an attempt after 15 s without progress. The controller reacts much earlier:
- `Watchdog.stuck()`: keeps the pose history over 3 s. Moving less than 2 cm counts as stuck, and spinning on the spot doesn't count as progress.
- `LoopGuard`: temporarily bans a target that keeps failing.
- Every `drive_axis` and `turn_to` call has a timeout, so no single move can hang.

---

## 11. Tuning guide (`config.py`)

| Setting | Default | Effect |
|---|---|---|
| `S1_CRUISE`, `S2_CRUISE` | 8.0 rad/s | Driving speed (0.16 m/s). Higher is faster but riskier. |
| `WHEEL_ACCEL` | 40 rad/s² | Lower is smoother and less slip; higher reacts faster. |
| `TURN_K`, `TURN_MAX`, `TURN_TOL` | 9, 6, 1.2° | Turn-in-place controller. |
| `HEAD_K`, `CROSS_K` | 7, 5 | Heading and line-keeping gains while driving. |
| `R_FORBID` / `R_HARD` / `R_SOFT` | 4.5 / 7 / 13 cm | How far Section 1 paths stay from cubes. |
| `S1_GOAL`, `S1_GOAL_TOL` | (0.22, 1.12), 3.5 cm | Target below the gap. |
| `WALL_THRESH_F/S` | 0.20 m | Axis-sensor reading below this means a wall in this cell. |
| `CAM_AHEAD_ROWS`, `CAM_NEAR_ROWS` | 1–8, 16–23 | Camera bands; retune if the camera pose changes. |
| `EARLY_STOP` | False | True = stop exploring once the majority is decided (shorter, but check the scoring rules). |
| `TIE_COLOUR` | 'blue' | Used if blue = green (not defined in the rulebook). |
| `TURN_COST` | 0.35 cells | Higher means maze routes prefer fewer turns. |
| `STUCK_WINDOW`, `STUCK_MOVE` | 3 s, 2 cm | Stuck-detection sensitivity. |
| `DEBUG` | True | Print progress to the Webots console. |

Set the environment variable `PERABOT_LOG=<file>` to also write the log to a file. When it isn't set, this does nothing.

---

## 12. Test results

| World | Section 1 | Maze exploration | Total | Tiles found | LED |
|---|---|---|---|---|---|
| `Arena.wbt`, full run | 16.9 s | 156 s | 186 s | B2 G3 (all 5) | green ✔ |
| Section 2 only (placed on the transition tile) | – | 156 s | 168 s | B2 G3 | green ✔ |
| Start zone, rotated heading | 16.8 s | 156 s | 188 s | B2 G3 | green ✔ |
| Random world 1 (new obstacles, maze, tiles) | 16.2 s | 131 s | 193 s | B1 G2 | green ✔ |
| Random world 2 | 16.0 s | 93 s | 124 s | G1 | green ✔ |

Every run stopped within about 3 mm of the end-cell centre with no tipping (max pitch 0.2°).

The test tools are in `../test_tools/`, **outside** the submission folder:
- `gen_random_world.py`: generates random mazes, tiles and obstacles
- `test_truth/`: supervisor logger that records the robot's true position (test only)
- test worlds

Don't include them in the submission zip.
"# maze_solving_robot_datalab2026" 
