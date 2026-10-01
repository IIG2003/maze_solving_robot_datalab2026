# PeraBots 2026 – University Category, Phase 1 (Webots)

Manual for the simulation project: how to run it, what the robot and controller contain, how to test it, and how to submit. The rulebook is the authority. If this file and the rulebook disagree, follow the rulebook. A running log of decisions and measurements is in [`PLAN.md`](PLAN.md).

## 1. Quick start

Requirement: **Webots R2025a** (the version the rulebook specifies).

1. Open the world: `open -a Webots worlds/Arena.wbt` (or **File → Open World…**). Open the `.wbt` file itself, not the folder.
2. In Webots, **Preferences → General → Python command** must be the **full path** of a working `python3` (for example `/opt/homebrew/bin/python3`; a bare `python3` fails when Webots is started from the Dock).
3. After you edit `protos/TwoWheelRobot.proto`, use **File → Reload World**. Webots caches PROTOs and keeps using the old one until the world is reloaded (a typical symptom: `Device "ds_f0" was not found on robot`).
4. Press play. The robot drives Red A → through the obstacle field → Red B → explores the maze → stops on the red end cell and lights the majority LED.

Two switches live in `controllers/main_controller/main_controller.py`:

| Switch | Default | Effect |
| --- | --- | --- |
| `RobotConfiguration.PRINT_PROGRESS` | `False` | `True` prints the start mode, tiles found, stalls and a final JSON dump of what the robot learned |
| `ENABLE_RUN_MONITOR` | `False` | `True` writes `run_log.csv` (speed, turn rate, wheel commands, clearance, tilt, stalls, phase) using `run_monitor.py`; chart it with `tests/plot_run.py` (see §7.1) |

## 2. Project layout

```
PeraBots_UC/
├── worlds/Arena.wbt                      # world to open
├── protos/
│   ├── Arena.proto                       # LOCKED – do not modify
│   ├── Obstacles.proto                   # LOCKED – do not modify
│   └── TwoWheelRobot.proto               # the robot (sensors + 10 mm wheel shift, see §3)
├── controllers/main_controller/
│   ├── main_controller.py                # the whole controller (single file)
│   └── run_monitor.py                    # OPTIONAL analytics – delete it for the submission
├── tests/                                # NOT part of the submission (see §7)
│   ├── maze_ground_truth.py              # reads the true maze out of Arena.proto
│   ├── compare_exploration_policies.py   # offline comparison of exploration policies
│   ├── run_maze_test.py                  # real-simulator test against the true maze
│   ├── run_ablation.py                   # sensor / speed / margin / turning / arc optimisation tests
│   └── plot_run.py (+ plot_run_template.html)  # turns run_log.csv into an HTML chart report
├── PLAN.md                               # decisions, measurements, open questions
└── README.md
```

Do not rename, move or restructure `TwoWheelRobot.proto`, `controllers/main_controller/` or `main_controller.py`.

## 3. The robot

### Devices
| Device | Name(s) | Notes |
| --- | --- | --- |
| Wheel motors / encoders | `left_wheel_motor`, `right_wheel_motor` / `left_wheel_sensor`, `right_wheel_sensor` | locked; wheel radius 0.02 m, track 0.064 m, limit 10 rad/s (0.2 m/s) |
| Majority LEDs | `led_blue`, `led_green` | locked; the controller lights one on arrival at the end cell |
| Distance sensors (16, `laser` type, 0.5 m range, readings in **metres**) | see table below | added through `extensionSlot` |
| Inertial unit | `imu` | world yaw (counter-clockwise positive) |
| Floor colour camera | `floor_cam` | 1×1 pixel, looking straight down, just past the chassis front |
| Look-ahead camera | `look_cam` | 32×24, 6 cm high, tilted 0.6 rad down; reads the colour of a dead-end cell from the cell before it (only used when `LOOK_AHEAD_ENABLED = 1`) |

### Distance sensors (robot frame: +x forward, +y left; mounted at z = 0.05 m, above the wheel tops)
| Sensor | Angle | Role (from the optimisation tests) |
| --- | --- | --- |
| `ds_f0` | 0° | essential: front distance, maze front wall |
| `ds_fl15`, `ds_fr15` | ±15° | helpful: front clearance and steering |
| `ds_fl30`, `ds_fr30` | ±30° | helpful: wider front view |
| `ds_cl`, `ds_cr` | ±45° | helpful: front corners while turning |
| `ds_left_front`, `ds_right_front` | ±67.5° | helpful: removing both made the course about 3.6 s slower; removing one added collisions |
| `ds_l`, `ds_r` | ±90° | essential: maze wall detection and centring |
| `ds_left_rear`, `ds_right_rear` | ±112.5° | no measurable effect in the tests; kept for pivot and stall edge cases |
| `ds_rear_left`, `ds_rear_right` | ±135° | same: kept for edge cases |
| `ds_rear` | 180° | same: kept for edge cases |

The sensors have no collision geometry or physics, so the locked collision shape and mass are unchanged.

### Balance: wheels moved 10 mm forward
The base robot's centre of gravity sits 0.43 mm behind the axle and 17 mm above it, with a caster only at the rear. Braking could tip it onto its nose (36.9° pitch) and it stayed there, tilting every sensor. Rulebook 4.4 allows minor positional adjustment of the two drive wheels for balance, so both wheels are 10 mm further forward in the proto (`anchor` and `translation` x = 0.01). In 25 acceleration/braking combinations: 0 mm tipped 15 times, 5 mm tipped 4 times, **10 mm tipped 0 times**. Odometry follows the axle (`AXLE_FORWARD_OFFSET_METERS`), so sensor poses are measured from it.

### Known risk: `extensionSlot`
The sensors are the **default value** of the proto's `extensionSlot` field. If the organizers' world sets `extensionSlot` on the robot node (even to an empty list) the sensors would not exist. The given `Arena.wbt` does not set it. This is an open question for the committee (see `PLAN.md` §8).

## 4. What you may and may not change (rulebook §4.3–4.4)

**Do not modify**
- `Arena` and `Obstacles` nodes and `Arena.proto` / `Obstacles.proto` (locked).
- The robot's chassis, motors, wheel size, physics and collision geometry. Only the drive-wheel **position** may change slightly, for balance.
- The `led_blue` and `led_green` devices. Do not enable Supervisor.

**Allowed**
- Add devices through the `extensionSlot` field of `TwoWheelRobot.proto`: `DistanceSensor`, `Camera`, `TouchSensor`, `Accelerometer`, `Gyroscope`, `InertialUnit`, `LightSensor`, `PositionSensor` (added joints), `Speaker`, extra LEDs. Prohibited: GPS, Emitter, Receiver.
- Chassis colour, sensor placement and orientation, controller logic.
- `Lidar`: the committee confirmed it is allowed for Phase 1 (keep that message). It is not used yet.

**Size limit:** 20 × 20 × 20 cm (the robot is about 9.4 × 7.4 × 5.8 cm including the sensors and camera).

Evaluation uses your submitted `TwoWheelRobot.proto` and `controllers/main_controller/` in the organizers' world. The world file's settings, such as `basicTimeStep`, are not submitted, so the controller reads the step from the world.

## 5. The task

**Section 1 – obstacle field.** 2.44 m × 1.22 m. The robot starts in a 25 × 25 cm red zone (Red A) and must reach the red transition tile (Red B). Obstacles are 10 cm cubes; clear passages are at least 25 cm. Scored on completion, time, distance and collisions.

**Section 2 – maze.** 9 × 4 cells, 25 cm pitch, walls 1.2 cm thick (clear cell about 23.8 cm). Blue and green tiles sit at some dead ends. The robot must find and count the tiles, determine the majority colour, reach the red end cell with its whole body, and within 3 s light the majority LED and stop.

**Evaluation notes**
- Three attempts per section. Evaluators may place the robot at the Section 2 entrance (Red B), so the controller decides its mode from the sensors.
- An attempt ends after 15 s without progress, or if the robot is fully off the track for more than 2 s.
- The official arena has a different obstacle layout, maze walls and tile positions. **Do not hard-code** any layout or route.
- The rulebook defines no tie rule for equal blue and green counts; the controller uses `TIE_DEFAULT_COLOR` (`"GREEN"`).

## 6. How the controller works

Flow: `detect_start_mode` → Section 1 (map + A\* + pure pursuit) → transition corridor → Section 2 (cell graph, nearest-unexplored exploration, tile counting, shortest route to the end, LED).

The detailed mathematics is in §9. The settings that matter most:

| Setting (`RobotConfiguration`) | Value | Why |
| --- | --- | --- |
| `MAXIMUM_SPEED_METERS_PER_SECOND`, `MAZE_CRUISE_SPEED_METERS_PER_SECOND` | 0.198 | the wheels cap straight-line speed at 0.2 m/s |
| `ACCELERATION_LIMIT…`, `DECELERATION_LIMIT…` | 1.2 m/s² | faster acceleration or braking gave no gain; braking at 6 m/s² fails |
| `PIVOT_PROPORTIONAL_GAIN` / max rate / min rate / tolerance | 32 / 8 rad/s / 1.0 rad/s / 0.02 rad | turning was where time was lost: 159 s → 135 s |
| `ARC_CORNERS_ENABLED`, `ARC_SPEED_METERS_PER_SECOND` | 1, 0.155 m/s | outer wheel must stay under 0.2 m/s: v ≤ 0.159 m/s |
| `NORMAL_INFLATION_RADIUS_METERS`, `CLEARANCE_PENALTY_*` | 0.078 m, radius 0.115 m, weight 4 | wider margins make routes longer or fail: the gaps are only 25 cm |
| `SIDE_CLEARANCE_MINIMUM_SPEED_FACTOR` | 0.7 | 1.0 (never slow down) causes collisions |
| `STALL_*` | 0.6 s, 7 cm back-off | fail-safe for unplanned stops |

### 6.1 Ported from the datalab2026 controller
A second controller for the same competition was reviewed (`maze_solving_robot_datalab2026`). These ideas were adopted. All of them act **only on failure paths**, so a normal run is unchanged:

| Safety net | Where | Behaviour |
| --- | --- | --- |
| Unexpected wall | `wall_blocks_leg`, `LegAborted` | a maze leg stops when the front beam sees a wall more than 15 cm nearer than the target cell, records the wall, returns to the cell centre and replans |
| Leg timeout | `LEG_TIMEOUT_SECONDS` = 8 s | a leg that takes too long is aborted and replanned (the rulebook ends an attempt after 15 s without progress) |
| Loop guard | `LoopGuard` | a maze target whose route keeps aborting (4 times in a row) is skipped for 30 s |
| No route in Section 1 | `OccupancyGrid.clear_around`, `reactive_drive_toward` | the first time: forget mapped obstacles within 20 cm of the robot and replan (drift can leave phantom obstacles); after that: drive reactively toward the goal for 2 s, then replan. Before, the robot just turned on the spot |
| Look-ahead (off) | `LOOK_AHEAD_ENABLED = 0` | 1 = the robot reads a dead-end tile from the neighbouring cell instead of entering it. A cell is only skipped when three beams (front and both ±30°) prove it is closed on three sides **and** the camera shows one clear colour; anything uncertain means it enters as before. A RED reading is never trusted (a wall can look red), so the end cell is always entered and read by `floor_cam`. Left OFF so every dead end is physically visited, because the rules rank "sufficient exploration" first. Simulator-checked, so it can be switched on if the committee accepts it (PLAN.md §0.49) |
| Entrance bias | `START_BIAS_WEIGHT = 2.0` | the next cell to explore is the one with the lowest (route cost + 2 x its route distance from the entrance), so the area near the entrance is finished before heading far away and the red end cell is usually reached late. 0 = plain nearest-first, 4 or more = too much backtracking |
| Early stop (off) | `EARLY_STOP_WHEN_MAJORITY_DECIDED = 0` | when set to 1, exploration ends once the blue/green lead exceeds the number of unvisited cells and the end cell is known. Only use it if the committee accepts that as "sufficient exploration" |
| Random mazes | `tests/gen_random_maze.py` | testing on layouts the controller has never seen, because the official maze differs |

Reviewed and **not** ported (yet): the 32×24 tilted camera that reads the next cell's tile, 1.0 m range sensors that map several cells at once, and skipping "resolved" cells. Together they could cut the cells visited, but they change the camera and the exploration and need simulator validation first. Absolute pose fixes against arena walls and a hardware wheel-acceleration limit were skipped because this controller is start-relative and already limits acceleration in software.

## 7. Tests (in `tests/`, not part of the submission)

Tests that use Webots run it **headless on temporary copies** of the project and never modify your files (they open a Webots window for the duration of a run). Set `WEBOTS=/path/to/webots` if Webots is not at `/Applications/Webots.app/Contents/MacOS/webots`.

| Command | What it does |
| --- | --- |
| `python3 tests/maze_ground_truth.py` | prints the true maze, tiles and end cell read from `Arena.proto` |
| `python3 tests/compare_exploration_policies.py` | compares exploration policies offline with the controller's own planner (no Webots) |
| `python3 tests/test_failure_handling.py` | offline (no Webots): unexpected-wall detection, loop guard, map forgetting, early stop |
| `python3 tests/test_random_mazes.py 300` | offline (no Webots): the route planner on 300 random mazes (all cells, all tiles, end reached); also checks that generated `Arena.proto` files read back exactly |
| `python3 tests/gen_random_maze.py 7` | prints a random maze; `build_project(spec, folder)` writes a copy of the project with that maze for a Webots run |
| `python3 tests/run_maze_test.py red_b_north red_b_south full_course` | runs the controller and compares what it learned with the true maze: coverage, every wall, tiles, end cell, contradicting readings, visit order and move count |
| `python3 tests/run_ablation.py <mode>` | optimisation runs on 6 start poses: `baseline`, `sensors`, `speed`, `margins`, `turning`, `combo`, `arcs`, `stress` (set `TIMESTEP=8` etc. to change the world step) |

A run only counts as passing if the robot finishes, visits all 36 cells, finds all 5 tiles and the end cell. A separate supervisor counts real contacts and the robot's real tilt.

### 7.1 Charts of a run: velocity, acceleration and more
1. In `controllers/main_controller/main_controller.py` set `ENABLE_RUN_MONITOR = True`.
2. Run the course once in Webots. `run_log.csv` appears in `controllers/main_controller/` (the console also prints a one-line summary).
3. `python3 tests/plot_run.py controllers/main_controller/run_log.csv` writes `run_report.html` next to the CSV. Open it in any browser; it works offline and follows your light/dark setting.
4. Set `ENABLE_RUN_MONITOR` back to `False` before submitting.

The report has summary tiles (duration, distance, mean and peak speed, peak acceleration, RMS jerk, peak tilt, stall recoveries) and six charts over time: **speed** (measured from the wheel encoders, and commanded), **acceleration** (measured speed smoothed over 0.1 s, and commanded, where the slew limit shows as flat steps), **turn rate**, **wheel speed commands** (with the 10 rad/s motor limit), **clearance** (front and closest beam) and **tilt** (pitch and roll, with the 37° tip-over mark). The path is drawn coloured by speed. Hover any chart (or tab to it and use the arrow keys) and every chart and the path share one cursor with the values; the phase buttons (whole run, Section 1, corridor, maze) re-scope everything; a table view lists the numbers.

### Results so far (simulator, 16 ms, 6 start poses)
| Stage | Whole course |
| --- | --- |
| first working version | about 241 s |
| after continuous motion, gate point, fail-safes (0 collisions) | about 159 s |
| after turn-gain tuning | about 135 s |
| **with arc corners (current)** | **about 128 s** (0 collisions, 0 stalls, max pitch 6.5°) |

All three ground-truth scenarios pass (0 of 144 walls wrong, all tiles, end cell). The maze takes 69 cell moves; an ideal walk that knew where the red cell was would take about 57. Timesteps 8, 16 and 32 ms work; 64 ms fails even with the old settings (the physics tips the robot). Not done: reversing out of dead ends instead of a 180° pivot (estimated 5–8 s); a start in the middle of the maze (the code path exists but is untested).

## 8. Troubleshooting

- **Webots crashes while loading the world (macOS).** `Arena.wbt` downloads `TexturedBackground` and `TexturedBackgroundLight` from GitHub. A Webots bug can segfault in the loading progress dialog when the downloads finish. Try: quit Webots (Cmd+Q) and reopen; use a stable network without VPN or proxy; delete `~/Library/Caches/Cyberbotics/Webots/`; or point the two `EXTERNPROTO` lines at the copies in `Webots.app/Contents/projects/objects/backgrounds/protos/`. Crash reports are in `~/Library/Logs/DiagnosticReports/` (`webots-*.ips`).
- **`Python was not found` / robot does not start.** Webots started from the Dock or Finder does not see your shell PATH, so a bare `python3` is not found. In Webots open **Webots → Preferences… → General → Python command**, enter the full path (here `/opt/homebrew/bin/python3`; find yours with `command -v python3`), press OK and use **File → Reload World**. Or quit Webots first and run `defaults write com.cyberbotics.Webots-R2025a General.pythonCommand /opt/homebrew/bin/python3` (Webots overwrites the setting if it is still open).
- **`Device '…' not found on the robot`.** You edited `TwoWheelRobot.proto` and did not reload: **File → Reload World**. Also check that `extensionSlot` is not overridden on the robot node.
- **Robot tilts or tips.** Both wheels must be at x = 0.01 in the proto (see §3).
- **Stuck.** Set `PRINT_PROGRESS = True`; stalls and the Section 1 map are printed.
- **Console shows nothing.** `PRINT_PROGRESS` is `False` by default.

## 9. Technical notes: mathematics and architecture

### 9.1 State machine
```
[Section 1: obstacle field] ──> [Transition corridor (Red B)] ──> [Section 2: maze]
 occupancy-grid mapping            alignment and wall               cell graph, Dijkstra,
 + inflated-grid A*                centring                         arc turns
 + pure pursuit
```
- `detect_start_mode`: black floor → `MAZE`; red floor with walls close on **both** sides → `TRANSITION`; otherwise `SECTION_1`.
- `main` loops up to 4 times: if the corridor entry finds white floor in both directions (a false red, or a bad approach) it returns `NOT_IN_CORRIDOR` and the robot goes back to Section 1 with the **original** start point, so the goal stays where it was. Section 1 only ends after red is seen on 4 consecutive readings while the robot is level.
- Mapping is occupancy-grid mapping from a dead-reckoned pose (no loop closure), not full SLAM.

### 9.2 Differential-drive kinematics and odometry
Wheel radius $R = 0.02\,\text{m}$, track $L = 0.064\,\text{m}$. From encoder increments $\Delta\theta_l, \Delta\theta_r$:
$$\Delta s_l = \Delta\theta_l R,\quad \Delta s_r = \Delta\theta_r R,\quad \Delta s = \tfrac{1}{2}(\Delta s_l + \Delta s_r)$$
Heading $\theta$ comes straight from the IMU (`getRollPitchYaw()[2]`), not from the encoder difference (tyre slip makes that drift):
$$x_{k+1} = x_k + \Delta s\cos\theta,\qquad y_{k+1} = y_k + \Delta s\sin\theta$$
Body commands: forward speed $v$ and turn rate $\omega$ (counter-clockwise positive) become
$$v_l = v - \omega\tfrac{L}{2},\quad v_r = v + \omega\tfrac{L}{2},\quad \dot\theta_l = v_l/R,\quad \dot\theta_r = v_r/R$$
and each wheel speed is clamped to $[-10, 10]\,\text{rad/s}$.

### 9.3 Sensor ray casting
Sensor $i$ is mounted at $(x_{s,i}, y_{s,i})$ with angle $\theta_{s,i}$ in the robot frame. The wheels sit 10 mm ahead of the proto origin and odometry follows the axle, so the axle offset $x_{\text{axle}} = 0.010\,\text{m}$ is subtracted: $x'_{s,i} = x_{s,i} - x_{\text{axle}}$, $y'_{s,i} = y_{s,i}$.
$$\begin{bmatrix} x_{w,i} \\ y_{w,i} \end{bmatrix} = \begin{bmatrix} x \\ y \end{bmatrix} + \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}\begin{bmatrix} x'_{s,i} \\ y'_{s,i} \end{bmatrix},\qquad \theta_{\text{beam},i} = \theta + \theta_{s,i}$$
A reading $d_i < 0.49\,\text{m}$ is a hit: $x_{\text{hit}} = x_{w,i} + d_i\cos\theta_{\text{beam},i}$, $y_{\text{hit}} = y_{w,i} + d_i\sin\theta_{\text{beam},i}$.

The grid has cells of $\Delta = 0.025\,\text{m}$: $\text{cell} = (\lfloor x/\Delta\rfloor, \lfloor y/\Delta\rfloor)$. A Bresenham walk marks every cell before the endpoint `FREE` and the endpoint cell `OCCUPIED` (sticky). The mapper writes about every 30 ms of simulated time, and is switched off in the maze, which uses its own cell graph.

### 9.4 Obstacle inflation and the clearance field
To plan for a point robot, obstacles are dilated with circular masks
$$\mathcal{D}(r) = \{(dx, dy) \in \mathbb{Z}^2 : (dx\Delta)^2 + (dy\Delta)^2 \le r^2\}$$
in three layers:
1. **Normal** ($r = 0.078\,\text{m}$): the robot reaches 0.056 m from the axle (rear corners), plus margin for the 2.5 cm quantisation.
2. **Tight** ($r = 0.062\,\text{m}$): fallback when the normal layer blocks every route, or for 4 s after a stall.
3. **Clearance penalty** ($r = 0.115\,\text{m}$): not blocked, but each step costs $\Delta s\,(1 + 4.0)$. A 25 cm passage has its centre-line 12.5 cm from both faces, so its centre stays penalty-free, while a narrow diagonal squeeze costs five times as much.

Measured: larger inflation, a wider penalty radius or a higher weight made Section 1 slower or less reliable, because the real gaps are only 25 cm.

### 9.5 Motion profile
Slew-rate limiter with separate limits, cycle time $\Delta t$:
$$\Delta v = \text{clamp}\big(v_{\text{desired}} - v_{\text{current}},\; -d_{\max}\Delta t,\; a_{\max}\Delta t\big),\qquad v_{k+1} = v_k + \Delta v$$
Front-clearance slow-down (Section 1), with $d_{\text{slow}} = 0.30\,\text{m}$, $d_{\text{stop}} = 0.05\,\text{m}$:
$$v_{\text{clearance}} = v_{\max}\,\text{clamp}\!\left(\frac{d_{\text{front}} - d_{\text{stop}}}{d_{\text{slow}} - d_{\text{stop}}},\, 0,\, 1\right)$$
Constant-deceleration braking toward a target at distance $d_{\text{rem}}$ (from $v_f^2 = v_i^2 - 2ad$):
$$v_{\text{desired}} = \min\!\big(v_{\text{cruise}},\; \sqrt{2\,a_{\text{brake}}\,d_{\text{rem}}}\big)$$

### 9.6 Section 1: planning and pure pursuit
- **Greedy A\*** on an 8-connected grid (orthogonal cost 1, diagonal $\sqrt2$). Heuristic $h(n) = 1.1\sqrt{(x_g - x_n)^2 + (y_g - y_n)^2}$; the weight bounds the path cost to within 10% of optimal and expands fewer nodes. Unknown cells count as free, so the robot replans as it learns. A diagonal step needs both orthogonal neighbours passable (no corner cutting).
- **Gate point:** the robot first lines up 10 cm below the Red B opening, then enters straight, instead of cutting the corner of the corridor wall.
- **Replanning:** only when the current route becomes blocked, or every 1.5 s.
- **Smoothing:** line-of-sight pruning with a Bresenham visibility test (look-ahead limited to 40 cells): if $\text{line\_is\_clear}(P_i, P_{i+k})$, the intermediate waypoints are dropped.
- **Pure pursuit:** target waypoint at lookahead $\ge 0.15\,\text{m}$; $\theta_e = \text{atan2}(W_y - y, W_x - x) - \theta$ wrapped to $[-\pi,\pi]$; $\omega_{\text{pursuit}} = \text{clamp}(3.0\,\theta_e, -2.5, 2.5)$.
- **Reactive repulsion** blended in: with $d_l$ the smallest of the left-side beams (front corner, ±30°, ±15°, and the side beams minus 2 cm) and $d_r$ likewise on the right,
  $$\text{push}_{l} = \max(0,\, 0.07 - d_l),\quad \text{push}_{r} = \max(0,\, 0.07 - d_r),\quad \omega_{\text{reactive}} = 20\,(\text{push}_r - \text{push}_l)$$
  and $\omega_{\text{total}} = \omega_{\text{pursuit}} + \omega_{\text{reactive}}$. Speed also drops (factor 0.7 to 1.0) when anything is close beside the robot.

### 9.7 Corridor centring (`corridor_turn_rate`)
With both side walls visible ($< 0.16\,\text{m}$): $e_{\text{lateral}} = (d_{\text{left}} - d_{\text{right}})/2$. With one wall, nominal distance $d_{\text{nom}} = 0.094\,\text{m}$: $e_{\text{lateral}} = d_{\text{left}} - d_{\text{nom}}$ or $d_{\text{nom}} - d_{\text{right}}$. Then
$$\theta_{\text{corr}} = \text{clamp}(6.0\,e_{\text{lateral}}, -0.2, 0.2),\qquad \omega = \text{clamp}\big(5.0\,[(\theta_{\text{target}} - \theta) + \theta_{\text{corr}}],\, -2, 2\big)$$

### 9.8 Pivot turns
In-place turns are a proportional controller on the IMU yaw: $\omega = \text{clamp}(K\,e_\theta, \pm 8\,\text{rad/s})$ with a minimum rate of 1.0 rad/s near the target and a tolerance of 0.02 rad. $K = 32$, but the code limits it to $0.8/\Delta t$ so the discrete loop stays stable at large timesteps. While pivoting, the robot creeps 4 cm/s away from anything very close.

### 9.9 Section 2: cell graph, Dijkstra and arc corners
- **Cell graph:** each cell has four edges (wall or open) in world axes. At a new cell centre the robot reads front, left and right (the entrance behind it is closed when it came through Red B), and the floor colour: blue/green tiles are counted once per cell; red marks the end cell.
- **Turn-penalised Dijkstra** (`find_route`) over $(\text{cell}, \text{heading})$ goes to the nearest unvisited cell; each 90° adds a penalty:
  $$\text{cost} = 1.0\cdot\text{steps} + 0.5\cdot\text{quarter-turns}$$
  After every cell is visited, the same search finds the shortest route to the red end cell. Moves use **absolute** cell-centre targets, so errors do not build up from cell to cell, and the front wall is trusted for stopping only when it agrees with odometry to within 5 cm.
- **Arc corners** replace stop-pivot-restart for 90° turns through cells that are **already visited** (a new cell still stops at its centre to read its walls; dead-end U-turns still pivot). The robot drives a quarter circle of radius $R = \text{pitch}/2 = 0.125\,\text{m}$ around the inside corner, entering at the middle of the entry edge and leaving at the middle of the exit edge:
  ```
         ▲ exit
         │
     ┌───┼───┐
     │  ╭╯   │   R = 0.125 m
     │ ╭╯    │
  ───┼─╯─────┼───
     │ entry
  ```
  The outer wheel travels on radius $R + L/2$, at speed $v_{\text{outer}} = v\,(1 + \tfrac{0.032}{0.125}) = 1.256\,v$. With the 0.2 m/s wheel limit, $v \le 0.159\,\text{m/s}$, so $v_{\text{arc}} = 0.155\,\text{m/s}$.
  With $s = +1$ for a left turn and $-1$ for a right turn, the circle centre is $\vec C = \vec C_{\text{cell}} - R\,\vec u_{\text{in}} + R\,\vec u_{\text{out}}$, and
  $$e_{\text{radial}} = \lVert\vec p - \vec C\rVert - R,\qquad \theta_e = \big(\text{atan2}(p_y - C_y,\, p_x - C_x) + s\tfrac{\pi}{2}\big) - \theta$$
  $$\omega = s\,\frac{v}{R} + K_\theta\,\theta_e + s\,K_r\,e_{\text{radial}},\qquad K_\theta = 4.0,\; K_r = 12.0$$
  The arc ends when the yaw has turned by $\pi/2 - 0.03$. It saved about 7–9 s on the whole course.

### 9.10 Floor colour classification and stall recovery
From the raw RGB of `floor_cam` (tile reds, greens and blues have exactly zero in the other channels, so the rules reject orange or tilted views):
- $\max(R,G,B) < 42 \Rightarrow$ BLACK
- $\min(R,G,B) > 0.7\max(R,G,B)$ and $\max > 120 \Rightarrow$ WHITE
- $R > 60$ and $G, B < 30 \Rightarrow$ RED; $B > 45$ and $R, G < 30 \Rightarrow$ BLUE; $G > 45$ and $R, B < 30 \Rightarrow$ GREEN

Measured colours in the development arena: white (220,220,220), black floor (29,29,29), red (108,0,0), green (0,66,0), blue (0,0,55).

A stall is flagged when any of these holds for $\ge 0.6\,\text{s}$:
1. a wheel commanded at $\ge 2\,\text{rad/s}$ turns at less than 35% of the command (blocked wheel);
2. the robot is commanded forward and a front beam reads $< 0.03\,\text{m}$ (or the same in reverse with the rear beams): the body is touching, while the tyres can keep spinning;
3. the robot is driving toward something within 0.25 m but the front distance has not changed by more than 1.2 cm in 1.2 s (wheels slipping against it).

Recovery: stop; reverse 0.07 m (forward if the rear beams see no room; if the wheels still cannot move it tries the other direction once); rotate $+25.8°$, then $-25.8°$, then back to the original heading so the side and corner beams sweep the surroundings and the mapper records them; in Section 1, route planning then uses the tight inflation layer for 4 s.

## 10. Submission (rulebook §4.5)

Zip the complete project folder, named with your team name or ID at the outer level only, for example `TeamName_Submission.zip`. It must include:

- The unmodified `Arena.proto` and `Obstacles.proto`.
- Your modified `TwoWheelRobot.proto`.
- The full `controllers/main_controller/` folder.

Checklist before zipping:
- [ ] `ENABLE_RUN_MONITOR = False` and `RobotConfiguration.PRINT_PROGRESS = False`.
- [ ] Delete `controllers/main_controller/run_monitor.py` (not needed when the monitor is off; nothing else changes).
- [ ] Leave `tests/` and (optionally) `PLAN.md` out of the zip.
- [ ] Diff `Arena.proto`, `Obstacles.proto` and the non-sensor parts of `TwoWheelRobot.proto` against the original package. Only the sensors and the 10 mm wheel position should differ.
- [ ] No team name inside any file or folder name.
- [ ] Unzip into a fresh folder, open it in Webots R2025a and run it once.

Follow the final instructions announced on the official WhatsApp channels.

## 11. Key dates

| Event | Date |
| --- | --- |
| Phase 1 begins | September 26 |
| Workshop series | September 28 – October 12 |
| Phase 1 submission deadline | October 28 |
| Finalists announced | November 5 |
| Grand Finale (physical) | December 12 |

## 12. Contacts

University Category coordinators: Sithum Vitharana (+94 76 430 4439), Chethiya Lakkana (+94 77 360 1039).
