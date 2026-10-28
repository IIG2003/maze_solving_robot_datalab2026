# PeraBots Lean-Sensor Robot Architecture

Oct 27, 2026 · Inuka

## 1. Goal and design targets

This design cuts the robot from 14 devices to **7** (5 distance sensors, 1 IMU, 1 camera) and keeps the full PeraBots task. It merges the tested PeraBots controller (grid + A*, Dijkstra, listeners) with the two-sector algorithm doc (goal-deferred frontier exploration, turn-cost planning, arcs on the final run).

| Target | Current robot | Lean robot |
| --- | --- | --- |
| Sensor devices (excl. wheel encoders) | 12 distance + IMU + camera = 14 | 5 distance + IMU + camera = **7** |
| Section 1 time | 16–17 s | ≤ 17 s (same planner, fewer rays) |
| Maze exploration | 93–156 s | target 30–40 % less |
| Dead ends physically entered | most | **none** (read from the neighbour cell) |

**What "non-repeating" can mean.** A maze with branches can't be covered with zero backtracking: in a tree maze with E open edges and start-to-end distance d, any walk that inspects every branch is at least 2E − d cells long. The design gets close to that bound by (1) never entering a dead end, (2) never visiting a cell whose walls and floor are already known, and (3) exploring the branch towards the end cell last, so the final leg has no repeated cells.

Assumption: wheel encoders ship with the motors and don't count towards the sensor budget. If they do count, the budget is 9.

## 2. Sensor suite

Seven devices cover everything: three forward rays for obstacles and look-ahead, two side rays for walls and centring, the IMU for heading, and the camera for floor colour. Keep the existing 1.0 m range, z = 0.045 m and metre output.

| # | Device | Mount | Section 1 job | Section 2 job |
| --- | --- | --- | --- | --- |
| 1 | `ds_f` | 0° | Obstacle ahead, emergency stop, start scan | Front wall + corridor length (up to 4 cells), longitudinal snap |
| 2 | `ds_fl30` | +30° | Obstacles in the front-left fan | **Next cell's left wall** (read from the current cell), centring |
| 3 | `ds_fr30` | −30° | Obstacles in the front-right fan | **Next cell's right wall**, centring |
| 4 | `ds_l` | +90° | Flank clearance, start-zone y fix | Current cell's left wall, side openings while driving, lateral snap |
| 5 | `ds_r` | −90° | Flank clearance | Current cell's right wall, side openings while driving, lateral snap |
| 6 | `imu` | – | Drift-free yaw | Drift-free yaw, exact 90° turns |
| 7 | `cam` 32×24, pitched down | – | (optional) red goal check | Tile colour of the **next** cell, transition/end red |

```
            ds_fl30   ds_f   ds_fr30
                 \     |     /
                  \ 30°|30° /
                   \   |   /
          ds_l ── [   ROBOT   ] ── ds_r
                  (imu, cam pitched down)
```

**Why 30° and not 45°.** From the cell centre, with the sensor about 4 cm forward of the axle, a 30° ray reaches the side-wall line (12.5 cm out) about 25 cm ahead, which is the middle of the next cell. A 45° ray lands on the corner post, where present and absent walls look alike. So 30° lets one stop reveal all four walls of the next cell: `ds_fl30`, `ds_fr30` for its sides and `ds_f` for its far wall. Expected reading is about 25 cm for a wall and much longer for an opening; measure the exact threshold in Webots.

**What was dropped and what replaces it**

- 20° and 70° pairs → the occupancy grid's memory. Driving forward sweeps every obstacle through the 0°/30° fan before it reaches the flank.
- 45° pair → the 30° pair.
- Back and back-diagonal sensors → one in-place spin at the start (§4); in the maze, the cell behind is always the one you came from.

**Fallback budgets**

| Budget | Drop | Cost |
| --- | --- | --- |
| 6 devices | `imu` | Heading from encoders, re-squared on every wall pair. Fine in noise-free Webots; riskier in the open Section 1 field. |
| 5 devices | `imu` + both 30° rays | Can't see the next cell's side walls, so dead ends must be entered (about +2 cells each). Section 1 relies on in-place scans. Not recommended. |

If the rulebook allows a Webots `Lidar`, one lidar + IMU + camera (3 devices) beats all of these. Check the rules before spending time on it.

## 3. System architecture

The layering stays the same as the current controller: one `Bot.step()` loop, with listeners so mapping and localisation run inside every motion primitive. Only the sensor layer and the maze brain change.

```
┌─ Mission FSM ──────────────────────────────────────────────────────────┐
│ Spin-scan → Section 1 → Gap fix → Explore maze → Final leg → LED+idle  │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ runs
┌─ Task logic ──────────────────────▼────────────────────────────────────┐
│ Section 1 brain        │ ★ Maze brain               │ Safety           │
│ Occupancy grid + A*    │ Next-best-view targets     │ Watchdog +       │
│ Pure pursuit, Bug0     │ Dijkstra (cell, heading)   │ LoopGuard,       │
│ fallback               │                            │ move timeouts    │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ calls
┌─ Primitives + state ──────────────▼────────────────────────────────────┐
│ Motion                 │ Vision                     │ Localisation     │
│ Straight runs, no stop │ Floor colour of next cell  │ Encoders + IMU   │
│ IMU turns, centring    │ 2–3 frame vote             │ Spin-scan, wall  │
│                        │                            │ + post snaps     │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ every primitive steps through
┌─ Hardware loop ───────────────────▼────────────────────────────────────┐
│ Bot.step(), 16 ms: read 7 devices → update odometry → run listeners    │
│ Listeners: grid mapper (S1), wall correction + post-edge snap (S2)     │
└───────────────────────────────────▲────────────────────────────────────┘
                                    │ 7 readings every 16 ms
┌─ Sensors (7 devices) ─────────────┴────────────────────────────────────┐
│  ds_f │ ds_fl30 │ ds_fr30 │ ds_l │ ds_r │ imu │ cam                    │
└────────────────────────────────────────────────────────────────────────┘
```

The FSM runs a section's brain; the brain calls motion and vision; every primitive advances time through `Bot.step()`, which reads the 7 devices and runs the listeners.

**Module map:** `section2.py` gets the new target logic; `hardware.py` and `TwoWheelRobot.proto` get the 7-device suite; `main_controller.py` swaps start detection for the spin-scan.

## 4. Localisation with fewer sensors

Heading comes from the IMU and distance from the encoders, as now. Without a back sensor, every absolute fix uses forward or side rays, plus one spin at the start.

**Start spin-scan (replaces the back sensor and fixes start-detection risk #1).** Spin 360° in place and log `ds_f` against IMU yaw every step. That gives a full range profile, like a slow lidar.

1. Decide S1 or S2 from the profile: a corridor shows two short walls on opposite sides, the open field doesn't. A single cube near the robot can't fake that, which removes risk #1 from the project report.
2. Fix x and y from the profile's minima towards the right wall and the bottom wall.
3. Seed the Section 1 occupancy grid with every ray from the spin, so cubes near the start are known before the first move.
4. Stop the spin facing the first A* waypoint, so no extra turn is needed.

Measure the spin time in Webots; it's the main cost of this design in Section 1.

**Fixes during the run**

| Where | Sensor | Correction |
| --- | --- | --- |
| Below the gap | `ds_f` facing west | x from the left arena wall (unchanged) |
| Maze, driving | `ds_l`, `ds_r` | Lateral offset from wall pairs (listener, unchanged) |
| Maze, driving | `ds_l` / `ds_r` edge | **Post-edge snap:** when a side reading jumps wall → opening (or back), the robot is at a cell boundary; snap the along-track position. New, and cheap. |
| Maze, approaching a wall | `ds_f` | Exact remaining distance (unchanged) |
| Maze, cell centre | `ds_f`, `ds_l`, `ds_r` | Snap x/y wherever a wall is close |

The post-edge snap is what lets the robot drive long runs without stopping: it corrects along-track drift at every wall end it passes.

## 5. Section 1: obstacle field

Keep the tested occupancy grid + A* + pure pursuit; it already runs in 16–17 s. Bug0 from the algorithm doc stays only as the fallback, because a map with memory is what stops the robot from revisiting ground.

**What changes with 5 rays**

- **Mapping:** same 2.5 cm grid, same clear-to-0.6 m / mark-within-0.9 m rule, now from 5 rays. The start spin-scan seeds it.
- **Turns map for free:** every in-place turn sweeps the 0°/30° fan across the field, and the mapper listener runs during turns. Before a large heading change, the robot has already seen what it is turning towards.
- **Unknown = free** stays (optimistic A*). Fewer rays mean more unknown cells, so replanning matters more.
- **Replan hysteresis (new):** switch to a new path only if it is at least 10 % cheaper than the remaining old path, or the old one is blocked. This stops left/right flip-flopping around a cube, which is the main source of repeated movement.
- **Speed from clearance (new):** `v = v_max × clamp((min(ds_f, ds_fl30, ds_fr30) − 0.05) / 0.25, 0.3, 1)`. Full speed in open space, slow only near cubes.

**Fallback chain (unchanged order):** emergency back-off if `ds_f` < 3.5 cm → clear the map near the robot → 2 s Bug0 boundary-follow using `ds_l`/`ds_r`, committed to one side → stuck watchdog (< 2 cm in 3 s).

**End of section:** drive to the point below the top-wall gap, face west, fix x with `ds_f` against the left wall, then drive north into the transition chamber.

## 6. Section 2: maze

The maze brain plans **where to look from**, not which cell to visit. The 30° rays and the camera resolve the next cell without entering it, so a dead end costs a glance instead of a round trip, and cells already seen are never driven into just to check them.

**Cell state.** `walls[4] ∈ {UNK, OPEN, WALL}`, `floor ∈ {UNK, none, blue, green}`, `visited`. A cell is *resolved* when all four walls and its floor are known. The outer boundary is always WALL.

**What one stop at cell c, heading h reveals**

| Sensor | Reveals |
| --- | --- |
| `ds_f` | Front wall of c, and the open run ahead up to 4 cells (every front wall along it) |
| `ds_l`, `ds_r` | Left and right walls of c; far wall of a side cell if the side is open |
| `ds_fl30`, `ds_fr30` | Left and right walls of the next cell c + h |
| `cam` (`ahead` rows) | Floor colour of c + h |

So from (c, h) the next cell c + h is fully resolved: its four walls and its floor. If it's a dead end, its tile is counted right there.

**Exploration: next-best-view over (cell, heading) states**

1. Targets = every reachable state (c, h) with c visited-or-resolved, the edge c → c + h OPEN, and c + h unresolved.
2. Run Dijkstra from the current (cell, heading) with cost 1.0 per cell and w = 0.17 per 90° turn (25 cm cells, from the algorithm doc's time model).
3. Score each target: `cost(c, h) − λ · manhattan(c + h, END)` with END = (4, 9) and λ = 2. The goal-deferral term pushes the branch nearest the end cell to the back of the queue, so exploration finishes close to the end.
4. Drive the best target's path as straight runs, sensing while moving; on arrival, turn to h if needed and sense.
5. Repeat until no target is left, then run the final leg.

Why this works: a side dead end becomes a target state *in the cell beside it*, one 90° turn away (cost 0.17), instead of a 2-cell round trip plus a 180° turn (about 2.3). The planner chooses the glance automatically, with no special case.

The end cell is known in advance here, so the full Manhattan distance is used. The algorithm doc preferred column-only only because its end row was unknown. Re-check λ with `maze_strategy_sim.py` on 4×9 mazes with END fixed at (4, 9).

```python
def pick_view(known, pos, h, w=0.17, lam=2.0, END=(4, 9)):
    dist, prev = dijkstra(known, pos, h, w)          # over (cell, heading)
    best = None
    for (c, hh), cost in dist.items():
        nxt = step(c, hh)
        if known.edge(c, hh) == OPEN and not known.resolved(nxt):
            score = cost - lam * manhattan(nxt, END)
            if best is None or score < best[0]:
                best = (score, (c, hh))
    return best[1] if best else None                  # None -> final leg
```

**Colour counting.** Per-cell register `tiles[cell] = colour`, written once after `stable()` agrees on 2–3 frames. A revisit can't double-count. Majority as now; tie rule from the rulebook, not a guess.

**Final leg.** Dijkstra on (cell, heading) over the finished map to (4, 9), consecutive equal headings merged into one straight run, then stop at the cell centre and light the LED.

**Recovery (unchanged).** A blocked or timed-out run re-localises to the nearest cell and records the wall; `LoopGuard` bans a target for 30 s after 3 failures. Add the stuck `Watchdog` to Section 2 as well (open risk #4 in the report).

## 7. Motion layer

Time is distance over speed plus turn time, so the motion layer stops only where a decision needs a fresh look.

- **Straight runs, no stops:** a planned run of k cells is one `drive_cells(k)`. Side walls are sensed at each cell midpoint while moving, and the post-edge snap corrects along-track drift.
- **Stop only at decision states:** where the plan turns, or where the target view is. A corridor cell with both side walls known is passed at full speed.
- **Centring while driving:** `ω = K·(ds_l − ds_r)` with both walls, single-wall hold with one, IMU heading hold with none. The 30° rays give an earlier, smoother error signal than the 90° ones.
- **Turns:** closed loop on IMU yaw, with a minimum wheel speed so a turn never stalls. Keep the 40 rad/s² acceleration limit (no tipping, no slip).
- **Final leg arcs (optional):** on the known path, replace stop-turn-go corners with arcs. The algorithm doc estimates about 0.4 s saved per 90° corner at 25 cm cells. Add only after the straight-line version is solid.

## 8. Gains, risks and adoption

The biggest gain is in the maze, where 85 % of the run time goes today. Never entering dead ends and planning views instead of visits should cut exploration by roughly a third; that's an estimate, so measure it.

| Risk | Why | Mitigation |
| --- | --- | --- |
| 30° ray misreads the next cell's side wall near a post | Wall thickness and sensor offset shift the hit point | Measure wall vs opening readings in Webots; require 2 agreeing frames |
| Camera tile read from the neighbour is wrong | `ahead` rows may catch the cell beyond | Calibrate rows on a known tile; if unsure, enter that one dead end |
| Start spin is slower than the current start | 360° in place | Stop the spin facing the first waypoint; measure the time |
| Cube hidden between 30° and 90° during a turn | Fewer rays | Grid memory + mapping during turns; emergency back-off stays |
| Fewer rays, less tested | New design | Run all 5 existing test worlds plus tie and near-start-cube worlds |

**Adoption, given the deadline is tomorrow.** Submit the current 12-sensor controller: it's 5/5 on tests. Build this as a branch afterwards (or for Phase 2), in this order, testing after each step:

1. Add next-best-view targets to `section2.py` using the existing 12 sensors (biggest gain, lowest risk).
2. Add the post-edge snap and replan hysteresis.
3. Swap the proto to the 7-device suite and replace the back-sensor start with the spin-scan.
4. Retune thresholds and rerun the full test set.

**Open question:** does the rulebook score fewer sensors, and does it allow a `Lidar`? Both change which budget to build.
