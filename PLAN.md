# Plan: PeraBots 2026 Phase 1 (and Phase 2 carry-over)

## 0. Where things stand

**Done**
- **Robot sensors:** `TwoWheelRobot.proto` has 12 laser distance sensors (front fan ±15/±30°, corners ±45°, sides ±90° mid-face, rear corners ±135°, rear 180°), an `imu`, and a 1×1 downward `floor_cam`.
- **Measured values:**
  - Colors (R, G, B): white (220,220,220), black floor (29,29,29), red (108,0,0), green (0,66,0), blue (0,0,55).
  - Yaw is counter-clockwise positive.
  - Distances are in meters, with 0.5 meaning no hit.
- **Controller scaffold** (`controllers/main_controller/`): `RobotConfiguration`, `MathHelpers`, `MotionProfile` and its builder, `RobotBuilder`, `DifferentialDriveRobot`, and optional `run_monitor.py`. The strategy slots are empty.
- **Fixed:** the rear-corner lasers (they were nested inside `ds_rear`), and the camera height.

**Not yet verified:** the rebuilt proto in the Webots window (reload the world and look), and no algorithm exists yet.

## 0.4 Findings from simulator testing (measured, not assumed)
- **The base robot tips onto its nose.** Its centre of gravity sits 0.43 mm behind the axle and 17 mm above it, with a caster only at the rear. A brake or start could tip it to 37 degrees nose-down, where it stayed. That tilted every distance beam and the camera (the camera read a false bright red).
- **Fix (allowed by rulebook 4.4, wheel position only):** both drive wheels moved 10 mm forward in `TwoWheelRobot.proto`. Over 25 acceleration and braking combinations: 0 mm tipped 15 times, 5 mm tipped 4 times, **10 mm tipped 0 times** (worst pitch 2.6 degrees). Odometry now follows the axle, so sensor poses are measured from `AXLE_FORWARD_OFFSET_METERS`.
- **Colours** (R,G,B) verified on every tile: white 220, black floor 29, red 108, green (0,66,0), blue (0,0,55).
- **Results so far** (headless, 16 ms step): Section 2 alone from Red B explores all 36 cells, finds blue 2 and green 3, reaches Red C and lights GREEN in about 136 s. The full course from Red A takes about 153 s.
- **Fixes found on the way:** the maze entrance must count as a wall (otherwise the robot explores back out into Section 1); the front-wall stopping correction must agree with odometry (otherwise it counts the next cell's wall); Section 1 needs a gate point in front of Red B so it enters the corridor straight.

## 0.45 Optimisation test results (tests/run_ablation.py, 6 start poses per setting, real simulator)
- **Sensors:** essential = `ds_f0`, `ds_l`, `ds_r` (robot fails without them). Helpful = front +-15, +-30, +-45 and the two side-front (+-67) beams. **No measurable effect** = all three rear beams and the two +-113 beams: an 11-sensor lean set (front fan 5, corners 2, sides 2, side-front 2) finishes in 134.9 s against 134.8 s for all 16, with 0 collisions and 0 stalls. The rear beams only matter in rare events (stall back-off, tight pivots) that did not occur.
- **Speed:** straight-line speed is at the motor limit (0.198 of 0.2 m/s). Faster acceleration and harder braking gave nothing (harder braking raises tilt and fails at 6 m/s^2). The time was being lost in the turns: turn gain 5 -> 32, minimum turn rate 1.0, tolerance 0.02 rad cut the whole course from 159 s to about 135 s (15 percent) with no extra collisions.
- **Safe margins:** wider margins did NOT keep the speed up. The gaps are only 25 cm, so bigger obstacle inflation, a wider clearance penalty or an earlier front slow-down made routes longer, caused stalls, or failed. Current margins are near the sweet spot. Only the side-slowdown minimum factor 0.45 -> 0.7 was neutral-to-slightly-better; 1.0 (never slow) causes collisions.
- **Timestep:** 8, 16 and 32 ms all work; 64 ms fails even with the old settings (the physics tips the robot). The package world uses 16 ms.
- **Bugs found and fixed by these tests:** an endless "turning around" loop in the corridor entry (now limited, then Section 1 resumes from the original start); a false red detection from an orange/tilted view (the colour rule now needs the other channels near zero, and red must be confirmed on 4 consecutive level readings); stalls where a wheel pushes a cube while spinning (new no-progress-toward-the-obstacle signal).

## 0.46 Arc corners (maze) - measured
- 90-degree turns through already-visited maze cells are now taken as a quarter circle (radius half a cell, centred on the inside cell corner) instead of stop, pivot and restart. New cells still stop at the centre because their walls must be read there; 180-degree turns at dead ends still pivot.
- Arc speed is limited by the wheels: the outer wheel must stay under 0.2 m/s, so v <= 0.2 / (1 + 0.032 / 0.125) = 0.159 m/s. Tested 0.10 / 0.12 / 0.15 / 0.158: 144 / 137 / 129 / 126 s; chosen 0.155 m/s for a little margin for steering feedback.
- Whole course, mean of 6 start poses at 16 ms: **127.6 s** with arcs, against 134.8 s with pivots only (and 159 s before the turning tuning). 0 collisions, 0 stalls, max pitch 6.5 degrees; also fine at 8 ms (130.3 s) and 32 ms (127.3 s). All three ground-truth maze scenarios still pass.
- Not done: reversing out of dead ends instead of a 180-degree pivot (9 of the 50 quarter-turns; estimated 5-8 s).

## 0.47 Port from the datalab2026 controller (reviewed, no simulation run)
- Added failure-path safety nets only: unexpected-wall abort and replan, 8 s leg timeout, loop guard for repeatedly failing maze targets, Section 1 "no route" handling (forget nearby obstacles, then reactive driving), and an off-by-default early stop. Normal driving is unchanged.
- Offline checks pass (`tests/test_failure_handling.py`, 33 checks). The planner explores 300 random mazes correctly (`tests/test_random_mazes.py`): all cells, all tiles, end reached, 65.8 moves on average against a lower bound of about 52.7.
- NOT yet verified in the simulator: the unexpected-wall and timeout paths inside the driving loops. Run `tests/run_maze_test.py` (three scenarios) and `tests/run_ablation.py baseline` to confirm nothing changed in normal runs before relying on them.
- Candidates that need simulator validation: look-ahead camera for tiles, longer-range beams, skipping resolved cells (fewer visits).

## 0.48 Decision: no absolute wall fixes
Decided not to add absolute pose fixes against the arena walls (nor a maze cell-centre snap). The controller stays start-relative, with no hard-coded arena coordinates beyond the existing soft Section 1 goal prior. Revisit only for the Phase 2 hardware robot, where wheel slip and gyro drift will matter.

## 0.5 Board layout (confirmed by the team)
One continuous board. Red squares mark every start and end:
- **Red A (bottom left of Section 1):** start of Section 1, in a corner of the arena.
- **Red B (between the white field and the maze):** end of Section 1 and start of Section 2. It sits in a short corridor with walls on both sides. **Separate Section 2 attempts start the robot on Red B.**
- **Red C (inside the maze):** end of Section 2.

Markers are always red. Terms: "Section 1/2" are parts of the board; "Phase 1/2" are online and physical rounds.

## 1. Rules to design around
- Locked: Arena, Obstacles, chassis, wheels, motors, LEDs, names `TwoWheelRobot.proto`, `controllers/main_controller/`, `main_controller.py`.
- Permitted: the devices listed in rulebook 4.4. The committee allowed Lidar, which we are deferring.
- No hard-coding of the obstacle layout, maze walls, tile positions or routes.
- Section 2 has its own attempts, possibly from the entrance. Attempts end after 15 s without progress or 2 s outside the track.
- LED within 3 s of reaching the end cell. The rulebook has no tie rule (see section 8).

## 0.49 Look-ahead camera (built and simulator-checked, switched OFF by decision)

- Decision: OFF. §4.6 lists "successful and sufficient exploration" first; skipping dead ends could count against it for a 2-14 s gain. Committee question to add in §8: does reading a dead-end tile from the neighbouring cell count as exploring it? Switch on with `LOOK_AHEAD_ENABLED = 1` if yes.
- The rules give no disqualification for reaching the red cell early; the robot never lights an LED before every reachable cell is visited. Open question: does passing through the end cell mid-exploration start the 3 s clock?

- `look_cam` (32x24, fov 1.0, 6 cm high, tilted 0.6 rad) sits next to the downward `floor_cam`, which stays the primary colour sensor. `LOOK_AHEAD_ENABLED = 0` by default; without the device the robot just enters every cell.
- A cell ahead is skipped only if ALL hold: door to it is known open, robot within 2 cm of the cell centre along and across the corridor facing the corridor within 0.05 rad, `ds_f0` ~ 0.329 (wall at the end of the NEXT cell), `ds_fl30`/`ds_fr30` ~ 0.198 (the next cell's side walls; the earlier 0.25 estimate was wrong, geometry gives (0.125-0.006-0.02)/sin 30 deg), and >= 70% of the sampled centre pixels agree on one colour (dark or washed-out pixels = UNKNOWN = enter).
- The peeked cell is stored as visited with its three far walls, and counted for BLUE/GREEN or as the end cell for RED; a route heading for it is stopped smoothly (`brake_to_stop`) instead of entering.
- Offline: 25 new checks in `tests/test_failure_handling.py` pass; 300 random mazes still 0 failures (planner unchanged).
- Simulator results (Webots, headless): real maze, 3 scenarios: 0 wrong walls, all 5 tiles, end cell (8,3), 0 contradictions. LED time (Section 2 only, same harness): 106.1 s -> 99.3 s; whole course 130.9 s -> 117.0 s (69 -> 61-65 moves).
- 10 random mazes (`tests/run_random_sim.py`, look-ahead OFF and ON): all correct walls/tiles/end/majority in both modes; mean LED time 98.4 s -> 96.1 s (saving 2.2 s; the gain depends on how many dead ends are approached head-on, 0-4 per maze).
- Found in testing: the look camera once reported RED mean (207,0,0) for cell (5,3), whose floor is black (the floor camera confirms black). Pitch was ~0 then, so the cause is NOT established (a red wall face or the far red end tile are candidates). Fix: RED from the look camera is never trusted, and a tilt guard (0.04 rad) was added. BLUE/GREEN/BLACK readings matched the truth in every run, but the unexplained red is the reason to keep the 70% agreement rule and the three-beam proof.
- `tests/run_maze_test.py` still fails its last two checks ("visit order" and "cell moves" equal the OLD planner) when look-ahead is on - expected, those compare with a plan that enters every cell.

## 0.50 Entrance-biased exploration (requested; offline-verified)

- `START_BIAS_WEIGHT = 2.0`: `find_route` ranks unvisited cells by route cost + 2 x route distance from the entrance (cell (0,0)). Offline on the real maze: 69 moves, est. 129.6 s (was 128.1 s with plain nearest-first); the pocket next to the entrance (cells (1,0),(1,1)) is visited at moves 14-23 instead of 45-58, and the red end cell is first entered at move 63 of 69 instead of 35. Weight 3: 81 moves / 155.8 s, weight 4: 93 moves / 182 s. 300 random mazes: 0 failures, mean 65.3 moves (65.8 before).
- Not yet run in Webots with this change.

## 0.51 Section 1 clearance (user report: robot passes too close to cubes/boundary; offline-checked, NOT yet run in Webots)

- Cause found offline (`tests/test_section1_clearance.py`, true cube/wall map, robot's own A* + smoothing): `smooth()` replaced A* waypoints by straight shortcuts that ignored the near-obstacle penalty A* uses to centre the robot in gaps. One shortcut passed 6.7 cm from the axle (rear corner reach is 5.6 cm).
- Change: `SMOOTH_RESPECTS_CLEARANCE = 1`: a shortcut is taken only if it touches no more penalised cells than the A* part it replaces. Offline: smoothed-route clearance away from the start pose rose from 0.067 m to >= 0.081 m; route 3.18 m -> 3.28 m (+3 %, about +0.5 s).
- Not solved: pure-pursuit corner cutting (carrot = first waypoint >= 0.15 m away). The crude offline tracker gives 0.044-0.108 m with the new smoothing, so tracking still needs a Webots measurement (supervisor monitor over the 6 start poses) before judging. Set `SMOOTH_RESPECTS_CLEARANCE = 0` to restore the validated behaviour (127.6 s mean, 0 collisions, 6 poses).

## 0.52 Sonar test and side-slowdown (Webots, 6 start poses, one run per pose, deterministic)

- Added `ds_sonar` (type sonar, 9 rays, aperture 2.0 rad, front) and `SONAR_ENABLED`. Result: sonar OFF 127.1 s / ON 127.1 s (no difference); sonar ON + `SIDE_CLEARANCE_MINIMUM_SPEED_FACTOR=1.0`: 123.5 s, 0 collisions, 0 stalls. Control with sonar OFF + factor 1.0: 123.45 s, 0 collisions, 0 stalls, same clearances. So the gain is from removing the side slowdown (now safe because of the clearance-aware smoothing, 0.51); the sonar adds nothing measurable.
- Decision: the sonar was REMOVED (proto node, controller code and test mode deleted); `SIDE_CLEARANCE_MINIMUM_SPEED_FACTOR = 1.0` kept. Baseline before: 127.1 s, 1 collision, 1 stall (south-west pose).
- Closest approach (monitor footprint 9 x 7.6 cm, slightly larger than the body) is still ~0 in the south-west pose in every variant: tracking/corner cutting is not solved, only measured.

## 2. Startup (every attempt)
1. Let sensors settle for a few steps.
2. Choose the mode from the floor: black = Section 2, white = Section 1. On red, use the walls: walls close on **opposite** sides (left and right) = Red B, Section 2 start; walls close on two **adjacent** sides (rear and one side) or none = Red A, Section 1 start. On Red B the heading is unknown: drive forward a little along the corridor, and if the floor turns white you are facing Section 1, so turn 180 degrees.
3. Save the start yaw and set odometry to (0, 0).
4. Check that every beam reads ≥ about 0.04 m, then run a smooth 45° one-way scan (all gaps are now ≤ 45°), binning by bearing from the robot's center.
5. Pick a heading and go.

## 3. Mapping (shared by both sections)
- An occupancy grid of 2.5 cm cells: free along each ray, occupied at the hit, nothing marked on no-hit.
- Marked dynamically, no pre-marked walls. It lives in the start-relative frame, using odometry and IMU yaw.
- A plain class with no Webots calls, so it carries over to the real robot. Measured cost is small: 0.02 ms per update and about 8 ms for a full A\*.

**Scan triggers (scanning is not continuous).** Driving forward, the beams keep updating the map on their own (front fan is 15° apart, so cubes are seen). Blind spots are the wider gaps between beams at the sides and rear (up to 45°), and they depend on the heading. Scan only when:
1. Startup (heading unknown).
2. Blocked ahead or close to a wall (front clearance below a threshold).
3. About to turn into a sector that is still unknown on the map. Do it as part of the turn: rotate slowly enough to sample every few degrees, so the turn is the scan.

Avoiding walls is separate and reactive: slow by clearance, back off if a corner beam reads very close.

## 4. Section 1 (obstacle field)
1. Inflate obstacles by about 6 cm.
2. A\* on free and unknown cells with a clearance cost, so it favors the middle of gaps. Replan when the map changes near the path.
3. Follow with pure pursuit, using `MotionProfile` (slow by clearance, limited acceleration).
4. Watch for loops and stalls. If stuck, back off and replan.
5. The red transition is the end; then hand over to Section 2.
6. Longest-free-ray is only a tie-break, not the goal (dead ends and corners break it).

## 5. Section 2 (maze)
1. Relative frame, start cell = (0, 0). Don't assume the start cell or heading. In a **continuous run** the robot arrives on Red B already moving forward and simply continues (Red B's side walls straighten it). Switch to Section 2 mode when the camera sees red with walls on both sides. The heading check only matters for separate attempts where the evaluators place the robot.
2. At each cell center, read three booleans (wall in front, left and right, thresholds about 0.16 m). Use the IMU for angle and the side beams for offset.
3. Explore by going to the nearest unexplored cell through known cells. Smooth arcs are an option later.
   - **Do not turn at every gap.** Record each side opening as a frontier (a known free cell next to an unknown one) and keep driving. The side beams see the opening beside you and the 45° corner beams see it slightly early.
   - Choose the next frontier by distance **plus a penalty per turn**, and prefer finishing the current corridor first. This avoids zigzagging and loops.
   - Optional look-ahead: cast the straight path through the grid and list unknown cells within sensor range beside it. Those are what you would miss by not stopping.
4. Count blue and green per cell (once each). The red end cell is found by color.
5. When nothing is left to explore, take the shortest route to the end cell.
6. On arrival, stop and light the majority LED at once. Tie: the rulebook defines nothing, so pick a fixed default and keep it in `RobotConfiguration`.
7. Count tiles while traversing. Open decision: explore every reachable dead end (safe for "correct majority") versus stopping early (shorter, but the majority can be wrong). Default to the full sweep until the committee defines "sufficient exploration".

## 6. Testing
- Sensor and color checks in a headless copy (already done for colors and sensors).
- Turn on `RunMonitor` to watch jerk, path and clearance.
- Test starting from: the start zone facing various ways, and the Section 2 entrance.
- Time the per-step cost.

## 7. Submission
Set `ENABLE_RUN_MONITOR = False` and delete `run_monitor.py`. Check the zip layout, and don't put team names in internal files.

## 8. Committee questions

**Answered**
- Section 2 attempts start on Red B (the middle red square). The heading does not matter: in a continuous run the robot arrives moving forward, and for placed starts the startup check handles either direction.
- Markers are always red.
- Tie: the rules define nothing; count tiles as the robot passes them.
- Red A pose and heading: does not matter. The startup scan finds the open directions. Direction to Red B comes from the IMU's world yaw as a soft bias (the arena is fixed in the world); if that assumption is wrong, the robot falls back to exploring until the camera sees red.

**Still open (ask in writing)**
1. What counts as "sufficient exploration" of the maze for the majority? Is it all tiles in the maze, or the tiles the robot encounters?
2. Does the evaluation world override `extensionSlot`? (If so our sensors vanish.) Ask this first.

## 9. Order of work
1. Reload and eyeball the proto.
2. Occupancy grid plus ASCII dump.
3. Startup mode detection.
4. Section 2 (simplest, grid-aligned).
5. Section 1 planning.
6. Smoothness tuning.
7. Final checks.

The biggest risk is question 2 above (extensionSlot). Ask that first.
