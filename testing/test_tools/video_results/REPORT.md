# Full Webots test suite

Controller: `PeraBots_UC/controllers/main_controller` (live code, via `test_run/`). Webots R2025a, Webots window shown with 3D rendering, fast mode, every run recorded to MP4. Arena outline, maze grid (4 x 9 cells, 25 cm), wall size and markers are identical to `Arena.proto`; only internal maze walls, tiles and Section 1 obstacles change.

Each design runs twice: **full** (start zone -> Section 1 -> maze) and **S2** (dedicated Section 2 attempt, robot placed on the transition tile). A run **PASSES** when it finishes with the correct LED, whole body inside the end cell, never enters the end cell early, never goes 15 s without progress and stays still after the LED. Missed tiles and collisions are listed as warnings.

## Summary

| Run | Verdict | LED / expected | In end cell (margin) | Finish t (s) | Maze t (s) | Distance | Collisions S1/maze | Tiles found | Tiles true | Longest no-progress |
|---|---|---|---|---|---|---|---|---|---|---|
| OFF Official arena (full) | **PASS** | green / green | yes (7.9 cm) | 117.1 | 96.3 | 13.1 m | 0 / 0 | B2 G3 | B2 G3 | 3 s |
| OFF Official arena (s2) | **PASS** | green / green | yes (7.9 cm) | 99.4 | 96.5 | 10.5 m | 0 / 0 | B2 G3 | B2 G3 | 3 s |
| D1 Snake (full) | **PASS** | green / green | yes (8.2 cm) | 109.0 | 88.8 | 13.3 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| D1 Snake (s2) | **PASS** | green / green | yes (8.2 cm) | 91.7 | 88.8 | 10.8 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| D2 Open hall (full) | **PASS** | green / green | yes (7.7 cm) | 104.2 | 84.2 | 12.4 m | 0 / 0 | B2 G3 | B2 G3 | 3 s |
| D2 Open hall (s2) | **PASS** | green / green | yes (7.7 cm) | 86.9 | 84.0 | 9.9 m | 0 / 0 | B2 G3 | B2 G3 | 3 s |
| D3 Comb (full) | **PASS** | blue / blue | yes (8.2 cm) | 150.2 | 130.2 | 17.3 m | 0 / 0 | B4 G3 | B4 G3 | 3 s |
| D3 Comb (s2) | **PASS** | blue / blue | yes (8.2 cm) | 133.0 | 130.1 | 14.8 m | 0 / 0 | B4 G3 | B4 G3 | 3 s |
| D4 Deep branches (full) | **PASS** | green / green | yes (8.2 cm) | 133.9 | 113.8 | 17.4 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| D4 Deep branches (s2) | **PASS** | green / green | yes (8.2 cm) | 116.6 | 113.8 | 14.9 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| D5 Loops (full) | **PASS** | green / green | yes (7.7 cm) | 114.2 | 94.2 | 13.8 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| D5 Loops (s2) | **PASS** | green / green | yes (7.7 cm) | 97.4 | 94.6 | 11.4 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| R1 Random 1 (full) | **PASS** | green / green | yes (7.2 cm) | 117.6 | 97.5 | 13.5 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| R1 Random 1 (s2) | **PASS** | green / green | yes (7.2 cm) | 100.3 | 97.5 | 10.9 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| R2 Random 2 (full) | **PASS** | green / green | yes (7.2 cm) | 96.0 | 76.0 | 11.2 m | 0 / 0 | B0 G1 | B0 G1 | 3 s |
| R2 Random 2 (s2) | **PASS** | green / green | yes (7.2 cm) | 78.7 | 75.9 | 8.8 m | 0 / 0 | B0 G1 | B0 G1 | 3 s |
| R3 Random 3 (full) | **PASS** | green / green | yes (7.2 cm) | 95.5 | 75.5 | 11.3 m | 0 / 0 | B0 G1 | B0 G1 | 3 s |
| R3 Random 3 (s2) | **PASS** | green / green | yes (7.2 cm) | 78.2 | 75.4 | 8.8 m | 0 / 0 | B0 G1 | B0 G1 | 3 s |
| A1 Staircase (full) | **PASS** | green / green | yes (8.0 cm) | 179.0 | 158.9 | 19.8 m | 0 / 0 | B2 G3 | B2 G3 | 3 s |
| A1 Staircase (s2) | **PASS** | green / green | yes (8.0 cm) | 161.8 | 158.9 | 17.3 m | 0 / 0 | B2 G3 | B2 G3 | 3 s |
| A2 Nine tiles (full) | **PASS** | green / green | yes (8.2 cm) | 149.3 | 129.3 | 18.1 m | 0 / 0 | B4 G5 | B4 G5 | 4 s |
| A2 Nine tiles (s2) | **PASS** | green / green | yes (8.2 cm) | 132.5 | 129.6 | 15.6 m | 0 / 0 | B4 G5 | B4 G5 | 4 s |
| A3 Long corridors (full) | **PASS** | green / green | yes (7.2 cm) | 128.6 | 108.6 | 17.8 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| A3 Long corridors (s2) | **PASS** | green / green | yes (7.2 cm) | 111.8 | 109.0 | 15.3 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| A4 Islands (full) | **PASS** | blue / blue | yes (7.2 cm) | 114.2 | 94.2 | 12.9 m | 0 / 0 | B2 G1 | B2 G1 | 3 s |
| A4 Islands (s2) | **PASS** | blue / blue | yes (7.2 cm) | 97.0 | 94.1 | 10.4 m | 0 / 0 | B2 G1 | B2 G1 | 3 s |
| A5 Behind the end (full) | **PASS** | blue / blue | yes (7.2 cm) | 115.2 | 95.2 | 14.4 m | 0 / 0 | B3 G2 | B3 G2 | 3 s |
| A5 Behind the end (s2) | **PASS** | blue / blue | yes (7.2 cm) | 98.4 | 95.5 | 11.9 m | 0 / 0 | B3 G2 | B3 G2 | 3 s |
| A6 Rooms (full) | **PASS** | green / green | yes (7.8 cm) | 124.3 | 104.2 | 14.1 m | 0 / 0 | B2 G3 | B2 G3 | 3 s |
| A6 Rooms (s2) | **PASS** | green / green | yes (7.8 cm) | 107.5 | 104.7 | 11.6 m | 0 / 0 | B2 G3 | B2 G3 | 3 s |
| A7 Ladder (full) | **PASS** | green / green | yes (8.0 cm) | 128.2 | 108.2 | 14.2 m | 0 / 0 | B3 G4 | B3 G4 | 3 s |
| A7 Ladder (s2) | **PASS** | green / green | yes (8.0 cm) | 110.9 | 108.0 | 11.6 m | 0 / 0 | B3 G4 | B3 G4 | 3 s |
| A8 Spiral (full) | **PASS** | blue / blue | yes (7.2 cm) | 130.1 | 110.0 | 18.7 m | 0 / 0 | B1 G0 | B1 G0 | 3 s |
| A8 Spiral (s2) | **PASS** | blue / blue | yes (7.2 cm) | 112.8 | 109.9 | 16.1 m | 0 / 0 | B1 G0 | B1 G0 | 3 s |
| A9 Hard perfect 1 (full) | **PASS** | blue / blue | yes (7.9 cm) | 99.8 | 79.8 | 11.3 m | 0 / 0 | B4 G3 | B4 G3 | 3 s |
| A9 Hard perfect 1 (s2) | **PASS** | blue / blue | yes (7.9 cm) | 82.6 | 79.7 | 8.9 m | 0 / 0 | B4 G3 | B4 G3 | 3 s |
| A10 Hard perfect 2 (full) | **PASS** | green / green | yes (7.9 cm) | 121.0 | 101.0 | 13.5 m | 0 / 0 | B3 G4 | B3 G4 | 4 s |
| A10 Hard perfect 2 (s2) | **PASS** | green / green | yes (7.9 cm) | 103.7 | 100.8 | 11.0 m | 0 / 0 | B3 G4 | B3 G4 | 4 s |
| A11 Braided 1 (full) | **PASS** | blue / blue | yes (7.8 cm) | 114.2 | 94.1 | 13.5 m | 0 / 0 | B2 G1 | B2 G1 | 3 s |
| A11 Braided 1 (s2) | **PASS** | blue / blue | yes (7.8 cm) | 97.0 | 94.1 | 10.9 m | 0 / 0 | B2 G1 | B2 G1 | 3 s |
| A12 Braided 2 (full) | **PASS** | green / green | yes (7.2 cm) | 110.4 | 90.4 | 12.8 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| A12 Braided 2 (s2) | **PASS** | green / green | yes (7.2 cm) | 93.1 | 90.3 | 10.3 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |

**42 / 42 runs passed.**

## Comparison: previous controller (backup_controller_2026-10-28)

Same worlds and judge. Maze time / cell moves / re-driven moves (moves back into a cell already visited), shown as previous controller (backup_controller_2026-10-28) -> controller under test.

| Run | Verdict | Maze t (s) | Cell moves | Re-driven moves |
|---|---|---|---|---|
| OFF Official arena (full) | PASS -> PASS | 148 -> 96 | 47 -> 39 | 17 -> 13 |
| OFF Official arena (s2) | PASS -> PASS | 148 -> 96 | 47 -> 39 | 17 -> 13 |
| D1 Snake (full) | PASS -> PASS | 125 -> 89 | 43 -> 41 | 9 -> 8 |
| D1 Snake (s2) | PASS -> PASS | 124 -> 89 | 43 -> 41 | 9 -> 8 |
| D2 Open hall (full) | PASS -> PASS | 109 -> 84 | 37 -> 37 | 11 -> 11 |
| D2 Open hall (s2) | PASS -> PASS | 110 -> 84 | 37 -> 37 | 11 -> 11 |
| D3 Comb (full) | PASS -> PASS | 187 -> 130 | 63 -> 55 | 29 -> 25 |
| D3 Comb (s2) | PASS -> PASS | 186 -> 130 | 63 -> 55 | 29 -> 25 |
| D4 Deep branches (full) | PASS -> PASS | 164 -> 114 | 63 -> 57 | 28 -> 25 |
| D4 Deep branches (s2) | PASS -> PASS | 164 -> 114 | 63 -> 57 | 28 -> 25 |
| D5 Loops (full) | PASS -> PASS | 157 -> 94 | 55 -> 43 | 24 -> 13 |
| D5 Loops (s2) | PASS -> PASS | 157 -> 95 | 55 -> 43 | 24 -> 13 |
| R1 Random 1 (full) | PASS -> PASS | 134 -> 97 | 43 -> 41 | 14 -> 11 |
| R1 Random 1 (s2) | PASS -> PASS | 134 -> 97 | 43 -> 41 | 14 -> 11 |
| R2 Random 2 (full) | PASS -> PASS | 102 -> 76 | 33 -> 33 | 1 -> 1 |
| R2 Random 2 (s2) | PASS -> PASS | 102 -> 76 | 33 -> 33 | 1 -> 1 |
| R3 Random 3 (full) | PASS -> PASS | 113 -> 76 | 37 -> 33 | 6 -> 2 |
| R3 Random 3 (s2) | PASS -> PASS | 113 -> 75 | 37 -> 33 | 6 -> 2 |
| A1 Staircase (full) | PASS -> PASS | 235 -> 159 | 71 -> 65 | 39 -> 36 |
| A1 Staircase (s2) | PASS -> PASS | 236 -> 159 | 71 -> 65 | 39 -> 36 |
| A2 Nine tiles (full) | PASS -> PASS | 187 -> 129 | 65 -> 59 | 35 -> 33 |
| A2 Nine tiles (s2) | PASS -> PASS | 187 -> 130 | 65 -> 59 | 35 -> 33 |
| A3 Long corridors (full) | PASS -> PASS | 156 -> 109 | 65 -> 59 | 30 -> 27 |
| A3 Long corridors (s2) | PASS -> PASS | 156 -> 109 | 65 -> 59 | 30 -> 27 |
| A4 Islands (full) | FAIL -> PASS | none -> 94 | 146 -> 39 | 137 -> 12 |
| A4 Islands (s2) | FAIL -> PASS | none -> 94 | 151 -> 39 | 142 -> 12 |
| A5 Behind the end (full) | PASS -> PASS | 154 -> 95 | 53 -> 45 | 25 -> 21 |
| A5 Behind the end (s2) | PASS -> PASS | 154 -> 96 | 53 -> 45 | 25 -> 21 |
| A6 Rooms (full) | PASS -> PASS | 150 -> 104 | 47 -> 43 | 17 -> 15 |
| A6 Rooms (s2) | PASS -> PASS | 150 -> 105 | 47 -> 43 | 17 -> 15 |
| A7 Ladder (full) | FAIL -> PASS | none -> 108 | 237 -> 43 | 224 -> 19 |
| A7 Ladder (s2) | FAIL -> PASS | none -> 108 | 245 -> 43 | 232 -> 19 |
| A8 Spiral (full) | PASS -> PASS | 146 -> 110 | 63 -> 63 | 29 -> 29 |
| A8 Spiral (s2) | PASS -> PASS | 145 -> 110 | 63 -> 63 | 29 -> 29 |
| A9 Hard perfect 1 (full) | PASS -> PASS | 106 -> 80 | 33 -> 33 | 5 -> 5 |
| A9 Hard perfect 1 (s2) | PASS -> PASS | 107 -> 80 | 33 -> 33 | 5 -> 5 |
| A10 Hard perfect 2 (full) | PASS -> PASS | 133 -> 101 | 41 -> 41 | 13 -> 13 |
| A10 Hard perfect 2 (s2) | PASS -> PASS | 133 -> 101 | 41 -> 41 | 13 -> 13 |
| A11 Braided 1 (full) | PASS -> PASS | 142 -> 94 | 47 -> 41 | 17 -> 13 |
| A11 Braided 1 (s2) | PASS -> PASS | 143 -> 94 | 47 -> 41 | 17 -> 13 |
| A12 Braided 2 (full) | PASS -> PASS | 119 -> 90 | 39 -> 39 | 11 -> 11 |
| A12 Braided 2 (s2) | PASS -> PASS | 119 -> 90 | 39 -> 39 | 11 -> 11 |

Full results of the previous controller (backup_controller_2026-10-28):

| Run | Verdict | LED / expected | In end cell (margin) | Finish t (s) | Maze t (s) | Distance | Collisions S1/maze | Tiles found | Tiles true | Longest no-progress |
|---|---|---|---|---|---|---|---|---|---|---|
| OFF Official arena (full) | **PASS** | green / green | yes (7.9 cm) | 169.9 | 148.4 | 15.1 m | 0 / 0 | B2 G3 | B2 G3 | 3 s |
| OFF Official arena (s2) | **PASS** | green / green | yes (7.9 cm) | 151.7 | 148.3 | 12.5 m | 0 / 0 | B2 G3 | B2 G3 | 3 s |
| D1 Snake (full) | **PASS** | green / green | yes (8.0 cm) | 145.4 | 124.5 | 13.8 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| D1 Snake (s2) | **PASS** | green / green | yes (8.0 cm) | 127.7 | 124.3 | 11.3 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| D2 Open hall (full) | **PASS** | green / green | yes (7.7 cm) | 130.1 | 109.3 | 12.3 m | 0 / 0 | B2 G3 | B2 G3 | 4 s |
| D2 Open hall (s2) | **PASS** | green / green | yes (7.7 cm) | 113.3 | 109.9 | 9.9 m | 0 / 0 | B2 G3 | B2 G3 | 4 s |
| D3 Comb (full) | **PASS** | blue / blue | yes (8.0 cm) | 207.4 | 186.6 | 19.2 m | 0 / 0 | B4 G3 | B4 G3 | 4 s |
| D3 Comb (s2) | **PASS** | blue / blue | yes (8.0 cm) | 189.6 | 186.2 | 16.7 m | 0 / 0 | B4 G3 | B4 G3 | 4 s |
| D4 Deep branches (full) | **PASS** | green / green | yes (8.1 cm) | 184.8 | 163.9 | 18.9 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| D4 Deep branches (s2) | **PASS** | green / green | yes (8.1 cm) | 167.5 | 164.1 | 16.4 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| D5 Loops (full) | **PASS** | green / green | yes (7.0 cm) | 177.6 | 156.8 | 16.9 m | 0 / 0 | B1 G2 | B1 G2 | 4 s |
| D5 Loops (s2) | **PASS** | green / green | yes (7.0 cm) | 160.3 | 156.9 | 14.4 m | 0 / 0 | B1 G2 | B1 G2 | 4 s |
| R1 Random 1 (full) | **PASS** | green / green | yes (7.0 cm) | 154.6 | 133.7 | 14.0 m | 0 / 0 | B1 G2 | B1 G2 | 4 s |
| R1 Random 1 (s2) | **PASS** | green / green | yes (7.0 cm) | 137.3 | 133.9 | 11.4 m | 0 / 0 | B1 G2 | B1 G2 | 4 s |
| R2 Random 2 (full) | **PASS** | green / green | yes (7.1 cm) | 122.9 | 102.1 | 11.2 m | 0 / 0 | B0 G1 | B0 G1 | 4 s |
| R2 Random 2 (s2) | **PASS** | green / green | yes (7.1 cm) | 105.6 | 102.2 | 8.7 m | 0 / 0 | B0 G1 | B0 G1 | 4 s |
| R3 Random 3 (full) | **PASS** | green / green | yes (7.1 cm) | 133.4 | 112.7 | 12.3 m | 0 / 0 | B0 G1 | B0 G1 | 3 s |
| R3 Random 3 (s2) | **PASS** | green / green | yes (7.1 cm) | 116.2 | 112.8 | 9.8 m | 0 / 0 | B0 G1 | B0 G1 | 3 s |
| A1 Staircase (full) | **PASS** | green / green | yes (8.0 cm) | 256.3 | 235.4 | 21.4 m | 0 / 0 | B2 G3 | B2 G3 | 4 s |
| A1 Staircase (s2) | **PASS** | green / green | yes (8.0 cm) | 239.0 | 235.6 | 18.8 m | 0 / 0 | B2 G3 | B2 G3 | 4 s |
| A2 Nine tiles (full) | **PASS** | green / green | yes (8.1 cm) | 207.4 | 186.6 | 19.6 m | 0 / 0 | B4 G5 | B4 G5 | 3 s |
| A2 Nine tiles (s2) | **PASS** | green / green | yes (8.1 cm) | 190.1 | 186.7 | 17.1 m | 0 / 0 | B4 G5 | B4 G5 | 3 s |
| A3 Long corridors (full) | **PASS** | green / green | yes (7.0 cm) | 176.6 | 155.9 | 19.2 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| A3 Long corridors (s2) | **PASS** | green / green | yes (7.0 cm) | 159.4 | 156.0 | 16.7 m | 0 / 0 | B1 G2 | B1 G2 | 3 s |
| A4 Islands (full) | **FAIL** | – / blue | no | – | – | 43.4 m | 0 / 0 | – | B2 G1 | 3 s |
| A4 Islands (s2) | **FAIL** | – / blue | no | – | – | 42.2 m | 0 / 0 | – | B2 G1 | 3 s |
| A5 Behind the end (full) | **PASS** | blue / blue | yes (7.0 cm) | 175.2 | 154.4 | 16.4 m | 0 / 0 | B3 G2 | B3 G2 | 4 s |
| A5 Behind the end (s2) | **PASS** | blue / blue | yes (7.0 cm) | 157.4 | 154.0 | 13.9 m | 0 / 0 | B3 G2 | B3 G2 | 4 s |
| A6 Rooms (full) | **PASS** | green / green | yes (7.8 cm) | 171.4 | 150.5 | 15.0 m | 0 / 0 | B2 G3 | B2 G3 | 4 s |
| A6 Rooms (s2) | **PASS** | green / green | yes (7.8 cm) | 153.6 | 150.2 | 12.5 m | 0 / 0 | B2 G3 | B2 G3 | 4 s |
| A7 Ladder (full) | **FAIL** | – / green | no | – | – | 64.5 m | 0 / 0 | – | B3 G4 | 3 s |
| A7 Ladder (s2) | **FAIL** | – / green | no | – | – | 64.0 m | 0 / 0 | – | B3 G4 | 3 s |
| A8 Spiral (full) | **PASS** | blue / blue | yes (7.0 cm) | 166.6 | 145.7 | 18.7 m | 0 / 0 | B1 G0 | B1 G0 | 3 s |
| A8 Spiral (s2) | **PASS** | blue / blue | yes (7.0 cm) | 148.8 | 145.4 | 16.1 m | 0 / 0 | B1 G0 | B1 G0 | 3 s |
| A9 Hard perfect 1 (full) | **PASS** | blue / blue | yes (7.9 cm) | 127.2 | 106.4 | 11.3 m | 0 / 0 | B4 G3 | B4 G3 | 4 s |
| A9 Hard perfect 1 (s2) | **PASS** | blue / blue | yes (7.9 cm) | 109.9 | 106.5 | 8.8 m | 0 / 0 | B4 G3 | B4 G3 | 4 s |
| A10 Hard perfect 2 (full) | **PASS** | green / green | yes (7.9 cm) | 153.6 | 132.8 | 13.4 m | 0 / 0 | B3 G4 | B3 G4 | 5 s |
| A10 Hard perfect 2 (s2) | **PASS** | green / green | yes (7.9 cm) | 136.3 | 132.9 | 10.9 m | 0 / 0 | B3 G4 | B3 G4 | 5 s |
| A11 Braided 1 (full) | **PASS** | blue / blue | yes (7.7 cm) | 163.2 | 142.3 | 15.0 m | 0 / 0 | B2 G1 | B2 G1 | 4 s |
| A11 Braided 1 (s2) | **PASS** | blue / blue | yes (7.7 cm) | 145.9 | 142.5 | 12.4 m | 0 / 0 | B2 G1 | B2 G1 | 4 s |
| A12 Braided 2 (full) | **PASS** | green / green | yes (7.0 cm) | 139.7 | 118.9 | 12.8 m | 0 / 0 | B1 G2 | B1 G2 | 4 s |
| A12 Braided 2 (s2) | **PASS** | green / green | yes (7.0 cm) | 122.4 | 119.0 | 10.3 m | 0 / 0 | B1 G2 | B1 G2 | 4 s |

**38 / 42 runs passed with the previous controller (backup_controller_2026-10-28).**

## Findings

**Current controller: 42 / 42 runs pass.** 21 arenas (official, D1-D5, R1-R3, A1-A12), each as a full run and a dedicated Section 2 attempt. Every run: correct LED, every tile found with the right colour, whole body inside the end cell (7.0-8.2 cm margin), no collisions, never entered the end cell early, still after the LED. Longest standstill in any run 3.7 s (rule limit 15 s). The controller at the start of the session passed 38 / 42 (looped until the time limit on A4 Islands and A7 Ladder).

**Speed.** Official arena full run 169.9 s at the start of the session -> 152.6 s (changes 1-3) -> 117.1 s now (Section 1 20.1 s, maze 96.3 s; Section 2 attempt 99.4 s). Over the 19 arenas the original controller could finish, full runs total 3163 s -> 2310 s (-27%). Full runs in all 21 arenas: 2552 s, 949 cell moves (original: 1328 moves including its looping runs).

**Change 1 - commit to the chosen target (fixes livelocks).** pick_target() re-optimised the whole tour at every stop while the robot drives only the first straight run of it, so two near-equal tours could alternate forever (A4, A7). The target chosen at a stop is now kept while it is still unresolved and reachable.

**Change 2 - glance into a dead end beside the robot before leaving.** A known dead end right beside the robot is viewed immediately (one turn) instead of being left for a later come-back that re-drives the same cells (R3: re-driven moves 6 -> 2).

**Change 3 - don't drive into a dead end whose tile the camera already saw.** Tiles only lie in dead ends, so a cell where the camera sees a blue/green tile from the neighbouring cell is counted, marked done and sealed (its other sides recorded as walls), and a planned drive into it is skipped. Before, the robot drove into 37 of the 82 tiles it had already seen. Full runs 5.7% faster in total.

**Change 4 - full speed over cells already driven through.** Straight runs of 2+ cells over visited cells use S2_KNOWN = 10 rad/s (motor maximum) instead of S2_CRUISE = 8: nothing new to scan there. Full runs 1.7% faster in total; A8 Spiral 9 s faster.

**Change 5 - quicker stops and turns (motion.py).** drive_axis brakes at a constant DRIVE_DECEL = 0.5 m/s^2 (full speed until ~4 cm before the stop) instead of slowing proportionally over the last 8 cm and creeping the final centimetres; turn_to brakes at TURN_DECEL = 15 rad/s^2 and finishes the last ~10 deg with TURN_K_END = 20 instead of a soft P controller. About 1 s saved at every stop: all 42 runs 22% faster (6105 -> 4743 s), Section 1 about 1 s faster, end-cell margins unchanged or slightly larger, no collisions.

**Remaining risk - long drives over known cells.** The robot never stands still, but in some mazes it drives back over known cells for more than 15 s before reaching a new one (A8 Spiral 42 s, A10 30 s, A1 25 s, A2 22 s, A12 20 s, A3 19 s, D4 19 s; official arena 11 s). These are the forced drives back out of long dead-end branches: the spiral's only exit is a 23-cell corridor, at least 29 s at the motor's top speed. A judge reading 'no meaningful progress' as stuck / looping / spinning should not count them.

**Test setup.** OFF and A1-A12: visible Webots window, real time (test_judge supervisor forces the speed mode). D1-D5 and R1-R3: rendered window, fast mode. Runs are deterministic: the same world gives the same times (within 0.05 s) rendered or hidden, real time or fast.

**Rollback.** test_tools/backup_controller_2026-10-28/ = start of the session; _v2 = changes 1-2; _v3 = changes 1-3; _v4 = changes 1-4. Changed files: section2.py, config.py, motion.py.

## OFF - Official arena

the provided Arena.wbt layout (Arena.proto + Obstacles.proto)

- Dead ends 8, internal walls 23, tiles 5 (blue 2, green 3) -> expected LED **green**; Section 1 obstacles `Obstacles`

```
+---+---+---+---+---+---+---+---+---+
|         B |           |     G | E |
+   +---+---+   +   +---+   +---+   +
|               |           |       |
+   +   +   +---+---+   +---+   +   +
|   |   |                       |   |
+   +---+   +---+---+   +---+---+   +
|   | G           B |         G |   |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_OFF`): PASS**

- LED green (expected green); finished at 117.1 s; Section 1 gap reached at 20.1 s; maze time 96.3 s; exploration 90.9 s; distance 13.1 m
- Tiles found B2 G3 vs true B2 G3 (all correct)
- Cell moves in the maze: 39, of which re-driven (back into a visited cell): 13
- Longest time in the maze without reaching a new cell: 11 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.9 cm), 0.8 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_OFF_s2`): PASS**

- LED green (expected green); finished at 99.4 s; Section 1 gap reached at 0.0 s; maze time 96.5 s; exploration 90.8 s; distance 10.5 m
- Tiles found B2 G3 vs true B2 G3 (all correct)
- Cell moves in the maze: 39, of which re-driven (back into a visited cell): 13
- Longest time in the maze without reaching a new cell: 11 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.9 cm), 0.8 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## D1 - Snake

one long serpentine path; pockets right beside the end cell

- Dead ends 4, internal walls 24, tiles 3 (blue 1, green 2) -> expected LED **green**; Section 1 obstacles `TestObstacles1`

```
+---+---+---+---+---+---+---+---+---+
|       |       |       | G | G | E |
+   +   +   +   +   +   +   +   +   +
|   |   |   |   |   |   |       |   |
+   +   +   +   +   +   +   +   +   +
|   |       |   |   |   |   |   |   |
+   +   +   +   +   +   +   +   +   +
|   | B |   |       |       |       |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_D1`): PASS**

- LED green (expected green); finished at 109.0 s; Section 1 gap reached at 19.4 s; maze time 88.8 s; exploration 74.6 s; distance 13.3 m
- Tiles found B1 G2 vs true B1 G2 (all correct)
- Cell moves in the maze: 41, of which re-driven (back into a visited cell): 8
- Longest time in the maze without reaching a new cell: 10 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 8.2 cm), 0.5 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_D1_s2`): PASS**

- LED green (expected green); finished at 91.7 s; Section 1 gap reached at 0.0 s; maze time 88.8 s; exploration 74.4 s; distance 10.8 m
- Tiles found B1 G2 vs true B1 G2 (all correct)
- Cell moves in the maze: 41, of which re-driven (back into a visited cell): 8
- Longest time in the maze without reaching a new cell: 10 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 8.2 cm), 0.5 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## D2 - Open hall

few walls, wide open areas (few walls to localise on), 5 alcove tiles

- Dead ends 5, internal walls 9, tiles 5 (blue 2, green 3) -> expected LED **green**; Section 1 obstacles `TestObstacles2`

```
+---+---+---+---+---+---+---+---+---+
| B |           | G |             E |
+   +   +   +   +   +   +   +   +   +
|                                   |
+   +   +   +   +   +   +   +   +   +
|       | G |                       |
+   +   +---+   +   +   +   +   +   +
|               | B |           | G |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_D2`): PASS**

- LED green (expected green); finished at 104.2 s; Section 1 gap reached at 19.3 s; maze time 84.2 s; exploration 78.4 s; distance 12.4 m
- Tiles found B2 G3 vs true B2 G3 (all correct)
- Cell moves in the maze: 37, of which re-driven (back into a visited cell): 11
- Longest time in the maze without reaching a new cell: 10 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.7 cm), 1.0 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_D2_s2`): PASS**

- LED green (expected green); finished at 86.9 s; Section 1 gap reached at 0.0 s; maze time 84.0 s; exploration 78.5 s; distance 9.9 m
- Tiles found B2 G3 vs true B2 G3 (all correct)
- Cell moves in the maze: 37, of which re-driven (back into a visited cell): 11
- Longest time in the maze without reaching a new cell: 10 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.7 cm), 1.0 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## D3 - Comb

16 dead ends off one corridor, 7 tiles, close 4:3 majority

- Dead ends 16, internal walls 24, tiles 7 (blue 4, green 3) -> expected LED **blue**; Section 1 obstacles `TestObstacles3`

```
+---+---+---+---+---+---+---+---+---+
|   | B |   | G | G |   | B |   | E |
+   +   +   +   +   +   +   +   +   +
|   |   |   |   |   |   |   |   |   |
+   +   +   +   +   +   +   +   +   +
|                                   |
+   +   +   +   +   +   +   +   +   +
|   |   | B |   |   | B |   | G |   |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_D3`): PASS**

- LED blue (expected blue); finished at 150.2 s; Section 1 gap reached at 19.3 s; maze time 130.2 s; exploration 118.5 s; distance 17.3 m
- Tiles found B4 G3 vs true B4 G3 (all correct)
- Cell moves in the maze: 55, of which re-driven (back into a visited cell): 25
- Longest time in the maze without reaching a new cell: 11 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 8.2 cm), 0.5 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_D3_s2`): PASS**

- LED blue (expected blue); finished at 133.0 s; Section 1 gap reached at 0.0 s; maze time 130.1 s; exploration 118.4 s; distance 14.8 m
- Tiles found B4 G3 vs true B4 G3 (all correct)
- Cell moves in the maze: 55, of which re-driven (back into a visited cell): 25
- Longest time in the maze without reaching a new cell: 11 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 8.2 cm), 0.5 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## D4 - Deep branches

tiles at the far ends of long corridors (camera sees 1 cell ahead)

- Dead ends 4, internal walls 24, tiles 3 (blue 1, green 2) -> expected LED **green**; Section 1 obstacles `TestObstacles1`

```
+---+---+---+---+---+---+---+---+---+
| B |                         G | E |
+   +   +---+---+---+---+---+---+   +
|   |   |                     G |   |
+   +   +   +---+---+---+---+---+   +
|   |   |               |       |   |
+   +   +---+---+---+   +   +---+   +
|                                   |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_D4`): PASS**

- LED green (expected green); finished at 133.9 s; Section 1 gap reached at 19.4 s; maze time 113.8 s; exploration 106.9 s; distance 17.4 m
- Tiles found B1 G2 vs true B1 G2 (all correct)
- Cell moves in the maze: 57, of which re-driven (back into a visited cell): 25
- Longest time in the maze without reaching a new cell: 19 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 8.2 cm), 0.5 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_D4_s2`): PASS**

- LED green (expected green); finished at 116.6 s; Section 1 gap reached at 0.0 s; maze time 113.8 s; exploration 106.9 s; distance 14.9 m
- Tiles found B1 G2 vs true B1 G2 (all correct)
- Cell moves in the maze: 57, of which re-driven (back into a visited cell): 25
- Longest time in the maze without reaching a new cell: 19 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 8.2 cm), 0.5 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## D5 - Loops

many loops and equal-length routes; 3 alcove tiles

- Dead ends 5, internal walls 16, tiles 3 (blue 1, green 2) -> expected LED **green**; Section 1 obstacles `TestObstacles2`

```
+---+---+---+---+---+---+---+---+---+
|       | G |                     E |
+   +---+   +   +---+---+   +   +   +
|               | G |               |
+   +---+---+---+   +---+---+---+   +
|                                   |
+   +   +   +---+   +   +   +   +   +
|                       | B |       |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_D5`): PASS**

- LED green (expected green); finished at 114.2 s; Section 1 gap reached at 19.3 s; maze time 94.2 s; exploration 85.8 s; distance 13.8 m
- Tiles found B1 G2 vs true B1 G2 (all correct)
- Cell moves in the maze: 43, of which re-driven (back into a visited cell): 13
- Longest time in the maze without reaching a new cell: 13 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.7 cm), 1.0 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_D5_s2`): PASS**

- LED green (expected green); finished at 97.4 s; Section 1 gap reached at 0.0 s; maze time 94.6 s; exploration 85.8 s; distance 11.4 m
- Tiles found B1 G2 vs true B1 G2 (all correct)
- Cell moves in the maze: 43, of which re-driven (back into a visited cell): 13
- Longest time in the maze without reaching a new cell: 13 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.7 cm), 1.0 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## R1 - Random 1

random DFS maze + 3 loops, random tiles and obstacle clusters (gen_random_world.py seed 1)

- Dead ends 5, internal walls 21, tiles 3 (blue 1, green 2) -> expected LED **green**; Section 1 obstacles `TestObstacles1`

```
+---+---+---+---+---+---+---+---+---+
| G                 | G           E |
+---+   +---+---+   +---+---+   +---+
|               |   |       |       |
+   +---+   +---+   +   +   +   +   +
|   | B |               |   |   |   |
+   +   +   +   +---+---+   +---+   +
|           |                       |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_R1`): PASS**

- LED green (expected green); finished at 117.6 s; Section 1 gap reached at 19.4 s; maze time 97.5 s; exploration 91.8 s; distance 13.5 m
- Tiles found B1 G2 vs true B1 G2 (all correct)
- Cell moves in the maze: 41, of which re-driven (back into a visited cell): 11
- Longest time in the maze without reaching a new cell: 13 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_R1_s2`): PASS**

- LED green (expected green); finished at 100.3 s; Section 1 gap reached at 0.0 s; maze time 97.5 s; exploration 91.7 s; distance 10.9 m
- Tiles found B1 G2 vs true B1 G2 (all correct)
- Cell moves in the maze: 41, of which re-driven (back into a visited cell): 11
- Longest time in the maze without reaching a new cell: 13 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## R2 - Random 2

random DFS maze + 3 loops, random tiles and obstacle clusters (gen_random_world.py seed 2)

- Dead ends 2, internal walls 21, tiles 1 (blue 0, green 1) -> expected LED **green**; Section 1 obstacles `TestObstacles2`

```
+---+---+---+---+---+---+---+---+---+
|       | G                       E |
+   +   +---+   +---+   +   +---+---+
|           |       |       |       |
+   +---+   +   +   +---+---+   +   +
|   |       |   |                   |
+   +   +---+---+---+---+---+---+   +
|   |                               |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_R2`): PASS**

- LED green (expected green); finished at 96.0 s; Section 1 gap reached at 19.3 s; maze time 76.0 s; exploration 68.4 s; distance 11.2 m
- Tiles found B0 G1 vs true B0 G1 (all correct)
- Cell moves in the maze: 33, of which re-driven (back into a visited cell): 1
- Longest time in the maze without reaching a new cell: 5 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_R2_s2`): PASS**

- LED green (expected green); finished at 78.7 s; Section 1 gap reached at 0.0 s; maze time 75.9 s; exploration 68.3 s; distance 8.8 m
- Tiles found B0 G1 vs true B0 G1 (all correct)
- Cell moves in the maze: 33, of which re-driven (back into a visited cell): 1
- Longest time in the maze without reaching a new cell: 6 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## R3 - Random 3

random DFS maze + 3 loops, random tiles and obstacle clusters (gen_random_world.py seed 3)

- Dead ends 3, internal walls 21, tiles 1 (blue 0, green 1) -> expected LED **green**; Section 1 obstacles `TestObstacles3`

```
+---+---+---+---+---+---+---+---+---+
|                       |         E |
+---+   +---+   +---+   +   +---+---+
|           |       |   |           |
+   +---+   +   +   +   +   +---+   +
|   |       | G |   |   |   |       |
+   +   +   +---+   +   +   +   +   +
|   |               |           |   |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_R3`): PASS**

- LED green (expected green); finished at 95.5 s; Section 1 gap reached at 19.3 s; maze time 75.5 s; exploration 70.2 s; distance 11.3 m
- Tiles found B0 G1 vs true B0 G1 (all correct)
- Cell moves in the maze: 33, of which re-driven (back into a visited cell): 2
- Longest time in the maze without reaching a new cell: 6 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_R3_s2`): PASS**

- LED green (expected green); finished at 78.2 s; Section 1 gap reached at 0.0 s; maze time 75.4 s; exploration 70.1 s; distance 8.8 m
- Tiles found B0 G1 vs true B0 G1 (all correct)
- Cell moves in the maze: 33, of which re-driven (back into a visited cell): 2
- Longest time in the maze without reaching a new cell: 6 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## A1 - Staircase

zig-zag staircase: a 90-degree turn after nearly every cell; stair-shaped side branches

- Dead ends 7, internal walls 24, tiles 5 (blue 2, green 3) -> expected LED **green**; Section 1 obstacles `TestObstacles1`

```
+---+---+---+---+---+---+---+---+---+
| B |     G |                   | E |
+   +   +---+   +   +---+---+---+   +
|       |       |       | B |       |
+   +---+   +   +---+   +   +   +---+
|   |       |       |       |   | G |
+   +   +---+---+   +---+   +   +   +
|               |     G |           |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_A1`): PASS**

- LED green (expected green); finished at 179.0 s; Section 1 gap reached at 19.4 s; maze time 158.9 s; exploration 155.3 s; distance 19.8 m
- Tiles found B2 G3 vs true B2 G3 (all correct)
- Cell moves in the maze: 65, of which re-driven (back into a visited cell): 36
- Longest time in the maze without reaching a new cell: 25 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 8.0 cm), 0.7 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_A1_s2`): PASS**

- LED green (expected green); finished at 161.8 s; Section 1 gap reached at 0.0 s; maze time 158.9 s; exploration 155.4 s; distance 17.3 m
- Tiles found B2 G3 vs true B2 G3 (all correct)
- Cell moves in the maze: 65, of which re-driven (back into a visited cell): 36
- Longest time in the maze without reaching a new cell: 25 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 8.0 cm), 0.7 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## A2 - Nine tiles

12 dead ends, 9 tiles, 5:4 majority; dead ends hidden behind walls next to the main corridor

- Dead ends 12, internal walls 24, tiles 9 (blue 4, green 5) -> expected LED **green**; Section 1 obstacles `TestObstacles2`

```
+---+---+---+---+---+---+---+---+---+
| G | B | B |   |   | B |   | B | E |
+   +   +   +   +   +   +   +   +   +
|       |       |       |       |   |
+   +   +   +   +   +   +   +   +   +
|   | G |   | G |   | G |   | G |   |
+   +---+   +---+   +---+   +---+   +
|                                   |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_A2`): PASS**

- LED green (expected green); finished at 149.3 s; Section 1 gap reached at 19.3 s; maze time 129.3 s; exploration 113.1 s; distance 18.1 m
- Tiles found B4 G5 vs true B4 G5 (all correct)
- Cell moves in the maze: 59, of which re-driven (back into a visited cell): 33
- Longest time in the maze without reaching a new cell: 22 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 8.2 cm), 0.5 cm from centre; longest no-progress 4 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_A2_s2`): PASS**

- LED green (expected green); finished at 132.5 s; Section 1 gap reached at 0.0 s; maze time 129.6 s; exploration 113.2 s; distance 15.6 m
- Tiles found B4 G5 vs true B4 G5 (all correct)
- Cell moves in the maze: 59, of which re-driven (back into a visited cell): 33
- Longest time in the maze without reaching a new cell: 22 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 8.2 cm), 0.5 cm from centre; longest no-progress 4 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## A3 - Long corridors

four parallel 2 m corridors with featureless walls; dead ends out of sensor range, one right beside the end cell

- Dead ends 3, internal walls 24, tiles 3 (blue 1, green 2) -> expected LED **green**; Section 1 obstacles `TestObstacles3`

```
+---+---+---+---+---+---+---+---+---+
|                                 E |
+   +---+---+---+---+---+---+---+---+
|                             G | G |
+   +---+---+---+---+---+---+---+   +
|                             B |   |
+   +---+---+---+---+---+---+---+   +
|                                   |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_A3`): PASS**

- LED green (expected green); finished at 128.6 s; Section 1 gap reached at 19.3 s; maze time 108.6 s; exploration 104.3 s; distance 17.8 m
- Tiles found B1 G2 vs true B1 G2 (all correct)
- Cell moves in the maze: 59, of which re-driven (back into a visited cell): 27
- Longest time in the maze without reaching a new cell: 19 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_A3_s2`): PASS**

- LED green (expected green); finished at 111.8 s; Section 1 gap reached at 0.0 s; maze time 109.0 s; exploration 104.3 s; distance 15.3 m
- Tiles found B1 G2 vs true B1 G2 (all correct)
- Cell moves in the maze: 59, of which re-driven (back into a visited cell): 27
- Longest time in the maze without reaching a new cell: 19 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## A4 - Islands

free-standing wall islands, loops everywhere, open areas with no side walls; only 3 dead ends (boundary alcoves)

- Dead ends 3, internal walls 15, tiles 3 (blue 2, green 1) -> expected LED **blue**; Section 1 obstacles `TestObstacles2`

```
+---+---+---+---+---+---+---+---+---+
| B |                   | G |     E |
+   +   +---+---+   +   +   +   +   +
|       |           |               |
+   +   +   +   +---+   +---+---+   +
|                   |       |       |
+---+   +---+   +   +   +   +   +---+
|                                 B |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_A4`): PASS**

- LED blue (expected blue); finished at 114.2 s; Section 1 gap reached at 19.3 s; maze time 94.2 s; exploration 90.5 s; distance 12.9 m
- Tiles found B2 G1 vs true B2 G1 (all correct)
- Cell moves in the maze: 39, of which re-driven (back into a visited cell): 12
- Longest time in the maze without reaching a new cell: 12 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_A4_s2`): PASS**

- LED blue (expected blue); finished at 97.0 s; Section 1 gap reached at 0.0 s; maze time 94.1 s; exploration 90.5 s; distance 10.4 m
- Tiles found B2 G1 vs true B2 G1 (all correct)
- Cell moves in the maze: 39, of which re-driven (back into a visited cell): 12
- Longest time in the maze without reaching a new cell: 12 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## A5 - Behind the end

6 cells (2 empty dead ends) reachable only THROUGH the end cell; exploration must give up on them and still finish

- Dead ends 7, internal walls 24, tiles 5 (blue 3, green 2) -> expected LED **blue**; Section 1 obstacles `TestObstacles3`

```
+---+---+---+---+---+---+---+---+---+
|       |     B |     G |         E |
+   +   +   +---+   +---+   +---+   +
|   |   |       |   | B |   |       |
+   +   +   +   +   +   +   +   +   +
|   | B |   | G |       |   |   |   |
+   +---+   +---+   +---+   +---+   +
|                           |       |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_A5`): PASS**

- LED blue (expected blue); finished at 115.2 s; Section 1 gap reached at 19.3 s; maze time 95.2 s; exploration 92.5 s; distance 14.4 m
- Tiles found B3 G2 vs true B3 G2 (all correct)
- Cell moves in the maze: 45, of which re-driven (back into a visited cell): 21
- Longest time in the maze without reaching a new cell: 13 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_A5_s2`): PASS**

- LED blue (expected blue); finished at 98.4 s; Section 1 gap reached at 0.0 s; maze time 95.5 s; exploration 92.4 s; distance 11.9 m
- Tiles found B3 G2 vs true B3 G2 (all correct)
- Cell moves in the maze: 45, of which re-driven (back into a visited cell): 21
- Longest time in the maze without reaching a new cell: 13 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## A6 - Rooms

open 2x2 / 2x3 rooms (no walls inside) joined by single doorways, tiles in closets, two empty closets in the end room

- Dead ends 7, internal walls 16, tiles 5 (blue 2, green 3) -> expected LED **green**; Section 1 obstacles `TestObstacles1`

```
+---+---+---+---+---+---+---+---+---+
| G |       | B |       |         E |
+   +   +   +   +   +   +---+   +   +
|           |           |           |
+   +---+---+---+   +   +---+   +---+
|                   |             G |
+   +   +   +   +   +   +   +   +---+
|       | G |                     B |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_A6`): PASS**

- LED green (expected green); finished at 124.3 s; Section 1 gap reached at 19.4 s; maze time 104.2 s; exploration 100.8 s; distance 14.1 m
- Tiles found B2 G3 vs true B2 G3 (all correct)
- Cell moves in the maze: 43, of which re-driven (back into a visited cell): 15
- Longest time in the maze without reaching a new cell: 13 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.8 cm), 0.8 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_A6_s2`): PASS**

- LED green (expected green); finished at 107.5 s; Section 1 gap reached at 0.0 s; maze time 104.7 s; exploration 100.8 s; distance 11.6 m
- Tiles found B2 G3 vs true B2 G3 (all correct)
- Cell moves in the maze: 43, of which re-driven (back into a visited cell): 15
- Longest time in the maze without reaching a new cell: 13 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.8 cm), 0.8 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## A7 - Ladder

two 9-cell rails joined by 5 rungs (many loops), 16 dead-end pockets beside the rails, 7 tiles 4:3

- Dead ends 16, internal walls 20, tiles 7 (blue 3, green 4) -> expected LED **green**; Section 1 obstacles `TestObstacles3`

```
+---+---+---+---+---+---+---+---+---+
|   | B |   |   | G |   | B | G | E |
+   +   +   +   +   +   +   +   +   +
|                                   |
+   +---+   +---+   +---+   +---+   +
|                                   |
+   +   +   +   +   +   +   +   +   +
|   |   | G |   |   | B |   | G |   |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_A7`): PASS**

- LED green (expected green); finished at 128.2 s; Section 1 gap reached at 19.3 s; maze time 108.2 s; exploration 102.2 s; distance 14.2 m
- Tiles found B3 G4 vs true B3 G4 (all correct)
- Cell moves in the maze: 43, of which re-driven (back into a visited cell): 19
- Longest time in the maze without reaching a new cell: 16 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 8.0 cm), 0.7 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_A7_s2`): PASS**

- LED green (expected green); finished at 110.9 s; Section 1 gap reached at 0.0 s; maze time 108.0 s; exploration 102.1 s; distance 11.6 m
- Tiles found B3 G4 vs true B3 G4 (all correct)
- Cell moves in the maze: 43, of which re-driven (back into a visited cell): 19
- Longest time in the maze without reaching a new cell: 16 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 8.0 cm), 0.7 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## A8 - Spiral

24-cell spiral dead end, the single tile at its centre right beside the end cell (behind a wall)

- Dead ends 1, internal walls 24, tiles 1 (blue 1, green 0) -> expected LED **blue**; Section 1 obstacles `TestObstacles1`

```
+---+---+---+---+---+---+---+---+---+
|                                 E |
+   +---+---+---+---+---+---+---+---+
|   |                               |
+   +   +---+---+---+---+---+---+   +
|   |                         B |   |
+   +---+---+---+---+---+---+---+   +
|                                   |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_A8`): PASS**

- LED blue (expected blue); finished at 130.1 s; Section 1 gap reached at 19.4 s; maze time 110.0 s; exploration 105.9 s; distance 18.7 m
- Tiles found B1 G0 vs true B1 G0 (all correct)
- Cell moves in the maze: 63, of which re-driven (back into a visited cell): 29
- Longest time in the maze without reaching a new cell: 42 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_A8_s2`): PASS**

- LED blue (expected blue); finished at 112.8 s; Section 1 gap reached at 0.0 s; maze time 109.9 s; exploration 105.7 s; distance 16.1 m
- Tiles found B1 G0 vs true B1 G0 (all correct)
- Cell moves in the maze: 63, of which re-driven (back into a visited cell): 29
- Longest time in the maze without reaching a new cell: 42 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## A9 - Hard perfect 1

no loops; the deepest / most branched of 3000 random perfect mazes (seed 969), 7 dead ends, 7 tiles

- Dead ends 7, internal walls 24, tiles 7 (blue 4, green 3) -> expected LED **blue**; Section 1 obstacles `TestObstacles2`

```
+---+---+---+---+---+---+---+---+---+
|               | G             | E |
+   +---+---+   +---+   +   +   +   +
|           |       | B |   |       |
+   +---+   +---+   +---+   +---+   +
| G |       | B |   |       | B |   |
+---+   +---+   +   +   +---+   +   +
|       | G             | B         |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_A9`): PASS**

- LED blue (expected blue); finished at 99.8 s; Section 1 gap reached at 19.3 s; maze time 79.8 s; exploration 70.6 s; distance 11.3 m
- Tiles found B4 G3 vs true B4 G3 (all correct)
- Cell moves in the maze: 33, of which re-driven (back into a visited cell): 5
- Longest time in the maze without reaching a new cell: 9 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.9 cm), 0.8 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_A9_s2`): PASS**

- LED blue (expected blue); finished at 82.6 s; Section 1 gap reached at 0.0 s; maze time 79.7 s; exploration 70.6 s; distance 8.9 m
- Tiles found B4 G3 vs true B4 G3 (all correct)
- Cell moves in the maze: 33, of which re-driven (back into a visited cell): 5
- Longest time in the maze without reaching a new cell: 9 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.9 cm), 0.8 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## A10 - Hard perfect 2

no loops; the deepest / most branched of 3000 random perfect mazes (seed 2463), 7 dead ends, 7 tiles

- Dead ends 7, internal walls 24, tiles 7 (blue 3, green 4) -> expected LED **green**; Section 1 obstacles `TestObstacles3`

```
+---+---+---+---+---+---+---+---+---+
|               | B             | E |
+   +   +---+   +---+   +---+   +   +
| G |   | G |           | B |       |
+---+   +   +---+---+---+   +---+   +
|       |             G |       |   |
+   +---+   +---+   +---+   +   +   +
|   | B       G |           |       |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_A10`): PASS**

- LED green (expected green); finished at 121.0 s; Section 1 gap reached at 19.3 s; maze time 101.0 s; exploration 71.9 s; distance 13.5 m
- Tiles found B3 G4 vs true B3 G4 (all correct)
- Cell moves in the maze: 41, of which re-driven (back into a visited cell): 13
- Longest time in the maze without reaching a new cell: 30 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.9 cm), 0.8 cm from centre; longest no-progress 4 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_A10_s2`): PASS**

- LED green (expected green); finished at 103.7 s; Section 1 gap reached at 0.0 s; maze time 100.8 s; exploration 71.8 s; distance 11.0 m
- Tiles found B3 G4 vs true B3 G4 (all correct)
- Cell moves in the maze: 41, of which re-driven (back into a visited cell): 13
- Longest time in the maze without reaching a new cell: 30 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.9 cm), 0.8 cm from centre; longest no-progress 4 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## A11 - Braided 1

random maze (seed 0) braided into 7 loops: open areas, many equal-cost routes (the A4 livelock pattern), only 3 dead ends

- Dead ends 3, internal walls 17, tiles 3 (blue 2, green 1) -> expected LED **blue**; Section 1 obstacles `TestObstacles1`

```
+---+---+---+---+---+---+---+---+---+
|         G |                     E |
+   +---+---+   +---+---+   +   +   +
|   |                               |
+   +   +---+   +---+   +   +---+   +
|       |           | B |           |
+---+   +   +---+   +---+---+   +   +
|           | B                     |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_A11`): PASS**

- LED blue (expected blue); finished at 114.2 s; Section 1 gap reached at 19.4 s; maze time 94.1 s; exploration 84.2 s; distance 13.5 m
- Tiles found B2 G1 vs true B2 G1 (all correct)
- Cell moves in the maze: 41, of which re-driven (back into a visited cell): 13
- Longest time in the maze without reaching a new cell: 13 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.8 cm), 0.8 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_A11_s2`): PASS**

- LED blue (expected blue); finished at 97.0 s; Section 1 gap reached at 0.0 s; maze time 94.1 s; exploration 84.2 s; distance 10.9 m
- Tiles found B2 G1 vs true B2 G1 (all correct)
- Cell moves in the maze: 41, of which re-driven (back into a visited cell): 13
- Longest time in the maze without reaching a new cell: 13 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.8 cm), 0.8 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

## A12 - Braided 2

random maze (seed 10) braided into 6 loops: open areas, many equal-cost routes (the A4 livelock pattern), only 3 dead ends

- Dead ends 3, internal walls 18, tiles 3 (blue 1, green 2) -> expected LED **green**; Section 1 obstacles `TestObstacles2`

```
+---+---+---+---+---+---+---+---+---+
|                       | B       E |
+   +   +   +---+---+   +---+   +---+
| G |               |       |       |
+---+   +---+---+   +---+   +   +   +
|                   | G |       |   |
+   +   +   +   +   +   +---+   +   +
|   |       |                       |
+---+---+---+---+---+---+---+---+---+
```

**Full run (`test_j_A12`): PASS**

- LED green (expected green); finished at 110.4 s; Section 1 gap reached at 19.3 s; maze time 90.4 s; exploration 86.4 s; distance 12.8 m
- Tiles found B1 G2 vs true B1 G2 (all correct)
- Cell moves in the maze: 39, of which re-driven (back into a visited cell): 11
- Longest time in the maze without reaching a new cell: 20 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

**Section 2 run (`test_j_A12_s2`): PASS**

- LED green (expected green); finished at 93.1 s; Section 1 gap reached at 0.0 s; maze time 90.3 s; exploration 86.2 s; distance 10.3 m
- Tiles found B1 G2 vs true B1 G2 (all correct)
- Cell moves in the maze: 39, of which re-driven (back into a visited cell): 11
- Longest time in the maze without reaching a new cell: 19 s (shows looping; the 15 s no-progress check only sees standing still)
- End cell: whole body inside (margin 7.2 cm), 1.1 cm from centre; longest no-progress 3 s; max tilt 0.0 deg; collisions S1 0 / maze 0

