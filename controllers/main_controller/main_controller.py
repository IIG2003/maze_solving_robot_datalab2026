"""
main_controller.py - PeraBots 2026 University Category, Phase 1.

Flow:
  1. Work out where the robot was placed:
       * Section 1 start zone  -> localise against the corner walls
       * Section 2 entrance     -> (judges' dedicated Section 2 attempts)
  2. Section 1: map + A* through the obstacle field to the gap in the top wall.
  3. Transition: drive north through the chamber into maze cell (1, 1).
  4. Section 2: explore the maze, count blue / green tiles, drive to the end
     cell, stop and light the majority LED.

File placement (must not change):
  controllers/main_controller/main_controller.py  (+ helper modules beside it)
"""
import math

import config as cfg
import section1
import section2
import vision
from hardware import Bot, log

NORTH = math.pi / 2


def nearest_axis(th):
    return min((0.0, NORTH, math.pi, -NORTH), key=lambda h: abs(math.remainder(th - h, 2 * math.pi)))


def detect_start(bot):
    """Return 'S1' (start zone) or 'S2' (maze entrance) and set the pose."""
    bot.turn_to(nearest_axis(bot.th))
    d = bot.d
    narrow_lr = d['ds_l'] + d['ds_r'] < 0.30
    narrow_fb = d['ds_f'] + d['ds_b'] < 0.30
    if not (narrow_lr or narrow_fb):
        # --- Section 1 start zone (bottom-right corner) ---
        bot.turn_to(math.pi)                  # face -x: back -> right wall, left -> bottom wall
        x, y = cfg.START_CENTRE
        if bot.d['ds_b'] < 0.35:
            x = cfg.ARENA_W - cfg.WALL_HALF - cfg.OFF_B - bot.d['ds_b']
        if bot.d['ds_l'] < 0.35:
            y = cfg.WALL_HALF + cfg.OFF_SIDE + bot.d['ds_l']
        bot.set_pose(x, y)
        log('start zone, pose (%.3f, %.3f)' % (x, y))
        return 'S1'

    # --- Section 2 entrance: transition chamber or maze cell (1, 1) ---
    bot.turn_to(-NORTH)                       # look back south at the floor
    col = vision.stable(bot, vision.near, 3)
    if col in ('white', 'red'):
        x, y = cfg.CHAMBER_CENTRE
    else:
        x, y = section2.centre(cfg.ENTRY_CELL)
    bot.set_pose(x, y)
    bot.turn_to(NORTH)
    log('maze entrance (floor behind: %s), pose (%.3f, %.3f)' % (col, x, y))
    return 'S2'


def fix_x_at_gap(bot):
    """
    At the Section 1 goal (below the gap), measure x against the arena's left
    wall, then line up under the gap again. Odometry drifts a few cm over the
    obstacle field and the gap only leaves ~8 cm of clearance on each side.
    """
    bot.turn_to(math.pi)
    f = bot.d['ds_f']
    if f < 0.5:
        x = cfg.WALL_HALF + cfg.OFF_F + f
        if abs(x - bot.x) < 0.15:
            log('gap x fix: %.3f -> %.3f' % (bot.x, x))
            bot.x = x
    gx, gy = cfg.S1_GOAL
    if abs(bot.x - gx) > 0.01:
        bot.drive_to(gx, gy, speed=4.0)


def enter_maze(bot):
    """From the Section 1 goal / chamber, drive north into cell (1, 1)."""
    bot.turn_to(NORTH)
    lat = section2.lateral_listener(bot)
    bot.listeners.append(lat)
    tx, ty = section2.centre(cfg.ENTRY_CELL)
    bot.drive_axis(tx, ty, NORTH, cfg.S2_CRUISE)
    bot.listeners.remove(lat)


def main():
    bot = Bot()
    mode = detect_start(bot)
    if mode == 'S1':
        section1.run(bot)
        fix_x_at_gap(bot)
    enter_maze(bot)
    section2.run(bot, cfg.ENTRY_CELL, section2.N)


main()
