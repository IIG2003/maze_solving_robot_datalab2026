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
    bot.turn_to(nearest_axis(bot.th))
    d = bot.d
    narrow_lr = d['ds_l'] + d['ds_r'] < 0.30
    narrow_fb = d['ds_f'] + d['ds_b'] < 0.30
    if not (narrow_lr or narrow_fb):
        bot.turn_to(math.pi)
        x, y = cfg.START_CENTRE
        if bot.d['ds_b'] < 0.35:
            x = cfg.ARENA_W - cfg.WALL_HALF - cfg.OFF_B - bot.d['ds_b']
        if bot.d['ds_l'] < 0.35:
            y = cfg.WALL_HALF + cfg.OFF_SIDE + bot.d['ds_l']
        bot.set_pose(x, y)
        log('start zone, pose (%.3f, %.3f)' % (x, y))
        return 'S1'

    bot.turn_to(-NORTH)
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
