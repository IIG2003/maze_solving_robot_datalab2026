"""
vision.py - floor colour classification with the single tilted camera.

  near  : floor of the current cell just ahead of the robot (bottom rows)
  ahead : floor of the next cell in the facing direction (upper rows)

Colours: 'white' (S1 floor / maze walls), 'black' (S2 floor / cubes),
'red' (start / transition / end markers), 'blue', 'green', or None.
"""
import config as cfg


def classify(r, g, b):
    mx = max(r, g, b)
    mn = min(r, g, b)
    if mx < 45:
        return 'black'
    if mn > 150:
        return 'white'
    if r > 80 and r > 2.0 * g and r > 2.0 * b:
        return 'red'
    if g > 45 and g > 1.6 * r and g > 1.3 * b:
        return 'green'
    if b > 35 and b > 1.6 * r and b > 1.3 * g:
        return 'blue'
    return None


def region_colour(bot, rows):
    """Majority colour over a band of rows (centre columns only)."""
    img = bot.cam.getImage()
    if not img:
        return None
    w = bot.cam_w
    c0 = int(w * cfg.CAM_COL_FRAC[0])
    c1 = int(w * cfg.CAM_COL_FRAC[1])
    votes = {}
    total = 0
    sums = [0, 0, 0]
    for y in rows:
        base = y * w
        for x in range(c0, c1):
            i = 4 * (base + x)              # BGRA
            b, g, r = img[i], img[i + 1], img[i + 2]
            if cfg.CAM_DEBUG:
                sums[0] += r; sums[1] += g; sums[2] += b
            c = classify(r, g, b)
            total += 1
            if c:
                votes[c] = votes.get(c, 0) + 1
    if cfg.CAM_DEBUG and total:
        print('[cam] rows %d-%d avg rgb=(%d,%d,%d) votes=%s' % (
            rows[0], rows[-1], sums[0] // total, sums[1] // total, sums[2] // total, votes))
    if not votes:
        return None
    best = max(votes, key=votes.get)
    # require a clear majority of the band
    return best if votes[best] >= 0.4 * total else None


def near(bot):
    return region_colour(bot, cfg.CAM_NEAR_ROWS)


def ahead(bot):
    return region_colour(bot, cfg.CAM_AHEAD_ROWS)


def stable(bot, fn, frames=2):
    """Read a region over several frames; return the colour only if consistent."""
    first = fn(bot)
    for _ in range(frames - 1):
        bot.step()
        if fn(bot) != first:
            return None
    return first
