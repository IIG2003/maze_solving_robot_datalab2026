# Build the advanced-maze test report (TEST ONLY).
#
# Reads advanced_designs.json, adv_results/offline.json and, per world,
# adv_results/<world>.json (test_judge verdict) + <world>.log (robot log).
# Writes adv_results/REPORT.md and adv_results/report.html (arena drawings
# with the robot's true path).
#
# usage: python make_report.py [--fix DIR]   (DIR: results of a patched controller,
#        shown next to the baseline; default adv_results_fix if it exists)
#        python make_report.py --suite      (whole suite from gen_suite.py:
#        suite.json, suite_results/, suite_results_fix/, suite_findings.json)
import argparse
import html
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, 'adv_results')
PROTOS = os.path.join(HERE, '..', 'test_run', 'protos')
END_C, END_H = (2.22, 2.205), 0.125
NO_PROGRESS_LIMIT = 15.0          # rulebook: attempt terminated after 15 s without progress
LED_LIMIT = 3.0                   # rulebook: LED within 3 s of arriving


def load(path, default=None):
    try:
        return json.load(open(path))
    except (OSError, ValueError):
        return default


def parse_log(path):
    info = {'tiles': {}, 'found': None, 'explore_s': None, 's1_s': None, 's2_s': None,
            'unexpected_walls': 0, 'run_fails': 0, 'skips': 0, 'stuck_s1': 0, 'lines': 0,
            'map': None, 'no_route': False}
    try:
        text = open(path, encoding='utf8', errors='replace').read()
    except OSError:
        return info
    info['lines'] = text.count('\n')
    for m in re.finditer(r'tile (blue|green) (?:seen ahead|under robot) in cell \((\d), (\d)\)', text):
        info['tiles'][(int(m.group(2)), int(m.group(3)))] = m.group(1)
    m = re.search(r'exploration done in ([\d.]+)s: blue=(\d+) green=(\d+)', text)
    if m:
        info['explore_s'] = float(m.group(1))
        info['found'] = (int(m.group(2)), int(m.group(3)))
    m = re.search(r'Section 1 done in ([\d.]+)s', text)
    info['s1_s'] = float(m.group(1)) if m else None
    m = re.search(r'Section 2 done in ([\d.]+)s', text)
    info['s2_s'] = float(m.group(1)) if m else None
    info['unexpected_walls'] = text.count('unexpected wall')
    info['run_fails'] = len(re.findall(r'ended "(?:blocked|timeout)"', text))
    info['skips'] = text.count('keeps failing')
    info['stuck_s1'] = text.count('S1: stuck')
    info['no_route'] = 'no route to the end cell' in text
    m = re.search(r'maze map:\n((?:\[bot\] )?[+|?][^\n]*\n(?:[+|?][^\n]*\n)+)', text)
    if m:
        info['map'] = m.group(1).rstrip()
    return info


def end_entries(track, t_finish):
    """Times the robot centre entered the end cell and left it again before the finish."""
    out, entered = [], None
    for t, x, y, _ in track:
        if t_finish is not None and t >= t_finish:
            break
        now = abs(x - END_C[0]) < END_H and abs(y - END_C[1]) < END_H
        if now and entered is None:
            entered = round(t, 1)
        elif not now and entered is not None:
            out.append(entered)
            entered = None
    return out


def no_new_cell(track, t_from, t_end):
    """Longest time (s) in the maze without the robot centre reaching a cell it never had."""
    seen, last, best = set(), t_from, 0.0
    for t, x, y, _ in track:
        if t_from is None or t < t_from or t > t_end:
            continue
        cell = (int(round((y - 1.455) / 0.25)) + 1, int(round((x - 0.22) / 0.25)) + 1)
        if cell not in seen:
            seen.add(cell)
            best, last = max(best, t - last), t
    return max(best, t_end - last) if t_from is not None else None


def moves(track, t_from, t_end):
    """(cell moves in the maze, moves back into a cell already visited)."""
    seen, last, n, again = set(), None, 0, 0
    for t, x, y, _ in track:
        if t_from is None or t < t_from or t > t_end or y < 1.33:
            continue
        cell = (int(round((y - 1.455) / 0.25)) + 1, int(round((x - 0.22) / 0.25)) + 1)
        if cell != last:
            if last is not None:
                n += 1
                again += cell in seen
            seen.add(cell)
            last = cell
    return n, again


def evaluate(world, design, r, lg):
    """One row of results + PASS / FAIL with reasons."""
    truth = {tuple(c): v for c, v in design['tiles']}
    row = {'world': world, 'ran': r is not None}
    if r is None:
        row.update(verdict='NO RESULT', problems=['Webots produced no verdict'])
        return row
    found = lg['found']
    tiles_ok = lg['tiles'] == truth
    missed = sorted(set(truth) - set(lg['tiles']))
    false_t = sorted(c for c in lg['tiles'] if c not in truth)
    wrong_col = sorted(c for c in lg['tiles'] if c in truth and lg['tiles'][c] != truth[c])
    early = end_entries(r['track'], r['t_finish'])
    nnc = no_new_cell(r['track'], r['t_maze'], r['t_finish'] or r['sim_time'])
    n_moves, n_again = moves(r['track'], r['t_maze'], r['t_finish'] or r['sim_time'])
    t_total = r['t_finish']
    t_maze = (r['t_finish'] - r['t_maze']) if r['t_finish'] and r['t_maze'] else None
    probs, warns = [], []
    if not r['finished']:
        probs.append('did not finish (%s)' % r['reason'])
    elif r['led'] != design['expect']:
        probs.append('wrong LED: %s, expected %s' % (r['led'], design['expect']))
    if r['finished'] and not r['in_end_cell']:
        probs.append('not fully inside the end cell (margin %.1f cm)' % r['end_margin_cm'])
    if r['longest_no_progress_s'] >= NO_PROGRESS_LIMIT:
        probs.append('%.0f s without progress (rule limit 15 s)' % r['longest_no_progress_s'])
    if r['max_outside_s'] > 2:
        probs.append('outside the track %.1f s' % r['max_outside_s'])
    if r['finished'] and r['moved_after_finish_cm'] > 1:
        probs.append('moved %.1f cm after the LED' % r['moved_after_finish_cm'])
    if early:
        probs.append('entered the end cell early at t=%s s' % ', '.join(map(str, early)))
    if missed:
        warns.append('missed tiles %s' % ', '.join(map(str, missed)))
    if false_t or wrong_col:
        warns.append('false/wrong tiles %s' % ', '.join(map(str, false_t + wrong_col)))
    if r['collisions_s1'] or r['collisions_maze']:
        warns.append('collisions: S1 %d, maze %d' % (r['collisions_s1'], r['collisions_maze']))
    if r['max_tilt_deg'] > 10:
        warns.append('tilted %.0f deg' % r['max_tilt_deg'])
    if lg['run_fails'] or lg['skips']:
        warns.append('%d blocked/timed-out runs, %d skipped targets' % (lg['run_fails'], lg['skips']))
    row.update(
        verdict='PASS' if not probs else 'FAIL', problems=probs, warnings=warns,
        led=r['led'], expect=design['expect'], finished=r['finished'],
        in_end=r['in_end_cell'], margin=r['end_margin_cm'], centre_err=r['end_centre_err_cm'],
        t_total=t_total, t_gap=r['t_gap'], t_maze=t_maze, explore=lg['explore_s'],
        dist=r['distance_m'], col_s1=r['collisions_s1'], col_maze=r['collisions_maze'],
        nop=r['longest_no_progress_s'], tilt=r['max_tilt_deg'], nnc=nnc, n_moves=n_moves, n_again=n_again,
        found=found, n_truth=(design['blue'], design['green']), tiles_ok=tiles_ok,
        missed=missed, false_t=false_t + wrong_col, early=early, map=lg['map'],
        unexpected=lg['unexpected_walls'], run_fails=lg['run_fails'], skips=lg['skips'])
    return row


# ------------------------------------------------------------------ drawing
def solids(proto_name):
    text = open(os.path.join(PROTOS, proto_name + '.proto'), encoding='utf8').read()
    out = []
    for blk in text.split('Solid {')[1:]:
        t = re.search(r'translation ([-\d.e]+) ([-\d.e]+) ([-\d.e]+)', blk)
        s = re.search(r'size ([-\d.e]+) ([-\d.e]+) ([-\d.e]+)', blk)
        n = re.search(r'name "([^"]+)"', blk)
        c = re.search(r'baseColor ([-\d.e]+) ([-\d.e]+) ([-\d.e]+)', blk)
        if t and s and n:
            out.append((n.group(1), [float(v) for v in t.groups()], [float(v) for v in s.groups()],
                        [float(v) for v in c.groups()] if c else None))
    return out


def svg(design, runs, size=420):
    k = size / 2.44
    X = lambda x: x * k                                       # noqa: E731
    Y = lambda y: (2.44 - y) * k                              # noqa: E731
    parts = ['<svg viewBox="0 0 %d %d" class="arena" role="img" aria-label="arena map">'
             % (size, size)]
    items = solids(design.get('arena') or 'TestArenaA%s' % design['n']) + solids(design['obstacles'])
    for name, (x, y, z), (sx, sy, sz), col in items:
        if name == 'section1_floor':
            cls = 'floor1'
        elif name == 'section2_floor':
            cls = 'floor2'
        elif name in ('start_marker', 'end_marker') or 'transition' in name:
            cls = 'marker'
        elif 'tile' in name and ('blue' in name or 'green' in name):
            cls = 'tile-blue' if 'blue' in name else 'tile-green'
        elif name.startswith('obstacle'):
            cls = 'obst'
        elif sz >= 0.05:
            cls = 'wall'
        else:
            continue
        inset = 0.03 if cls.startswith('tile') else 0
        parts.append('<rect class="%s" x="%.1f" y="%.1f" width="%.1f" height="%.1f"/>' % (
            cls, X(x - sx / 2 + inset), Y(y + sy / 2 - inset),
            max(1.2, (sx - 2 * inset) * k), max(1.2, (sy - 2 * inset) * k)))
    for cls, r in runs:
        if r and r.get('track'):
            pts = ' '.join('%.1f,%.1f' % (X(p[1]), Y(p[2])) for p in r['track'])
            parts.append('<polyline class="%s" points="%s"/>' % (cls, pts))
            for e in r.get('collision_events', []):
                parts.append('<circle class="hit" cx="%.1f" cy="%.1f" r="4"/>' % (X(e['x']), Y(e['y'])))
            x, y = r['final_pose'][:2]
            parts.append('<circle class="%s-end" cx="%.1f" cy="%.1f" r="4"/>' % (cls, X(x), Y(y)))
    parts.append('</svg>')
    return '\n'.join(parts)


def fmt(v, f='%.1f', none='–'):
    return none if v is None else f % v


def md_row(label, row):
    if not row['ran']:
        return '| %s | NO RESULT | | | | | | | | | |' % label
    return '| %s | **%s** | %s / %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
        label, row['verdict'], row['led'] or '–', row['expect'],
        'yes (%.1f cm)' % row['margin'] if row['in_end'] else 'no',
        fmt(row['t_total']), fmt(row['t_maze']), fmt(row['dist'], '%.1f m'),
        '%d / %d' % (row['col_s1'], row['col_maze']),
        '%s' % (('B%d G%d' % row['found']) if row['found'] else '–'),
        'B%d G%d' % row['n_truth'], fmt(row['nop'], '%.0f s'))


def collect(designs, res_dir):
    rows = {}
    for i, (key, d) in enumerate(designs.items(), 1):
        d['n'] = i
        for suffix, mode in (('', 'full'), ('_s2', 's2')):
            world = d['worlds'][mode] if 'worlds' in d else 'test_adv%d%s' % (i, suffix)
            r = load(os.path.join(res_dir, world + '.json'))
            lg = parse_log(os.path.join(res_dir, world + '.log'))
            rows[(key, mode)] = (evaluate(world, d, r, lg), r)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--suite', action='store_true')
    ap.add_argument('--fix', help='comparison results directory')
    ap.add_argument('--results')
    ap.add_argument('--label', help='name of the comparison controller')
    ap.add_argument('--setup', default='headless (`--mode=fast --no-rendering`)',
                    help='how Webots was run (report text)')
    a = ap.parse_args()
    global RES, FINDINGS, SETUP
    SETUP = a.setup
    if a.suite:
        RES = a.results or os.path.join(HERE, 'suite_results_new')
        a.fix = a.fix or os.path.join(HERE, 'suite_results_old')
        a.label = a.label or 'previous controller (backup_controller_2026-10-28)'
        designs = load(os.path.join(HERE, 'suite.json'))
        FINDINGS = [tuple(f) for f in load(os.path.join(HERE, 'suite_findings.json'), [])]
        title = 'Full Webots test suite'
        scope = ('%d test arenas (official, 5 designed, 3 random, 12 advanced), all inside the '
                 'unchanged arena' % len(designs))
    else:
        a.fix = a.fix or os.path.join(HERE, 'adv_results_fix')
        a.label = a.label or 'proposed fix'
        designs = load(os.path.join(HERE, 'advanced_designs.json'))
        title = 'Advanced maze test report'
        scope = '6 new maze designs inside the unchanged arena'
    offline = load(os.path.join(RES, 'offline.json'), {})
    rows = collect(designs, RES)
    fix = collect(designs, a.fix) if os.path.isdir(a.fix) else None

    # ---------------------------------------------------------- markdown
    md = ['# %s' % title, '',
          'Controller: `PeraBots_UC/controllers/main_controller` (live code, via `test_run/`). '
          'Webots R2025a, %s. Arena outline, maze grid ' % SETUP +
          '(4 x 9 cells, 25 cm), wall size and markers are identical to `Arena.proto`; only '
          'internal maze walls, tiles and Section 1 obstacles change.', '',
          'Each design runs twice: **full** (start zone -> Section 1 -> maze) and **S2** '
          '(dedicated Section 2 attempt, robot placed on the transition tile). A run **PASSES** '
          'when it finishes with the correct LED, whole body inside the end cell, never enters '
          'the end cell early, never goes 15 s without progress and stays still after the LED. '
          'Missed tiles and collisions are listed as warnings.', '',
          '## Summary', '',
          '| Run | Verdict | LED / expected | In end cell (margin) | Finish t (s) | Maze t (s) '
          '| Distance | Collisions S1/maze | Tiles found | Tiles true | Longest no-progress |',
          '|---|---|---|---|---|---|---|---|---|---|---|']
    for (key, mode), (row, _) in rows.items():
        md.append(md_row('%s %s (%s)' % (key, designs[key]['name'], mode), row))
    n_pass = sum(r[0].get('verdict') == 'PASS' for r in rows.values())
    md += ['', '**%d / %d runs passed.**' % (n_pass, len(rows)), '']
    if fix:
        f_pass = sum(r[0].get('verdict') == 'PASS' for r in fix.values())
        md += ['## Comparison: %s' % a.label, '',
               'Same worlds and judge. Maze time / cell moves / re-driven moves (moves back into a '
               'cell already visited), shown as %s -> controller under test.' % a.label, '',
               '| Run | Verdict | Maze t (s) | Cell moves | Re-driven moves |',
               '|---|---|---|---|---|']
        for k_ in rows:
            new, old = rows[k_][0], fix[k_][0]
            if not (new['ran'] and old['ran']):
                continue
            md.append('| %s %s (%s) | %s -> %s | %s -> %s | %d -> %d | %d -> %d |' % (
                k_[0], designs[k_[0]]['name'], k_[1], old['verdict'], new['verdict'],
                fmt(old['t_maze'], '%.0f', 'none'), fmt(new['t_maze'], '%.0f', 'none'),
                old['n_moves'], new['n_moves'], old['n_again'], new['n_again']))
        md += ['', 'Full results of the %s:' % a.label, '',
               '| Run | Verdict | LED / expected | In end cell (margin) | Finish t (s) | Maze t (s) '
               '| Distance | Collisions S1/maze | Tiles found | Tiles true | Longest no-progress |',
               '|---|---|---|---|---|---|---|---|---|---|---|']
        for (key, mode), (row, _) in fix.items():
            md.append(md_row('%s %s (%s)' % (key, designs[key]['name'], mode), row))
        md += ['', '**%d / %d runs passed with the %s.**' % (f_pass, len(fix), a.label), '']
    md += ['## Findings', '']
    for title, text in FINDINGS:
        md += ['**%s.** %s' % (title, text), '']
    for key, d in designs.items():
        md += ['## %s - %s' % (key, d['name']), '', d['about'], '',
               '- Dead ends %d, internal walls %d, tiles %d (blue %d, green %d) -> expected LED '
               '**%s**; Section 1 obstacles `%s`' % (d['dead_ends'], d['walls'], len(d['tiles']),
                                                     d['blue'], d['green'], d['expect'],
                                                     d['obstacles'])]
        off = offline.get(key)
        if off:
            md.append('- Offline planner baseline (perfect sensing): %d cells, %d turns, %d stops, '
                      '~%.0f s, LED %s, reaches end %s' % (
                          off['cells'], off['turns'], off['stops'], off['time'],
                          'correct' if off['ok'] else 'WRONG', 'yes' if off['at_end'] else 'NO'))
        md += ['', '```', d['map'], '```', '']
        for mode in ('full', 's2'):
            row, _ = rows[(key, mode)]
            md.append('**%s run (`%s`): %s**' % ('Full' if mode == 'full' else 'Section 2',
                                                 row['world'], row.get('verdict')))
            md.append('')
            if not row['ran']:
                md += ['- no result', '']
                continue
            md.append('- LED %s (expected %s); finished at %s s; Section 1 gap reached at %s s; '
                      'maze time %s s; exploration %s s; distance %.1f m' % (
                          row['led'] or 'none', row['expect'], fmt(row['t_total']),
                          fmt(row['t_gap']), fmt(row['t_maze']), fmt(row['explore']), row['dist']))
            md.append('- Tiles found %s vs true B%d G%d%s' % (
                ('B%d G%d' % row['found']) if row['found'] else 'none',
                row['n_truth'][0], row['n_truth'][1], ' (all correct)' if row['tiles_ok'] else ''))
            md.append('- Cell moves in the maze: %d, of which re-driven (back into a visited cell): '
                      '%d' % (row['n_moves'], row['n_again']))
            md.append('- Longest time in the maze without reaching a new cell: %s s '
                      '(shows looping; the 15 s no-progress check only sees standing still)'
                      % fmt(row['nnc'], '%.0f'))
            md.append('- End cell: %s, %.1f cm from centre; longest no-progress %.0f s; '
                      'max tilt %.1f deg; collisions S1 %d / maze %d' % (
                          'whole body inside (margin %.1f cm)' % row['margin'] if row['in_end']
                          else 'NOT inside', row['centre_err'], row['nop'], row['tilt'],
                          row['col_s1'], row['col_maze']))
            for p in row['problems']:
                md.append('- FAIL: %s' % p)
            for w in row['warnings']:
                md.append('- warning: %s' % w)
            md.append('')
    open(os.path.join(RES, 'REPORT.md'), 'w', encoding='utf8').write('\n'.join(md) + '\n')

    # ---------------------------------------------------------- html
    h = ['<!doctype html><html lang="en"><head><meta charset="utf-8">',
         '<meta name="viewport" content="width=device-width, initial-scale=1">',
         '<title>%s</title><style>' % ('Webots Test Suite' if a.suite else 'Advanced Maze Tests'),
         CSS, '</style></head><body><main>', '<h1>%s</h1>' % title,
         '<p class="lede">%d of %d runs passed. %s, each '
         'run from the start zone (full) and as a dedicated Section&nbsp;2 attempt.</p>'
         % (n_pass, len(rows), scope),
         '<div class="scroll"><table><thead><tr><th>Run</th><th>Verdict</th><th>LED</th>'
         '<th>End cell</th><th class="num">Finish&nbsp;s</th><th class="num">Maze&nbsp;s</th>'
         '<th class="num">Dist&nbsp;m</th><th class="num">Hits</th><th>Tiles found / true</th>'
         '<th class="num">No-prog&nbsp;s</th></tr></thead><tbody>']
    for (key, mode), (row, _) in rows.items():
        name = '%s %s <span class="mode">%s</span>' % (key, html.escape(designs[key]['name']), mode)
        if not row['ran']:
            h.append('<tr><td>%s</td><td><span class="badge fail">NO RESULT</span></td>'
                     '<td colspan="8"></td></tr>' % name)
            continue
        ok = row['verdict'] == 'PASS'
        h.append('<tr><td>%s</td><td><span class="badge %s">%s</span></td><td>%s / %s</td>'
                 '<td>%s</td><td class="num">%s</td><td class="num">%s</td><td class="num">%.1f</td>'
                 '<td class="num">%d</td><td>%s / B%d G%d</td><td class="num">%.0f</td></tr>' % (
                     name, 'pass' if ok else 'fail', row['verdict'], row['led'] or '–',
                     row['expect'], 'inside' if row['in_end'] else 'no', fmt(row['t_total']),
                     fmt(row['t_maze']), row['dist'], row['col_s1'] + row['col_maze'],
                     ('B%d G%d' % row['found']) if row['found'] else '–',
                     row['n_truth'][0], row['n_truth'][1], row['nop']))
    h.append('</tbody></table></div>')
    if fix:
        f_pass = sum(r[0].get('verdict') == 'PASS' for r in fix.values())
        h.append('<p class="meta">With the %s: <b>%d of %d</b> runs pass.</p>' % (
            html.escape(a.label), f_pass, len(fix)))
    h.append('<h2 style="margin-top:24px">Findings</h2>')
    for title, text in FINDINGS:
        h.append('<p><b>%s.</b> %s</p>' % (html.escape(title), html.escape(text)))
    for key, d in designs.items():
        full, rf = rows[(key, 'full')]
        s2, rs = rows[(key, 's2')]
        h.append('<section><h2>%s · %s</h2><p>%s</p>' % (key, html.escape(d['name']),
                                                          html.escape(d['about'])))
        h.append('<p class="meta">%d dead ends · %d internal walls · tiles blue %d / green %d → '
                 'expected <b>%s</b> · obstacles %s</p>' % (
                     d['dead_ends'], d['walls'], d['blue'], d['green'], d['expect'],
                     d['obstacles']))
        h.append('<div class="pair"><figure>%s<figcaption><span class="key full"></span>full run'
                 ' &nbsp;<span class="key s2"></span>Section 2 run &nbsp;<span class="key hit">'
                 '</span>collision</figcaption></figure><div class="runs">' %
                 svg(d, [('full', rf), ('s2', rs)]))
        for label, row in (('Full run', full), ('Section 2 run', s2)):
            if not row['ran']:
                h.append('<div class="card"><h3>%s <span class="badge fail">NO RESULT</span></h3>'
                         '</div>' % label)
                continue
            h.append('<div class="card"><h3>%s <span class="badge %s">%s</span></h3><ul>' % (
                label, 'pass' if row['verdict'] == 'PASS' else 'fail', row['verdict']))
            h.append('<li>LED <b>%s</b> (expected %s), finished at %s s, maze %s s, '
                     '%.1f m</li>' % (row['led'] or 'none', row['expect'], fmt(row['t_total']),
                                      fmt(row['t_maze']), row['dist']))
            h.append('<li>Tiles found %s, true B%d G%d</li>' % (
                ('B%d G%d' % row['found']) if row['found'] else 'none', *row['n_truth']))
            h.append('<li>%d cell moves, %d re-driven · longest without a new cell %s s</li>' % (
                row['n_moves'], row['n_again'], fmt(row['nnc'], '%.0f')))
            h.append('<li>End cell: %s · longest no-progress %.0f s · tilt %.1f°</li>' % (
                ('inside, margin %.1f cm' % row['margin']) if row['in_end'] else 'not inside',
                row['nop'], row['tilt']))
            for p in row['problems']:
                h.append('<li class="bad">%s</li>' % html.escape(p))
            for w in row['warnings']:
                h.append('<li class="warn">%s</li>' % html.escape(w))
            h.append('</ul></div>')
        h.append('</div></div></section>')
    h.append('</main></body></html>')
    open(os.path.join(RES, 'report.html'), 'w', encoding='utf8').write('\n'.join(h))
    print('\n'.join(md[:16 + len(rows)]))


FINDINGS = [
    ('A4 Islands: exploration livelock (both runs fail)',
     'The robot shuttles between cells (3,6) and (4,6) for ~550 s until the 600 s limit and never '
     'reaches the end cell. Sensing and driving are correct (debug log: every run succeeds, every '
     'wall reading matches the maze). Cause: pick_target() re-optimises the full tour at every '
     'stop and the robot only drives the first straight run of it. At (3,6) facing S the best tour '
     'starts with (3,3) - first run north to (4,6); at (4,6) facing N the best tour starts with '
     '(3,7) - first run back south to (3,6). LoopGuard only counts failed runs and the Watchdog only '
     'counts standing still, so neither fires. In the competition this is "looping without '
     'meaningful progress" and the attempt would be terminated after 15 s.'),
    ('Proposed fix: commit to the chosen target',
     'section2.py: pick_target(..., keep) returns the previously chosen target cell while it is '
     'still a candidate (unresolved and reachable), and only re-optimises the tour once it is '
     'resolved, unreachable or banned. run() remembers the target cell. Tested in test_run_dbg/: '
     '12/12 Webots runs pass, A4 finishes in 132 s of maze time, the other five designs are '
     'unchanged (A1 about 1 s faster); offline on 200 random mazes: 200/200 correct, 42.4 vs 42.3 '
     'cells driven on average.'),
    ('Everything else held up',
     'All other designs: correct LED, every tile found with the right colour (including 9 tiles '
     'at 5:4), whole body inside the end cell with 7-8 cm margin, no collisions, no tipping, never '
     'entered the end cell early (also in A5, where 6 cells are only reachable through it), '
     'longest standstill 4 s. Section 1 (gap reached) took about 20 s on all three obstacle layouts.'),
]

CSS = '''
:root{--bg:#f7f7f5;--fg:#1c1c1a;--muted:#6b6b66;--card:#fff;--line:#e2e1dc;--floor1:#fbfbf9;
--floor2:#2a2a28;--wall:#8a8a84;--obst:#55554f;--pass:#1f7a4d;--fail:#b3261e;--full:#e07a1f;
--s2:#2d6cdf;--blue:#2f5bd3;--green:#2f9a4a;--marker:#d64545}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#161615;--fg:#ecebe6;
--muted:#9c9b95;--card:#20201e;--line:#34332f;--floor1:#3a3a37;--floor2:#0d0d0c;--wall:#b5b4ad;
--obst:#d0cfc8;--pass:#4cc38a;--fail:#ff6b5f;--full:#ffa04d;--s2:#6aa0ff}}
:root[data-theme="dark"]{--bg:#161615;--fg:#ecebe6;--muted:#9c9b95;--card:#20201e;--line:#34332f;
--floor1:#3a3a37;--floor2:#0d0d0c;--wall:#b5b4ad;--obst:#d0cfc8;--pass:#4cc38a;--fail:#ff6b5f;
--full:#ffa04d;--s2:#6aa0ff}
body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.5 system-ui,sans-serif}
main{max-width:1060px;margin:0 auto;padding:24px 16px 64px}
h1{font-size:26px;margin:0 0 4px}h2{font-size:19px;margin:0 0 4px}h3{font-size:15px;margin:0 0 6px}
.lede,.meta{color:var(--muted)}.scroll{overflow-x:auto}
table{border-collapse:collapse;width:100%;font-size:13.5px;background:var(--card)}
th,td{padding:6px 8px;border-bottom:1px solid var(--line);text-align:left;white-space:nowrap}
.num{text-align:right;font-variant-numeric:tabular-nums}.mode{color:var(--muted);font-size:12px}
.badge{font-size:11.5px;font-weight:700;padding:1px 7px;border-radius:9px;color:#fff}
.badge.pass{background:var(--pass)}.badge.fail{background:var(--fail)}
section{margin-top:28px;padding:18px;background:var(--card);border:1px solid var(--line);border-radius:10px}
.pair{display:flex;gap:18px;flex-wrap:wrap}.pair figure{margin:0;flex:0 1 420px;min-width:0}
.runs{flex:1 1 300px;min-width:0}.card{margin-bottom:12px}.card ul{margin:0;padding-left:18px}
.bad{color:var(--fail)}.warn{color:var(--muted)}
svg.arena{width:100%;height:auto;display:block;border-radius:6px}
figcaption{font-size:12px;color:var(--muted);margin-top:4px}
.floor1{fill:var(--floor1)}.floor2{fill:var(--floor2)}.wall{fill:var(--wall)}.obst{fill:var(--obst)}
.marker{fill:var(--marker);opacity:.55}.tile-blue{fill:var(--blue)}.tile-green{fill:var(--green)}
polyline{fill:none;stroke-width:1.6;stroke-linejoin:round;opacity:.9}
polyline.full{stroke:var(--full)}polyline.s2{stroke:var(--s2);stroke-dasharray:4 3}
.full-end{fill:var(--full)}.s2-end{fill:var(--s2)}.hit{fill:none;stroke:var(--fail);stroke-width:2}
.key{display:inline-block;width:14px;height:3px;vertical-align:middle;margin-right:3px}
.key.full{background:var(--full)}.key.s2{background:var(--s2)}
.key.hit{width:8px;height:8px;border:2px solid var(--fail);border-radius:50%}
'''

if __name__ == '__main__':
    main()
