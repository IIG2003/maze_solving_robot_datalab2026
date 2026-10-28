# Export one standalone SVG per test arena (TEST ONLY).
#
# Same drawing as report.html (arena, walls, tiles, obstacles, the robot's true
# path for the full run and the Section 2 run), but with the colours embedded so
# the files render on their own, e.g. as images in the GitHub README.
#
# usage: python export_svgs.py [--results DIR] [--out DIR]
#        (default: video_results/ -> ../../docs/paths/)
import argparse
import os

import make_report as mr

STYLE = ('<style>'
         '.bg{fill:#ffffff}.floor1{fill:#fbfbf9}.floor2{fill:#2a2a28}.wall{fill:#8a8a84}'
         '.obst{fill:#55554f}.marker{fill:#d64545;opacity:.55}'
         '.tile-blue{fill:#2f5bd3}.tile-green{fill:#2f9a4a}'
         'polyline{fill:none;stroke-width:1.6;stroke-linejoin:round;opacity:.9}'
         'polyline.full{stroke:#e07a1f}polyline.s2{stroke:#4f8dff;stroke-dasharray:4 3}'
         '.full-end{fill:#e07a1f}.s2-end{fill:#4f8dff}.hit{fill:none;stroke:#b3261e;stroke-width:2}'
         '</style>')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results', default=os.path.join(mr.HERE, 'video_results'))
    ap.add_argument('--out', default=os.path.join(mr.HERE, '..', '..', 'docs', 'paths'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    designs = mr.load(os.path.join(mr.HERE, 'suite.json'))
    rows = mr.collect(designs, a.results)
    for key, d in designs.items():
        runs = [(mode, rows[(key, mode)][1]) for mode in ('full', 's2')]
        body = mr.svg(d, runs).split('\n')
        head = body[0].replace('class="arena"', 'xmlns="http://www.w3.org/2000/svg" '
                               'width="420" height="420"')
        out = [head, STYLE, '<rect class="bg" width="420" height="420"/>'] + body[1:]
        path = os.path.join(a.out, '%s.svg' % key)
        open(path, 'w', encoding='utf8').write('\n'.join(out) + '\n')
        print(path)


if __name__ == '__main__':
    main()
