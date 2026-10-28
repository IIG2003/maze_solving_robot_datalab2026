# generate a random test arena + obstacles + world (TEST ONLY)
import random, sys
seed = int(sys.argv[1]); random.seed(seed)
ROOT = 'C:/Users/User/Documents/robotic/pera_bot_2026/PeraBots_UC_v2/PeraBots_UC/'
R, C = 4, 9
X0, Y0, CELL = 0.22, 1.455, 0.25
# ---- maze: DFS perfect maze + a few loops
open_e = set()
def key(a, b): return tuple(sorted((a, b)))
seen = {(1, 1)}; stack = [(1, 1)]
while stack:
    r, c = stack[-1]
    nb = [(r+dr, c+dc) for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)) if 1 <= r+dr <= R and 1 <= c+dc <= C and (r+dr, c+dc) not in seen]
    if not nb: stack.pop(); continue
    n = random.choice(nb); open_e.add(key((r, c), n)); seen.add(n); stack.append(n)
edges = [key((r, c), (r+dr, c+dc)) for r in range(1, R+1) for c in range(1, C+1) for dr, dc in ((1,0),(0,1)) if r+dr <= R and c+dc <= C]
closed = [e for e in edges if e not in open_e]
for e in random.sample(closed, 3): open_e.add(e)
def deg(cell): return sum(1 for e in open_e if cell in e)
dead = [(r, c) for r in range(1, R+1) for c in range(1, C+1) if deg((r, c)) == 1 and (r, c) not in ((1, 1), (4, 9))]
k = min(len(dead), random.choice([1, 3, 5]))
if k % 2 == 0: k -= 1
tiles = {cell: random.choice(['blue', 'green']) for cell in random.sample(dead, k)}
def cx(c): return X0 + CELL*(c-1)
def cy(r): return Y0 + CELL*(r-1)
def solid(name, x, y, z, sx, sy, sz, color=None, bound=True):
    col = 'baseColor %s\n              ' % color if color else ''
    b = '\n        boundingObject Box {\n          size %g %g %g\n        }' % (sx, sy, sz) if bound else ''
    return '''      Solid {
        translation %.4f %.4f %g
        children [
          Shape {
            appearance PBRAppearance {
              %sroughness 1
              metalness 0
            }
            geometry Box {
              size %g %g %g
            }
          }
        ]
        name "%s"%s
      }
''' % (x, y, z, col, sx, sy, sz, name, b)
nodes = ''
for e in edges:
    if e in open_e: continue
    (r1, c1), (r2, c2) = e
    if r1 == r2:
        nodes += solid('w_v_%d_%d' % (r1, c1), cx(c1)+0.125, cy(r1), 0.05, 0.012, 0.262, 0.1)
    else:
        nodes += solid('w_h_%d_%d' % (r1, c1), cx(c1), cy(r1)+0.125, 0.05, 0.262, 0.012, 0.1)
for (r, c), col in tiles.items():
    nodes += solid('tile_%s_%d_%d' % (col, r, c), cx(c), cy(r), 0.001, 0.25, 0.25, 0.001,
                   '0 0 0.5' if col == 'blue' else '0 0.6 0', bound=False)
a = open(ROOT + 'protos/Arena.proto', encoding='utf8').read()
cut = a.index('name "end_marker"\n      }\n') + len('name "end_marker"\n      }\n')
a = a[:cut].replace('PROTO Arena [', 'PROTO TestArena%d [' % seed) + nodes + '    ]\n  }\n}\n'
open(ROOT + 'protos/TestArena%d.proto' % seed, 'w', encoding='utf8', newline='\n').write(a)
# ---- obstacles: clusters of touching cubes, >= 0.25 m passages between clusters
cubes = []
def ok(x, y, cluster):
    if not (0.056 <= x <= 2.384 and 0.056 <= y <= 1.164): return False
    if x > 2.0 and y < 0.35: return False            # start zone
    if x < 0.6 and y > 0.85: return False            # approach to the gap
    for (a, b, cl) in cubes:
        if cl == cluster: continue
        if max(abs(a-x), abs(b-y)) < 0.37: return False
    # wall gap: either touching (<0.06) or >= 0.30 from the wall face
    for d in (x - 0.006, 2.434 - x, y - 0.006, 1.214 - y):
        if 0.055 < d < 0.36: return False
    return True
for cl in range(400):
    if len(cubes) >= 40: break
    x, y = random.uniform(0.05, 2.39), random.uniform(0.05, 1.17)
    if not ok(x, y, cl): continue
    group = [(x, y)]
    for _ in range(random.randint(0, 3)):
        gx, gy = random.choice(group)
        dx, dy = random.choice(((0.1, 0), (-0.1, 0), (0, 0.1), (0, -0.1)))
        if ok(gx+dx, gy+dy, cl) and (gx+dx, gy+dy) not in group: group.append((gx+dx, gy+dy))
    for g in group: cubes.append((g[0], g[1], cl))
o = '#VRML_SIM R2025a utf8\nPROTO TestObstacles%d [\n] {\n  Group {\n    children [\n' % seed
for i, (x, y, _) in enumerate(cubes):
    o += solid('obstacle_%d' % i, x, y, 0.05, 0.1, 0.1, 0.1, '0.02 0.02 0.02')
o += '    ]\n  }\n}\n'
open(ROOT + 'protos/TestObstacles%d.proto' % seed, 'w', encoding='utf8', newline='\n').write(o)
w = open(ROOT + 'worlds/test_truth.wbt', encoding='utf8').read()
w = w.replace('../protos/Arena.proto', '../protos/TestArena%d.proto' % seed).replace('../protos/Obstacles.proto', '../protos/TestObstacles%d.proto' % seed)
w = w.replace('\nArena {', '\nTestArena%d {' % seed).replace('\nObstacles {', '\nTestObstacles%d {' % seed)
open(ROOT + 'worlds/test_rand%d.wbt' % seed, 'w', encoding='utf8', newline='\n').write(w)
b = sum(v == 'blue' for v in tiles.values()); g = len(tiles) - b
print('seed', seed, 'cubes', len(cubes), 'tiles', sorted(tiles.items()), 'blue', b, 'green', g, 'expect', 'blue' if b > g else 'green')
