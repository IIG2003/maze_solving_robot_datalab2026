# TEST ONLY - not part of the submission. Logs the true robot pose.
import os, math
from controller import Supervisor
sup = Supervisor()
dt = int(sup.getBasicTimeStep())
bot = sup.getFromDef('BOT')
path = os.environ.get('PERABOT_LOG', 'truth.log')
k = 0
while sup.step(dt) != -1:
    k += 1
    if k % 31 == 0:
        x, y, z = bot.getPosition()
        R = bot.getOrientation()
        th = math.atan2(R[3], R[0])
        with open(path, 'a') as f:
            f.write('[truth] t=%.2f x=%.3f y=%.3f th=%.1f\n' % (sup.getTime(), x, y, math.degrees(th)))
