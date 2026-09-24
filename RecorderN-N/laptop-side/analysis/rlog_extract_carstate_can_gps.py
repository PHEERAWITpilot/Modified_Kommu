import sys, zstandard, capnp, numpy as np
import os
# Intermediate .npz/.npy files from the extract scripts. Set BYD_ODOM_DATA
# to wherever you re-run the extraction; defaults to the current directory.
DATA_DIR = os.environ.get("BYD_ODOM_DATA", ".") + "/"
sys.path.insert(0, '/home/inc/Desktop/Kommu.AI/bumpbump_clone')
capnp.remove_import_hook()
log = capnp.load('/home/inc/Desktop/Kommu.AI/bumpbump_clone/cereal/log.capnp')
base = '/home/inc/Desktop/Kommu.AI/claude/longitudinal_analysis/rlogs/2026-07-13--07-05-29--%d'

cs = []   # t, vEgo, steerDeg
yw = []   # t, yaw_rate_phys, yaw_offset_phys
gp = []   # t, lat, lon, flags, speed, bearing
lp = []   # t, angleOffsetDeg
for n in range(15, 26):
    with open(base % n + '/rlog.zst', 'rb') as f:
        raw = zstandard.ZstdDecompressor().stream_reader(f).read()
    for m in log.Event.read_multiple_bytes(raw):
        t = m.logMonoTime / 1e9
        if t < 300: continue          # drop stale boot-time events
        w = m.which()
        if w == 'carState':
            c = m.carState; cs.append((t, c.vEgo, c.steeringAngleDeg))
        elif w == 'can':
            for x in m.can:
                if x.src == 0 and x.address == 546:
                    v = int.from_bytes(bytes(x.dat), 'little')
                    yr = ((v >> 0) & 0xFFF) * 0.002132603 - 2.094216146
                    yo = ((v >> 12) & 0xFFF) * 0.002132603 - 0.130088783
                    yw.append((t, yr, yo))
        elif w in ('gpsLocation', 'gpsLocationExternal'):
            g = getattr(m, w)
            gp.append((t, g.latitude, g.longitude, g.flags, g.speed, g.bearingDeg))
        elif w == 'liveParameters':
            lp.append((t, m.liveParameters.angleOffsetDeg))

for nm, arr in (('cs', cs), ('yw', yw), ('gp', gp), ('lp', lp)):
    print(nm, len(arr))
np.savez('route.npz',
         cs=np.array(cs), yw=np.array(yw), gp=np.array(gp), lp=np.array(lp) if lp else np.zeros((0,2)))
print('saved')
