import sys, zstandard, capnp, numpy as np
import os
# Intermediate .npz/.npy files from the extract scripts. Set BYD_ODOM_DATA
# to wherever you re-run the extraction; defaults to the current directory.
DATA_DIR = os.environ.get("BYD_ODOM_DATA", ".") + "/"
sys.path.insert(0,'/home/inc/Desktop/Kommu.AI/bumpbump_clone')
capnp.remove_import_hook()
log=capnp.load('/home/inc/Desktop/Kommu.AI/bumpbump_clone/cereal/log.capnp')
base='/home/inc/Desktop/Kommu.AI/claude/longitudinal_analysis/rlogs/2026-07-13--07-05-29--%d/rlog.zst'
cs=[]
for n in range(15,26):
    with open(base%n,'rb') as f: raw=zstandard.ZstdDecompressor().stream_reader(f).read()
    for m in log.Event.read_multiple_bytes(raw):
        if m.which()!='carState': continue
        t=m.logMonoTime/1e9
        if t<300: continue
        c=m.carState
        cs.append((t,c.vEgo,c.steeringAngleDeg,1.0 if str(c.gearShifter)!='reverse' else -1.0))
cs=np.array(cs)
np.save('cs_gear.npy',cs)
print("saved",cs.shape,"reverse samples:",int((cs[:,3]<0).sum()))
