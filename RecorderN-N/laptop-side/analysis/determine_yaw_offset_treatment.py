import numpy as np, math
import os
# Intermediate .npz/.npy files from the extract scripts. Set BYD_ODOM_DATA
# to wherever you re-run the extraction; defaults to the current directory.
DATA_DIR = os.environ.get("BYD_ODOM_DATA", ".") + "/"
S=DATA_DIR
d=np.load(S+'route.npz'); cs,yw,gp,lp=d['cs'],d['yw'],d['gp'],d['lp']
cs=cs[np.argsort(cs[:,0])]; yw=yw[np.argsort(yw[:,0])]; gp=gp[np.argsort(gp[:,0])]
t=cs[:,0]; v=cs[:,1]; steer=cs[:,2]
yr=np.interp(t,yw[:,0],yw[:,1]); yo=np.interp(t,yw[:,0],yw[:,2])
aoff=np.interp(t,lp[:,0],lp[:,1])
dt=np.clip(np.diff(t,prepend=t[0]),0,0.05)

print("=== GPS bearing field populated? ===")
print("  bearingDeg range %.2f..%.2f  distinct %d"%(gp[:,5].min(),gp[:,5].max(),len(np.unique(gp[:,5]))))

# GPS course over ground from successive fixes
lat0,lon0=gp[0,1],gp[0,2]
mlat=111132.0; mlon=111320.0*math.cos(math.radians(lat0))
gx=(gp[:,2]-lon0)*mlon; gy=(gp[:,1]-lat0)*mlat
gt=gp[:,0]; gs=gp[:,4]
dx=np.diff(gx); dy=np.diff(gy); dm=np.hypot(dx,dy)
mid=0.5*(gt[1:]+gt[:-1])
ok=(dm>2.0)&(gs[1:]>3.0)
course=np.unwrap(np.arctan2(dy[ok],dx[ok]))
ct=mid[ok]
print("  usable GPS course points: %d over %.0f s"%(ok.sum(), ct[-1]-ct[0]))
print("  GPS net heading change: %+.1f deg"%math.degrees(course[-1]-course[0]))

# integrate yaw with candidate bias, compare net heading change to GPS
def net_head(bias):
    h=np.cumsum((yr-bias)*dt)
    return np.interp(ct,t,h)
gps_rel=course-course[0]
def err(bias):
    h=net_head(bias); h=h-h[0]
    return h-gps_rel
print("\n=== which bias treatment matches GPS heading? ===")
cands={
 'raw, no correction        ':0.0,
 'subtract YAW_OFFSET (mean)':float(yo.mean()),
 'subtract parked rest value':0.00416,
}
for nm,b in cands.items():
    e=np.degrees(err(b))
    print("  %s bias=%.6f  final %+8.2f deg  RMS %7.2f"%(nm,b,e[-1],np.sqrt(np.mean(e**2))))
# solve least-squares optimal bias
from numpy.polynomial import polynomial as P
bs=np.linspace(-0.002,0.010,1201)
rms=[np.sqrt(np.mean(np.degrees(err(b))**2)) for b in bs]
bopt=bs[int(np.argmin(rms))]
print("  %s bias=%.6f  final %+8.2f deg  RMS %7.2f"%('LEAST-SQUARES OPTIMAL      ',bopt,
      np.degrees(err(bopt))[-1],min(rms)))
print("\n  YAW_OFFSET observed values (rad/s):",sorted(set(np.round(yw[:,2],6).tolist())))
print("  optimal bias / YAW_OFFSET mean = %.3f"%(bopt/yo.mean()))
print("  YAW_RATE zero-code=982 -> raw code at optimal bias = %.2f"%((bopt+2.094216146)/0.002132603))
print("  YAW_OFFSET zero-code=61 -> mean raw code = %.2f"%((yo.mean()+0.130088783)/0.002132603))
