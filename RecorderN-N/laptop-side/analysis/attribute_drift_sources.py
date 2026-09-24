import numpy as np, math
import os
# Intermediate .npz/.npy files from the extract scripts. Set BYD_ODOM_DATA
# to wherever you re-run the extraction; defaults to the current directory.
DATA_DIR = os.environ.get("BYD_ODOM_DATA", ".") + "/"
S=DATA_DIR
d=np.load(S+'route.npz'); yw,gp,lp=d['yw'],d['gp'],d['lp']
cs=np.load(S+'cs_gear.npy'); cs=cs[np.argsort(cs[:,0])]
yw=yw[np.argsort(yw[:,0])]; gp=gp[np.argsort(gp[:,0])]
t=cs[:,0]; v=cs[:,1]; steer=cs[:,2]; sgn=cs[:,3]
yr=np.interp(t,yw[:,0],yw[:,1]); aoff=np.interp(t,lp[:,0],lp[:,1])
dt=np.clip(np.diff(t,prepend=t[0]),0,0.05)
hT=np.cumsum((yr-0.004170)*dt)
WB=2.92
lat0,lon0=gp[0,1],gp[0,2]; mlat=111132.0; mlon=111320.0*math.cos(math.radians(lat0))
gx=(gp[:,2]-lon0)*mlon; gy=(gp[:,1]-lat0)*mlat
def head(sr,signed,extra=0.0):
    vv=v*sgn if signed else v
    return np.cumsum((vv*np.tan(np.radians(steer-aoff-extra)/sr)/WB)*dt)
def pos(h,signed):
    vv=v*sgn if signed else v
    best=None;bh=0.0
    for h0 in np.radians(np.arange(0,360,0.25)):
        hh=h+h0; x=np.cumsum(vv*np.cos(hh)*dt); y=np.cumsum(vv*np.sin(hh)*dt)
        xi=np.interp(gp[:,0],t,x); yi=np.interp(gp[:,0],t,y)
        m=gp[:,0]-gp[0,0]<60
        e=np.mean((xi[m]-gx[m])**2+(yi[m]-gy[m])**2)
        if best is None or e<best: best,bh=e,h0
    hh=h+bh; x=np.cumsum(vv*np.cos(hh)*dt); y=np.cumsum(vv*np.sin(hh)*dt)
    return np.hypot(np.interp(gp[:,0],t,x)-gx,np.interp(gp[:,0],t,y)-gy)
K=1.1974
cases=[
 ("current: SR=16.0, unsigned v      ",16.0,False,0.0),
 ("geometry fix only (SR=13.362)     ",16.0/K,False,0.0),
 ("reverse-sign fix only (SR=16.0)   ",16.0,True,0.0),
 ("reverse + geometry                ",16.0/K,True,0.0),
 ("reverse + geometry + centre +1.2deg",16.0/K,True,1.2),
]
print("%-36s %12s %11s %12s"%("","final head","RMS head","final dev"))
base=None
for nm,sr,sg,ex in cases:
    h=head(sr,sg,ex); e=np.degrees(h-hT); dv=pos(h,sg)
    if base is None: base=(np.sqrt(np.mean(e**2)),dv[-1])
    print("%-36s %+11.2f° %10.2f° %11.1f m   (RMS %.0f%%, pos %.0f%% closed)"%(
        nm,e[-1],np.sqrt(np.mean(e**2)),dv[-1],
        100*(1-np.sqrt(np.mean(e**2))/base[0]),100*(1-dv[-1]/base[1])))
