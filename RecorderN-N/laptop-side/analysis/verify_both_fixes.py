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
yr=np.interp(t,yw[:,0],yw[:,1]); yo=np.interp(t,yw[:,0],yw[:,2])
aoff=np.interp(t,lp[:,0],lp[:,1])
dt=np.clip(np.diff(t,prepend=t[0]),0,0.05)
WB=2.92; SR=16.0

# --- GPS ground truth: position + course over ground ---
lat0,lon0=gp[0,1],gp[0,2]; mlat=111132.0; mlon=111320.0*math.cos(math.radians(lat0))
gx=(gp[:,2]-lon0)*mlon; gy=(gp[:,1]-lat0)*mlat; gt=gp[:,0]
dx=np.diff(gx); dy=np.diff(gy); dm=np.hypot(dx,dy); mid=0.5*(gt[1:]+gt[:-1])
ok=(dm>2.0)&(gp[1:,4]>3.0)
course=np.unwrap(np.arctan2(dy[ok],dx[ok])); ct=mid[ok]
course_rel=course-course[0]

# --- heading sources ---
# bicycle-model yaw rate is proportional to velocity, so the gear sign must be
# applied here too -- reversing with the wheel turned rotates the other way.
# This mirrors odom_node.py, where v_for_integration feeds heading_rate_rad_s().
bike=lambda vv: vv*np.tan(np.radians(steer-aoff)/SR)/WB
h_kin=np.cumsum(bike(v)*dt)                   # unsigned v
h_kin_s=np.cumsum(bike(v*sgn)*dt)             # gear-signed v
h_yaw=np.cumsum((yr-yo)*dt)                   # AS DEPLOYED in carstate.py
h_yaw_opt=np.cumsum((yr-0.004170)*dt)         # best-fit bias (ceiling)

def track(h,signed):
    vv=v*sgn if signed else v
    vv=np.where(np.abs(vv)<0.05,0.0,vv)
    best=None;bh=0.0
    for h0 in np.radians(np.arange(0,360,0.25)):
        hh=h+h0; x=np.cumsum(vv*np.cos(hh)*dt); y=np.cumsum(vv*np.sin(hh)*dt)
        xi=np.interp(gt,t,x); yi=np.interp(gt,t,y); m=gt-gt[0]<60
        e=np.mean((xi[m]-gx[m])**2+(yi[m]-gy[m])**2)
        if best is None or e<best: best,bh=e,h0
    hh=h+bh; x=np.cumsum(vv*np.cos(hh)*dt); y=np.cumsum(vv*np.sin(hh)*dt)
    dev=np.hypot(np.interp(gt,t,x)-gx,np.interp(gt,t,y)-gy)
    hi=np.interp(ct,t,h); herr=np.degrees((hi-hi[0])-course_rel)
    return x,y,dev,herr

CASES=[("1  raw kinematic (SR=16, unsigned v)",h_kin,False),
       ("2  reverse-sign fix only (SR=16)    ",h_kin_s,True),
       ("3  yaw sensor alone (unsigned v)    ",h_yaw,False),
       ("4  NEW gear-sign + yaw sensor       ",h_yaw,True)]
print("=== 3b  scored against GPS (661 s, 5271 m, 618 course points) ===")
print("%-38s %12s %11s %12s %12s"%("","final head","RMS head","max dev","final dev"))
R={}
for nm,h,sg in CASES:
    x,y,dev,herr=track(h,sg); R[nm[0]]=(x,y,dev,herr)
    print("%-38s %+11.2f° %10.2f° %11.1f m %11.1f m"%(nm,herr[-1],np.sqrt(np.mean(herr**2)),dev.max(),dev[-1]))
x,y,dev,herr=track(h_yaw_opt,True); R['5']=(x,y,dev,herr)
print("%-38s %+11.2f° %10.2f° %11.1f m %11.1f m"%("5  (ceiling) gear-sign + best-fit bias",herr[-1],np.sqrt(np.mean(herr**2)),dev.max(),dev[-1]))
print("\n=== 3d  does the combination beat yaw-sensor alone? ===")
d3,d4=R['3'][2],R['4'][2]; h3,h4=R['3'][3],R['4'][3]
print("  final dev   %.1f m -> %.1f m   (%+.1f%%)"%(d3[-1],d4[-1],100*(1-d4[-1]/d3[-1])))
print("  max dev     %.1f m -> %.1f m   (%+.1f%%)"%(d3.max(),d4.max(),100*(1-d4.max()/d3.max())))
print("  RMS heading %.2f° -> %.2f°  (identical by construction: same heading source)"%(
      np.sqrt(np.mean(h3**2)),np.sqrt(np.mean(h4**2))))
np.savez(S+'stage3.npz',gx=gx,gy=gy,gt=gt,t=t,ct=ct,
         **{f'x{k}':R[k][0] for k in R},**{f'y{k}':R[k][1] for k in R},
         **{f'd{k}':R[k][2] for k in R},**{f'h{k}':R[k][3] for k in R})
print("\nsaved stage3.npz")
