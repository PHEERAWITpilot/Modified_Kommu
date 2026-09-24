import numpy as np, math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os
# Intermediate .npz/.npy files from the extract scripts. Set BYD_ODOM_DATA
# to wherever you re-run the extraction; defaults to the current directory.
DATA_DIR = os.environ.get("BYD_ODOM_DATA", ".") + "/"
SURFACE="#fcfcfb"; INK="#0b0b0b"; INK2="#52514e"; GRID="#e5e4e0"
ORANGE="#e07b1f"; PURPLE="#7b4fc9"; GREEN="#1baf7a"
plt.rcParams.update({"figure.facecolor":SURFACE,"axes.facecolor":SURFACE,"axes.edgecolor":GRID,
 "axes.labelcolor":INK2,"text.color":INK,"xtick.color":INK2,"ytick.color":INK2,"axes.grid":True,
 "grid.color":GRID,"grid.linewidth":0.6,"axes.spines.top":False,"axes.spines.right":False,
 "font.size":10,"figure.dpi":130})
S=DATA_DIR
d=np.load(S+'route.npz'); yw,gp,lp=d['yw'],d['gp'],d['lp']
cs=np.load(S+'cs_gear.npy'); cs=cs[np.argsort(cs[:,0])]
yw=yw[np.argsort(yw[:,0])]; gp=gp[np.argsort(gp[:,0])]
t=cs[:,0]; v=cs[:,1]; steer=cs[:,2]; sgn=cs[:,3]
yr=np.interp(t,yw[:,0],yw[:,1]); aoff=np.interp(t,lp[:,0],lp[:,1])
dt=np.clip(np.diff(t,prepend=t[0]),0,0.05); hT=np.cumsum((yr-0.004170)*dt)
WB=2.92; K=1.1974; tr=t-t[0]
lat0,lon0=gp[0,1],gp[0,2]; mlat=111132.0; mlon=111320.0*math.cos(math.radians(lat0))
gx=(gp[:,2]-lon0)*mlon; gy=(gp[:,1]-lat0)*mlat; gtr=gp[:,0]-gp[0,0]
def head(sr,signed): 
    vv=v*sgn if signed else v
    return np.cumsum((vv*np.tan(np.radians(steer-aoff)/sr)/WB)*dt)
def track(h,signed):
    vv=v*sgn if signed else v
    best=None;bh=0.0
    for h0 in np.radians(np.arange(0,360,0.25)):
        hh=h+h0; x=np.cumsum(vv*np.cos(hh)*dt); y=np.cumsum(vv*np.sin(hh)*dt)
        xi=np.interp(gp[:,0],t,x); yi=np.interp(gp[:,0],t,y); m=gtr<60
        e=np.mean((xi[m]-gx[m])**2+(yi[m]-gy[m])**2)
        if best is None or e<best: best,bh=e,h0
    hh=h+bh; x=np.cumsum(vv*np.cos(hh)*dt); y=np.cumsum(vv*np.sin(hh)*dt)
    return x,y,np.hypot(np.interp(gp[:,0],t,x)-gx,np.interp(gp[:,0],t,y)-gy)
res={}
for tag,signed in (("aslog",False),("revfix",True)):
    hA=head(16.0,signed); hB=head(16.0/K,signed)
    xA,yA,dA=track(hA,signed); xB,yB,dB=track(hB,signed)
    res[tag]=dict(hA=hA,hB=hB,xA=xA,yA=yA,xB=xB,yB=yB,dA=dA,dB=dB,
                  eA=np.degrees(hA-hT),eB=np.degrees(hB-hT))
fig=plt.figure(figsize=(14.5,9.6))
gs=fig.add_gridspec(2,2,hspace=0.30,wspace=0.22)
for col,(tag,title) in enumerate([("aslog","(a) as logged — vEgo unsigned (current refstart)"),
                                  ("revfix","(b) same, with reverse-gear sign applied to vEgo")]):
    r=res[tag]; ax=fig.add_subplot(gs[0,col])
    ax.plot(gx,gy,color=ORANGE,lw=2.4,label="GNSS baseline (ground truth)",zorder=5)
    ax.plot(r['xA'],r['yA'],color=PURPLE,lw=1.5,label="refstart, current  WB×SR=46.720",zorder=4)
    ax.plot(r['xB'],r['yB'],color=GREEN,lw=1.5,label="refstart, geometry-corrected  39.018",zorder=4)
    ax.plot(gx[0],gy[0],'o',color=INK,ms=5,zorder=6)
    ax.set_aspect('equal'); ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")
    ax.set_title(title,fontsize=10.5,loc='left')
    ax.legend(loc='lower left',framealpha=0.92,fontsize=8.5)
ax3=fig.add_subplot(gs[1,0])
ax3.plot(tr,res['aslog']['eA'],color=PURPLE,lw=1.1,label="current, as logged")
ax3.plot(tr,res['aslog']['eB'],color=GREEN,lw=1.1,label="geom-corrected, as logged")
ax3.plot(tr,res['revfix']['eA'],color=PURPLE,lw=1.4,ls="--",label="current, reverse fixed")
ax3.plot(tr,res['revfix']['eB'],color=GREEN,lw=1.4,ls="--",label="geom-corrected, reverse fixed")
ax3.axhline(0,color=ORANGE,lw=1.6); ax3.axvspan(360,378,color=INK2,alpha=0.10,lw=0)
ax3.annotate("reverse manoeuvre (13.8 s)",(369,ax3.get_ylim()[1]*0.97),ha='center',va='top',fontsize=8,color=INK2)
ax3.set_xlabel("time (s)"); ax3.set_ylabel("heading error (deg)")
ax3.set_title("heading error vs yaw-sensor reference",fontsize=10.5,loc='left')
ax3.legend(fontsize=8,framealpha=0.92,ncol=2)
ax4=fig.add_subplot(gs[1,1])
ax4.plot(gtr,res['aslog']['dA'],color=PURPLE,lw=1.1)
ax4.plot(gtr,res['aslog']['dB'],color=GREEN,lw=1.1)
ax4.plot(gtr,res['revfix']['dA'],color=PURPLE,lw=1.4,ls="--")
ax4.plot(gtr,res['revfix']['dB'],color=GREEN,lw=1.4,ls="--")
ax4.axhline(0,color=ORANGE,lw=1.6); ax4.set_yscale('symlog',linthresh=10)
ax4.set_xlabel("time (s)"); ax4.set_ylabel("deviation from baseline (m, log)")
ax4.set_title("lateral deviation  (solid = as logged, dashed = reverse fixed)",fontsize=10.5,loc='left')
fig.suptitle("Figure 3 — refstart integration: current vs geometry-corrected constants, "
             "route 2026-07-13--07-05-29 seg 15–25  (661 s, 5271 m)",fontsize=12,x=0.5,y=0.975)
OUT="/home/inc/Desktop/Kommu.AI/claude/refstart_geometry_comparison.png"
fig.savefig(OUT,bbox_inches="tight"); print("saved",OUT)
print("\n=== 2d METRICS ===")
for tag,lbl in (("aslog","AS LOGGED (unsigned vEgo)"),("revfix","REVERSE-SIGN FIXED")):
    r=res[tag]; print("\n "+lbl)
    for nm,e,dv in (("purple  current   WB×SR=46.720",r['eA'],r['dA']),
                    ("green   corrected WB×SR=39.018",r['eB'],r['dB'])):
        print("   %-32s final head %+8.2f°  RMS %7.2f°  max dev %8.1f m  final dev %8.1f m"%(
            nm,e[-1],np.sqrt(np.mean(e**2)),dv.max(),dv[-1]))
    print("   drift closed by geometry alone: final head %+.1f%%  RMS head %+.1f%%  max dev %+.1f%%  final dev %+.1f%%"%(
        100*(1-abs(r['eB'][-1])/abs(r['eA'][-1])),
        100*(1-np.sqrt(np.mean(r['eB']**2))/np.sqrt(np.mean(r['eA']**2))),
        100*(1-r['dB'].max()/r['dA'].max()),100*(1-r['dB'][-1]/r['dA'][-1])))
