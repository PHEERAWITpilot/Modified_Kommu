import numpy as np, math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os
# Intermediate .npz/.npy files from the extract scripts. Set BYD_ODOM_DATA
# to wherever you re-run the extraction; defaults to the current directory.
DATA_DIR = os.environ.get("BYD_ODOM_DATA", ".") + "/"
SURFACE="#fcfcfb"; INK="#0b0b0b"; INK2="#52514e"; GRID="#e5e4e0"
ORANGE="#e07b1f"; PURPLE="#7b4fc9"; BLUE="#2a78d6"; GREEN="#1baf7a"
plt.rcParams.update({"figure.facecolor":SURFACE,"axes.facecolor":SURFACE,"axes.edgecolor":GRID,
 "axes.labelcolor":INK2,"text.color":INK,"xtick.color":INK2,"ytick.color":INK2,"axes.grid":True,
 "grid.color":GRID,"grid.linewidth":0.6,"axes.spines.top":False,"axes.spines.right":False,
 "font.size":10,"figure.dpi":130})
S=DATA_DIR
z=np.load(S+'stage3.npz')
gx,gy,gt,t=z['gx'],z['gy'],z['gt'],z['t']; gtr=gt-gt[0]
# 3 and 4 nearly coincide (same heading source), so 3 is drawn last, dashed, on top.
SER=[("1",PURPLE,1.4,"-","raw kinematic — current constants, unsigned v, no yaw sensor"),
     ("2",BLUE,  1.4,"-","reverse-sign fix only — kinematic heading"),
     ("4",GREEN, 2.2,"-","NEW: gear-signed v + yaw-sensor heading"),
     ("3",INK2,  1.1,(0,(5,3)),"yaw sensor alone, offset-corrected (unsigned v)")]
fig=plt.figure(figsize=(14.5,7.2))
gs=fig.add_gridspec(2,2,width_ratios=[1.5,1],hspace=0.40,wspace=0.20)
ax=fig.add_subplot(gs[:,0])
ax.plot(gx,gy,color=ORANGE,lw=2.6,label="GPS baseline (ground truth)",zorder=6)
for i,(k,c,lw,ls,lab) in enumerate(SER):
    ax.plot(z['x'+k],z['y'+k],color=c,lw=lw,ls=ls,label=lab,zorder=4+i)
ax.plot(gx[0],gy[0],'o',color=INK,ms=6,zorder=7)
ax.set_aspect('equal'); ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")
ax.set_title("Figure 4 — both fixes, offline reprocessing\n"
             "route 2026-07-13--07-05-29 seg 15–25   661 s   5271 m",fontsize=11,loc='left')
ax.legend(loc='lower left',framealpha=0.93,fontsize=8.5)
ax2=fig.add_subplot(gs[0,1])
for i,(k,c,lw,ls,_) in enumerate(SER): ax2.plot(z['ct']-z['ct'][0],z['h'+k],color=c,lw=lw,ls=ls,zorder=4+i)
ax2.axhline(0,color=ORANGE,lw=1.6)
ax2.set_ylabel("heading error (deg)"); ax2.set_xlabel("time (s)")
ax2.set_title("heading error vs GPS course over ground",fontsize=10,loc='left')
ax3=fig.add_subplot(gs[1,1])
for i,(k,c,lw,ls,_) in enumerate(SER): ax3.plot(gtr,z['d'+k],color=c,lw=lw,ls=ls,zorder=4+i)
ax3.axhline(0,color=ORANGE,lw=1.6); ax3.set_yscale('symlog',linthresh=10)
ax3.set_ylabel("deviation (m, log)"); ax3.set_xlabel("time (s)")
ax3.set_title("position deviation from GPS baseline",fontsize=10,loc='left')
OUT="/home/inc/Desktop/Kommu.AI/claude/stage3_both_fixes.png"
fig.savefig(OUT,bbox_inches="tight"); print("saved",OUT)
