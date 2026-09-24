#!/usr/bin/env python3
"""Gap / tick-decay vs corrected-track drift across recorded Odom_record runs.
Baselines `corrected` against trackA_persample: the `measured` track is frozen
at yaw 0.000 in the five 2026-09-09 evening runs, so it is not a valid
reference there. Excludes the six park-only runs (zero wheel speed).
Written 2026-09-10."""
import csv, math, os, json
ROOT=os.environ.get("BYD_ODOM_RECORD", os.path.expanduser("~/Desktop/Kommu.AI/Odom_record"))
GAP=0.3
def wrap(d): return (d+180.0)%360.0-180.0
DRIVE=["2026/09/09/1646","2026/09/09/1857","2026/09/09/1908","2026/09/09/1918","2026/09/09/1923",
       "2026/09/09/1935","2026/09/10/1548","2026/09/10/1549","2026/09/10/1607","2026/09/10/1625",
       "2026/09/10/1634","2026/09/10/1638","2026/09/10/1651"]
REF="trackA_persample"
out=[]
for run in DRIVE:
    p=os.path.join(ROOT,run,"odom.csv")
    t=[];v=[];cx=[];cy=[];cyaw=[];rx=[];ry=[];ryaw=[]
    for r in csv.DictReader(open(p)):
        t.append(float(r["t_mono"])); v.append(float(r["v_integrated_ms"]))
        cx.append(float(r["corrected_x"]));cy.append(float(r["corrected_y"]));cyaw.append(float(r["corrected_yaw_deg"]))
        rx.append(float(r[REF+"_x"]));ry.append(float(r[REF+"_y"]));ryaw.append(float(r[REF+"_yaw_deg"]))
    n=len(t); dur=t[-1]-t[0]
    gaps=[(i,t[i]-t[i-1]) for i in range(1,n) if t[i]-t[i-1]>GAP]
    gap_time=sum(g[1] for g in gaps); gap_dist=sum(abs(v[i])*dt for i,dt in gaps)
    maxgap=max((g[1] for g in gaps),default=0.0)
    def hz(a,b):
        c=sum(1 for x in t if a<=x<b); return c/(b-a) if b>a else 0
    hz0=hz(t[0],t[0]+30); hz1=hz(t[-1]-30,t[-1]); decay=hz1/hz0 if hz0 else float("nan")
    plen=sum(math.hypot(cx[i]-cx[i-1],cy[i]-cy[i-1]) for i in range(1,n))
    dev=[math.hypot(cx[i]-rx[i],cy[i]-ry[i]) for i in range(n)]
    dyaw=[abs(wrap(cyaw[i]-ryaw[i])) for i in range(n)]
    fin=dev[-1]; mx=max(dev)
    rms=math.sqrt(sum(d*d for d in dev)/n)
    yaw_rms=math.sqrt(sum(d*d for d in dyaw)/n)
    pct=100.0*fin/plen if plen>1 else float("nan")
    out.append(dict(run=run,n=n,dur=dur,hz0=hz0,hz1=hz1,decay=decay,ngap=len(gaps),
        gap_time=gap_time,gap_dist=gap_dist,maxgap=maxgap,plen=plen,fin=fin,mx=mx,
        rms=rms,yaw_fin=dyaw[-1],yaw_rms=yaw_rms,pct=pct))
h=f"{'run':<16}{'dur_s':>7}{'path_m':>8}{'Hz_1st':>7}{'Hz_lst':>7}{'decay':>6}{'gaps':>5}{'gap_s':>7}{'gap_m':>7}{'maxgap':>7}{'dev_fin':>8}{'dev_max':>8}{'dev_rms':>8}{'dev%':>6}{'dYaw_f':>7}{'dYawrms':>8}"
print(h);print("-"*len(h))
for r in sorted(out,key=lambda x:x["pct"]):
    print(f"{r['run']:<16}{r['dur']:>7.0f}{r['plen']:>8.0f}{r['hz0']:>7.1f}{r['hz1']:>7.1f}{r['decay']:>6.2f}{r['ngap']:>5}{r['gap_time']:>7.1f}{r['gap_dist']:>7.1f}{r['maxgap']:>7.2f}{r['fin']:>8.1f}{r['mx']:>8.1f}{r['rms']:>8.1f}{r['pct']:>6.1f}{r['yaw_fin']:>7.1f}{r['yaw_rms']:>8.1f}")
json.dump(out, open(os.environ.get("BYD_ODOM_OUT", "/tmp/byd_odom_gap_drift.json"), "w"), indent=1)
