#!/usr/bin/env python3
"""Pearson/Spearman correlation of gap and tick-decay metrics against drift.
Consumes the JSON written by odom_record_gap_drift.py. Written 2026-09-10."""
import json, math, os
d=json.load(open(os.environ.get("BYD_ODOM_OUT", "/tmp/byd_odom_gap_drift.json")))
def pear(x,y):
    n=len(x); mx=sum(x)/n; my=sum(y)/n
    sx=math.sqrt(sum((a-mx)**2 for a in x)); sy=math.sqrt(sum((b-my)**2 for b in y))
    if sx==0 or sy==0: return float("nan")
    return sum((a-mx)*(b-my) for a,b in zip(x,y))/(sx*sy)
def rank(v):
    s=sorted(range(len(v)),key=lambda i:v[i]); r=[0]*len(v)
    for pos,i in enumerate(s): r[i]=pos
    return r
def spear(x,y): return pear(rank(x),rank(y))
preds=["ngap","gap_time","gap_dist","maxgap","decay","hz1","dur","plen"]
targs=["pct","rms","yaw_rms","fin","yaw_fin"]
print("Correlation of gap/decay predictors vs corrected-track drift (n=%d driving runs)"%len(d))
print(f"{'predictor':<12}" + "".join(f"{t:>20}" for t in targs))
print(f"{'':12}" + "".join(f"{'Pearson  Spearman':>20}" for t in targs))
print("-"*(12+20*len(targs)))
for p in preds:
    x=[r[p] for r in d]
    row=f"{p:<12}"
    for t in targs:
        y=[r[t] for r in d]
        row+=f"{pear(x,y):>9.2f}{spear(x,y):>11.2f}"
    print(row)
print()
print("Drift rate normalized (dev_rms per 100 m of path):")
for r in sorted(d,key=lambda z:z["rms"]/z['plen']*100):
    print(f"  {r['run']:<16} {r["rms"]/r['plen']*100:>6.2f} m/100m   gaps={r['ngap']:>4}  gap_s={r['gap_time']:>6.1f}  decay={r['decay']:.2f}")
