"""Diagnose SPMS1 repeats using common valid timestamps and map-update logs."""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import evaluate_ntu_viral as ntu

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'experiments/adaptive_map_v1/validation/ntu_spms1'
OUT=ROOT/'experiments/adaptive_map_v1/summary/spms1_repeats_audit'
OUT.mkdir(exist_ok=True)
gt=ROOT/'bag/NTU/spms_01/ground_truth/ground_truth.csv'
gt_t,gt_p=ntu.read_gt(gt)
zero=float(np.genfromtxt(gt,delimiter=',',names=True,max_rows=1)['time'])
logs={};matched={};own={}
for mode in ['off','on']:
    for i in range(1,4):
        name=f'{mode}{i}';method='ours_adaptive' if mode=='on' else 'ours_no_adaptive'
        run=BASE/method/'run02';source=run/f'runtime{i}.csv'
        rows=list(csv.DictReader(source.open()))
        logs[name]={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
        t,p=ntu.read_estimate(source,zero)
        t,p,g=ntu.interpolate_gt(gt_t,gt_p,t,p)
        # All files share scan timestamps when the same scan was logged.
        keys=np.rint(t*1e4).astype(np.int64)
        assert len(set(keys))==len(keys)
        matched[name]={int(k):(tt,pp,gg) for k,tt,pp,gg in zip(keys,t,p,g)}
        own[name]=json.loads((run/f'evaluation_runtime{i}/ntu_ate_metrics.json').read_text())['ate_rmse_m']
common=sorted(set.intersection(*[set(v) for v in matched.values()]))
def align(p,g,mask):
    a=p[mask].mean(axis=0);b=g[mask].mean(axis=0)
    u,_,vt=np.linalg.svd((p[mask]-a).T@(g[mask]-b));d=np.eye(3);d[-1,-1]=np.linalg.det(u@vt)
    return (p-a)@(u@d@vt)+b
t0=(logs['off1']['lidar_end_time'][0]-zero*1e-9)
metrics={'common_sample_count':len(common),'time_origin_gt_relative_s':t0,'runs':{}}
metrics['max_common_timestamp_spread_s']=float(max(np.ptp([v[k][0] for v in matched.values()]) for k in common))
errors={};full_errors={};window=[]
for name,lookup in matched.items():
    t=np.array([lookup[k][0] for k in common])-t0
    p=np.array([lookup[k][1] for k in common]);g=np.array([lookup[k][2] for k in common])
    full=np.linalg.norm(align(p,g,np.ones(len(t),bool))-g,axis=1)
    prefix=np.linalg.norm(align(p,g,t<60)-g,axis=1)
    errors[name]=prefix;full_errors[name]=full
    metrics['runs'][name]={'original_ate':own[name],'common_ate':float(np.sqrt(np.mean(full**2))),
                           'prefix_alignment_samples':int((t<60).sum()),
                           'mode_counts':{str(int(v)):int((logs[name]['degeneracy_mode']==v).sum()) for v in np.unique(logs[name]['degeneracy_mode'])}}
    for lo,hi in [(0,60),(60,300),(300,320),(320,330),(330,340),(340,350),(350,360),(360,370),(370,390),(390,420)]:
        mask=(t>=lo)&(t<hi)
        d=logs[name];lt=d['lidar_end_time']-zero*1e-9-t0;m=(lt>=lo)&(lt<hi)
        r={'run':name,'start_s':lo,'end_s':hi,'common_samples':int(mask.sum()),
           'prefix_error_median':float(np.median(prefix[mask])) if mask.any() else None,
           'log_rows':int(m.sum()),'effective_median':float(np.median(d['effective_points'][m])),
           'effective_min':int(np.min(d['effective_points'][m])),
           'residual_median':float(np.median(d['residual_mean'][m])),
           'transient_fraction':float(np.mean(d['degeneracy_mode'][m]==1)),
           'map_added':int(d['map_added'][m].sum())}
        for k in ['quality_rejected','invalid_quality_rejected','direction_rejected','novel_rejected','persistent_quota_rejected','total_rejected']:
            r[k]=int(d[k][m].sum())
        r['candidate_rejection_fraction']=r['total_rejected']/max(1,r['total_rejected']+r['map_added'])
        window.append(r)
with (OUT/'windows.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=list(window[0]));w.writeheader();w.writerows(window)
with (OUT/'common_errors.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['relative_s']+[k+'_prefix_error' for k in errors]);w.writerows(zip(t,*errors.values()))
fig,ax=plt.subplots(5,1,figsize=(13,15))
for name in errors:
    # Do not join across intervals without valid ground-truth associations.
    ax[0].scatter(t,full_errors[name],label=name,s=3)
    ax[1].scatter(t,errors[name],label=name,s=3)
for name in ['on1','on2','on3']:
    d=logs[name];lt=d['lidar_end_time']-zero*1e-9-t0
    ax[2].plot(lt,d['effective_points'],label=name)
    # Five-second bins avoid making frame-rate variation look like an insertion change.
    bins=np.arange(300,421,5);centres=(bins[:-1]+bins[1:])/2
    added=[];rejected=[]
    for lo,hi in zip(bins[:-1],bins[1:]):
        m=(lt>=lo)&(lt<hi);a=d['map_added'][m].sum();b=d['total_rejected'][m].sum()
        added.append(a/(hi-lo));rejected.append(b/max(1,a+b))
    ax[3].plot(centres,added,label=name)
    ax[4].plot(centres,rejected,label=name)
ax[0].set(ylabel='Full SE(3) APE (m)',title='SPMS1: common valid timestamps (gaps are not interpolated)')
ax[1].set(ylabel='First-60s aligned error (m)')
ax[2].set(ylabel='Effective points',xlim=(300,420))
ax[3].set(ylabel='Map candidates / second',xlim=(300,420))
ax[4].set(ylabel='Candidate rejection fraction',xlim=(300,420),xlabel='Seconds from first logged scan')
for a in ax:a.grid(alpha=.3);a.legend(ncol=3,fontsize=8)
fig.tight_layout();fig.savefig(OUT/'spms1_repeats_diagnostics.png',dpi=160);plt.close(fig)
(OUT/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
print(json.dumps(metrics,indent=2))
for r in window:
    if r['run']=='on2' and r['start_s']>=300:print(r)
