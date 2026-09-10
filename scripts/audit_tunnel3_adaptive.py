"""Compare current Tunnel3 frontend runs; produce diagnostics without modifying them."""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evo.core.sync import matching_time_indices

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'experiments/adaptive_map_v1/validation/geode_tunnel3'
OUT=ROOT/'experiments/adaptive_map_v1/summary/tunnel3_adaptive_audit'
OUT.mkdir(exist_ok=True)
methods={'off':'ours_no_adaptive_blind2','on':'ours_adaptive'}
data={};summ={};pairs={}
named=['quality_rejected','invalid_quality_rejected','direction_rejected','persistent_quota_rejected','novel_rejected']
for name,method in methods.items():
    run=BASE/method/'run01'
    rows=list(csv.DictReader((run/'runtime.csv').open()))
    d={k:np.array([float(r[k]) for r in rows]) for k in rows[0]}
    # All counted rejection branches are exclusive. Remaining rejected candidates
    # map to the two uncounted range checks in current allow_map_insert_point.
    d['unclassified_range_reject']=d['total_rejected']-sum(d[k] for k in named)
    assert np.all(d['unclassified_range_reject']>=0)
    assert np.all(d['map_added']+d['total_rejected']+d['voxel_rejected']==d['downsampled_points'])
    data[name]=d
    est=np.loadtxt(run/'evaluation_v1'/f'{method}.txt')
    gt=np.loadtxt(run/'evaluation_v1/ground_truth.txt')
    i,j=matching_time_indices(est[:,0],gt[:,0],max_diff=.1,offset_2=.1)
    best={}
    for a,b in zip(i,j):
        distance=abs(est[a,0]-(gt[b,0]+.1))
        if b not in best or distance<best[b][0]:
            best[b]=(distance,est[a,1:4])
    pairs[name]={int(b):value[1] for b,value in best.items()}
    summ[name]={'rows':len(rows),'start':d['lidar_end_time'][0],'end':d['lidar_end_time'][-1],
                'modes':{str(int(k)):int((d['degeneracy_mode']==k).sum()) for k in np.unique(d['degeneracy_mode'])}}
t0=data['off']['lidar_end_time'][0]
common=sorted(set(pairs['off']) & set(pairs['on']))
truth=gt[common,1:4]; ts=gt[common,0]+.1-t0
errors={}
for name in methods:
    pos=np.array([pairs[name][i] for i in common])
    a=pos-pos.mean(axis=0);b=truth-truth.mean(axis=0)
    u,s,v=np.linalg.svd(a.T@b); fix=np.eye(3);fix[-1,-1]=np.linalg.det(u@v)
    rot=u@fix@v;aligned=a@rot+truth.mean(axis=0)
    errors[name]=np.linalg.norm(aligned-truth,axis=1)
    summ[name]['common_gt_rmse']=float(np.sqrt(np.mean(errors[name]**2)))
summ['common_gt_count']=len(common)
# Compare poses at the same raw scan begin time; both runs share initial body frame.
a=data['off'];b=data['on']
i,j=matching_time_indices(a['lidar_begin_time'],b['lidar_begin_time'],max_diff=1e-5)
pos=lambda d:np.column_stack([d[k] for k in ['pos_x','pos_y','pos_z']])
delta=np.linalg.norm(pos(a)[i]-pos(b)[j],axis=1);td=a['lidar_end_time'][i]-t0
summ['raw_pose_first_difference_s']={str(k):float(td[np.flatnonzero(delta>k)[0]]) if np.any(delta>k) else None for k in [.1,.5,1,2,5]}
window_rows=[]
for name,d in data.items():
    t=d['lidar_end_time']-t0
    for lo,hi in [(0,120),(120,130),(130,132),(132,134),(134,136),(136,140),(140,160),(160,253)]:
        mask=(t>=lo)&(t<hi)
        r={'run':name,'start_s':lo,'end_s':hi,'rows':int(mask.sum())}
        for key in ['downsampled_points','effective_points','residual_mean','map_size']:
            r[key+'_median']=float(np.median(d[key][mask])) if mask.any() else None
        for key in ['map_added','total_rejected','unclassified_range_reject']+named:
            r[key+'_sum']=int(d[key][mask].sum())
        r['transient_rows']=int((d['degeneracy_mode'][mask]==1).sum())
        window_rows.append(r)
        summ.setdefault('windows',[]).append(r)
    with (OUT/f'{name}_timeline.csv').open('w') as f:
        keys=['lidar_begin_time','lidar_end_time','downsampled_points','effective_points','residual_mean','degeneracy_mode','map_added','total_rejected','unclassified_range_reject']+named
        w=csv.writer(f);w.writerow(['relative_s']+keys)
        w.writerows(zip(t,*[d[k] for k in keys]))
with (OUT/'window_statistics.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=list(window_rows[0]));w.writeheader();w.writerows(window_rows)
fig,axes=plt.subplots(6,1,figsize=(13,17))
def gap_values(t,y):
    out=np.array(y,dtype=float).copy()
    out[np.flatnonzero(np.diff(t)>.2)+1]=np.nan
    return out
for name in methods:
    axes[0].plot(ts,errors[name],label=name)
axes[0].set(ylabel='Common-GT APE (m)',title='Tunnel3: blind=2m; full-trajectory SE(3) alignment')
axes[1].plot(td,delta,color='purple',label='Raw pose difference at same scan')
axes[1].set(ylabel='Position difference (m)')
for name,d in data.items():
    t=d['lidar_end_time']-t0
    axes[2].plot(t,gap_values(t,d['effective_points']),label=name)
    axes[3].plot(t,gap_values(t,d['map_added']),label=name)
axes[2].set(ylabel='Effective points',xlim=(120,145))
axes[3].set(ylabel='Map insertion candidates',xlim=(120,145))
d=data['on'];t=d['lidar_end_time']-t0
for key in ['unclassified_range_reject','quality_rejected','invalid_quality_rejected','direction_rejected','novel_rejected']:
    axes[4].plot(t,gap_values(t,d[key]),label=key)
axes[4].set(ylabel='Rejected candidates (on)',xlim=(120,145))
axes[5].step(t,d['degeneracy_mode'],where='post',label='on mode')
axes[5].set(ylabel='0 Normal / 1 Transient',xlim=(120,145),xlabel='Seconds from off first CSV end stamp')
for ax in axes:
    ax.axvspan(132.4,134.0,color='grey',alpha=.15)
    ax.grid(alpha=.3);ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(OUT/'tunnel3_on_off_diagnostics.png',dpi=170);plt.close(fig)
(OUT/'metrics.json').write_text(json.dumps(summ,indent=2)+'\n')
print(json.dumps({k:v for k,v in summ.items() if k!='windows'},indent=2))
for r in window_rows:
    if 120<=r['start_s']<140: print(r)
