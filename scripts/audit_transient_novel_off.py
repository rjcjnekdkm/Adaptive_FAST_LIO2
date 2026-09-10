"""Locate SPMS1 quota-off run02/run03 divergence; no algorithm changes."""
import csv
import json
import hashlib
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import evaluate_ntu_viral as ntu

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'experiments/adaptive_map_v1/validation/ntu_spms1/transient_novel_ab/control_off'
parser=argparse.ArgumentParser()
parser.add_argument('--replacement',action='store_true',help='Use explicitly re-evaluated replacement run02, not a quota-off claim')
args=parser.parse_args()
OUT=ROOT/'experiments/adaptive_map_v1/summary/spms1_transient_novel_ab'/('replacement_divergence' if args.replacement else 'divergence')
OUT.mkdir(exist_ok=True)
gt=ROOT/'bag/NTU/spms_01/ground_truth/ground_truth.csv'
gt_t,gt_p=ntu.read_gt(gt)
zero=float(np.genfromtxt(gt,delimiter=',',names=True,max_rows=1)['time'])
data={}; matched={}; tables=[]; metrics={}
for run in ['run02','run03']:
    folder=BASE.parent/'control'/run if args.replacement and run=='run02' else BASE/run
    src=folder/'runtime.csv'
    manifest=json.loads((folder/'evaluation_v1/manifest.json').read_text())
    actual_hash=hashlib.sha256(src.read_bytes()).hexdigest()
    if actual_hash != manifest['source_sha256']:
        raise RuntimeError(f'{src}: source changed since evaluation; do not mix old ATE with new logs')
    d=np.genfromtxt(src,delimiter=',',names=True)
    t,p=ntu.read_estimate(src,zero)
    mt,mp,mg=ntu.interpolate_gt(gt_t,gt_p,t,p)
    matched[run]={float(a):(b,c) for a,b,c in zip(mt,mp,mg)}
    data[run]=(d,t,p)
common=np.array(sorted(set(matched['run02'])&set(matched['run03'])))
origin=data['run02'][1][0]
rel=common-origin
aligned={};errors={}
fig,ax=plt.subplots(6,1,figsize=(13,16))
for run,(d,t,p) in data.items():
    pair=matched[run]
    cp=np.array([pair[a][0] for a in common]);cg=np.array([pair[a][1] for a in common])
    mask=rel<60
    a=cp[mask].mean(axis=0);b=cg[mask].mean(axis=0)
    u,_,vt=np.linalg.svd((cp[mask]-a).T@(cg[mask]-b))
    fix=np.eye(3);fix[-1,-1]=np.linalg.det(u@vt)
    rot=u@fix@vt
    aligned[run]=(p-a)@rot+b
    errors[run]=np.linalg.norm((cp-a)@rot+b-cg,axis=1)
    lt=t-origin;dt=np.r_[np.nan,np.diff(t)]
    pos=np.column_stack([d['pos_'+x] for x in 'xyz'])
    speed=np.r_[np.nan,np.linalg.norm(np.diff(pos,axis=0),axis=1)/np.diff(t)]
    q=np.column_stack([d['quat_'+x] for x in 'xyzw']);q/=np.linalg.norm(q,axis=1)[:,None]
    omega=np.r_[np.nan,2*np.arccos(np.clip(np.abs((q[1:]*q[:-1]).sum(axis=1)),0,1))/np.diff(t)]
    ax[0].scatter(rel,errors[run],s=5,label=run)
    ax[1].plot(lt,d['residual_mean'],label=run)
    ax[2].plot(lt,d['novel_accepted'],label=run)
    ax[3].plot(lt,d['invalid_quality_rejected'],label=run)
    ax[4].plot(lt,speed,label=run)
    ax[5].plot(lt,dt,label=run)
    metrics[run]={'prefix_alignment_samples':int(mask.sum()),'common_samples':len(common),
                 'source_sha256':hashlib.sha256((BASE.parent/'control'/run/'runtime.csv' if args.replacement and run=='run02' else BASE/run/'runtime.csv').read_bytes()).hexdigest()}
    for lo in range(240,401):
        m=(lt>=lo)&(lt<lo+1);g=(rel>=lo)&(rel<lo+1)
        if not m.any(): continue
        tables.append(dict(run=run,start_s=lo,rows=int(m.sum()),gt_common_samples=int(g.sum()),
            error_median=float(np.median(errors[run][g])) if g.any() else None,
            residual_median=float(np.median(d['residual_mean'][m])),
            transient_frames=int((d['degeneracy_mode'][m]==1).sum()),
            effective_min=int(d['effective_points'][m].min()),
            novel_accepted=int(d['novel_accepted'][m].sum()),
            invalid_rejected=int(d['invalid_quality_rejected'][m].sum()),
            added=int(d['map_added'][m].sum()),
            speed_max=float(np.nanmax(speed[m])),angular_speed_max=float(np.nanmax(omega[m])),
            csv_gap_max=float(np.nanmax(dt[m]))))
for a,y in zip(ax,['First-60s aligned error (m)','Mean residual (m)','Novel accepted / scan',
                   'Invalid quality rejected / scan','Estimated speed (m/s)','CSV interval (s)']):
    a.set(xlim=(240,400),ylabel=y);a.grid(alpha=.3);a.legend()
ax[-1].set_xlabel('Seconds from first logged scan')
fig.tight_layout();fig.savefig(OUT/'diagnostics.png',dpi=150);plt.close(fig)
with (OUT/'seconds.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(tables[0]));w.writeheader();w.writerows(tables)
with (OUT/'common_errors.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['relative_s','run02_error','run03_error']);w.writerows(zip(rel,errors['run02'],errors['run03']))
(OUT/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
print(metrics)
for row in tables:
    if 330<=row['start_s']<360: print(row)
