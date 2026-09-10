"""Evaluate NTU SPMS novel-quota A/B without altering source records."""
import argparse
import csv
import json
import statistics
import numpy as np
import evaluate_baseline_round as e

parser=argparse.ArgumentParser()
parser.add_argument('--sequence',type=int,choices=[1,2,3],default=1)
n=parser.parse_args().sequence
base = e.BASE/f'validation/ntu_spms{n}/transient_novel_ab'
out = e.BASE/f'summary/spms{n}_transient_novel_ab'
out.mkdir(exist_ok=True)
records = []
windows = []
previous_records=list(csv.DictReader((out/'ate.csv').open(encoding='utf-8-sig'))) if (out/'ate.csv').exists() else []
previous_windows=list(csv.DictReader((out/'windows.csv').open(encoding='utf-8-sig'))) if (out/'windows.csv').exists() else []
for group in ['control', 'control_off']:
    for i in range(1, 4):
        folder = base/group/f'run{i:02d}'
        source = folder/'runtime.csv'
        if not source.exists():
            if not (folder/'evaluation_v1/manifest.json').exists():
                print(f'MISSING: {source}',flush=True)
                continue
            # User-overwritten quota-off run02: retain its historical score,
            # never substitute a different group's CSV or fabricate raw data.
            m=json.loads((folder/'evaluation_v1/manifest.json').read_text())
            old=next(r for r in previous_records if r['group']==group and r['run']==folder.name)
            assert old['source_sha256']==m['source_sha256']
            old['ate_rmse_m']=float(old['ate_rmse_m'])
            old['raw_status']='missing_historical_evaluation_only'
            records.append(old)
            windows.extend(r for r in previous_windows if r['group']==group and r['run']==folder.name)
            continue
        d = np.genfromtxt(source, delimiter=',', names=True)
        assert np.all(d['adaptive_map'] == 1), source
        assert all(np.isfinite(d[k]).all() for k in ['lidar_end_time','pos_x','pos_y','pos_z','quat_x','quat_y','quat_z','quat_w']), source
        assert np.all(np.diff(d['lidar_end_time']) > 0), source
        ev = folder/'evaluation_v1'
        if not ev.exists():
            e.evaluate(f'ntu_spms{n}','ntu',e.ROOT/f'bag/NTU/spms_0{n}/ground_truth/ground_truth.csv',
                       method=f'transient_novel_ab/{group}',run_name=folder.name)
        m = json.loads((ev/'manifest.json').read_text())
        assert m['source_sha256'] == e.digest(source)
        transient = d['degeneracy_mode'] == 1
        records.append(dict(group=group,run=folder.name,ate_rmse_m=m['ate_rmse_m'],
            rows=len(d),associated_samples=m['associated_samples'],duration_s=m['duration_s'],
            first_stamp=float(d['lidar_end_time'][0]),last_stamp=float(d['lidar_end_time'][-1]),
            max_gap_s=m['max_gap_s'],transient_frames=int(transient.sum()),
            persistent_frames=int((d['degeneracy_mode']==2).sum()),
            transient_novel_rejected=int(d['novel_rejected'][transient].sum()),
            transient_max_novel_accepted=int(d['novel_accepted'][transient].max()) if transient.any() else 0,
            source_sha256=m['source_sha256'],source=m['source'],raw_status='available'))
        t = d['lidar_end_time']-d['lidar_end_time'][0]
        intervals=([(0,340),(340,350),(350,360),(360,370),(370,390),(390,420)] if n==1
                   else [(lo,lo+30) for lo in range(0,int(t[-1])+1,30)])
        for lo,hi in intervals:
            mask=(t>=lo)&(t<hi)
            if not mask.any(): continue
            added=float(d['map_added'][mask].sum()); rejected=float(d['total_rejected'][mask].sum())
            windows.append(dict(group=group,run=folder.name,start_s=lo,end_s=hi,rows=int(mask.sum()),
                residual_median=float(np.median(d['residual_mean'][mask])),
                effective_min=int(d['effective_points'][mask].min()),
                transient_fraction=float(transient[mask].mean()),map_added=added,
                rejection_fraction=rejected/max(1,added+rejected),
                novel_rejected=int(d['novel_rejected'][mask].sum())))
        print(records[-1],flush=True)
summary=[]
for group in ['control','control_off']:
    rs=[r for r in records if r['group']==group]; values=[r['ate_rmse_m'] for r in rs]
    if not values: continue
    summary.append(dict(group=group,n=len(rs),mean_ate_m=statistics.mean(values),
        sample_std_m=statistics.stdev(values) if len(values)>1 else None,min_ate_m=min(values),max_ate_m=max(values),
        unique_hashes=len({r['source_sha256'] for r in rs})))
for name,data in [('ate',records),('windows',windows),('statistics',summary)]:
    with (out/f'{name}.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
print(summary,flush=True)
