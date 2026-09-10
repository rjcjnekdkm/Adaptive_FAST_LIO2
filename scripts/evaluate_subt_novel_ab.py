"""Score available SubT quota A/B records using the established evo protocol."""
import csv
import json
import statistics
import numpy as np
import evaluate_baseline_round as e

base=e.BASE/'validation/subt_hawkins/transient_novel_ab'
out=e.BASE/'summary/subt_transient_novel_ab'
out.mkdir(exist_ok=True)
records=[]
for group in ['control','control_off']:
    for i in range(1,4):
        folder=base/group/f'run{i:02d}';src=folder/'runtime.csv'
        if not src.exists():
            print('MISSING',src,flush=True);continue
        d=np.genfromtxt(src,delimiter=',',names=True)
        assert np.all(d['adaptive_map']==1)
        assert np.all(np.diff(d['lidar_end_time'])>0)
        assert all(np.isfinite(d[k]).all() for k in ['pos_x','pos_y','pos_z','quat_x','quat_y','quat_z','quat_w'])
        ev=folder/'evaluation_v1'
        if not ev.exists():
            e.evaluate('subt_hawkins','subt',e.ROOT/'bag/SubT_MRS/ground_truth_path.csv',
                       method=group,run_name=folder.name,folder_override=folder)
        m=json.loads((ev/'manifest.json').read_text())
        assert m['source_sha256']==e.digest(src)
        t=d['degeneracy_mode']==1
        records.append(dict(group=group,run=folder.name,ate_rmse_m=m['ate_rmse_m'],rows=len(d),
            duration_s=m['duration_s'],max_gap_s=m['max_gap_s'],first_stamp=float(d['lidar_end_time'][0]),
            last_stamp=float(d['lidar_end_time'][-1]),transient_frames=int(t.sum()),
            persistent_frames=int((d['degeneracy_mode']==2).sum()),
            transient_novel_rejected=int(d['novel_rejected'][t].sum()),
            transient_max_novel_accepted=int(d['novel_accepted'][t].max()) if t.any() else 0,
            source=m['source'],source_sha256=m['source_sha256']))
        print(records[-1],flush=True)
summary=[]
for group in ['control','control_off']:
    rs=[r for r in records if r['group']==group];a=[r['ate_rmse_m'] for r in rs]
    if not a:continue
    summary.append(dict(group=group,n=len(a),mean_ate_m=statistics.mean(a),
        sample_std_m=statistics.stdev(a) if len(a)>1 else None,
        min_ate_m=min(a),max_ate_m=max(a),unique_hashes=len({r['source_sha256'] for r in rs})))
for name,data in [('ate',records),('statistics',summary)]:
    if not data:continue
    with (out/f'{name}.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
print(summary,flush=True)
