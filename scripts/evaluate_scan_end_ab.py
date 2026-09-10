"""Score SPMS3 scan-end ablation, audit provenance, compare common physical times."""
import csv
import json
import statistics
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import evaluate_baseline_round as e
import evaluate_ntu_viral as ntu

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'experiments/adaptive_map_v1/validation/ntu_spms3/scan_end_ab'
OUT = ROOT / 'experiments/adaptive_map_v1/summary/spms3_scan_end_ab'
GT = ROOT / 'bag/NTU/spms_03/ground_truth/ground_truth.csv'


def write(name, rows):
    with (OUT / name).open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)


def align(p, g):
    a, b = p.mean(0), g.mean(0)
    u, _, vt = np.linalg.svd((p-a).T @ (g-b))
    fix = np.eye(3); fix[-1,-1] = np.linalg.det(u @ vt)
    return (p-a) @ u @ fix @ vt + b


def main():
    OUT.mkdir(exist_ok=True)
    records, arrays, provenance = [], {}, []
    zero = float(np.genfromtxt(GT, delimiter=',', names=True, max_rows=1)['time'])
    for group in ['max', 'last']:
        for run in ['run01', 'run02', 'run03']:
            folder = BASE / group / run; src = folder / 'runtime.csv'
            d = np.genfromtxt(src, delimiter=',', names=True)
            assert len(d) > 10 and np.all(np.diff(d['lidar_end_time']) > 0)
            for key in ['lidar_end_time', 'pos_x','pos_y','pos_z','quat_x','quat_y','quat_z','quat_w']:
                assert np.isfinite(d[key]).all(), (src,key)
            assert np.all(d['adaptive_map'] == 0) and np.all(d['window_ready'] == 0)
            assert np.all(d['scan_end_use_last_point'] == (group == 'last'))
            chosen = d['scan_last_offset_s'] if group == 'last' else d['scan_max_offset_s']
            no_fallback = d['scan_end_fallback'] == 0
            delta = np.abs(d['lidar_end_time'] - d['lidar_begin_time'] - chosen)
            assert delta[no_fallback].max() < 1e-6
            launch = json.loads((folder/'manifest.json').read_text())
            log = (folder/'console.log').read_text(errors='replace')
            assert 'adaptive_window=0' in log and 'adaptive_map=0' in log
            assert e.digest(folder/'config.yaml') == launch['config_sha256']
            provenance.append(dict(group=group,run=run,binary_sha256=launch['binary_sha256'],
                config_sha256=launch['config_sha256'],bag=launch['bag'],
                play_command=(folder/'play_command.txt').read_text().strip(),
                launch_command=launch['launch_command'],
                source_hashes={k:v for k,v in launch['source_hashes'].items() if '__pycache__' not in k},
                init_lines=[s for s in log.splitlines() if '[IMU Init] finished' in s],
                exit_code=(folder/'exit_code.txt').read_text().strip() if (folder/'exit_code.txt').exists() else 'missing'))
            ev = folder/'evaluation_v1'
            if not ev.exists():
                e.evaluate('ntu_spms3','ntu',GT,method=f'scan_end_ab/{group}',run_name=run)
            m = json.loads((ev/'manifest.json').read_text())
            assert e.digest(src) == m['source_sha256']
            r = dict(group=group,run=run,ate_rmse_m=m['ate_rmse_m'],rows=len(d),
                     associated_samples=m['associated_samples'],first_stamp=float(d['lidar_end_time'][0]),
                     last_stamp=float(d['lidar_end_time'][-1]),max_gap_s=float(np.diff(d['lidar_end_time']).max()),
                     fallback_frames=int((~no_fallback).sum()),
                     max_minus_last_median_ms=float(np.median(d['scan_max_offset_s']-d['scan_last_offset_s'])*1000),
                     imu_boundary_violations=int((d['sync_imu_last_time']>d['lidar_end_time']+1e-6).sum()),
                     source=m['source'],source_sha256=m['source_sha256'])
            records.append(r)
            arrays[group+'/'+run] = ntu.read_estimate(src,zero)
            print(r,flush=True)
    (OUT/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    # Common physical-time grid, no interpolation over estimate gaps >0.2s.
    start=max(t[0] for t,p in arrays.values()); end=min(t[-1] for t,p in arrays.values())
    grid=np.arange(np.ceil(start*10)/10,end,.1)
    gt_t,gt_p=ntu.read_gt(GT)
    grid,_,g=ntu.interpolate_gt(gt_t,gt_p,grid,np.zeros((len(grid),3)))
    good=np.ones(len(grid),dtype=bool); points={}
    for name,(t,p) in arrays.items():
        i=np.searchsorted(t,grid,side='right')-1
        valid=(i>=0)&(i<len(t)-1); i=np.clip(i,0,len(t)-2)
        good &= valid & (t[i+1]-t[i]<=.2)
        points[name]=p[i]+((grid-t[i])/(t[i+1]-t[i]))[:,None]*(p[i+1]-p[i])
    grid=grid[good]; g=g[good]
    assert len(grid)>100
    fig,ax=plt.subplots(figsize=(12,5)); common=[]
    for r in records:
        name=r['group']+'/'+r['run']; p=points[name][good]
        error=np.linalg.norm(align(p,g)-g,axis=1)
        r['common_grid_samples']=len(grid); r['common_grid_rmse_m']=float(np.sqrt(np.mean(error**2)))
        ax.scatter(grid-start,error,s=3,label=name)
        common.extend(dict(run=name,relative_s=float(t-start),error_m=float(v)) for t,v in zip(grid,error))
    ax.legend(); ax.set(xlabel='Seconds from common start',ylabel='Common-time SE(3) aligned position error (m)'); ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(OUT/'common_errors.png',dpi=150);plt.close(fig)
    write('ate.csv',records);write('common_errors.csv',common)
    stats=[]
    for group in ['max','last']:
        rs=[r for r in records if r['group']==group]
        for metric in ['ate_rmse_m','common_grid_rmse_m']:
            values=[r[metric] for r in rs]
            stats.append(dict(group=group,metric=metric,n=len(rs),mean=statistics.mean(values),sample_std=statistics.stdev(values)))
    write('statistics.csv',stats)
    print(stats,flush=True)
    print('binary hashes',len({p['binary_sha256'] for p in provenance}),'config hashes',len({p['config_sha256'] for p in provenance}),flush=True)


if __name__ == '__main__':
    main()
