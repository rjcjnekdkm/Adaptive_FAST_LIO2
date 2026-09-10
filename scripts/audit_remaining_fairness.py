"""Read-only remaining fairness checks using current code and SPMS3 records."""
import csv
import hashlib
import json
import sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'experiments/adaptive_map_v1/summary/baseline_fairness_audit'


def main():
    deps = []
    a = ROOT/'src/FAST_LIO/include'; b = ROOT/'src/adaptive_fast_lio2/include'
    for sub in ['ikd-Tree', 'IKFoM_toolkit']:
        for path in sorted((a/sub).rglob('*')):
            if not path.is_file():
                continue
            rel=path.relative_to(a); other=b/rel
            deps.append(dict(path=str(rel),baseline_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                             ours_sha256=hashlib.sha256(other.read_bytes()).hexdigest() if other.exists() else None,
                             identical=other.exists() and path.read_bytes()==other.read_bytes()))
    (OUT/'dependency_comparison.json').write_text(json.dumps(deps,indent=2)+'\n')
    db=ROOT/'bag/NTU/spms_03/spms_03_ros2/spms_03_ros2.db3'
    con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
    con.row_factory=sqlite3.Row
    topic=dict(con.execute('select * from topics where name=?',('/imu/imu',)).fetchone())
    stamps=[]
    for row in con.execute('select data from messages where topic_id=? order by timestamp,id',(topic['id'],)):
        m=deserialize_message(row['data'],Imu)
        stamps.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
    con.close()
    ts=np.array(stamps); gaps=np.diff(ts)
    sorted_ts=np.sort(ts)
    records=[]
    for group in ['max','last']:
        for run in ['run01','run02','run03']:
            folder=ROOT/f'experiments/adaptive_map_v1/validation/ntu_spms3/scan_end_ab/{group}/{run}'
            d=np.genfromtxt(folder/'runtime.csv',delimiter=',',names=True)
            log=(folder/'console.log').read_text(errors='replace')
            valid=d['sync_imu_samples']>0
            lo=d['sync_imu_first_time'][valid]; hi=d['sync_imu_last_time'][valid]
            expected=np.searchsorted(sorted_ts,hi+1e-6,side='right')-np.searchsorted(sorted_ts,lo-1e-6,side='left')
            diff=expected-d['sync_imu_samples'][valid]
            records.append(dict(group=group,run=run,rows=len(d),min_logged_map_size=int(d['map_size'].min()),
                local_map_move_lines=log.count('[LocalMap] moved.'),
                loopback_lines=[s for s in log.splitlines() if 'loop back' in s],
                map_init_lines=[s for s in log.splitlines() if '[MapInit]' in s],
                empty_imu_rows=int((~valid).sum()),
                rows_raw_imu_count_matches=int((diff==0).sum()),
                rows_raw_imu_count_differs=int((diff!=0).sum()),max_abs_imu_count_difference=int(np.abs(diff).max()),
                max_end_extrapolation_s=float((d['lidar_end_time'][valid]-hi).max()),
                max_csv_interval_s=float(np.diff(d['lidar_end_time']).max())))
    result=dict(dependency_files=len(deps),identical_dependency_files=sum(r['identical'] for r in deps),
        imu_topic_metadata=topic,raw_imu_samples=len(ts),raw_imu_nonpositive_intervals=int((gaps<=0).sum()),
        raw_imu_gaps_over_200ms=int((gaps>.2).sum()),raw_imu_max_gap_s=float(gaps.max()),runs=records)
    (OUT/'remaining_runtime_checks.json').write_text(json.dumps(result,indent=2)+'\n')
    # Synthetic branch demonstration; not a measured frequency in any experiment.
    cases=[]
    for n in range(1,6):
        # Query lies in voxel [0,.5]^3; nearest point is exactly its center.
        query=np.array([.4,.4,.4]); center=np.array([.25,.25,.25])
        neighbors=[center]+[np.array([10.+j,10.,10.]) for j in range(n-1)]
        distance=lambda p: float(np.sum((p-center)**2))
        baseline_reject=n>=5 and any(distance(p)<distance(query) for p in neighbors[:5])
        ours_reject=any(distance(p)<distance(query) for p in neighbors[:5])
        cases.append(dict(neighbors=n,baseline_accept=not baseline_reject,ours_accept=not ours_reject))
    (OUT/'partial_neighbor_counterexample.json').write_text(json.dumps(cases,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
