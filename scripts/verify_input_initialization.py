"""Offline NTU input counterfactual, not a replay or historical state reconstruction."""
import csv
import json
import sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'experiments/adaptive_map_v1/summary/baseline_fairness_audit'


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    results = {}
    for sequence in (1, 2, 3):
        db = ROOT / f'bag/NTU/spms_0{sequence}/spms_0{sequence}_ros2/spms_0{sequence}_ros2.db3'
        con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        topics = {n: i for i, n in con.execute('select id,name from topics')}
        lid, imu = topics['/os1_cloud_node1/points'], topics['/imu/imu']
        first = con.execute('select min(timestamp) from messages where topic_id=?', (lid,)).fetchone()[0]
        scans, inertial = [], []
        for topic, data in con.execute('select topic_id,data from messages where topic_id in (?,?) and timestamp<=? order by timestamp,id', (lid, imu, first + int(10e9))):
            if topic == imu:
                m = deserialize_message(data, Imu)
                inertial.append((stamp(m), [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z],
                                 [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]))
                continue
            m = deserialize_message(data, PointCloud2)
            fields = {f.name: f for f in m.fields}
            assert fields['t'].datatype == 6 and all(fields[k].datatype == 7 for k in 'xyz')
            def field(k, dtype):
                return np.ndarray((m.height, m.width), dtype=('>' if m.is_bigendian else '<') + dtype,
                                  buffer=m.data, offset=fields[k].offset, strides=(m.row_step, m.point_step)).ravel()
            xyz = np.column_stack([field(k, 'f4') for k in 'xyz']).astype(float)
            times = field('t', 'u4')
            picked = np.arange(len(times)) % 3 == 0
            ranges = (xyz * xyz).sum(axis=1)
            baseline = picked & (ranges >= 1.0)
            ours = picked & (ranges > 1.0) & np.isfinite(xyz).all(axis=1)
            assert baseline.any() and ours.any()
            # Both implementations store relative milliseconds in float curvature.
            bt = np.float32(times[baseline].astype(float) * 1e-6).astype(float) / 1000
            ot = np.float32(times[ours].astype(float) * 1e-6).astype(float) / 1000
            scans.append(dict(begin=stamp(m), baseline_last=float(bt[-1]), ours_max=float(ot.max()),
                              mask_difference=int(np.count_nonzero(baseline != ours)),
                              max_minus_last_s=float(bt.max() - bt[-1])))
        con.close()
        ends = {}
        for name, key, initial in [('baseline', 'baseline_last', 0.), ('ours', 'ours_max', .1)]:
            mean, n, values = initial, 0, []
            for s in scans:
                duration = s[key]
                if duration < .5 * mean or (name == 'ours' and duration <= 0):
                    duration = mean
                else:
                    n += 1
                    mean += (duration - mean) / n
                values.append(s['begin'] + duration)
            ends[name] = np.array(values)
        ts = np.array([r[0] for r in inertial])
        acc = np.array([r[1] for r in inertial]); gyr = np.array([r[2] for r in inertial])
        init = {}
        for name in ['baseline', 'ours']:
            start = int(np.searchsorted(ts, ends[name][0], side='right')) if name == 'baseline' else 0
            for i in range(1 if name == 'baseline' else 0, len(scans)):
                stop = int(np.searchsorted(ts, ends[name][i], side='right'))
                count = stop - start
                if count >= (10 if name == 'baseline' else 200):
                    init[name] = dict(scan_index=i, samples=count, end_relative_s=float(ends[name][i]-scans[0]['begin']),
                                      mean_acc=acc[start:stop].mean(0).tolist(), mean_gyro=gyr[start:stop].mean(0).tolist())
                    break
        a, b = np.array(init['baseline']['mean_acc']), np.array(init['ours']['mean_acc'])
        logged = {}
        for method in ['fastlio2', 'ours_no_adaptive']:
            src = ROOT / f'experiments/adaptive_map_v1/validation/ntu_spms{sequence}/{method}/run01/runtime.csv'
            with src.open() as f:
                data = list(csv.DictReader(f))
            key = 'stamp' if method == 'fastlio2' else 'lidar_end_time'
            observed = np.array([float(d[key]) for d in data])
            observed = observed[(observed >= scans[0]['begin']) & (observed <= min(ends['baseline'][-1], ends['ours'][-1]))]
            logged[method] = dict(samples=len(observed),
                                  first_output_relative_s=float(observed[0]-scans[0]['begin']) if len(observed) else None)
            for rule, predicted in ends.items():
                distances = np.min(np.abs(observed[:, None] - predicted[None, :]), axis=1) if len(observed) else np.array([])
                logged[method][rule + '_rule_matches_1us'] = int((distances <= 1e-6).sum())
                logged[method][rule + '_median_distance_s'] = float(np.median(distances)) if len(distances) else None
        r = dict(bag=str(db.relative_to(ROOT)), scans=len(scans), imu_samples=len(inertial),
                 historical_output_timestamps=logged,
                 candidate_mask_difference=sum(s['mask_difference'] for s in scans),
                 scans_max_not_last=sum(s['max_minus_last_s'] > 1e-9 for s in scans),
                 endpoint_difference_max_s=float(np.max(np.abs(ends['ours'] - ends['baseline']))),
                 ideal_fifo_initialization=init,
                 mean_acc_direction_difference_deg=float(np.degrees(np.arccos(np.clip(a.dot(b)/np.linalg.norm(a)/np.linalg.norm(b), -1, 1)))))
        results[f'spms{sequence}'] = r
    (OUT / 'ntu_input_initialization.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
