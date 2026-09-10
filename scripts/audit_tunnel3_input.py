"""Read-only bag audit; writes only derived diagnostics beside current evaluation."""
import csv
import json
import math
import sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'experiments/adaptive_map_v1/validation/geode_tunnel3/ours_no_adaptive/run01'
rows = list(csv.DictReader((RUN / 'runtime.csv').open()))
t0 = float(rows[0]['lidar_end_time'])
db = next((ROOT / 'bag/GEODE/Tunneling_tunnel_gamma/Tunneling_tunnel3_gamma_ros2').glob('*.db3'))
con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
topics = {name: ident for ident, name in con.execute('select id,name from topics')}

def messages(name, cls):
    for record, data in con.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
                                   (topics[name], int((t0+125)*1e9), int((t0+145)*1e9))):
        msg = deserialize_message(data, cls)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yield stamp, msg

scans = []
for stamp, msg in messages('/livox/lidar', CustomMsg):
    pts = msg.points
    xyz = np.array([(p.x,p.y,p.z) for p in pts], dtype=np.float64)
    ranges = np.linalg.norm(xyz, axis=1)
    valid = np.array([p.line < 6 and (p.tag & 0x30) in (0,16) for p in pts])
    valid[0] = False
    picked = valid & (np.cumsum(valid) % 3 == 0)
    duplicate = np.zeros(len(pts), dtype=bool)
    duplicate[1:] = np.all(np.abs(xyz[1:] - xyz[:-1]) < 1e-7, axis=1)
    kept = picked & np.isfinite(xyz).all(axis=1) & (ranges > 4) & ~duplicate
    # This is a pre-deskew voxel count proxy, not the runtime PCL result.
    voxel_proxy = len(np.unique(np.floor(xyz[kept]/.5), axis=0))
    scans.append(dict(relative_s=stamp-t0,stamp=stamp,raw_points=len(pts),declared_points=msg.point_num,
                      valid_line_tag=int(valid.sum()),picked=int(picked.sum()),
                      picked_near4=int((picked & (ranges<=4)).sum()),
                      raw_above4=int((ranges>4).sum()),raw_nonzero=int((ranges>0).sum()),
                      kept_above1=int((picked & (ranges>1) & ~duplicate).sum()),
                      kept_above2=int((picked & (ranges>2) & ~duplicate).sum()),
                      kept=int(kept.sum()),duplicate_removed=int((picked & (ranges>4) & duplicate).sum()),
                      voxel_proxy=voxel_proxy,
                      offset_span_s=(max(p.offset_time for p in pts)-min(p.offset_time for p in pts))*1e-9))
imu = [(s,m) for s,m in messages('/imu/data', Imu)]
ts = np.array([s for s,m in imu]); acc=np.array([(m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z) for s,m in imu]); gyro=np.array([(m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z) for s,m in imu])
summary = dict(time_origin=t0,window_relative_s=[125,145],scans=len(scans),
               lidar_max_header_gap_s=float(np.diff([s['stamp'] for s in scans]).max()),
               imu_samples=len(imu),imu_max_header_gap_s=float(np.diff(ts).max()),
               imu_nonpositive_gaps=int((np.diff(ts)<=0).sum()),
               imu_acc_finite=bool(np.isfinite(acc).all()),imu_gyro_finite=bool(np.isfinite(gyro).all()),
               acc_norm_range=[float(x) for x in [np.linalg.norm(acc,axis=1).min(),np.linalg.norm(acc,axis=1).max()]],
               gyro_norm_max_rad_s=float(np.linalg.norm(gyro,axis=1).max()))
for lo,hi in [(125,130),(131,136),(140,145)]:
    selected=[s for s in scans if lo<=s['relative_s']<hi]
    summary[f'{lo}_{hi}']={k:dict(min=min(s[k] for s in selected),median=float(np.median([s[k] for s in selected])),max=max(s[k] for s in selected)) for k in ['raw_points','raw_nonzero','valid_line_tag','picked','picked_near4','raw_above4','kept','kept_above1','kept_above2','duplicate_removed','voxel_proxy']}
out=RUN/'input_audit';out.mkdir(exist_ok=True)
with (out/'scan_counts.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=list(scans[0]));w.writeheader();w.writerows(scans)
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
