"""Score six explicitly supplied tunnel repeat CSVs, preserving every run."""
import argparse
import csv
import json
import math
import statistics
import evaluate_baseline_round as e

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--tunnel',type=int,choices=range(1,6),default=2)
    tunnel=parser.parse_args().tunnel
    gt=e.ROOT/'bag/GEODE/Tunneling_tunnel_gamma'
    records=[]
    for method,flag in [('ours_no_adaptive_blind2','0'),('ours_adaptive_blind2','1')]:
        for i in range(1,4):
            folder=e.BASE/f'validation/geode_tunnel{tunnel}'/method/'run02'
            source=folder/f'runtime{i}.csv'
            rows=list(csv.DictReader(source.open()))
            assert {r['adaptive_map'] for r in rows}=={flag},source
            assert all(math.isfinite(float(r[k])) for r in rows for k in ['pos_x','pos_y','pos_z','quat_x','quat_y','quat_z','quat_w']),source
            out=folder/f'evaluation_runtime{i}'
            if not out.exists():
                e.evaluate(f'geode_tunnel{tunnel}','geode',gt/f'groudtruth/Tunneling_tunnel{tunnel}.txt',
                           gt/'gamma2GT_leica.py',gt/'rmse.py',method=method,run_name='run02',
                           source_filename=source.name,output_name=out.name)
            m=json.loads((out/'manifest.json').read_text())
            assert m['source_sha256']==e.digest(source),'stale evaluation: '+str(out)
            record=dict(method=method,repeat=i,ate_rmse_m=m['ate_rmse_m'],rows=len(rows),
                        duration_s=m['duration_s'],max_csv_gap_s=m['max_gap_s'],
                        first_stamp=float(rows[0]['lidar_end_time']),last_stamp=float(rows[-1]['lidar_end_time']),
                        final_z_m=float(rows[-1]['pos_z']),adaptive_map=flag,
                        source=str(source.relative_to(e.ROOT)),source_sha256=m['source_sha256'],
                        evaluation=str(out.relative_to(e.ROOT)))
            records.append(record);print(record,flush=True)
    summary=[]
    for method in dict.fromkeys(r['method'] for r in records):
        values=[r['ate_rmse_m'] for r in records if r['method']==method]
        unique_hashes=len({r['source_sha256'] for r in records if r['method']==method})
        summary.append(dict(method=method,n=len(values),mean_ate_m=statistics.mean(values),
                            sample_std_ate_m=statistics.stdev(values),min_ate_m=min(values),max_ate_m=max(values),
                            unique_csv_hashes=unique_hashes,
                            independence_note=('identical CSV contents; ' if unique_hashes==1 else 'CSV contents differ; ')+
                            'independent launch provenance not captured'))
    base=e.BASE/'summary'
    for name,data in [(f'tunnel{tunnel}_repeats_ate.csv',records),(f'tunnel{tunnel}_repeats_statistics.csv',summary)]:
        with (base/name).open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
    print('STATISTICS',summary,flush=True)

if __name__=='__main__':
    main()
