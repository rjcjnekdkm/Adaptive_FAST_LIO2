"""Evaluate NTU repeat records with the existing local NTU VIRAL protocol."""
import argparse
import csv
import json
import math
import statistics
import evaluate_baseline_round as e

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--sequence',type=int,choices=[1,2,3],default=1)
    n=parser.parse_args().sequence
    seq=f'ntu_spms{n}'
    gt=e.ROOT/f'bag/NTU/spms_0{n}/ground_truth/ground_truth.csv'
    records=[]
    for method,flag in [('ours_no_adaptive','0'),('ours_adaptive','1')]:
        for i in range(1,4):
            folder=e.BASE/'validation'/seq/method/'run02'
            source=folder/f'runtime{i}.csv'
            rows=list(csv.DictReader(source.open()))
            assert {r['adaptive_map'] for r in rows}=={flag},source
            assert all(math.isfinite(float(r[k])) for r in rows for k in ['pos_x','pos_y','pos_z','quat_x','quat_y','quat_z','quat_w']),source
            out=folder/f'evaluation_runtime{i}'
            if not out.exists():
                e.evaluate(seq,'ntu',gt,method=method,run_name='run02',source_filename=source.name,output_name=out.name)
            manifest=json.loads((out/'manifest.json').read_text())
            assert manifest['source_sha256']==e.digest(source),'stale evaluation: '+str(out)
            record=dict(sequence=seq,method=method,repeat=i,ate_rmse_m=manifest['ate_rmse_m'],
                        rows=len(rows),associated_samples=manifest['associated_samples'],
                        duration_s=manifest['duration_s'],max_csv_gap_s=manifest['max_gap_s'],
                        first_stamp=float(rows[0]['lidar_end_time']),last_stamp=float(rows[-1]['lidar_end_time']),
                        final_z_m=float(rows[-1]['pos_z']),source=str(source.relative_to(e.ROOT)),
                        source_sha256=manifest['source_sha256'],evaluation=str(out.relative_to(e.ROOT)))
            records.append(record);print(record,flush=True)
    summaries=[]
    for method in ['ours_no_adaptive','ours_adaptive']:
        runs=[r for r in records if r['method']==method]
        values=[r['ate_rmse_m'] for r in runs]
        summaries.append(dict(sequence=seq,method=method,n=3,mean_ate_m=statistics.mean(values),
                              sample_std_ate_m=statistics.stdev(values),min_ate_m=min(values),max_ate_m=max(values),
                              unique_csv_hashes=len({r['source_sha256'] for r in runs})))
    for suffix,data in [('ate',records),('statistics',summaries)]:
        with (e.BASE/'summary'/f'{seq}_repeats_{suffix}.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
    print('STATISTICS',summaries,flush=True)

if __name__=='__main__':
    main()
