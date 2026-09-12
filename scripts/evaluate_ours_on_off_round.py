"""Evaluate current ours_on/off recordings using the existing dataset protocols."""
import csv
import json
import statistics
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import evaluate_baseline_round as baseline

ROOT = baseline.ROOT
BASE = baseline.BASE


def evaluate_source(source):
    sequence, method, run = source.relative_to(BASE / 'validation').parts[:3]
    if sequence.startswith('geode_tunnel'):
        folder = ROOT / 'bag/GEODE/Tunneling_tunnel_gamma'
        args = ('geode', folder / f'groudtruth/Tunneling_tunnel{sequence[-1]}.txt', folder / 'gamma2GT_leica.py', folder / 'rmse.py')
    elif sequence.startswith('geode_offroad'):
        folder = ROOT / 'bag/GEODE/offroad'
        args = ('geode', folder / f'groudtruth/Offroad{sequence[-1]}.txt', folder / 'gamma2GT_gnss.py', folder / 'rmse.py')
    elif sequence.startswith('geode_waterway'):
        folder = ROOT / 'bag/GEODE/water'
        args = ('geode', folder / f'groudtruth/Inland_Waterways_{sequence.split("_")[-1].title()}_beta.txt', folder / 'gamma2GT_gnss.py', folder / 'rmse.py')
    elif sequence.startswith('ntu_spms'):
        args = ('ntu', ROOT / f'bag/NTU/spms_0{sequence[-1]}/ground_truth/ground_truth.csv')
    else:
        args = ('subt', ROOT / 'bag/SubT_MRS/ground_truth_path.csv')
    name = 'evaluation_on_off_v1_' + source.stem
    manifest = source.parent / name / 'manifest.json'
    if not manifest.exists():
        baseline.evaluate(sequence, *args, method=method, run_name=run,
                          source_filename=source.name, output_name=name, folder_override=source.parent)
    result = json.loads(manifest.read_text())
    if result['source_sha256'] != baseline.digest(source):
        raise RuntimeError(f'Source changed since evaluation: {source}')
    with source.open() as f:
        rows = list(csv.DictReader(f))
    time_key = 'stamp' if 'stamp' in rows[0] else 'lidar_end_time'
    audit = {'sequence': sequence, 'method': method, 'run': run, 'file': source.name,
             'ate_rmse_m': result['ate_rmse_m'], 'rows': len(rows),
             'first_stamp': rows[0][time_key], 'last_stamp': rows[-1][time_key],
             'max_gap_s': result['max_gap_s'],
             'adaptive_values': ','.join(sorted({r.get('adaptive_map', 'missing') for r in rows})),
             'core_revision': ','.join(sorted({r.get('frontend_core_revision', 'missing') for r in rows})),
             'manifest': str(manifest.relative_to(ROOT))}
    for key in ('degenerate', 'window_ready', 'quality_rejected', 'invalid_quality_rejected',
                'direction_rejected', 'persistent_quota_rejected', 'novel_accepted', 'novel_rejected'):
        audit[key + '_sum'] = sum(float(r.get(key, 0)) for r in rows)
    return audit


def main():
    sources = sorted(p for method in ('ours_on', 'ours_off')
                     for p in (BASE / 'validation').glob(f'*/{method}/run*/runtime*.csv'))
    records = []
    with ProcessPoolExecutor(max_workers=4) as pool:
        for result in pool.map(evaluate_source, sources):
            records.append(result)
            print(result['sequence'], result['method'], result['run'], result['file'], result['ate_rmse_m'], flush=True)
    out = BASE / 'summary/ours_on_off_round'
    out.mkdir(exist_ok=True)
    with (out / 'run_details.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    comparisons = []
    for sequence in sorted({r['sequence'] for r in records}):
        on = [r for r in records if r['sequence'] == sequence and r['method'] == 'ours_on']
        off = [r for r in records if r['sequence'] == sequence and r['method'] == 'ours_off']
        latest = max(r['run'] for r in off)
        off = [r for r in off if r['run'] == latest]
        a, b = [r['ate_rmse_m'] for r in off], [r['ate_rmse_m'] for r in on]
        end_delta = min(float(r['last_stamp']) for r in on) - max(float(r['last_stamp']) for r in off)
        status = 'incomplete_recording' if end_delta < -1 else 'scored_not_success_certified'
        comparisons.append(dict(sequence=sequence, off_run=latest, off_n=len(a), on_n=len(b),
                                off_mean=statistics.mean(a), off_sd=statistics.stdev(a) if len(a)>1 else 0,
                                on_mean=statistics.mean(b),
                                improvement_pct=(1-statistics.mean(b)/statistics.mean(a))*100 if end_delta >= -1 else '',
                                on_end_delta_s=end_delta, status=status))
    with (out / 'comparison.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(comparisons[0]))
        writer.writeheader()
        writer.writerows(comparisons)
    print(json.dumps(comparisons, indent=2), flush=True)


if __name__ == '__main__':
    main()
