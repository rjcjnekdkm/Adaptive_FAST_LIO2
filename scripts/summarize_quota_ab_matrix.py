"""Build provenance-checked quota ablation matrix from existing scores."""
import csv
import hashlib
import json
import statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SUMMARY = ROOT / 'experiments/adaptive_map_v1/summary'
SCENES = dict(spms1='ntu_spms1', spms3='ntu_spms3', tunnel2='geode_tunnel2',
              tunnel5='geode_tunnel5', subt='subt_hawkins')


def read_csv(path):
    with path.open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    with path.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main():
    details, groups = [], []
    for short, scene in SCENES.items():
        folder = SUMMARY / f'{short}_transient_novel_ab'
        scores = read_csv(folder / ('current_results.csv' if short == 'spms1' else 'ate.csv'))
        for row in scores:
            group = row.get('group', row.get('directory'))
            src = ROOT / f'experiments/adaptive_map_v1/validation/{scene}/transient_novel_ab/{group}/{row["run"]}/runtime.csv'
            entry = dict(sequence=scene, group=group, run=row['run'], ate_rmse_m=float(row['ate_rmse_m']),
                         raw_status='available' if src.exists() else 'historical_raw_missing',
                         source=str(src.relative_to(ROOT)), sha256='', transient_frames='',
                         transient_novel_rejected='', transient_max_novel_accepted='')
            if src.exists():
                digest = hashlib.sha256(src.read_bytes()).hexdigest()
                manifest = json.loads((src.parent / 'evaluation_v1/manifest.json').read_text())
                if digest != manifest['source_sha256']:
                    raise ValueError(f'Stale evaluation: {src}')
                if abs(entry['ate_rmse_m'] - float(manifest['ate_rmse_m'])) > 1e-6:
                    raise ValueError(f'Score mismatch: {src}')
                transient = [r for r in read_csv(src) if int(r['degeneracy_mode']) == 1]
                entry.update(sha256=digest, transient_frames=len(transient),
                             transient_novel_rejected=sum(int(r['novel_rejected']) for r in transient),
                             transient_max_novel_accepted=max((int(r['novel_accepted']) for r in transient), default=0))
            details.append(entry)
        for group in ['control', 'control_off']:
            selected = [r for r in details if r['sequence'] == scene and r['group'] == group]
            valid = [r for r in selected if r['raw_status'] == 'available']
            values = [r['ate_rmse_m'] for r in valid]
            groups.append(dict(sequence=scene, group=group, scored_records=len(selected), raw_available=len(valid),
                               unique_raw_hashes=len({r['sha256'] for r in valid}),
                               mean_ate_m=statistics.mean(values),
                               sample_std_m=statistics.stdev(values) if len(values) > 1 else '',
                               transient_novel_rejected=sum(r['transient_novel_rejected'] for r in valid),
                               transient_max_novel_accepted=max(r['transient_max_novel_accepted'] for r in valid),
                               missing_runs=';'.join(f'run0{i}' for i in (1, 2, 3) if f'run0{i}' not in {r['run'] for r in valid})))
    write_csv(SUMMARY / 'transient_novel_ab_runs.csv', details)
    write_csv(SUMMARY / 'transient_novel_ab_matrix.csv', groups)
    for row in groups:
        print(row)


if __name__ == '__main__':
    main()
