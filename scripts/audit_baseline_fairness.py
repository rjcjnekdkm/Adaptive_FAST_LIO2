"""Read-only source/run audit; write evidence tables, never relabel historical runs."""
import csv
import hashlib
import json
import yaml
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'experiments/adaptive_map_v1/summary/baseline_fairness_audit'


def rows(path):
    with path.open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def main():
    OUT.mkdir(exist_ok=True)
    result = []
    for row in rows(ROOT / 'experiments/adaptive_map_v1/summary/ate_experiment_matrix.csv'):
        for method in ['fastlio2', 'ours_no_adaptive', 'ours_adaptive']:
            src = ROOT / row[method + '_source']
            data = rows(src) if src.exists() else []
            digest = hashlib.sha256(src.read_bytes()).hexdigest() if src.exists() else ''
            manifest_path = src.parent / 'evaluation_v1/manifest.json'
            manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
            stamp_key = next((k for k in ('lidar_end_time', 'stamp') if data and k in data[0]), None)
            stamps = [float(d[stamp_key]) for d in data] if stamp_key else []
            entry = dict(sequence=row['sequence'], method=method, source=str(src.relative_to(ROOT)),
                         sha256=digest, manifest_hash_match=digest == manifest.get('source_sha256'),
                         rows=len(data), first_stamp=stamps[0] if stamps else '', last_stamp=stamps[-1] if stamps else '',
                         max_gap_s=max((b-a for a,b in zip(stamps, stamps[1:])), default=0) if stamps else '',
                         raw_status='available' if src.exists() else 'missing',
                         adaptive_map_values=';'.join(sorted({d['adaptive_map'] for d in data})) if data and 'adaptive_map' in data[0] else 'not_logged',
                         window_ready_values=';'.join(sorted({d['window_ready'] for d in data})) if data and 'window_ready' in data[0] else 'not_logged',
                         runtime_source_revision=manifest.get('runtime_source_revision', 'unknown'),
                         run_metadata_files=';'.join(sorted(str(p.relative_to(src.parent)) for p in (src.parent.iterdir() if src.parent.exists() else [])
                                                          if p.is_file() and p.suffix in ('.yaml', '.yml', '.txt', '.log', '.md', '.json'))))
            result.append(entry)
    with (OUT / 'main_run_audit.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(result[0])); w.writeheader(); w.writerows(result)
    paths = list((ROOT / 'src/adaptive_fast_lio2/src').glob('adaptive_*.cpp'))
    paths += [ROOT / 'src/FAST_LIO/src' / n for n in ['laserMapping.cpp', 'preprocess.cpp', 'IMU_Processing.hpp']]
    paths += list((ROOT / 'src/adaptive_fast_lio2/config').glob('*.yaml'))
    paths += list((ROOT / 'src/FAST_LIO/config').glob('*.yaml'))
    paths += [ROOT / 'src/adaptive_fast_lio2/launch/adaptive_fast_lio2.launch.py']
    (OUT / 'audited_source_hashes.json').write_text(json.dumps({str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}, indent=2)+'\n')
    print('runs', len(result), 'hash_matches', sum(r['manifest_hash_match'] for r in result))
    for r in result:
        if r['method'] == 'ours_no_adaptive':
            print(r['sequence'], r['adaptive_map_values'], r['window_ready_values'], r['run_metadata_files'] or 'no_run_metadata')
    comparisons = []
    fields = ['common.lid_topic', 'common.imu_topic', 'common.time_sync_en', 'common.time_offset_lidar_to_imu',
              'preprocess.blind', 'preprocess.lidar_type', 'preprocess.scan_line', 'preprocess.timestamp_unit',
              'point_filter_num', 'feature_extract_enable', 'mapping.acc_cov', 'mapping.gyr_cov',
              'mapping.b_acc_cov', 'mapping.b_gyr_cov', 'mapping.extrinsic_T', 'mapping.extrinsic_R',
              'mapping.extrinsic_est_en', 'mapping.det_range']
    pairs = [('GEODE_blind2', 'geode_gamma_source_check_baseline.yaml', 'geode_gamma_blind2.yaml'),
             ('NTU', 'ntu_spms_baseline.yaml', 'ntu_spms.yaml'),
             ('SubT', 'subt_mrs_hawkins_long_corridor.yaml', 'subt_mrs_hawkins_long_corridor.yaml')]
    def value(d, key):
        for k in key.split('.'):
            d = d.get(k, {}) if isinstance(d, dict) else {}
        return d if d != {} else 'not_set'
    for scene, baseline, adaptive in pairs:
        a = next(iter(yaml.safe_load((ROOT / 'src/FAST_LIO/config' / baseline).read_text()).values()))['ros__parameters']
        b = next(iter(yaml.safe_load((ROOT / 'src/adaptive_fast_lio2/config' / adaptive).read_text()).values()))['ros__parameters']
        keys = [(k, k) for k in fields] + [('filter_size_surf', 'mapping.filter_size_surf'),
                ('filter_size_map', 'mapping.filter_size_map'), ('max_iteration', 'mapping.scan_match_max_iteration'),
                ('cube_side_length', 'mapping.cube_len')]
        for ka, kb in keys:
            va, vb = value(a, ka), value(b, kb)
            comparisons.append(dict(scene=scene, baseline_file=baseline, ours_file=adaptive, baseline_key=ka,
                                    ours_key=kb, baseline_value=json.dumps(va), ours_value=json.dumps(vb), equal=va == vb))
    with (OUT / 'current_config_comparison.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(comparisons[0])); w.writeheader(); w.writerows(comparisons)
    print('Current YAML mismatches:', [r for r in comparisons if not r['equal']])


if __name__ == '__main__':
    main()
