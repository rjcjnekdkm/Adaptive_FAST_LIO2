"""Evaluate the recorded baseline round; preserve source CSVs and official scripts."""
import csv
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from decimal import Decimal

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'experiments/adaptive_map_v1'
ENV = dict(os.environ, MPLBACKEND='Agg', MPLCONFIGDIR='/tmp/adaptive_eval_mpl')
ENV['PATH'] = str(Path(sys.executable).parent) + ':' + ENV['PATH']


def run(cmd, cwd, name):
    proc = subprocess.run([str(x) for x in cmd], cwd=cwd, env=ENV, capture_output=True, text=True)
    (cwd / (name + '.log')).write_text(proc.stdout + '\n' + proc.stderr)
    if proc.returncode:
        raise RuntimeError(f'{name} failed: {cwd}')
    return proc.stdout


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def evaluate(key, kind, gt, converter=None, scorer=None, method='fastlio2', run_name='run01',
             source_filename='runtime.csv', output_name='evaluation_v1', folder_override=None):
    folder = Path(folder_override) if folder_override is not None else BASE / 'validation' / key / method / run_name
    source = folder / source_filename
    output = folder / output_name
    output.mkdir(exist_ok=False)
    rows = list(csv.DictReader(source.open()))
    # Aligned Ours baseline uses FAST-LIO2's external recorder as well.
    time_key = 'stamp' if 'stamp' in rows[0] else 'lidar_end_time'
    stamps = [float(r[time_key]) for r in rows]
    raw = output / 'trajectory_end.tum'
    fields = [time_key, 'pos_x', 'pos_y', 'pos_z', 'quat_x', 'quat_y', 'quat_z', 'quat_w']
    raw.write_text(''.join(' '.join(r[f] for f in fields) + '\n' for r in rows))
    manifest = dict(source=str(source.relative_to(ROOT)), source_sha256=digest(source),
                    gt=str(gt.relative_to(ROOT)), gt_sha256=digest(gt),
                    stamp=time_key + '; end-of-scan pose', method=method,
                    runtime_source_revision='not captured at run time; do not infer from current HEAD',
                    rows=len(rows), duration_s=stamps[-1]-stamps[0],
                    max_gap_s=max(b-a for a,b in zip(stamps,stamps[1:])))
    if kind == 'geode':
        rawdir = output / ('shield_tunnel1' if 'leica' in converter.name else 'Offroad1')
        rawdir.mkdir()
        (output/'tran2body').mkdir()
        trajectory_name = method + '.txt'
        shutil.copyfile(raw, rawdir/trajectory_name)
        shutil.copyfile(converter, output/converter.name)
        run([sys.executable, output/converter.name], output, 'official_conversion')
        shutil.copyfile(output/'tran2body'/trajectory_name, output/trajectory_name)
        shutil.copyfile(gt, output/'ground_truth.txt')
        shutil.copyfile(scorer, output/'rmse.py')
        text = run([sys.executable, output/'rmse.py', method, 'ground_truth.txt', '0.1'], output, 'official_rmse')
        match = re.search(r'Extracted RMSE:\s*([\d.]+)', text)
        if not match:
            raise RuntimeError(f'Official RMSE did not produce a result: {key}')
        rmse = float(match.group(1))
        manifest.update(protocol='GEODE original local conversion + rmse.py',
                        converter_sha256=digest(converter), scorer_sha256=digest(scorer),
                        time_offset_s=0.1, offset_applies_to='GT (second evo input)',
                        offset_basis='previous user-specified protocol; not an official mandated offset')
        # Same order/parameters as official rmse.py, retaining plots and full results.
        cmd = ['evo_ape','tum',trajectory_name,'ground_truth.txt','-va',
               '--t_max_diff','0.1','--t_offset','0.1']
    elif kind == 'ntu':
        evaluator = ROOT/'scripts/evaluate_ntu_viral.py'
        run([sys.executable,evaluator,gt,source,output], output, 'ntu_evaluation')
        metrics = json.loads((output/'ntu_ate_metrics.json').read_text())
        rmse = metrics['ate_rmse_m']
        manifest.update(protocol='Local Python implementation of NTU VIRAL protocol; not official MATLAB execution',
                        evaluator_sha256=digest(evaluator), associated_samples=metrics['associated_samples'])
        import numpy as np
        import matplotlib.pyplot as plt
        a = np.genfromtxt(output/'ntu_aligned_trajectory.csv', delimiter=',', names=True)
        fig, axes = plt.subplots(1,2,figsize=(11,4))
        axes[0].plot(a['gt_x'],a['gt_y'],label='GT')
        axes[0].plot(a['est_prism_x'],a['est_prism_y'],label=method + ' prism')
        axes[0].set(xlabel='x (m)',ylabel='y (m)',title=key)
        axes[0].axis('equal'); axes[0].legend()
        axes[1].plot(a['time_s']-a['time_s'][0],a['err_norm'])
        axes[1].set(xlabel='time (s)',ylabel='position error (m)',title=f'ATE RMSE {rmse:.4f} m')
        fig.tight_layout(); fig.savefig(output/'trajectory_error.png',dpi=160); plt.close(fig)
        cmd = None
    else:
        gr = list(csv.DictReader(gt.open()))
        columns = ['p_w_b_x','p_w_b_y','p_w_b_z','q_w_b_x','q_w_b_y','q_w_b_z','q_w_b_w']
        (output/'ground_truth.tum').write_text(''.join(
            f"{Decimal(r['timestamp']) / Decimal(1000000000):.9f} " +
            ' '.join(r[c] for c in columns)+'\n' for r in gr))
        manifest.update(protocol='SubT evo SE(3) alignment, max_diff=0.1s, offset=0',time_offset_s=0)
        cmd=['evo_ape','tum','ground_truth.tum','trajectory_end.tum','-va','--t_max_diff','0.1']
        text=run(cmd, output, 'subt_ape')
        rmse=float(re.search(r'\brmse\s+([\d.]+)',text).group(1))
        run(['evo_ape','tum','ground_truth.tum','trajectory_end.tum','-va','--t_max_diff','0.2'],output,'subt_ape_sensitivity_0p2')
    if cmd:
        run(cmd+['--save_results','ape.zip','--save_plot','ape.png','--plot_mode','xy'],output,'ape_artifacts')
    manifest['ate_rmse_m']=rmse
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return dict(sequence=key,ate_rmse_m=rmse,rows=len(rows),max_gap_s=manifest['max_gap_s'],
                protocol=manifest['protocol'],path=str(output.relative_to(ROOT)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--method', choices=['fastlio2', 'ours_no_adaptive'], default='fastlio2')
    method = parser.parse_args().method
    jobs=[]
    off=ROOT/'bag/GEODE/offroad'; tun=ROOT/'bag/GEODE/Tunneling_tunnel_gamma'; water=ROOT/'bag/GEODE/water'
    for i in range(1,4):
        jobs.append((f'geode_offroad{i}','geode',off/f'groudtruth/Offroad{i}.txt',off/'gamma2GT_gnss.py',off/'rmse.py'))
    for i in range(1,6):
        jobs.append((f'geode_tunnel{i}','geode',tun/f'groudtruth/Tunneling_tunnel{i}.txt',tun/'gamma2GT_leica.py',tun/'rmse.py'))
    for seq in ('short','medium'):
        jobs.append((f'geode_waterway_{seq}','geode',water/f'groudtruth/Inland_Waterways_{seq.title()}_beta.txt',water/'gamma2GT_gnss.py',water/'rmse.py'))
    for i in range(1,4):
        jobs.append((f'ntu_spms{i}','ntu',ROOT/f'bag/NTU/spms_0{i}/ground_truth/ground_truth.csv'))
    jobs.append(('subt_hawkins','subt',ROOT/'bag/SubT_MRS/ground_truth_path.csv'))
    results=[]
    for job in jobs:
        results.append(evaluate(*job, method=method))
        print(job[0],results[-1]['ate_rmse_m'],flush=True)
    summary=BASE/'summary'; summary.mkdir(exist_ok=True)
    with (summary/f'{method}_round1_ate.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(results[0]));w.writeheader();w.writerows(results)
    lines=[f'# {method} round 1 evaluation','',
           'GEODE: original local conversion/rmse.py; end stamp; offset +0.1 s applied to GT by official input order.',
           'NTU: existing local Python implementation, not official MATLAB execution. SubT: evo max_diff 0.1 s; 0.2 s sensitivity log retained.',
           'Single runs, no significance claim. Runtime code/config snapshots were not captured; manifests identify this limitation.',
           'Trajectory gaps and NTU start/end coverage remain audit items. ATE is computed over associated samples, not proof of full-run success.','',
           '| Sequence | ATE RMSE (m) | Rows | Max gap (s) |','|---|---:|---:|---:|']
    lines += [f"| {r['sequence']} | {r['ate_rmse_m']:.6f} | {r['rows']} | {r['max_gap_s']:.3f} |" for r in results]
    (summary/f'{method}_round1_ate.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    main()
