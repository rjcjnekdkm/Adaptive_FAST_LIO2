"""Audit available SPMS quota runs on common GT support; no map-quality claim."""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import evaluate_ntu_viral as ntu

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'experiments/adaptive_map_v1/summary/spms_quota_mapping_audit'


def write_csv(path, rows):
    with path.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def audit(sequence):
    base = ROOT / f'experiments/adaptive_map_v1/validation/ntu_spms{sequence}/transient_novel_ab'
    gt = ROOT / f'bag/NTU/spms_0{sequence}/ground_truth/ground_truth.csv'
    gt_t, gt_p = ntu.read_gt(gt)
    zero = float(np.genfromtxt(gt, delimiter=',', names=True, max_rows=1)['time'])
    runs, missing = {}, []
    for group in ['control', 'control_off']:
        for run in ['run01', 'run02', 'run03']:
            folder = base / group / run
            src = folder / 'runtime.csv'
            name = f'{group}/{run}'
            if not src.exists():
                missing.append(name)
                continue
            manifest = json.loads((folder / 'evaluation_v1/manifest.json').read_text())
            digest = hashlib.sha256(src.read_bytes()).hexdigest()
            if digest != manifest['source_sha256']:
                raise ValueError(f'Stale evaluation: {src}')
            d = np.genfromtxt(src, delimiter=',', names=True)
            t, p = ntu.read_estimate(src, zero)
            mt, mp, mg = ntu.interpolate_gt(gt_t, gt_p, t, p)
            runs[name] = dict(d=d, t=t, p=p, matched={float(a): (b, c) for a, b, c in zip(mt, mp, mg)},
                              source=str(src.relative_to(ROOT)), sha256=digest)
    common = np.array(sorted(set.intersection(*(set(r['matched']) for r in runs.values()))))
    origin = min(r['t'][0] for r in runs.values())
    rel = common - origin
    prefix = rel < 60
    if prefix.sum() < 20:
        raise ValueError('Insufficient common prefix support')
    windows, summary, error_rows = [], [], []
    fig, axes = plt.subplots(4, 1, figsize=(13, 13), sharex=True)
    for name, r in runs.items():
        d, t = r['d'], r['t'] - origin
        cp = np.array([r['matched'][a][0] for a in common])
        cg = np.array([r['matched'][a][1] for a in common])
        a, b = cp[prefix].mean(0), cg[prefix].mean(0)
        u, singular, vt = np.linalg.svd((cp[prefix] - a).T @ (cg[prefix] - b))
        fix = np.eye(3)
        fix[-1, -1] = np.linalg.det(u @ vt)
        rotation = u @ fix @ vt
        err = np.linalg.norm((cp - a) @ rotation + b - cg, axis=1)
        transient = d['degeneracy_mode'] == 1
        axes[0].scatter(rel, err, s=2, label=name)
        bins = []
        for lo in range(0, int(np.ceil(t[-1])), 10):
            m, g = (t >= lo) & (t < lo + 10), (rel >= lo) & (rel < lo + 10)
            if not m.any():
                continue
            tr = m & transient
            row = dict(run=name, start_s=lo, frames=int(m.sum()), common_gt_samples=int(g.sum()),
                       prefix_error_median=float(np.median(err[g])) if g.any() else None,
                       residual_median=float(np.median(d['residual_mean'][m])),
                       transient_frames=int(tr.sum()),
                       novel_accepted=int(d['novel_accepted'][tr].sum()),
                       novel_rejected=int(d['novel_rejected'][tr].sum()),
                       map_added=int(d['map_added'][m].sum()),
                       invalid_quality_rejected=int(d['invalid_quality_rejected'][m].sum()),
                       map_size_last=int(d['map_size'][m][-1]))
            windows.append(row)
            bins.append(row)
        for ax, key in zip(axes[1:], ['novel_accepted', 'novel_rejected', 'residual_median']):
            ax.plot([x['start_s'] + 5 for x in bins], [x[key] for x in bins], label=name)
        summary.append(dict(run=name, source=r['source'], sha256=r['sha256'], frames=len(d),
                            common_gt_samples=len(common), prefix_samples=int(prefix.sum()),
                            prefix_alignment_singular_values=singular.tolist(),
                            prefix_aligned_rmse=float(np.sqrt(np.mean(err ** 2))),
                            last_20s_error_median=float(np.median(err[rel >= rel[-1] - 20])),
                            transient_novel_accepted=int(d['novel_accepted'][transient].sum()),
                            transient_novel_rejected=int(d['novel_rejected'][transient].sum()),
                            submitted_map_points=int(d['map_added'].sum())))
        error_rows.extend(dict(run=name, relative_s=float(s), prefix_aligned_error_m=float(e)) for s, e in zip(rel, err))
    for ax, label in zip(axes, ['Prefix-aligned position error (m)', 'Transient novel accepted / 10 s',
                               'Transient novel rejected / 10 s', 'Median mean residual (m) / 10 s']):
        ax.set_ylabel(label)
        ax.grid(alpha=.3)
    axes[0].legend(ncol=3, fontsize=8)
    axes[-1].set_xlabel('Seconds from first logged scan; errors shown only at common valid GT timestamps')
    fig.tight_layout()
    fig.savefig(OUT / f'spms{sequence}_diagnostics.png', dpi=150)
    plt.close(fig)
    write_csv(OUT / f'spms{sequence}_windows.csv', windows)
    write_csv(OUT / f'spms{sequence}_common_errors.csv', error_rows)
    result = dict(missing_raw=missing, gt_sha256=hashlib.sha256(gt.read_bytes()).hexdigest(),
                  common_gt_gaps_over_1s=[dict(start=float(rel[i]), end=float(rel[i+1])) for i in np.where(np.diff(rel) > 1)[0]], runs=summary)
    (OUT / f'spms{sequence}_metrics.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    for sequence in (1, 3):
        audit(sequence)
