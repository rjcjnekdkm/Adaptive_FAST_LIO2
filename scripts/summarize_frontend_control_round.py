"""Join existing single-run results without changing alignment or raw data."""
import csv
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SUMMARY = ROOT / 'experiments/adaptive_map_v1/summary'


def main():
    tables = []
    for method in ('fastlio2', 'ours_no_adaptive'):
        with (SUMMARY / f'{method}_round1_ate.csv').open() as f:
            tables.append({r['sequence']: r for r in csv.DictReader(f)})
    if tables[0].keys() != tables[1].keys():
        raise ValueError('Sequence sets differ')
    out = []
    for sequence, baseline in tables[0].items():
        control = tables[1][sequence]
        a, b = float(baseline['ate_rmse_m']), float(control['ate_rmse_m'])
        out.append(dict(sequence=sequence, fastlio2_ate_m=a, ours_no_adaptive_ate_m=b,
                        delta_ours_minus_fastlio2_m=b-a,
                        fastlio2_rows=baseline['rows'], ours_rows=control['rows']))
    with (SUMMARY/'frontend_control_comparison.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(out[0])); w.writeheader(); w.writerows(out)
    lines = [
        '# FAST-LIO2 与关闭 Adaptive 对照组：本轮描述性比较', '',
        '每组仅一次当前保留的运行；不是 Adaptive Map 增益实验。',
        'GEODE 两组均执行本地官方转换及 rmse.py，帧尾时间戳，offset=+0.1 s（作用于第二输入 GT）。',
        'NTU 均为现有 Python 协议实现，非官方 MATLAB；SubT 均为 evo SE(3)，max_diff=0.1 s，offset=0。',
        '当前表使用各次运行自身的关联样本，尚未统一共同时间支持；初始化、记录频率和覆盖范围存在差异。',
        '本轮配置为 Adaptive 初始化 200；FAST-LIO2 当前源码 MAX_INI_COUNT=10，也按 IMU 样本计数。运行时二进制/参数快照缺失，不能仅凭当前源码证明当时设置。',
        'SPMS1 的关闭 Adaptive run01 是重跑覆盖结果；原始 3077 行 CSV 已被用户删除，只保留历史摘要。',
        '因此差异不能归因于自适应模块，不能据此宣称工程等价或统计显著。', '',
        'Tunnel3 的关闭 Adaptive run01 已在补齐下采样点<5保护后再次同配置重跑覆盖，其他序列未随之重跑，因此本表存在代码版本差异。最新末尾 z≈−6.924 m，ATE≈9.625 m；上一次同配置ATE≈65.849 m，保护前≈6165.233 m。相同配置仍明显波动，不能将本次结果视为稳定性能或把差异全部归因于保护。', '',
        '| 序列 | FAST-LIO2 ATE (m) | 关闭 Adaptive ATE (m) | 后者减前者 (m) |',
        '|---|---:|---:|---:|',
    ]
    for r in out:
        lines.append(f"| {r['sequence']} | {r['fastlio2_ate_m']:.6f} | {r['ours_no_adaptive_ate_m']:.6f} | {r['delta_ours_minus_fastlio2_m']:+.6f} |")
    (SUMMARY/'frontend_control_comparison.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
