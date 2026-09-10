"""Evaluate recorded runs and build traceable ATE matrices; never choose best runs."""
import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path
import evaluate_baseline_round as ev

METHODS = ['fastlio2', 'ours_no_adaptive', 'ours_adaptive']
SEQUENCES = ([f'geode_offroad{i}' for i in range(1,4)] +
             [f'geode_tunnel{i}' for i in range(1,6)] +
             ['geode_waterway_short','geode_waterway_medium'] +
             [f'ntu_spms{i}' for i in range(1,4)] + ['subt_hawkins'])

def job(seq):
    if seq.startswith('geode_offroad'):
        base=ev.ROOT/'bag/GEODE/offroad'
        return ('geode',base/f'groudtruth/Offroad{seq[-1]}.txt',base/'gamma2GT_gnss.py',base/'rmse.py')
    if seq.startswith('geode_tunnel'):
        base=ev.ROOT/'bag/GEODE/Tunneling_tunnel_gamma'
        return ('geode',base/f'groudtruth/Tunneling_tunnel{seq[-1]}.txt',base/'gamma2GT_leica.py',base/'rmse.py')
    if seq.startswith('geode_waterway'):
        base=ev.ROOT/'bag/GEODE/water'
        return ('geode',base/f'groudtruth/Inland_Waterways_{seq.split("_")[-1].title()}_beta.txt',base/'gamma2GT_gnss.py',base/'rmse.py')
    if seq.startswith('ntu_'):
        return ('ntu',ev.ROOT/f'bag/NTU/spms_0{seq[-1]}/ground_truth/ground_truth.csv')
    return ('subt',ev.ROOT/'bag/SubT_MRS/ground_truth_path.csv')

def write_csv(path, rows):
    with path.open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def evaluate_run(seq, method, adaptive_blind, run_name='run01'):
    folder=ev.BASE/'validation'/seq/method/run_name
    source=folder/'runtime.csv'
    rows=list(csv.DictReader(source.open()))
    sha=ev.digest(source)
    output=folder/'evaluation_v1'
    manifest_path=output/'manifest.json'
    cached=json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    args=job(seq)
    expected={'source_sha256':sha,'gt_sha256':ev.digest(args[1])}
    if args[0]=='geode':
        expected.update(converter_sha256=ev.digest(args[2]),scorer_sha256=ev.digest(args[3]))
    if not cached or any(cached.get(k)!=v for k,v in expected.items()):
        if output.exists():
            output.rename(folder/f'evaluation_previous_{datetime.now().strftime("%Y%m%d_%H%M%S_%f")}')
        ev.evaluate(seq,*args,method=method,run_name=run_name)
        cached=json.loads(manifest_path.read_text())
    flags=sorted(set(r.get('adaptive_map','NA') for r in rows))
    valid_flags=(flags==['1']) if method.startswith('ours_adaptive') else (flags==['0'] if method.startswith('ours_no') else True)
    finite=all(math.isfinite(float(r[k])) for r in rows for k in ['pos_x','pos_y','pos_z','quat_x','quat_y','quat_z','quat_w'])
    blind='2' if method.endswith('blind2') else ('4' if seq.startswith('geode_') else 'dataset_config')
    if seq.startswith('geode_tunnel') and method=='ours_adaptive':
        blind=adaptive_blind
    duplicates=[m for m in METHODS if m!=method and (folder.parent.parent/m/'run01/runtime.csv').exists() and ev.digest(folder.parent.parent/m/'run01/runtime.csv')==sha]
    return dict(sequence=seq,method=method,run=run_name,ate_rmse_m=cached['ate_rmse_m'],
                role_valid=valid_flags,adaptive_flags=';'.join(flags),finite_trajectory=finite,
                blind_m=blind,blind_evidence='directory/history or user confirmation; no runtime config snapshot',
                rows=len(rows),duration_s=cached['duration_s'],max_gap_s=cached['max_gap_s'],
                normal_frames=sum(r.get('degeneracy_mode')=='0' for r in rows),
                transient_frames=sum(r.get('degeneracy_mode')=='1' for r in rows),
                persistent_frames=sum(r.get('degeneracy_mode')=='2' for r in rows),
                zero_effective_frames=sum(r.get('effective_points')=='0' for r in rows),
                quality_rejected_sum=sum(int(r.get('quality_rejected',0)) for r in rows),
                final_z_m=float(rows[-1]['pos_z']),
                identical_csv_to=';'.join(duplicates),
                source=str(source.relative_to(ev.ROOT)),source_sha256=sha,
                evaluation=str(output.relative_to(ev.ROOT)),protocol=cached['protocol'])

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--adaptive-tunnel-blind',choices=['2','4','unknown'],default='unknown')
    opts=parser.parse_args()
    details=[]
    for seq in SEQUENCES:
        for method in METHODS+(['fastlio2_blind2','ours_no_adaptive_blind2'] if seq.startswith('geode_tunnel') else []):
            if not (ev.BASE/'validation'/seq/method/'run01/runtime.csv').exists():
                continue
            result=evaluate_run(seq,method,opts.adaptive_tunnel_blind)
            details.append(result)
            print(seq,method,result['ate_rmse_m'],'role_valid',result['role_valid'],flush=True)
        if seq.startswith('geode_tunnel'):
            details.append(evaluate_run(seq,'ours_adaptive_blind2','2',run_name='run02'))
    summary=ev.BASE/'summary';summary.mkdir(exist_ok=True)
    write_csv(summary/'ate_run_details.csv',details)
    index={(r['sequence'],r['method']):r for r in details}
    def matrix(blind2=False):
        output=[]
        for seq in SEQUENCES:
            selected={m:index.get((seq,m+('_blind2' if blind2 and seq.startswith('geode_tunnel') and m!='ours_adaptive' else ''))) for m in METHODS}
            if blind2 and seq.startswith('geode_tunnel'):
                selected['ours_adaptive']=index[(seq,'ours_adaptive_blind2')]
            row={'sequence':seq}
            for m,r in selected.items():
                row[m+'_ate_rmse_m']=f'{r["ate_rmse_m"]:.6f}' if r and r['role_valid'] and r['finite_trajectory'] else ''
            flags=[]
            for m,r in selected.items():
                row[m+'_status']='missing' if not r else ('switch_mismatch' if not r['role_valid'] else ('nonfinite' if not r['finite_trajectory'] else 'scored'))
                row[m+'_blind_m']=r['blind_m'] if r else ''
                row[m+'_source']=r['source'] if r else ''
                if row[m+'_status']!='scored':flags.append(m+':'+row[m+'_status'])
            if seq.startswith('geode_tunnel'):
                blinds={r['blind_m'] for r in selected.values() if r}
                if 'unknown' in blinds: flags.append('adaptive_blind_unconfirmed')
                elif len(blinds)>1: flags.append('blind_mismatch')
            row['comparison_notes']=';'.join(flags) or 'single_run;configuration_and_coverage_audit_required'
            if blind2 and seq.startswith('geode_tunnel'):
                row['comparison_notes']+=';adaptive_min_range=2m;adaptive_run=run02'
            output.append(row)
        return output
    rows=matrix(opts.adaptive_tunnel_blind=='2')
    write_csv(summary/'ate_experiment_matrix.csv',rows)
    write_csv(summary/'ate_experiment_matrix_tunnel_blind2.csv',matrix(True))
    write_csv(summary/'ate_experiment_matrix_original_directories.csv',matrix())
    lines=['# ATE实验矩阵（单位：m）','',
           '主矩阵在2米设置下：Tunnel1～5的FAST-LIO2和关闭Adaptive使用blind2/run01，开启组使用ours_adaptive_blind2/run02（入图下限2米）；其他序列仍用同名目录run01。按用户指定更新，不择优。每格来源见CSV的source列。原目录参考表保留此前结果。',
           '空单元格由status列说明；switch_mismatch表示记录开关与实验角色不符，评分仍保留在ate_run_details.csv。scored仅表示评分完成，不等于轨迹成功或完整覆盖。',
           'GEODE：原始本地官方坐标转换脚本+rmse.py，帧尾时间，SE(3)，max_diff=0.1s，offset=+0.1s作用于第二输入GT（沿用约定，非官方强制offset）。',
           'NTU：现有Python实现的NTU协议，未执行官方MATLAB。SubT：evo SE(3)，max_diff=0.1s，offset=0。',
           '各组按自身关联样本评分，尚未统一时间支持；初始化、代码版本及运行波动仍需审计。不能把现有矩阵直接作为模块因果增益结论。',
           f'开启Adaptive的隧道盲区：{opts.adaptive_tunnel_blind}。','',
           '| 序列 | FAST-LIO2 | Ours no adaptive | Ours adaptive |','|---|---:|---:|---:|']
    for r in rows:
        lines.append('| '+r['sequence']+' | '+' | '.join(r[m+'_ate_rmse_m'] or '待有效记录' for m in METHODS)+' |')
    (summary/'ate_experiment_matrix.md').write_text('\n'.join(lines)+'\n')

if __name__=='__main__':
    main()
