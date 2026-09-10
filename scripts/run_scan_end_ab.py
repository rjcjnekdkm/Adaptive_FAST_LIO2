"""Launch one isolated SPMS2 scan-end arm after the user builds the package."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('rule', choices=['max', 'last'])
    parser.add_argument('run', choices=['run01', 'run02', 'run03'])
    args = parser.parse_args()
    prefix = Path(subprocess.check_output(['ros2', 'pkg', 'prefix', 'adaptive_fast_lio2'], text=True).strip())
    binary = prefix / 'lib/adaptive_fast_lio2/adaptive_fastlio_mapping'
    binary_data = binary.read_bytes()
    if b'mapping.scan_end_use_last_point' not in binary_data:
        sys.exit('Installed binary lacks scan-end switch. Build and source install/setup.bash first.')
    out = ROOT / f'experiments/adaptive_map_v1/validation/ntu_spms3/scan_end_ab/{args.rule}/{args.run}'
    out.mkdir(parents=True, exist_ok=False)
    config = ROOT / 'src/adaptive_fast_lio2/config/ntu_spms.yaml'
    (out / 'config.yaml').write_bytes(config.read_bytes())
    # Freeze the launch file as well: run the inspected workspace launch, not a stale installed copy.
    launch = ROOT / 'src/adaptive_fast_lio2/launch/adaptive_fast_lio2.launch.py'
    (out / 'launch_snapshot.py').write_bytes(launch.read_bytes())
    cmd = ['ros2', 'launch', str(out / 'launch_snapshot.py'),
           f'config_path:={out}', 'config_file:=config.yaml', 'use_sim_time:=true',
           'rviz:=false', 'backend:=false', 'adaptive_map_enable:=false',
           'adaptive_window_enable:=false', 'transient_novel_quota_enable:=false',
           f'scan_end_use_last_point:={str(args.rule == "last").lower()}',
           f'frontend_runtime_csv_path:={out / "runtime.csv"}']
    bag = ROOT / 'bag/NTU/spms_03/spms_03_ros2'
    play = ['ros2', 'bag', 'play', str(bag), '--clock', '--rate', '1.0']
    (out / 'launch_command.txt').write_text(shlex.join(cmd)+'\n')
    (out / 'play_command.txt').write_text(shlex.join(play)+'\n')
    (out / 'git_commit.txt').write_text(subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True))
    (out / 'source_diff.patch').write_bytes(subprocess.check_output(['git', 'diff', '--', 'src/adaptive_fast_lio2'], cwd=ROOT))
    source_hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in (ROOT / 'src/adaptive_fast_lio2').rglob('*') if p.is_file()}
    (out / 'manifest.json').write_text(json.dumps(dict(rule=args.rule, binary=str(binary),
        binary_sha256=hashlib.sha256(binary_data).hexdigest(), source_hashes=source_hashes,
        bag=str(bag), bag_identity='path only; verify bag checksum before formal use',
        config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(), launch_command=cmd,
        state='started_not_scored'), indent=2)+'\n')
    print('After the node is ready, run in a second sourced terminal:\n'+shlex.join(play), flush=True)
    print('Output: '+str(out), flush=True)
    with (out / 'console.log').open('w') as log:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            for line in proc.stdout:
                print(line, end='', flush=True)
                log.write(line); log.flush()
            code = proc.wait()
        except KeyboardInterrupt:
            # Terminal SIGINT also reaches ros2 launch; let it flush runtime.csv.
            code = proc.wait(timeout=30)
    (out / 'exit_code.txt').write_text(str(code)+'\n')
    return code


if __name__ == '__main__':
    sys.exit(main())
