"""C5와 같은 정지차·시설·속도로 현재 경로 선택을 검증한다. 별도 출력에 한 번만 실행."""
import argparse
import hashlib
import inspect
import json
import subprocess
import sys
import zipfile
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PREVIOUS = ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'tests')]
import validate_g1_traffic as target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name')
    parser.add_argument('--recessed', action='store_true')
    args = parser.parse_args()
    if not args.name.isidentifier(): raise ValueError('single folder identifier required')
    folder = HERE/args.name
    folder.mkdir(exist_ok=False)
    base = json.loads((PREVIOUS/'c_stationary_posts/args_run5.json').read_text())
    cli = list(base['args'])
    cli[cli.index('--output')+1] = str(folder/'run1')
    if args.recessed:
        i = cli.index('--config')+1
        data = json.loads((ROOT/cli[i]).read_text())
        data['actor']['visual_outline'] = 'recessed_sides'
        config = folder/'recessed_config.json'
        config.write_text(json.dumps(data, indent=2)+'\n')
        cli[i] = str(config)
    extra = [Path(__file__), HERE/'stationary_run.sh', HERE/'analyze_continuity.py',
             *[PREVIOUS/name for name in ('stationary_goal_eval.py', 'evaluate_traffic_posts.py',
                                          'stationary_posts_assess.py')]]
    if args.recessed:
        extra.extend([HERE/'audit_recessed.py',
            ROOT/'artifacts/validation/2026-11-08/medium_recessed_pass_v1/outline_moving.py',
            ROOT/'scripts/audit_actor_outline_returns.py'])
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in extra}
    record = dict(registered_at_utc=datetime.now(timezone.utc).isoformat(),
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        args=cli, parent='11/9 C5', added_launch_arguments=base['added_launch_arguments'],
        recessed_visual=args.recessed, extras=hashes,
        scope='same stationary C5 setup; current code; visual-only recessed shape if explicitly requested',
        gates=dict(original_report_preserved=True, original_C5_goal_preserved=True,
                   required=['no_contact', 'facility_clear', 'source_unchanged', 'STOP', 'clean_shutdown'],
                   continuity='compare reason-aligned stop episodes; no post-hoc threshold replacement'))
    if args.recessed:
        record['shape_audit'] = dict(original_collision_box_preserved=True,
            recessed_center_returns_minimum=2, unique_scans_minimum=2,
            more_center_returns_than_old_box_side=True, time_shift_s=[-.01, .01],
            scope='visual concavity at lidar plane only; not arbitrary vehicle shapes')
    (folder/'registered.json').write_text(json.dumps(record, indent=2)+'\n')
    with zipfile.ZipFile(folder/'extra_sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in extra: archive.write(p,p.relative_to(ROOT))
    original = target.start_demo_process
    def start(*a, **kw):
        bound = inspect.signature(original).bind(*a,**kw);bound.apply_defaults()
        bound.arguments['extra_arguments'] = [*bound.arguments['extra_arguments'],*base['added_launch_arguments']]
        return original(*bound.args,**bound.kwargs)
    target.start_demo_process = start
    sys.argv = cli
    try:
        return target.main()
    finally:
        changed = [p for p,h in hashes.items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        (folder/'extra_integrity.json').write_text(json.dumps(dict(changed=changed),indent=2)+'\n')


if __name__ == '__main__': raise SystemExit(main())
