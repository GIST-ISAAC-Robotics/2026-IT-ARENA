#!/usr/bin/env python3
"""보존 shadow 기하와 새 순수 모듈의 동일 입력 대조. 런타임 연결 없음."""
import argparse
import json
from pathlib import Path
import zipfile

from arena_autonomy.corner_corridor import corridor_evidence
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from probe_corner_corridor import ROOT, digest
from probe_corner_temporal import baseline_matches


def compare_events(events, report):
    saved = {r['seq']:r for r in report['rows']}
    config = motion = None
    rows = []
    for event in events:
        data = event['data']
        if event['kind'] == 'header':
            config, motion = FollowConfig(**data['follow_config']), MotionConfig(**data['motion_config'])
        if event['kind'] != 'call' or event['seq'] not in saved:
            continue
        old = saved[event['seq']]['shadow'].get('geometry')
        if old is None:
            rows.append(dict(seq=event['seq'], evaluated=False, reason='geometry_not_evaluated_by_original'))
            continue
        inputs, scan, history = restore_call(data['input'], motion)
        scene = motion_scene(scan, history, inputs['reference_s'], timing_verified=inputs['timing_verified'],
                             epoch=inputs['epoch'], rear_half_angle_deg=inputs['rear_half_angle_deg'])
        new = corridor_evidence(scene.occupied_endpoints_xy_m, inputs['side'], config)
        checks = dict(support_equal=new['supported'] == old['geometry_supported'],
                      reason_equal=new['reason'] == old['reason'], path_equal=new['path'] == old['path'])
        if new['supported']:
            checks.update(prefix_equal=new['common_prefix_m'] == old['common_prefix_m'],
                          witness_indices_equal=[w['source_point_indices'] for w in new['witnesses']] ==
                                                [w['source_point_indices'] for w in old['witnesses']])
        rows.append(dict(seq=event['seq'], time_s=inputs['reference_s'], evaluated=True,
                         supported=new['supported'], matched=all(checks.values()), checks=checks))
    if len(rows) != len(saved):
        raise ValueError('saved source calls missing')
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--shadow', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = json.loads(args.shadow.read_text())
    baseline_matches(report, args.input)
    inputs = {str(p):digest(p) for p in (args.input,args.shadow)}
    files = list(report['source_sha256'])+['src/arena_autonomy/arena_autonomy/corner_corridor.py',
        'scripts/compare_corridor_module.py','tests/test_corner_corridor_module.py']
    sources = {p:digest(ROOT/p) for p in files}
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in files:
            archive.write(ROOT/p, p)
    with args.input.open() as stream:
        rows = compare_events((json.loads(line) for line in stream), report)
    summary = dict(saved_calls=len(rows), evaluated=sum(r['evaluated'] for r in rows),
                   matched=sum(r.get('matched',False) for r in rows),
                   mismatched=sum(r['evaluated'] and not r.get('matched',False) for r in rows),
                   supported=sum(r.get('supported',False) for r in rows))
    result = dict(scope='pure_geometry_module_equivalence_no_runtime_connection', summary=summary, rows=rows,
        source_sha256=sources, input_sha256=inputs,
        input_unchanged=all(digest(p) == h for p,h in inputs.items()),
        source_unchanged=all(digest(ROOT/p) == h for p,h in sources.items()),
        runtime_changed=False, driving_authorized=False)
    with (args.output/'report.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({**summary, 'input_unchanged':result['input_unchanged'],'source_unchanged':result['source_unchanged']}))
    return 0 if summary['mismatched'] == 0 and result['input_unchanged'] and result['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
