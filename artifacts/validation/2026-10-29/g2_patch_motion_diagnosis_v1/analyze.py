"""저장 G2 입력에서 조각 이동과 운동 이송 오차를 분리한다. 제어 출력/새 주행 없음.

평가 자세는 같은 시점의 정답 이송과 비교하는 데만 사용한다. 실제 monitor에는
원래 모터축/IMU 이력을 전달한다. 입력과 실행 당시 분석 소스를 보존한다.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.bypass_handoff import BlockingPersistence, HandoffExecutionConfig, front_patch
from arena_autonomy.follow_trace import restore_call, thaw
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene, transport_points


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def truth_transport(points, previous, current):
    """평가 전용: 과거 base 좌표 → world → 현재 base 좌표."""
    a, b = previous, current
    ca, sa, cb, sb = math.cos(a[2]), math.sin(a[2]), math.cos(b[2]), math.sin(b[2])
    world = points @ np.array([[ca, sa], [-sa, ca]]) + a[:2]
    return (world-b[:2]) @ np.array([[cb, -sb], [sb, cb]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--case', default='right_far_combined')
    parser.add_argument('--policy', default='observed_entry_space_v1')
    parser.add_argument('--source', default='g2_stop_entry_v1/closed_space1')
    parser.add_argument('--until', type=float, default=3.7)
    args = parser.parse_args()
    folder = BASE.parent/args.source
    name = args.case+'__'+args.policy
    trace = folder/(name+'.jsonl')
    evaluation = folder/(name+'.records.json')
    out = BASE/args.output
    out.mkdir(exist_ok=False)
    source_paths = [Path(__file__), ROOT/'src/arena_autonomy/arena_autonomy/bypass_handoff.py',
                    ROOT/'src/arena_autonomy/arena_autonomy/scene_motion.py']
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in [trace, evaluation]+source_paths}
    with zipfile.ZipFile(out/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as z:
        for p in source_paths:
            z.write(p, p.relative_to(ROOT))
    records = {round(r['t'], 6): r for r in json.loads(evaluation.read_text(encoding='utf-8'))}
    rows = []
    last = None
    with trace.open(encoding='utf-8') as f:
        for line in f:
            event = json.loads(line)
            kind, data = event['kind'], event['data']
            if kind == 'header':
                motion = MotionConfig(**data['motion_config'])
                monitor = BlockingPersistence(HandoffExecutionConfig.from_mapping(data['execution_config']))
                continue
            if kind == 'reset':
                monitor.reset(); last = None
                continue
            if kind != 'call':
                continue
            restored, scan, history = restore_call(data['input'], motion)
            now = restored['reference_s']
            if now > args.until+1e-9:
                break
            scene = motion_scene(scan, history, now, timing_verified=restored['timing_verified'],
                epoch=restored['epoch'], rear_half_angle_deg=restored['rear_half_angle_deg'])
            if last is not None and scene.source_first_s == last['source']:
                continue
            patch, reason = front_patch(scene)
            rec = records[round(now, 6)]
            pose = np.array(rec['evaluation']['center_pose'])
            expected = thaw(data['output'])[2]
            old_rows = list(monitor.rows)
            pairs = []
            for old in old_rows if patch is not None else []:
                projected = transport_points(old['patch'], history, old['time'], now)
                old_pose = np.array(records[round(old['time'], 6)]['evaluation']['center_pose'])
                truth = truth_transport(old['patch'], old_pose, pose)
                pairs.append(dict(source=old['source'], time=old['time'],
                    median_measured_m=float(np.linalg.norm(np.median(patch, axis=0)-np.median(projected, axis=0))),
                    median_truth_m=float(np.linalg.norm(np.median(patch, axis=0)-np.median(truth, axis=0))),
                    transport_error_max_m=float(np.linalg.norm(projected-truth, axis=1).max()),
                    previous_median=np.median(old['patch'], axis=0).tolist(),
                    projected_median=np.median(projected, axis=0).tolist(),
                    truth_median=np.median(truth, axis=0).tolist()))
            result = monitor.update(scene, history)
            row = dict(t=now, source=scene.source_first_s, speed=restored['speed_mps'],
                actual_speed=rec['actual_speed_mps'], pose=pose.tolist(), patch_reason=reason,
                observed_persistence=expected.get('persistence'), recomputed_persistence=result,
                motion_evidence=monitor.motion_evidence(), motion_rate=monitor.motion_rate_mps,
                original_reason=expected.get('reason'), preparation=expected.get('preparation'),
                entry_space=expected.get('entry_space'), pairs=pairs)
            if patch is not None:
                row.update(patch_count=len(patch), patch_median=np.median(patch, axis=0).tolist(),
                           patch_span=np.ptp(patch, axis=0).tolist(), patch=patch.tolist())
            if last is not None and patch is not None and last['patch'] is not None:
                measured = transport_points(last['patch'], history, last['time'], now)
                exact = truth_transport(last['patch'], last['pose'], pose)
                row['successive'] = dict(
                    measured_m=float(np.linalg.norm(np.median(patch, axis=0)-np.median(measured, axis=0))),
                    truth_m=float(np.linalg.norm(np.median(patch, axis=0)-np.median(exact, axis=0))),
                    transport_error_max_m=float(np.linalg.norm(measured-exact, axis=1).max()))
            rows.append(row)
            last = dict(source=scene.source_first_s, time=now, pose=pose, patch=patch)
    (out/'details.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding='utf-8')
    resets = [r for r in rows if r['motion_evidence'] and r['recomputed_persistence'].get('sample_count') == 1]
    compact = [{k:r[k] for k in ('t', 'speed', 'actual_speed', 'pose', 'patch_count', 'patch_median',
                                  'patch_span', 'motion_rate', 'pairs', 'successive') if k in r} for r in resets]
    report = dict(scope='saved-input decomposition only; truth used only for evaluation transport',
        fresh_scans=len(rows), motion_reset_events=compact, hashes=hashes,
        source_unchanged=all(digest(ROOT/p)==value for p,value in hashes.items()))
    (out/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(fresh_scans=len(rows), resets=compact, source_unchanged=report['source_unchanged']), ensure_ascii=False))


if __name__ == '__main__':
    main()
