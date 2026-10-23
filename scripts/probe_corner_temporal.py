#!/usr/bin/env python3
"""회랑 shadow의 서로 다른 스캔 확인. 런타임 경계 기억/명령 발행에 연결하지 않는다."""
import argparse
from collections import Counter
import json
from pathlib import Path
import zipfile

import numpy as np

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, path_agrees
from arena_autonomy.scene_motion import motion_scene, transport_points
from probe_corner_corridor import shadow_scene, digest, ROOT


class TemporalCornerShadow:
    """매 호출 현재 기하를 요구한다. 과거 경로만 남은 상태에서는 진행 가설도 거절한다."""
    def __init__(self, config=None):
        self.config = config or FollowConfig()
        self.reset()

    def reset(self):
        self.key = self.last_ref = self.last_source = None
        self.path = self.path_ref = self.path_source = None
        self.confirmations = self.release_count = 0

    def clear_evidence(self):
        self.path = self.path_ref = self.path_source = None
        self.confirmations = self.release_count = 0

    def observe(self, scene, history, speed, side, previous_steering, dt):
        now, source, key = scene.t_ref_s, scene.source_first_s, (scene.source_epoch,side)
        reset_cause = None
        if self.key is not None and key != self.key:
            reset_cause = 'epoch_or_side_changed'
            self.reset()
        elif (self.last_ref is not None and now < self.last_ref) or (
                self.last_source is not None and source is not None and source < self.last_source):
            self.reset()
            self.key, self.last_ref, self.last_source = key, now, source
            return dict(temporal_candidate=False, reason='reference_or_source_regressed',
                        fresh=False, confirmations=0, release_count=0, proposed_speed_mps=0.,
                        driving_authorized=False, runtime_connected=False)
        fresh = source is not None and (self.last_source is None or source > self.last_source)
        self.key, self.last_ref, self.last_source = key, now, source
        proposal = shadow_scene(scene, speed, side, previous_steering, dt, self.config)
        detail = dict(temporal_candidate=False, reason=proposal['reason'], fresh=fresh,
                      geometry_supported=proposal['geometry_supported'],
                      current_point_occupancy_supported=proposal['preliminary_motion_supported'],
                      proposed_speed_mps=0., source_first_s=source, reference_s=now,
                      swept_clearance_m=proposal.get('swept_clearance_m'),
                      confirmations=0, release_count=0, reset_cause=reset_cause,
                      driving_authorized=False, runtime_connected=False, memory_refresh_authorized=False,
                      old_path_only_allowed=False, wall_identity_certified=False)
        if not proposal['geometry_supported'] or proposal['reason'] == 'reverse_motion':
            self.clear_evidence()
            return detail
        path = np.asarray(proposal['geometry']['path'])
        compatible = False
        prior_age = None if self.path_source is None else now-self.path_source
        transport_error = None
        displacement = None
        if self.path is not None and 0 <= prior_age < self.config.memory_ttl_s:
            try:
                transported = transport_points(self.path, history, self.path_ref, now)
                displacement = float(np.max(np.linalg.norm(transported-self.path,axis=1)))
                compatible = (displacement <= self.config.memory_motion_limit_m and
                              path_agrees(path, transported, self.config.boundary_agreement_m))
            except (ValueError, MotionUnavailable) as error:
                transport_error = str(error)
        detail.update(previous_source_age_s=prior_age, previous_path_agrees=bool(compatible),
                      transport_displacement_m=displacement, transport_error=transport_error)
        if not fresh and not compatible:
            # 같은 스캔을 다시 맞춘 경로가 갑자기 바뀌어도 이전 확인 횟수를 쓰지 않는다.
            self.clear_evidence()
            return {**detail, 'reason':'reused_scan_path_unconfirmed'}
        if fresh:
            self.confirmations = self.confirmations+1 if compatible else 1
            if not compatible:
                self.release_count = 0
            self.path, self.path_ref, self.path_source = path.copy(), now, source
            if self.confirmations >= self.config.confirmations and proposal['preliminary_motion_supported']:
                self.release_count += 1
        if not proposal['preliminary_motion_supported']:
            self.release_count = 0
        detail.update(confirmations=self.confirmations, release_count=self.release_count)
        if self.confirmations < self.config.confirmations:
            detail['reason'] = 'distinct_geometry_confirmation_wait'
        elif not proposal['preliminary_motion_supported']:
            detail['reason'] = proposal['reason']
        elif self.release_count < self.config.confirmations:
            detail['reason'] = 'distinct_motion_release_wait'
        else:
            detail.update(temporal_candidate=True, reason='confirmed_shadow_candidate_only',
                          proposed_speed_mps=proposal['proposed_speed_mps'], steering_rad=proposal['steering_rad'])
        return detail


def inspect_events(events):
    controller = motion = None
    rows = []
    for event in events:
        kind, data = event['kind'], event['data']
        if kind == 'header':
            controller = TemporalCornerShadow(FollowConfig(**data['follow_config']))
            motion = MotionConfig(**data['motion_config'])
        elif kind == 'reset':
            controller.reset()
        elif kind == 'call':
            inputs, scan, history = restore_call(data['input'], motion)
            try:
                scene = motion_scene(scan, history, inputs['reference_s'], timing_verified=inputs['timing_verified'],
                                     epoch=inputs['epoch'], rear_half_angle_deg=inputs['rear_half_angle_deg'])
                result = controller.observe(scene, history, inputs['speed_mps'], inputs['side'],
                                            inputs['previous_steering_rad'], inputs['dt_s'])
            except (ValueError, MotionUnavailable) as error:
                controller.reset()
                result = dict(temporal_candidate=False, reason='input_rejected', error=str(error),
                              proposed_speed_mps=0., driving_authorized=False, fresh=False)
            rows.append(dict(seq=event['seq'], time_s=inputs['reference_s'], original_speed_mps=data['output'][0],
                             original_reason=data['output'][2]['reason'], shadow=result))
    return rows


def baseline_matches(report, input_path):
    if (report.get('scope') != 'fixed_input_stateless_shadow_not_driving_policy' or
            not report.get('input_unchanged') or not report.get('source_unchanged') or
            report.get('runtime_changed') is not False or report['input_sha256'] != digest(input_path)):
        raise ValueError('baseline provenance mismatch')
    for path, expected in report['source_sha256'].items():
        # 보존 파일에 대한 조회이며 임의 경로 실행/쓰기에는 쓰지 않는다.
        resolved = (ROOT/path).resolve()
        if ROOT not in resolved.parents or digest(resolved) != expected:
            raise ValueError(f'baseline source differs: {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--baseline-shadow', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    baseline = json.loads(args.baseline_shadow.read_text())
    baseline_matches(baseline, args.input)
    input_hash = digest(args.input)
    baseline_hash = digest(args.baseline_shadow)
    files = list(baseline['source_sha256'])+['scripts/probe_corner_temporal.py','tests/test_corner_temporal_shadow.py']
    sources = {p:digest(ROOT/p) for p in files}
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in files:
            archive.write(ROOT/p,p)
    with args.input.open() as stream:
        rows = inspect_events(json.loads(line) for line in stream)
    summary = dict(calls=len(rows), fresh_calls=sum(r['shadow']['fresh'] for r in rows),
                   temporal_candidate_calls=sum(r['shadow']['temporal_candidate'] for r in rows),
                   positive_during_original_hold=sum(r['original_speed_mps']==0 and r['shadow']['temporal_candidate'] for r in rows),
                   reasons=dict(Counter(r['shadow']['reason'] for r in rows)))
    report = dict(scope='fixed_input_temporal_shadow_not_closed_loop', summary=summary, rows=rows,
                  reused_baseline_calls=baseline['summary']['baseline_calls_matched'],
                  baseline_reexecution=False, baseline_sha256=baseline_hash, input_sha256=input_hash,
                  source_sha256=sources, source_unchanged=all(digest(ROOT/p)==v for p,v in sources.items()),
                  input_unchanged=digest(args.input)==input_hash and digest(args.baseline_shadow)==baseline_hash,
                  runtime_changed=False, driving_authorized=False, wall_identity_certified=False,
                  closed_loop_verified=False)
    with (args.output/'report.json').open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({**summary,'source_unchanged':report['source_unchanged'],'input_unchanged':report['input_unchanged']}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] else 2


if __name__=='__main__':
    raise SystemExit(main())
