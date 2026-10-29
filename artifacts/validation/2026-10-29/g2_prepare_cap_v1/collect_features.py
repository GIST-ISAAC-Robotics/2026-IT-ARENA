"""기존 교통1랩 저장 입력의 전방 반환 조각 특징을 모은다. 새 주행/제어 입력 아님.

현재 연결(run3과 같은 코드)을 같은 입력으로 재생하면서, 매 호출의 센서 장면에서
전방 조각·두 벽 직선 기준·G1 현재 원호와의 관계·연속 스캔 이동량을 계산한다.
평가용 Gazebo 자세(ego/actor)는 **사후 분류에만** 쓰며 제어기에 전달하지 않는다.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import zipfile

import numpy as np
from shapely.geometry import Point, Polygon

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.bypass_handoff import BypassHandoffFollow, front_patch
from arena_autonomy.bypass_entry_trial import straight_reference
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_path import swept_limit
from arena_autonomy.follow_trace import restore_call, freeze, comparable_decision, equal_output
from arena_autonomy.scene_motion import motion_scene, transport_points

RUN = ROOT/'artifacts/validation/2026-10-28/bump_traffic_v1/run4'
INPUT = RUN/'follow_decisions.jsonl'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Track:
    """[t, x, y, yaw] 평가 기록의 최근접 보간. 분류 전용."""
    def __init__(self, rows):
        self.rows = rows
        self.times = [r[0] for r in rows]

    def at(self, t):
        i = bisect.bisect_left(self.times, t)
        if i <= 0:
            return self.rows[0]
        if i >= len(self.rows):
            return self.rows[-1]
        a, b = self.rows[i-1], self.rows[i]
        w = (t-a[0])/(b[0]-a[0]) if b[0] > a[0] else 0.
        yaw = a[3]+w*math.atan2(math.sin(b[3]-a[3]), math.cos(b[3]-a[3]))
        return [t, a[1]+w*(b[1]-a[1]), a[2]+w*(b[2]-a[2]), yaw]

    def speed(self, t, span=.1):
        a, b = self.at(t-span), self.at(t)
        return math.hypot(b[1]-a[1], b[2]-a[2])/span


def box_in_ego(ego, actor, length=.2, width=.15):
    c, s = math.cos(ego[3]), math.sin(ego[3])
    corners = []
    for dx, dy in ((length/2, width/2), (length/2, -width/2), (-length/2, -width/2), (-length/2, width/2)):
        wx = actor[1]+math.cos(actor[3])*dx-math.sin(actor[3])*dy
        wy = actor[2]+math.sin(actor[3])*dx+math.cos(actor[3])*dy
        rx, ry = wx-ego[1], wy-ego[2]
        corners.append((c*rx+s*ry, -s*rx+c*ry))
    return Polygon(corners)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='features1')
    args = parser.parse_args()
    out = BASE/args.output
    out.mkdir(exist_ok=False)
    sources = [p for p in (ROOT/'src').rglob('*.py') if '__pycache__' not in p.parts]+[Path(__file__)]
    hashes = {p.relative_to(ROOT).as_posix(): digest(p) for p in sources}
    with zipfile.ZipFile(out/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in sources:
            archive.write(p, p.relative_to(ROOT))
    ego = Track([json.loads(line) for line in (RUN/'ego_poses.jsonl').open(encoding='utf-8')])
    lead = Track([json.loads(line) for line in (RUN/'actor_poses.jsonl').open(encoding='utf-8')])
    extra = json.loads((RUN/'additional_actors.json').read_text(encoding='utf-8'))
    side = Track(extra['poses']['eval_e3_side'])
    scans = []  # (source_first_s, t_ref, face_median) 분석용 연속 스캔 이력
    matched = mismatched = calls = 0
    began_all = time.perf_counter()
    with INPUT.open(encoding='utf-8') as source, (out/'features.jsonl').open('x', encoding='utf-8') as detail:
        for line in source:
            event = json.loads(line)
            kind, data = event['kind'], event['data']
            if kind == 'header':
                if data['schema'] != 1:
                    raise ValueError('expected preserved G1 input')
                ctl = BypassHandoffFollow(FollowConfig(**data['follow_config']), allow_simulation_trial=True)
                motion = MotionConfig(**data['motion_config'])
                config = ctl.config
                continue
            if kind == 'reset':
                ctl.reset(); scans = []
                continue
            if kind != 'call':
                continue
            calls += 1
            restored, scan, history = restore_call(data['input'], motion)
            now = restored['reference_s']
            try:
                scene = motion_scene(scan, history, now, timing_verified=restored['timing_verified'],
                    epoch=restored['epoch'], rear_half_angle_deg=restored['rear_half_angle_deg'])
            except (MotionUnavailable, ValueError) as error:
                ctl.reset(); scans = []
                detail.write(json.dumps(dict(seq=event['seq'], t=now, scene_error=str(error)))+'\n')
                continue
            fresh = not scans or scene.source_first_s != scans[-1][0]
            try:
                actual = freeze(ctl.command(scene, history, restored['speed_mps'], restored['side'],
                    restored['previous_steering_rad'], restored['dt_s'], raw_scan=scan))
            except (MotionUnavailable, ValueError) as error:
                # replay_call과 같은 처리. 이 입력에서는 발생하지 않아야 한다.
                ctl.reset(); scans = []
                detail.write(json.dumps(dict(seq=event['seq'], t=now, command_error=str(error)))+'\n')
                mismatched += 1
                continue
            expected = data['output']
            d = actual[2]
            normal = d.get('normal') or {}
            if normal and equal_output(comparable_decision([expected[0], expected[1], normal]),
                                       comparable_decision(expected)):
                matched += 1
            else:
                mismatched += 1
            row = dict(seq=event['seq'], t=now, fresh=fresh, g1=expected[0], g1_steer=expected[1],
                out=actual[0], reason=d.get('reason'), measured=restored['speed_mps'],
                previous=restored['previous_steering_rad'], age=now-scene.source_first_s,
                g1_reason=normal.get('reason'), g1_selector=normal.get('selector_mode'),
                g1_clearance=normal.get('swept_clearance_m'), persistence=d.get('persistence'))
            face, patch_reason = front_patch(scene)
            row['patch_reason'] = patch_reason
            if face is not None:
                median = np.median(face, axis=0)
                row.update(face_x=float(median[0]), face_y=float(median[1]), face_rays=len(face),
                    face_y_span=float(np.ptp(face[:, 1])), face_x_min=float(face[:, 0].min()))
                arc = min(swept_limit(face, angle, config.wheelbase_m, horizon=2.5,
                    margin=.025+config.measurement_padding_m, length=config.length_m, width=config.width_m)
                    for angle in (expected[1], restored['previous_steering_rad']))
                row['face_arc_clearance'] = float(arc)
                # 직전 스캔들의 조각 중앙값을 측정 운동으로 현재 시각에 이송한 이동량.
                motions = {}
                for back in (1, 2, 3):
                    if len(scans) >= back and scans[-back][2] is not None:
                        old_ref, old_source, old_face = scans[-back][1], scans[-back][0], scans[-back][2]
                        try:
                            moved = transport_points(old_face[None, :], history, old_ref, now)[0]
                            motions[str(back)] = dict(dx=float(median[0]-moved[0]), dy=float(median[1]-moved[1]),
                                dt=float(scene.source_first_s-old_source))
                        except (ValueError, MotionUnavailable):
                            pass
                row['face_motion'] = motions
            reference, evidence = straight_reference(scene, config)
            row['straight_ok'] = reference is not None
            row['straight_reason'] = evidence.get('reason')
            # 평가용 정답: 사후 분류만. 제어기 입력이 아니다.
            truth_ego = ego.at(now)
            labels = {}
            for name, track in (('lead', lead), ('side', side)):
                pose = track.at(now)
                polygon = box_in_ego(truth_ego, pose)
                center = polygon.centroid
                labels[name] = dict(x=center.x, y=center.y, speed=track.speed(now),
                    face_gap=(polygon.distance(Point(row['face_x'], row['face_y'])) if face is not None else None))
            row['truth'] = labels
            row['truth_ego_speed'] = ego.speed(now, .05)
            if fresh:
                scans.append((scene.source_first_s, now, np.median(face, axis=0) if face is not None else None))
                scans = scans[-4:]
            elif face is not None:
                # 같은 원시 스캔의 재사용은 같은 관측이다. 이송 기준 시각만 최신으로 둔다.
                scans[-1] = (scans[-1][0], now, np.median(face, axis=0))
            detail.write(json.dumps(row, allow_nan=False)+'\n')
    report = dict(input=str(INPUT.relative_to(ROOT)), input_sha256=digest(INPUT), calls=calls,
        normal_detail_matched=matched, normal_detail_mismatched=mismatched,
        source_sha256=hashes, source_unchanged=all(digest(ROOT/p) == v for p, v in hashes.items()),
        seconds=time.perf_counter()-began_all,
        scope='Feature collection on stored G1 trajectory with current handoff code. Evaluation poses are post-hoc labels only.')
    (out/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('calls', 'normal_detail_matched', 'normal_detail_mismatched', 'source_unchanged', 'seconds')}))


if __name__ == '__main__':
    main()
