"""38.3–38.9 s 스캔의 전방 무반환 광선을 팀 월드 상자 기하와 대조(평가 전용, 정답 자세 사용).

11/2 solo_observation_diagnosis_v1/inspect_rays.py 방식을 이번 C1 위치(x 0.06, 광축 0.075)로 옮겼다.
자세는 ego_poses.jsonl(약 1 kHz [t,x,y,yaw]), 광선 시각은 stamp + i·time_increment. 상자 visual만 보며
렌더 메시·자차 기하·구형 광원은 재현하지 않는다. 제어 입력이 아니다.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
RUN = HERE.parent/'run1'
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf'
REGISTERED = json.loads((HERE.parent/'registered.json').read_bytes())['input_sha256']
SENSOR_X, SENSOR_Z = .06, .0655+.0125   # 차체 링크 정답 z(0.0655) + (광축 0.075 − 링크 0.0625)
WINDOW = (38.3, 38.9)


def rotation(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def transform(text):
    v = list(map(float, (text or '0 0 0 0 0 0').split()))
    m = np.eye(4)
    m[:3, :3] = rotation(*v[3:])
    m[:3, 3] = v[:3]
    return m


def boxes():
    found = []

    def visit(node, parent):
        here = parent
        if node.tag in ('model', 'link', 'visual'):
            pose = node.find('pose')
            assert pose is None or not pose.get('relative_to'), 'relative frame unsupported'
            here = parent@transform(node.findtext('pose'))
        if node.tag == 'visual':
            box = node.find('geometry/box/size')
            if box is not None:
                found.append((node.get('name'), here[:3, 3], here[:3, :3], np.array(list(map(float, box.text.split())))/2))
            return
        for child in node:
            if child.tag in ('sdf', 'world', 'model', 'link', 'visual'):
                visit(child, here)
    visit(ET.parse(WORLD).getroot(), np.eye(4))
    return [v[0] for v in found], np.array([v[1] for v in found]), np.array([v[2] for v in found]), np.array([v[3] for v in found])


def main():
    world_hash = hashlib.sha256(WORLD.read_bytes()).hexdigest()
    labels, centers, rotations, half = boxes()
    poses = np.array([json.loads(x) for x in (RUN/'ego_poses.jsonl').read_text(encoding='utf-8').splitlines() if x.strip()])
    yaw = np.unwrap(poses[:, 3])
    traj = [json.loads(x) for x in (RUN/'trajectory.jsonl').read_text(encoding='utf-8').splitlines() if x.strip()]
    tt, ss = np.array([r['time'] for r in traj]), np.array([r['progress_m'] for r in traj])
    seen, out, residuals = set(), [], []
    with (RUN/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        next(stream)
        for line in stream:
            e = json.loads(line)
            if e['kind'] != 'call' or not WINDOW[0] <= e['data']['input']['reference_s'] <= WINDOW[1]:
                continue
            scan = e['data']['input']['scan']
            first = scan['sec']+scan['nanosec']*1e-9
            if first in seen:
                continue
            seen.add(first)
            raw = np.array([float(v['nonfinite']) if isinstance(v, dict) else v for v in scan['ranges']])
            angles = scan['angle_min']+np.arange(len(raw))*scan['angle_increment']
            unknown = (~np.isfinite(raw)) | (raw < scan['range_min']) | (raw > scan['range_max'])
            wrapped = (angles+np.pi) % (2*np.pi)-np.pi
            front = np.abs(wrapped) <= math.radians(100)
            ids = np.flatnonzero(unknown & front)
            neighbors = sorted({n for i in ids for n in (i-1, i+1) if 0 <= n < len(raw) and not unknown[n]})
            rays = []
            for index in sorted(set(ids) | set(neighbors)):
                t = first+index*scan['time_increment']
                x, y, h = (np.interp(t, poses[:, 0], poses[:, 1]), np.interp(t, poses[:, 0], poses[:, 2]),
                           np.interp(t, poses[:, 0], yaw))
                origin = np.array([x+SENSOR_X*math.cos(h), y+SENSOR_X*math.sin(h), SENSOR_Z])
                direction = np.array([math.cos(h+angles[index]), math.sin(h+angles[index]), 0.])
                lo = np.einsum('ni,nij->nj', origin-centers, rotations)
                ld = np.einsum('i,nij->nj', direction, rotations)
                with np.errstate(divide='ignore', invalid='ignore'):
                    a, b = (-half-lo)/ld, (half-lo)/ld
                    enter, leave = np.max(np.minimum(a, b), axis=1), np.min(np.maximum(a, b), axis=1)
                dist = np.where((leave >= np.maximum(enter, 0)) & (leave > 0), np.maximum(enter, 0), np.inf)
                best = int(np.argmin(dist))
                hit = float(dist[best])
                row = dict(index=int(index), body_angle_deg=round(math.degrees(wrapped[index]), 2), unknown=bool(unknown[index]),
                           raw=None if unknown[index] else round(float(raw[index]), 4),
                           expected_first_box_m=None if not math.isfinite(hit) else round(hit, 4),
                           expected_label=labels[best] if math.isfinite(hit) else None)
                if unknown[index]:
                    row['class'] = 'beyond_12m' if hit > scan['range_max'] else 'box_within_12m'
                else:
                    row['residual_m'] = round(float(raw[index]-hit), 4) if math.isfinite(hit) else None
                    if math.isfinite(hit):
                        residuals.append(abs(raw[index]-hit))
                rays.append(row)
            out.append(dict(source_first_s=round(first, 3), progress_m=round(float(np.interp(first, tt, ss)), 2),
                            front_unknown=len(ids), rays=rays))
    summary = dict(scans=len(out), unknown_rays=sum(s['front_unknown'] for s in out),
                   beyond_12m=sum(r.get('class') == 'beyond_12m' for s in out for r in s['rays']),
                   box_within_12m=sum(r.get('class') == 'box_within_12m' for s in out for r in s['rays']),
                   neighbor_residual_median_m=float(np.median(residuals)) if residuals else None,
                   neighbor_residual_max_m=float(np.max(residuals)) if residuals else None,
                   world_sha256=world_hash, world_matches_registration=REGISTERED.get(
                       'src/arena_gazebo/worlds/it_arena_official/world.sdf') == world_hash)
    (HERE/'raycheck.json').write_text(json.dumps(dict(scope=__doc__.strip(), summary=summary, scans=out), ensure_ascii=False, indent=1),
                                      encoding='utf-8', newline='\n')
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    main()
