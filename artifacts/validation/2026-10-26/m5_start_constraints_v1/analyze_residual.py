"""저장 입력의 마지막 회전에서 원판 초기 간섭과 별도 노면 거절을 분리한다. 제어 실행 없음."""
import hashlib
import json
import math
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def main():
    inputs = [HERE/'run7/data/report.json', HERE/'run8/data/report.json',
              ROOT/'src/arena_description/config/b0183_c1.yaml']
    raw = {str(p.relative_to(ROOT)): p.read_bytes() for p in inputs}
    observed, compared = [json.loads(p.read_text(encoding='utf-8')) for p in inputs[:2]]
    sensor_path = str(inputs[2].relative_to(ROOT)).replace('\\', '/')
    assert hashlib.sha256(inputs[2].read_bytes()).hexdigest() == observed['source_sha256'][sensor_path]
    lidar = inputs[2].read_text(encoding='utf-8').split('\nlidar:\n')[1].split('\nimu:')[0]
    sx, sy, _ = map(float, re.search(r'xyz_m: \[([^]]+)\]', lidar).group(1).split(','))
    mask = float(re.search(r'masked_rear_half_angle_deg: ([\d.]+)', lidar).group(1))
    out = []
    for case in observed['cases']:
        if case['slot'] != 5 or case['fault_selected']:
            continue
        matched = next(c for c in compared['cases'] if c['slot'] == case['slot'] and
                       c['bias'] == case['bias_m'] and c['delay'] == case['additional_delay_s'] and not c['fault'])
        calls = matched['comparisons'][1]['calls'][-5:]
        scan = case['scans'][-1]
        rows = []
        for call in calls:
            candidates = []
            for i, distance in enumerate(scan['ranges']):
                angle = -math.pi+i*math.tau/500
                if abs(angle) > math.radians(180-mask)+1e-12 or not isinstance(distance, (int, float)):
                    continue
                if not math.isfinite(distance):
                    continue
                x, y = sx+distance*math.cos(angle), sy+distance*math.sin(angle)
                age = call['t_s']-(scan['first_s']+i*.0002)
                # 거리 오차 .03 + 물체 이동 .5*age. 기계 여유 .025는 직사각형에 유지.
                rectangle_distance = math.hypot(max(abs(x)-.125, 0), max(abs(y)-.1, 0))
                radius = .03+.5*age
                candidates.append(dict(index=i, point_xy_m=[x, y], age_s=age,
                    rectangle_distance_m=rectangle_distance, disk_radius_m=radius,
                    initial_signed_gap_m=rectangle_distance-radius))
            witness = min(candidates, key=lambda r: r['initial_signed_gap_m'])
            meta = call['meta']
            checks = meta.get('m3_lateral', {}).get('checks', {})
            rows.append(dict(t_s=call['t_s'], reason=meta['reason'],
                reported_clearance_m=meta.get('swept_clearance_m'), witness=witness,
                plan_rejections={name: c.get('reasons') for name, c in checks.items()}))
        out.append(dict(slot=case['slot'], bias_m=case['bias_m'], delay_s=case['additional_delay_s'], calls=rows))
    report = dict(scope='stored last scan diagnostic, no driving or new controller execution',
        inputs_sha256={name: hashlib.sha256(data).hexdigest() for name, data in raw.items()},
        input_unchanged=all((ROOT/name).read_bytes() == data for name, data in raw.items()), cases=out)
    path = HERE/'residual_analysis.json'
    with path.open('x', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
        f.write('\n')
    print(json.dumps([dict(bias=c['bias_m'], delay=c['delay_s'],
        initial_gap_mm=[round(r['witness']['initial_signed_gap_m']*1000, 3) for r in c['calls']],
        reasons=[r['reason'] for r in c['calls']]) for c in out], ensure_ascii=False))


if __name__ == '__main__':
    main()
