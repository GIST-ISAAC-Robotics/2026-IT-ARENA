"""E3 저장 자세의 횡투영 간격 분해. 제어·반사실 주행·새 합격 판정이 아니다."""
import hashlib
import itertools
import json
import math
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
SOURCES = [OUT/'analyze.py', OUT.parent/'e3_onroad_analysis_v1/g1.json',
           OUT.parent/'e3_onroad_geometry_v1/report.json',
           ROOT/'config/tests/e3_onroad_encounter_v1.json', ROOT/'scripts/g1_traffic_eval.py']


def half_span(length, width, yaw):
    return (length*abs(math.sin(yaw)) + width*abs(math.cos(yaw)))/2


def main():
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCES}
    # 식을 독립적인 네 모서리 회전과 대조한다. 측정 오차·속도 모형 검사는 아니다.
    for theta in (0., .1, -.2, math.pi/2, math.pi, -math.pi/3):
        corners_y = [x*math.sin(theta)+y*math.cos(theta)
                     for x,y in itertools.product((-.10,.10), (-.075,.075))]
        assert math.isclose(half_span(.20,.15,theta), (max(corners_y)-min(corners_y))/2, abs_tol=1e-12)
    source = json.loads(SOURCES[1].read_text())
    geometry = json.loads(SOURCES[2].read_text())
    config = json.loads(SOURCES[3].read_text())
    point = source['minimum_at_decision']
    y0, y1, yaw = geometry['ego_initial_lateral_m'], point['ego_lateral_m'], point['yaw_from_road_rad']
    side = config['additional_actors'][0]['lateral_offset_m']
    assert config['additional_actors'][0]['route_kind'] == 'side_road_straight'
    assert y0 < y1 < side and abs(yaw) < math.pi/2
    # 상대차는 등록된 직선 일정의 횡위치/도로 정렬 자세를 쓴다.
    # 아래 signed gap은 도로 횡축 투영의 간격. 양수면 공간 분리 하한이고,
    # 음수여도 종방향 분리·기울어진 모서리 때문에 실제 접촉을 뜻하지 않는다.
    def gap(y, theta):
        return side-y-half_span(.20,.15,theta)-.075
    rows = [dict(case=name, ego_y_m=y, yaw_rad=theta, lateral_projection_gap_m=gap(y,theta))
            for name,y,theta in [('initial_parallel',y0,0.), ('translation_only',y1,0.),
                                  ('yaw_only',y0,yaw), ('observed_translation_and_yaw',y1,yaw)]]
    translation_loss = y1-y0
    rotation_loss = half_span(.20,.15,yaw)-.075
    assert math.isclose(rows[0]['lateral_projection_gap_m']-translation_loss-rotation_loss,
                        rows[-1]['lateral_projection_gap_m'], abs_tol=1e-12)
    report = dict(scope='static lateral projection decomposition, not counterfactual control or a new verdict',
        source_sha256=hashes, formula='gap = y_actor-y_ego-(L*abs(sin(yaw))+W*abs(cos(yaw)))/2-W_actor/2',
        dimensions_m=dict(ego_length=.20,ego_width=.15,actor_width=.15),
        time_s=point['t'], nominal_actor_y_m=side, actor_yaw_from_road_assumed_rad=0., rows=rows,
        translation_loss_m=translation_loss, rotation_loss_m=rotation_loss,
        total_projection_loss_m=translation_loss+rotation_loss,
        original_sampled_polygon_gap_m=point['sampled_gap_m'],
        original_requested_speed_mps=point['decision_speed_mps'],
        original_motor_feedback_mps=point['measured_motor_speed_mps'],
        original_behavior=point['behavior'], geometric_identity_checks=7,
        limitations=['registered straight actor pose, not exact contact reconstruction',
            'four static combinations do not prove dynamically achievable trajectories',
            'no sensor-noise allowance or lateral safety threshold adopted',
            'positive projection gap is a separating-axis lower bound, not general polygon distance'])
    assert all(hashlib.sha256(p.read_bytes()).hexdigest()==hashes[str(p.relative_to(ROOT))] for p in SOURCES)
    with (OUT/'report.json').open('x') as stream:
        json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps({k:report[k] for k in ('time_s','translation_loss_m','rotation_loss_m','total_projection_loss_m',
        'original_sampled_polygon_gap_m','rows')},ensure_ascii=False))


if __name__ == '__main__':
    main()
