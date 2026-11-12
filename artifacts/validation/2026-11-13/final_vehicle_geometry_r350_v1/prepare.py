"""확정 CAD의 주행 기하를 명시 후보로 준비한다. 공용 기본 설정은 그대로 둔다."""
import copy
import hashlib
import json
from pathlib import Path
import yaml

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
SOURCE=ROOT/'outputs/cad_final_2026_11_13'


def main():
    source=json.loads((SOURCE/'source.json').read_bytes())
    assert source['sha256']=='c9c203b5a66585ccde71313ea888c16804a1ddb71e6e91514a46bebededbfce6'
    rows=json.loads((SOURCE/'assembly.json').read_bytes())
    rear=[next(r for r in rows if r['id']==i)['bbox'] for i in (10,11)]
    tire_widths=[b[3]-b[0] for b in rear]
    track=abs(sum(rear[0][::3])/2-sum(rear[1][::3])/2)
    assert all(abs(w-26.1)<1e-5 for w in tire_widths)
    assert abs(track-123.9)<.001
    baseline=ROOT/'src/arena_description/config/vehicle.yaml'
    before=yaml.safe_load(baseline.read_bytes())
    candidate=copy.deepcopy(before)
    v=candidate['vehicle']
    v['status']='cad_20261113_geometry_candidate_mass_tires_mount_not_measured'
    v['footprint'].update(status='symmetric_planning_envelope_about_axle_midpoint_not_rule_size',
        length_m=.207,width_m=.15,
        source_bbox_m=[-.0962,.1035,-.075,.075],
        note='실제 CAD 전후 비대칭을 덮는 계획/보호용 대칭 상자. 검차 길이207mm라는 뜻이 아니다. 최종 마커/배선 외곽은 미측정.')
    v['body'].update(length_m=.1997,width_m=.15,geometry_center_x_m=.00365,
        height_and_mass_status='old_box_height_and_2kg_mass_proxy_retained_not_final_CAD_inertia')
    v['drivetrain'].update(wheelbase_m=.128,wheelbase_status='user_final_CAD_128mm_installed_unmeasured',
        track_width_m=.1239,track_width_status='rear_tire_bbox_centers_CAD_123_89985mm_rounded',
        wheel_width_m=.0261,wheel_width_status='CAD_tire_bbox_26_1mm',
        geometry_source_sha256=source['sha256'])
    # 공칭65mm 구동 환산·질량·마찰·토크·제동·조향 한계는 이번 치수 반영으로 바꾸지 않는다.
    v['cad_geometry_provenance']=dict(source_sha256=source['sha256'],
        frame='X=-c-64 Y=21.2-a Z=b+32.2 mm; axle midpoint ground origin',
        effective_rolling_radius_measured=False,steering_limit_measured=False,
        installed_mass_inertia_measured=False,full_CAD_collision_mesh=False)
    output=A/'vehicle.final.yaml'
    assert not output.exists()
    output.write_text('# 선택형 확정 CAD 주행 기하. 질량/접지/조향 한계는 실측 전 기존 근사.\n'+
                     yaml.safe_dump(candidate,allow_unicode=True,sort_keys=False),encoding='utf-8')
    (A/'geometry.json').write_text(json.dumps(dict(source=source['sha256'],
        rear_track_mm=track,rear_tire_widths_mm=tire_widths,
        wheelbase_m=.128,wheel_width_m=.0261,nominal_wheel_diameter_m=.065,
        body_box_size_m=[.1997,.15],body_box_x_m=.00365,
        symmetric_planning_envelope_m=[.207,.15],
        baseline_sha256=hashlib.sha256(baseline.read_bytes()).hexdigest(),
        controller_truth_input=False,default_changed=False),ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':main()
