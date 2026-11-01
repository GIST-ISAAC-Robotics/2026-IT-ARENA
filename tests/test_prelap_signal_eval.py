import json
import pytest
from build_experimental_track import obstacle_polygons
from prelap_signal_eval import MANIFEST,STAGES,fixture_inputs,sequence_evidence


def sequence(tmp_path):
    lamps=json.loads(MANIFEST.read_text(encoding='utf-8'))['lamps']
    rows=[]
    for i,stage in enumerate(STAGES):
        red=int(stage[-1]) if stage.startswith('red') else 0
        rows.append(dict(stage=stage,applied_sim_s=float(i+1),receipts=[
            dict(lamp=v['name'],lit=v['column']<red or (v['column']==4 and stage in ('green','green_probe')),
                 parsed=True,executed=True,service_result=True,applied=True) for v in lamps]))
    p=tmp_path/'signal_sequence.jsonl'
    return rows,p


def evaluate(rows,path,first_start):
    path.write_text('\n'.join(json.dumps(r) for r in rows))
    return sequence_evidence(path.parent,{'green_probe','red','green'},
                             [dict(time=first_start,autonomy={'started':True})])


def test_precourse_fixture_matches_registered_lamps_and_marker():
    assert any(k.endswith('fixture/world.sdf') for k in fixture_inputs())


def test_actual_green_after_four_red_groups_and_preprobe(tmp_path):
    rows,p=sequence(tmp_path)
    assert evaluate(rows,p,7.1)['passed']
    assert not evaluate(rows,p,1.2)['passed']  # 순서가 맞아도 첫 초록 시험점등에 출발한 경우
    rows[4]['receipts'][2]['lit']=not rows[4]['receipts'][2]['lit']
    assert not evaluate(rows,p,7.1)['passed']  # 명령 수락과 올바른 광원 상태는 다르다


def test_omitted_red_group_or_missing_lamp_is_not_complete(tmp_path):
    rows,p=sequence(tmp_path)
    assert not evaluate(rows[:3]+rows[4:],p,7.1)['passed']
    rows[-1]['receipts'].pop()
    assert not evaluate(rows,p,7.1)['passed']


def test_tilted_housing_above_car_and_low_or_offset_box_have_different_outcomes(tmp_path):
    # 평면 검증이 상부 구조 때문에 실행되지 않던 실제 회귀. 회전 중심에서 아래로
    # 늘어진 상자까지 단순히 링크 원점 높이만 보고 건너뛰면 안 된다.
    def fixture(z,offset):
        box='<geometry><box><size>.02 .16 .05</size></box></geometry>'
        path=tmp_path/'world.sdf'
        path.write_text(f'<sdf><world><model name="it_arena_track_static"><link name="wall"><pose>0 0 .06 0 0 0</pose>'
            f'<collision>{box}</collision></link><link name="housing"><pose>0 0 {z} 0 .52 0</pose>'
            f'<collision><pose>0 0 {offset} 0 0 0</pose>{box}</collision></link></model></world></sdf>')
        return path
    assert len(obstacle_polygons(fixture(.424,0)))==1
    for z,offset in ((.09,0),(.424,-.35)):
        with pytest.raises(ValueError,match='기울어진 충돌체'):
            obstacle_polygons(fixture(z,offset))
