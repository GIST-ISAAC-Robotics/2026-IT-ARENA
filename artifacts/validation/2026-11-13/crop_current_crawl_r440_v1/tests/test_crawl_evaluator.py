"""실제 전체Evaluator를 통한 대표 입력의 소수 합성 반례. 새 주행이 아니다."""
import copy
import json
import math
from pathlib import Path

import pytest
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.race_runtime_policy import crop_current_candidate as crop
from arena_autonomy.runtime_dual_state import current_request

ROOT=Path(__file__).resolve().parents[1]
EVIDENCE=[]

@pytest.fixture
def record():return json.loads((ROOT/'inputs/r437_representative.json').read_bytes())

def evaluate(record,change=None):
    header=json.loads((ROOT/'inputs/r437_header.json').read_bytes())
    data=current_request(record['plan_output'],record['plan_input'],record['current'],retained=record['retained'],crop_current_selection=True)
    if change:change(data)
    original=copy.deepcopy(data);worker=Evaluator('current',header)
    try:result=worker(data)
    finally:worker.close()
    assert data==original
    return data,result

def test_real_clean_evaluator_uses_two_checks(record):
    data,result=evaluate(record);trial=result[crop.DIAGNOSTIC]
    assert result['speed_mps']==.1 and trial['attempted_targets']==2 and trial['final_full_checks']==2
    assert trial['crawl_retry']['applied'] is True
    assert trial['region']['masked_tail_extent_m']<.01 and trial['proof']['accepted']
    EVIDENCE.append(dict(case='unchanged_representative',result=result))

@pytest.mark.parametrize('fault',['front_obstacle','side_obstacle','expanded_unknown','reference_mismatch','epoch_mismatch'])
def test_actual_evaluator_rejects_changed_inputs_with_old_positive_plan(record,fault):
    # 원 plan의 positive/crop 표식은 의도적으로 남긴다. 그것이 새 측정을 대신하지 못해야 한다.
    members=set(record['plan_output'][2]['normal']['partial_observation']['crop_edge_component']['source_indices'])
    changed_indices=[]
    if fault in ('front_obstacle','side_obstacle','expanded_unknown'):
        for snapshot in (record['plan_input'],record['current']):
            raw=snapshot['scan']['measurements']
            for index,angle in enumerate(raw['angles_rad']):
                degrees=math.degrees(angle)
                choose=((fault=='front_obstacle' and abs(degrees)<10)
                    or (fault=='side_obstacle' and 80<degrees<125 and index not in members)
                    or (fault=='expanded_unknown' and index==max(members)+1))
                if choose:
                    raw['ranges_m'][index]={'nonfinite':'inf'} if fault=='expanded_unknown' else .12
                    changed_indices.append(index)
        assert changed_indices
    def changed(data):
        if fault=='reference_mismatch':data['submitted']['reference_s']=data['current']['reference_s']+.01
        if fault=='epoch_mismatch':data['current']['epoch']+=1
    data,result=evaluate(record,changed)
    assert result['speed_mps']==0,(fault,result)
    trial=result.get(crop.DIAGNOSTIC) or {}
    assert not (trial.get('crawl_retry') or {}).get('applied')
    EVIDENCE.append(dict(case=fault,changed_indices=sorted(set(changed_indices)),result=result,
        current_reference_s=data['current']['reference_s'],submitted_reference_s=data['submitted']['reference_s'],
        current_epoch=data['current']['epoch'],submitted_epoch=data['submitted']['epoch']))

@pytest.fixture(scope='session',autouse=True)
def evidence():
    yield
    (ROOT/'unit_result/synthetic_evidence.json').write_text(json.dumps(EVIDENCE,ensure_ascii=False,indent=2),encoding='utf-8')
