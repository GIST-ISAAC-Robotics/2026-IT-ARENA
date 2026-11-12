"""r217 조건을 보존하며 공통 옵션/80초 평가만 사전 준비한다."""
from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;P=A.parent
old=P/'expiry_null_runtime_r217_v1'
for name in ('run.py','assess.py','postprocess.py','launch_prefix.py','delay_worker.py','node_entry.py'):
    assert not (A/name).exists();(A/name).write_bytes((old/name).read_bytes())
def edit(name,before,after):
    p=A/name;s=p.read_text();assert s.count(before)==1,(name,before)
    p.write_text(s.replace(before,after))
edit('launch_prefix.py',"'g1_released_body_gap_trial:=true']", "'g1_released_body_gap_trial:=true', 'g1_expiry_route_reference_trial:=true']")
edit('node_entry.py','from expiry_worker import ExpiryFreshDelivery, ReasonedWorker\n','')
edit('node_entry.py',"    from expiry_worker import worker_entry\n    import arena_autonomy.decision_process as decision_process\n    decision_process._worker=worker_entry\n",'')
edit('node_entry.py',"        node.decision_delivery=ExpiryFreshDelivery(\n            DelayedWorker(ReasonedWorker(node.decision_delivery.worker),node.now_s,timeline),\n            align_first_source=True)",
     "        assert node.follow.config.expiry_route_reference_trial is True\n        assert node.decision_delivery.retain_expired_reference is True\n        node.decision_delivery.worker=DelayedWorker(node.decision_delivery.worker,node.now_s,timeline)")
edit('node_entry.py',"expiry_route_reference='selected_wall_recent_v3_null_guard'", "expiry_route_reference='common_selected_recent_v3'")
edit('run.py','r217: 같은 지연 교통에 r216 None 경로 예외 보완를 명시 선택한 1회.',
     'r219: r218 공통 최근 경로 참조와80초 지연 교통1회.')
edit('run.py',"latest='config/tests/static_side_elapsed_r209_sha256.json'", "latest='config/tests/expiry_reference_r218_sha256.json'")
edit('run.py','explicit r217 delay/reference prefix; common r209 selected;', 'explicit r219 delay only prefix; common r218 selected;')
edit('run.py',"    config['vehicle_and_sensor_unchanged']", "    case=config['cases'][0]\n    assert case['name']=='two_actor_course_lap' and case['duration_s']==60.\n    case['duration_s']=80.\n    case['schedule'][0]['to_s']=80.\n    case['expected']='80초 동일 교통 지속 주행/1랩 확인. 원60초 실패는 보존.'\n    config['vehicle_and_sensor_unchanged']")
edit('run.py',"    (d/'additional_actors.json').write_bytes((BASE/'two_actor_run1/additional_actors.json').read_bytes())",
    "    overlay=json.loads((BASE/'two_actor_run1/additional_actors.json').read_bytes())\n    assert overlay['additional_actors'][0]['schedule'][0]['to_s']==60.\n    overlay['additional_actors'][0]['schedule'][0]['to_s']=80.\n    overlay['scope']='two centerline actors 0.8m/s,80seconds,initial6m apart,no map input'\n    (d/'additional_actors.json').write_text(json.dumps(overlay,indent=2)+'\\n')")
edit('run.py',"baseline='static_side_runtime_r212_v1'", "baseline='expiry_null_runtime_r217_v1', duration_s=80., expiry_reference='common_selected_recent_v3'")
edit('assess.py',"gates.update(delay_profile_registered=", "gates.update(common_expiry_reference_selected=rows[0]['data']['follow_config'].get('expiry_route_reference_trial') is True and any(e['kind']=='expiry_reference_policy' for e in rows),delay_profile_registered=")
record=json.loads((old/'delivery_profile.json').read_bytes())
record['source_sha256'][(old/'delivery_profile.json').relative_to(A.parents[3]).as_posix()]=hashlib.sha256((old/'delivery_profile.json').read_bytes()).hexdigest()
rows=list(record['samples'])
record['samples']=rows+[dict(row,offset_s=row['offset_s']+60.) for row in rows if row['offset_s']<20.]
record['extension']=dict(period_s=60.,duration_s=80.,method='repeat first20s of preserved60s sample, no final-value hold')
record['scope']+='; r219 repeats first20s after60s for preregistered80s window'
(A/'delivery_profile.json').write_text(json.dumps(record,indent=2)+'\n')
print('Prepared80s common selection; unchanged controller/physics/stop criteria.')
