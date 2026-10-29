"""재관측 후보 소스의 이전 바이트와 후속 변경을 연결한다. 과거 계약은 수정하지 않는다."""
import hashlib,json,zipfile,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];BASE=Path(__file__).resolve().parent
sha=lambda b:hashlib.sha256(b).hexdigest()
parent='config/tests/local_start_signal_r0_sha256.json'
reasons={
 'src/arena_autonomy/arena_autonomy/partial_observation.py':'기본 계산 동일 protected hook; 새 후보는 별도 계산.',
 'src/arena_autonomy/arena_autonomy/local_follow.py':'기본false 교통 회복·선행 계약과 독립 정책 생성.',
 'src/arena_autonomy/arena_autonomy/lidar_safety.py':'명시 v4 회복 객체를 독립 생성. STOP/보호 출력 유지.',
 'src/arena_autonomy/arena_autonomy/local_pursuit.py':'명시 시뮬레이션 선택 옵션을 FollowConfig와 기록에 전달.',
 'src/arena_autonomy/arena_autonomy/pass_release_trial.py':'후보ON 통과 이력·같은 경로 검사와 같은 호출 계산 재사용.',
 'src/arena_autonomy/arena_autonomy/rear_fringe_trial.py':'원래1mm 조건을 유지하며 공통 무반환/점유 검사를 분리.',
 'src/arena_bringup/launch/demo.launch.py':'기본false 선택형 옵션 전달.',
 'src/arena_bringup/launch/local_pursuit.launch.py':'기본false 선택형 옵션 전달.',
 'scripts/validate_g1_traffic.py':'교통 회복 명시CLI·선행 검사·소스/선택 기록. 기존 평가선 유지.',
 'scripts/validate_sensor_input_ros.py':'실제ROS 고장 시험의 회복 옵션과 소스 보존.'}
archives=[
 ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_replay_v1/source.zip',
 ROOT/'artifacts/validation/2026-10-27/official_departure_v2/dense5_local_uniform30_run1/source_snapshot.zip',
 ROOT/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_source_snapshot.zip']
latest={};ref=parent
while ref:
 c=json.loads((ROOT/ref).read_text(encoding='utf-8'))
 for group in ('approved_changes','approved_evaluation_scripts','changes'):
  for name,row in c.get(group,{}).items():latest.setdefault(name,row['after'])
 for name,digest in c.get('new_dependencies',{}).items():latest.setdefault(name,digest)
 ref=c.get('parent_contract')
rows={};origins={};before={}
for path,reason in reasons.items():
 for archive in archives:
  with zipfile.ZipFile(archive) as z:
   if path not in z.namelist():continue
   data=z.read(path)
  if path in latest and sha(data)!=latest[path]:continue
  before[path]=data;origins[path]=archive.relative_to(ROOT).as_posix();break
 else:raise ValueError('previous version not found: '+path)
 rows[path]=dict(before=sha(before[path]),after=sha((ROOT/path).read_bytes()),reason=reason)
 assert rows[path]['before']!=rows[path]['after']
if '--check-only' in sys.argv:
 print('previous bytes matched',len(rows),'files',json.dumps(origins));raise SystemExit(0)
with zipfile.ZipFile(BASE/'before.zip','x',zipfile.ZIP_DEFLATED) as z:
 for name,data in before.items():z.writestr(name,data)
(BASE/'before_origins.json').write_text(json.dumps(origins,indent=2),encoding='utf-8')
names=['recovery_geometry','traffic_recovery_trial','surface_anchor_trial','same_path_gap_trial']
new={f'src/arena_autonomy/arena_autonomy/{name}.py':sha((ROOT/f'src/arena_autonomy/arena_autonomy/{name}.py').read_bytes()) for name in names}
contract=dict(schema_version=1,scope='선택형 교통 회복 출처. 기본 활성화 또는 실물 안전 승인 아님',
 parent_contract=parent,parent_contract_sha256=sha((ROOT/parent).read_bytes()),
 before_snapshot=(BASE/'before.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha((BASE/'before.zip').read_bytes()),
 approved_changes={k:v for k,v in rows.items() if k.startswith('src/')},
 approved_evaluation_scripts={k:v for k,v in rows.items() if k.startswith('scripts/')},new_dependencies=new)
target=ROOT/'config/tests/traffic_recovery_trial_r0_sha256.json'
with target.open('x',encoding='utf-8') as f:json.dump(contract,f,ensure_ascii=False,indent=2);f.write('\n')
for name in ('test_corner_follow.py','test_g1_dense_stop.py','test_physics_step_option.py','test_source_provenance_r0_plus_b3.py'):
 p=ROOT/'tests'/name;s=p.read_text(encoding='utf-8')
 old="'local_start_signal_r0_sha256.json')";new="'local_start_signal_r0_sha256.json', 'traffic_recovery_trial_r0_sha256.json')"
 if name=='test_source_provenance_r0_plus_b3.py':
  old="'config/tests/local_start_signal_r0_sha256.json')";new="'config/tests/local_start_signal_r0_sha256.json', 'config/tests/traffic_recovery_trial_r0_sha256.json')"
  s=s.replace('    name = M3_SUCCESSORS[-1]', "    name = 'config/tests/local_start_signal_r0_sha256.json'")
  s=s.replace("    assert child['parent_contract'] == M3_SUCCESSORS[-2]", "    assert child['parent_contract'] == 'config/tests/short_boundary_prefix_r0_sha256.json'")
 assert old in s,name;s=s.replace(old,new);p.write_text(s,encoding='utf-8')
print('registered',len(rows),'changes and',len(contract['new_dependencies']),'new dependencies; prior snapshots preserved')
