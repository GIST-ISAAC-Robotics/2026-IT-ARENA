"""10/29 왼쪽 시험 기구를 복사해 중앙 표적 1회 시험을 실행 전에 고정한다. 주행 없음.

바꾼 것은 표적 횡편이 0, 시험 창 20초, g2_corridor_policy=corridor_v1, 쪽 판정의 중앙 처리, 평가 경로뿐이다.
월드·launch 사본·신호·하네스 패치·독립 보호·가상 MCU·차량/센서 물성·G2 판정선은 그대로다.
"""
import hashlib
import json
from pathlib import Path
import shutil

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
LEFT=ROOT/'artifacts/validation/2026-10-29/g2_entry_left_v1'


def text(path):
    return Path(path).read_bytes().decode('utf-8').replace('\r\n','\n')


def patched(name,pairs,target=None):
    content=text(LEFT/name)
    for old,new in pairs:
        if content.count(old)<1:
            raise ValueError(f'{name}: {old[:60]}')
        content=content.replace(old,new)
    with (BASE/(target or name)).open('x',encoding='utf-8',newline='\n') as stream:
        stream.write(content)


shutil.copytree(LEFT/'fixture',BASE/'fixture')
manifest=json.loads((BASE/'fixture/manifest.json').read_text(encoding='utf-8'))
manifest.update(scope='G2 explicit center-target trial launch copy; world is the unchanged v2 signal fixture',
    case='g2_center_stopped',duration_s=20.)
manifest['target']['lateral_offset_m']=0.
(BASE/'fixture/manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

config=json.loads((LEFT/'g2_config_v2_left.json').read_text(encoding='utf-8'))
config.update(registered_at='2026-10-30 before the single G2 center-target Gazebo run',
    scope=('G2 explicit sensor entry trial with corridor_v1: one stationary 20x15x12 cm box on the road center, ego on the '
           'centerline of the official start straight, official 4-red visual start, rendered sequential C1, independent LiDAR '
           'safety, virtual MCU. Limited simulation assumption test. Not race default, team/vehicle risk acceptance, left '
           'corridor, cut-in, biased range, multi-vehicle or hardware evidence.'),
    corridor_assumptions=dict(policy='corridor_v1',observed_rate_term_mps=.225,onset_acceleration_mps2=3.,snap_budget_m=.03,
        note='0.225 m/s considers average-change error over 0.4 s only; it is not a physical bound on instantaneous or future speed. '
             '3 m/s^2 is a test assumption. 30 mm alignment is an exploratory assumption, not deterministic margin preservation.'))
case=config['cases'][0]
case.update(name='g2_center_stopped',route_kind='straight',duration_s=20.,actor_lateral_offset_m=0.,
    expected=('G2 corridor_v1 enters the wider measured corridor, passes the stationary center target and returns to the centre. '
              'The preserved G1 stopped_center gates are expected to fail by design.'))
config['acceptance']['g2_center_stopped']=config['acceptance'].pop('g2_edge_stopped_left')
g2=config['acceptance']['g2']
g2['version']='g2_acceptance_v2_center'
g2['bypass_side_rule']='center target has no prescribed side: either side counts, the observed side is recorded'
g2['trace_header_expected']['execution_config']['corridor_policy']='corridor_v1'
(BASE/'g2_center_config_v1.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

patched('run.py',[
    ('"""G2 명시 진입 왼쪽 가장자리 1회 Gazebo 시험 어댑터(g2_entry_gazebo_v1/run.py 복사본).','"""G2 corridor_v1 중앙 표적 1회 Gazebo 시험 어댑터(g2_entry_left_v1/run.py 복사본).'),
    ("바꾼 것: config(g2_config_v2_left.json)·case·평가기(g2_eval_v2)·범위 문구.","바꾼 것: config(g2_center_config_v1.json)·case·g2_corridor_policy·평가기 쪽 판정·범위 문구."),
    ("CONFIG=BASE/'g2_config_v2_left.json'","CONFIG=BASE/'g2_center_config_v1.json'"),
    ("CASE='g2_edge_stopped_left'","CASE='g2_center_stopped'"),
    ("settings=dict(g1_follow_enabled=True,g2_entry_simulation_trial=True,g2_trace_path=str(output/'g2_decisions.jsonl'),",
     "settings=dict(g1_follow_enabled=True,g2_entry_simulation_trial=True,g2_corridor_policy='corridor_v1',g2_trace_path=str(output/'g2_decisions.jsonl'),"),
    ("paths.add(BASE.parent/'g2_entry_gazebo_v1'/'g2_eval.py')","paths.add(ROOT/'artifacts/validation/2026-10-29/g2_entry_gazebo_v1/g2_eval.py')"),
    ("scope=('single G2 explicit entry trial: LEFT-edge stationary box (run1 mirror), ego on centerline, official signal v2 fixture, '",
     "scope=('single G2 corridor_v1 trial: center stationary box, ego on centerline, official signal v2 fixture, '"),
    ("g2_execution_defaults=header.get('execution_config')==asdict(BypassExecutionConfig()),",
     "g2_execution_defaults=header.get('execution_config')==asdict(BypassExecutionConfig(corridor_policy='corridor_v1')),"),
    ("g2_node_started='G2 explicit sensor entry trial' in log,",
     "g2_node_started='G2 explicit sensor entry trial' in log and 'G2 corridor test policy corridor_v1' in log,"),
])
patched('g2_eval_v2.py',[
    ("RUN1=BASE.parent/'g2_entry_gazebo_v1'","RUN1=ROOT/'artifacts/validation/2026-10-29/g2_entry_gazebo_v1'"),
    ("gates['target_lateral_sign_matches_case']=bool(np.sign(target_lateral)==np.sign(case['actor_lateral_offset_m']))",
     "gates['target_lateral_sign_matches_case']=bool(abs(target_lateral-case['actor_lateral_offset_m'])<=.005)"),
])
patched('g2_eval_core.py',[
    ("""    if pass_t is None or target_lateral_m==0:return dict(expected=None,observed=None,matches=False)
    expected='left' if target_lateral_m<0 else 'right'
""","""    if pass_t is None:return dict(expected=None,observed=None,matches=False)
    if target_lateral_m==0:
        # 중앙 표적은 쪽을 미리 정하지 않는다. 실제 통과 쪽만 기록한다.
        sample=next(r for r in rows if r['t']>=pass_t)
        return dict(expected=None,observed='left' if sample['lateral']>0 else 'right',matches=True)
    expected='left' if target_lateral_m<0 else 'right'
"""),
])
patched('preflight.py',[
    ("g2_config_v2_left.json","g2_center_config_v1.json"),
    ("run1=BASE.parent/'g2_entry_gazebo_v1'","run1=ROOT/'artifacts/validation/2026-10-29/g2_entry_gazebo_v1'"),
    ("target_on_left=bool(target_lateral>0)","target_on_left=bool(abs(target_lateral)<=.005)"),
    ("and result['target_on_left'] and result['fixture_launch_identical_to_run1'] and not control_changed and not control_added\n        and not other_changed)",
     "and result['target_on_left'] and result['fixture_launch_identical_to_run1'])"),
])
print('prepared center trial files')
print({p.name:hashlib.sha256(p.read_bytes()).hexdigest()[:12] for p in sorted(BASE.glob('*.py'))})
