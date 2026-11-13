"""후속 GUARD 저장 비교의 원자료 추출만 수행. 후보 계산/ROS 실행 없음."""
import collections
import hashlib
import json
import math
from pathlib import Path
import zipfile

B = Path(__file__).resolve().parent
P, O = B / 'run1', B / 'analysis_v1'
rows = [json.loads(x) for x in (P / 'follow_decisions.jsonl').read_text(encoding='utf-8').splitlines()]
calls = [r for r in rows if r['kind'] == 'call']
trajectory = [json.loads(x) for x in (P / 'trajectory.jsonl').read_text(encoding='utf-8').splitlines()]
additional = json.loads((P / 'additional_actors.json').read_text(encoding='utf-8'))
selected = []
for r in calls:
    d = r['data']
    if not 33.92 <= d['input']['reference_s']:
        continue
    n = d['output'][2]['normal']
    cand = n.get('selector_guard_candidate', {})
    record = dict(seq=r['seq'], original_jsonl_line=r['seq'] + 1,
                  reference_sim_s=d['input']['reference_s'], measured_speed_mps=d['input']['speed_mps'],
                  previous_steering_rad=d['input']['previous_steering_rad'],
                  plan_output_speed_mps=d['output'][0], plan_reason=n.get('reason'),
                  selector_reason=n.get('selector_reason'), mode=n.get('selector_mode'),
                  choice=n.get('selector_choice'), sides=n.get('selector_sides'), evidence=n.get('selector_evidence'),
                  original_candidate=cand, lower_cap_attempts=n.get('selector_guard_speed_recheck'),
                  current_approach_recorded='current_approach' in cand,
                  scope='원 측정/제어 결과 추출, 새 후보 계산 없음')
    selected.append(record)
(O / 'guard_original_rows.json').write_text(json.dumps(selected, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

evaluation_only = []
for target in (34.12, 44.42):
    ego = min(trajectory, key=lambda row: abs(row['t'] - target))
    actors = [('primary_lead', ego['actor_stamp_s'], ego['actor_x'], ego['actor_y'])]
    actors += [(name, *min(poses, key=lambda pose: abs(pose[0] - ego['t']))[:3])
               for name, poses in additional['poses'].items()]
    positions = []
    for name, stamp, x, y in actors:
        dx, dy = x - ego['x'], y - ego['y']
        c, s = math.cos(ego['yaw']), math.sin(ego['yaw'])
        positions.append(dict(name=name, actor_sim_s=stamp, x=x, y=y,
            center_distance_m=math.hypot(dx, dy), body_forward_m=c * dx + s * dy,
            body_left_m=-s * dx + c * dy))
    evaluation_only.append(dict(target_reference_sim_s=target, ego_sim_s=ego['t'],
        ego={k: ego[k] for k in ('x', 'y', 'yaw')}, actors=positions))

with zipfile.ZipFile(B / 'sources.zip') as z:
    source = z.read('src/arena_autonomy/arena_autonomy/pass_release_trial.py')
    lines = source.decode('utf-8').splitlines()
    (O / 'guard_full_check_frozen.excerpt.txt').write_text('\n'.join(
        f'{i}: {line}' for i, line in enumerate(lines, 1) if 377 <= i <= 555) + '\n', encoding='utf-8')

result = dict(
    scope='Windows 원자료/동결 소스 읽기. 원본 및 공통 수정 없음, 새 주행/저장 후보 재평가 없음.',
    original_rows='guard_original_rows.json', row_count=len(selected),
    guard_entry=dict(seq=891, reference_sim_s=33.92, before='SOLO', after='GUARD',
        sides=next(r for r in selected if r['seq'] == 891)['sides']),
    final_sides=selected[-1]['sides'],
    measured_speed_counts=dict(collections.Counter(str(r['measured_speed_mps']) for r in selected)),
    additional_approach=dict(
        original_candidate_values='각 원본 행의 original_candidate.guarded_sides',
        lower_cap_values_saved=False,
        reason='기존 lower_cap_attempts는 target/accepted/reason/region만 기록. .25/.15의 guarded_sides 수치는 미기록.',
        current_approach_evaluated_in_final_stall=False,
        reason_for_not_evaluated='동결 소스 530–532행 region_blocked에서 반환하여 533–543행 observed_guard_approach에 도달하지 않음.',
        implication='0.1에서 mask 통과하더라도 양측 누적/추가접근·현재 관측 검사를 그대로 통과해야 함.'),
    evaluator_truth_only=dict(samples=evaluation_only,
        inference='두 평가용 actor 모두 인접 측방 차량 위치와 떨어져 있음. 마지막 양측 present=true는 단순 과거 기억만은 아님. 센서 위협 표본이 실제 actor인지 벽/다른 반환인지 이 자료만으로 개체 동일성 확정 불가.',
        prohibited_use='정답 위치를 제어 입력/side 상태 제거/자유공간 증거로 사용하지 않음.'),
    replay_feasibility=dict(
        raw_scan_history_reference_epoch_speed_steering_available=True,
        full_header_and_frozen_source_available=True,
        original_trace_footer=rows[-1],
        standalone_selector_state_snapshot_available=False,
        reason='snapshot_call은 센서/운동 이력 입력이며 GUARD anchors/sides/누적 selector 내부 상태의 독립 snapshot이 아님.',
        minimum_reconstruction='동결 native plan Evaluator로 선행 call/reset 순서를 재생하여 원래 .35/.25/.15 출력과 GUARD 상태의 동일성을 먼저 확인. 해당 호출 내부의 실제 args/state에서 단일 .1 비교만 분리.',
        unproven='아직 실행하지 않아 baseline exact 재현과 worker 버림/순서의 완전성은 확인되지 않음. seen/present/passed/vanish나 anchor를 임의 설정하면 원 입력 재현으로 취급할 수 없음.'),
    bounded_next_comparison=dict(
        worth_one_saved_comparison=True,
        basis='104개 연속 정체가 unobserved/front mask 0이고 마지막 .15 꼬리가 1 cm를 조금 초과. 정지근처에서 더 짧은 목표의 기하 비교는 유의미하나 통과 보장 없음.',
        required_invariants=['원 .35/.25/.15 계산과 순서 보존', '원 실측속도/scan/reference/epoch/조향/기하/1 cm 유지',
                             '동일 GUARD full-region 및 양측 누적/추가접근 검사', 'reference_not_straight와 위협 상태 그대로'],
        extra_cost='추가 .1은 GUARD 전체 검사를 1회 더 계산하므로 실패 시 순수 추가 비용. r441 plan max 55.44 wall ms만으로 .1 추가 후 deadline을 보장할 수 없음.',
        authority='새 주행 또는 공통 구현 승인으로 확대하지 않음.'),
    frozen_source_sha256=hashlib.sha256(source).hexdigest(),
    executed_candidate_replays=0)
(O / 'guard_followup.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps({k: result[k] for k in ('row_count', 'guard_entry', 'final_sides', 'measured_speed_counts', 'evaluator_truth_only')}, ensure_ascii=False, indent=2))
