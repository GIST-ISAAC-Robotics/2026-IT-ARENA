"""동일 조건 r6 두 실행의 시작 반복과 전달/물리 변동을 별도로 판정한다."""
import argparse
import json
from pathlib import Path

from analyze_initial_startup import analyze_run, PLUGIN_SHA, ROOT
from analyze_apply_trace import analyze_run as trace_run, classify, divergence
from analyze_apply_trace_v2 import first_application_difference
from analyze_scheduled_phase import compare_metrics


def startup_pair(a, b):
    fields = ('source_sha256', 'meshes_sha256', 'profile', 'spawn_pose',
              'runtime_vehicle_sha256', 'command_start_ns')
    checks = {'distinct_runs': a['name'] != b['name'],
              'both_embedded': a['mode'] == b['mode'] == 'embedded_paused',
              'both_valid': a['passed'] is True and b['passed'] is True,
              'initial_commands_zero': all(
                  (r.get('startup') or {}).get('first_update', {}).get('speed') == 0
                  and (r.get('startup') or {}).get('first_update', {}).get('steering') == 0 for r in (a,b)),
              'same_world': a['shape']['world_without_vehicle_sha256'] == b['shape']['world_without_vehicle_sha256'],
              **{'same_'+k: a[k] == b[k] for k in fields},
              'first_step_expected': a['trace']['spawn'] == b['trace']['spawn'] ==
                  {'iteration': 1, 'sim_ns': 1_000_000, 'dt_ns': 1_000_000}}
    return {'checks': checks, 'bounded_initialization_repeat_confirmed': all(checks.values()),
            'scope': '유효한 두 실행의 초기 배치 반복. 물리 결과 결정성이나 모든 환경의 재현성을 뜻하지 않는다.'}


def stage_differences(a, b):
    """같은 프로필 값과 짝지어진 전환만 비교한다. 누락을 0차이로 채우지 않는다."""
    out = {}
    for channel in ('speed', 'steering'):
        ca, cb = a[channel], b[channel]
        stages_a, stages_b = ca['stages_rel_start_ns'], cb['stages_rel_start_ns']
        out[channel] = {}
        for stage in ('request', 'request_observed', 'safe', 'mcu', 'output'):
            if channel == 'steering' and stage == 'output':
                out[channel][stage] = {'comparable': False, 'reason': '출력 진단에 조향각 필드 없음'}
                continue
            x, y = stages_a.get(stage), stages_b.get(stage)
            comparable = (isinstance(x, list) and isinstance(y, list) and
                          len(x) == len(y) == ca['events'] == cb['events'] and len(x) > 0)
            record = {'comparable': comparable}
            if comparable:
                diffs = [q-p for p,q in zip(x,y)]
                record.update(delta_b_minus_a_ns=diffs, differing=sum(d != 0 for d in diffs),
                              max_abs_diff_ns=max(abs(d) for d in diffs),
                              first_difference_index=next((i for i,d in enumerate(diffs) if d), None))
            else:
                record['reason'] = '전환 개수/값 대응 누락 또는 불일치'
            out[channel][stage] = record
    return out


def application_rows(a, b):
    out = {}
    for channel in ('speed', 'steering'):
        x, y = a[channel], b[channel]
        comparison = first_application_difference(a,b)[channel]
        if not comparison['comparable']:
            out[channel] = {'comparable': False}
            continue
        out[channel] = {'comparable': True, 'transitions': [
            {'index': i, 'value': p['to'], 'a_apply_rel_ns': p['apply_rel_ns'], 'b_apply_rel_ns': q['apply_rel_ns'],
             'apply_delta_ns': q['apply_rel_ns']-p['apply_rel_ns'],
             'rx_after_delta_ns': q['rx_after_rel_ns']-p['rx_after_rel_ns'],
             'steps_after_rx': [p['steps_after_rx'], q['steps_after_rx']]}
            for i,(p,q) in enumerate(zip(x,y))]}
    return out


def analyze_pair(paths, contract):
    if paths[0].resolve() == paths[1].resolve():
        raise ValueError('two distinct executions required')
    initial = [analyze_run(p,contract) for p in paths]
    traces = [trace_run(p,contract,PLUGIN_SHA) for p in paths]
    # trace_run v1의 완화된 출처 검사는 판정 근거로 삼지 않는다.
    # initial의 run_result는 v2 필수 소스/원시 무결성·STOP·종료 감사를 수행한다.
    a,b = [t[0] for t in traces]
    init = startup_pair(*initial)
    same_start = initial[0]['command_start_ns'] == initial[1]['command_start_ns']
    application = first_application_difference(a['trace'],b['trace'])
    div = divergence(traces[0][1],traces[1][1],initial[0]['command_start_ns']) if same_start else None
    metrics,within,secondary = compare_metrics(a,b)
    comparable = init['bounded_initialization_repeat_confirmed'] and same_start
    return {'runs': initial, 'startup_pair': init,
            'application_comparison_valid': comparable,
            'stage_differences': stage_differences(a['delivery'],b['delivery']),
            'delivery': [a['delivery'],b['delivery']],
            'applications': application, 'application_rows': application_rows(a['trace'],b['trace']),
            'divergence': div,
            'classification': classify(True,application,div) if comparable else 'not_comparable',
            'metrics': metrics, 'metrics_within_tolerance': within, 'secondary_metrics': secondary,
            'limits': ['두 실행은 통계적 분포가 아니다. 인과 확정이 아니라 관측 순서의 비교다.',
                       '출력 시각은 ROS 진단 stamp, 수신 시각은 직전 PreUpdate 경계. 정확한 DDS/Gazebo 전송 지연은 아니다.',
                       '임시 수동 제동 근사 결과이며 MD20A/실차나 G1 주행 개선 증거가 아니다.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if len(args.run) != 2: parser.error('exactly two runs required')
    contract = json.loads((ROOT/'config/tests/scheduled_command_r6_sha256.json').read_text())
    result = analyze_pair(args.run,contract)
    with args.output.open('x',encoding='utf-8') as f: json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps({k:result[k] for k in ('startup_pair','application_comparison_valid','stage_differences',
                                         'applications','divergence','classification','metrics_within_tolerance')},
                     ensure_ascii=False,indent=2))
