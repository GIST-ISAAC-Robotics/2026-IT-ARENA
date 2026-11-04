"""기존 정지 강제 평가를 보존하고 관측 우회/두 바퀴 노면의 별도 기능 판정을 낸다.

새 기준의 보조 분석이다. 원래 report.json을 다시 쓰거나 실패를 소급 성공 처리하지 않는다.
"""
import hashlib
import json
from pathlib import Path


def evaluate(report, trace, surface):
    evaluation = report['evaluation']
    original = evaluation['gates']
    result = dict(scope='supplementary continuous-response assessment; original verdict unchanged',
                  original_passed=report['passed'], original_gates=original,
                  policy='hold_or_observed_bypass_two_wheels_v2',
                  not_proven=['independent opponent','real hardware','all traffic situations'])
    # 차체 정답은 이 파일의 사후 평가에서만 사용한다. 기존 접촉·간격·진행 기준은 그대로다.
    gates = {k: v for k, v in original.items() if k not in ('hold','inside_road')}
    gates['run_valid'] = report['run_valid'] is True
    stop = report.get('stop_checks', {})
    gates['stop_and_termination'] = bool(stop) and all(value is True for value in stop.values())
    wheels = surface.get('interpretations', {})
    gates['surface_evidence'] = (surface.get('samples', 0) > 0 and
                                 surface.get('sample_gaps_over_limit') == 0 and
                                 surface.get('wall_wheel_samples') == 0)
    gates['at_most_two_wheels_outside'] = (set(wheels) == {'any','center','full'} and
        all(value.get('max_off_wheels', 5) <= 2 for value in wheels.values()))
    if 'hold' in original:
        # 이전 장면과 같은 3–8초 상대 정지 창. 이름만 우회인 정체는 대안으로 인정하지 않는다.
        green = evaluation['t_green']
        active = [r for r in trace if green+3 <= r['t'] <= green+8]
        longest, span, progress, maximum_progress = 0., 0., 0., 0.
        for a, b in zip(active, active[1:]):
            dt = b['t']-a['t']
            valid = (0 < dt <= .2 and a['autonomy'].get('behavior') == 'BYPASS'
                     and b['autonomy'].get('behavior') == 'BYPASS'
                     and a['safe_speed_mps'] > 0 and b['safe_speed_mps'] > 0)
            if valid:
                span += dt
                progress += max(0., b['progress_m']-a['progress_m'])
                longest, maximum_progress = max(longest, span), max(maximum_progress, progress)
            else:
                span, progress = 0., 0.
        bypass = longest >= 1. and maximum_progress >= .20
        gates['held_or_executed_bypass'] = original['hold'] or bypass
        result['alternative_response'] = dict(hold=original['hold'], observed_bypass=bypass,
            continuous_bypass_s=longest, bypass_progress_m=maximum_progress,
            basis='original 1 s response duration, at least one body length (0.20 m) actual route progress; '
                  'contact/gap/rejoin/exit criteria retained')
    result['gates'] = gates
    result['supplementary_passed'] = bool(gates) and all(gates.values())
    return result


def main(run, surface, output):
    paths = dict(report=run/'report.json', trace=run/'trace.json', surface=surface)
    data = {k: json.loads(p.read_bytes()) for k, p in paths.items()}
    result = evaluate(**data)
    result['input_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths.values()}
    with output.open('x', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('surface', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    main(args.run, args.surface, args.output)
