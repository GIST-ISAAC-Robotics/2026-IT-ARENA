"""폐루프 비교 1회의 사후 판정(평가 전용). 정답 자세는 지표 계산에만 쓰며 제어기 입력이 아니다.

python -I -B review.py --output review.json
- 비교 지표: 상위 compare.py의 metrics()를 기준 run1과 이 run1에 같은 정의로 적용한다.
- 런타임 확인: 판단 기록 머리말 continuous_forward_pace, 실행 인자, report.json source_sha256의 r7/r8 after 일치.
- 앞차 이동: actor_poses 창 안 변위와 1 s 구간 속도(각본 0.30 m/s 계속 이동인지).
- 판정선은 registered.json acceptance 그대로이며 완화·재판정하지 않는다.
"""
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
spec = importlib.util.spec_from_file_location('compare', HERE.parent/'compare.py')
compare = importlib.util.module_from_spec(spec); spec.loader.exec_module(compare)
CONTRACTS = ['config/tests/lidar_mount_structure_gate_r7_sha256.json', 'config/tests/continuous_forward_pace_r8_sha256.json']


def header(run):
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as f:
        first = json.loads(f.readline())
    assert first['kind'] == 'header'
    return first['data']['execution_config']


def actor_motion(run, report):
    tg, te = report['case_window']['green_sim_s'], report['case_window']['end_sim_s']
    rows = compare.jl(run/'actor_poses.jsonl'); ts = [r[0] for r in rows]
    p = lambda t: compare.at(rows, ts, t)
    seg = []
    t = tg
    while t+1. <= te+1e-9:
        a, b = p(t), p(t+1.)
        seg.append(math.hypot(b[0]-a[0], b[1]-a[1]))
        t += 1.
    a, b = p(tg), p(te)
    return dict(window_displacement_m=math.hypot(b[0]-a[0], b[1]-a[1]), one_second_speeds_mps=[round(v, 4) for v in seg],
                min_one_second_speed_mps=min(seg), max_one_second_speed_mps=max(seg))


def runtime(run, report, reg):
    src = report['source_sha256']
    contracts = {}
    for c in CONTRACTS:
        d = json.loads((ROOT/c).read_text(encoding='utf-8'))
        files = {**d.get('approved_changes', {}), **d.get('approved_evaluation_scripts', {})}
        contracts[c] = {p: src.get(p) == v['after'] for p, v in files.items()}
    cfg = header(run)
    return dict(header_continuous_forward_pace=cfg.get('continuous_forward_pace'),
                header_observed_forward_pace=cfg.get('observed_forward_pace'),
                launch_argument='evaluation_handoff_continuous_pace:=true' in report['handoff_launch_arguments'],
                handoff_execution_header_check=report['handoff_execution']['checks'].get('header'),
                contract_after_match=contracts,
                registered_sources_match={p: src.get(p) == v for p, v in reg['sources'].items() if p in src})


def main(argv):
    reg = json.loads((HERE/'registered.json').read_text(encoding='utf-8'))
    base_run, run = ROOT/reg['baseline']['run'], HERE/'run1'
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    base_report = json.loads((base_run/'report.json').read_text(encoding='utf-8'))
    base, new = (compare.metrics(r, r.parent/(r.name+'_surface.json')) for r in (base_run, run))
    rt = runtime(run, report, reg)
    act, base_act = actor_motion(run, report), actor_motion(base_run, base_report)
    valid = bool(new['run_valid'] and new['harness_valid'] and new['source_unchanged'] and all(new['stop_checks'].values()))
    runtime_ok = bool(rt['header_continuous_forward_pace'] is True and rt['launch_argument'] and rt['handoff_execution_header_check']
                      and all(all(v.values()) for v in rt['contract_after_match'].values()))
    surf = new['surface'] or {}
    gates = bool(all(new['gates'].values()) and new['contact_messages'] == 0 and surf.get('body_outside_samples') == 0
                 and surf.get('wall_wheel_samples') == 0)
    actor_moving = bool(act['min_one_second_speed_mps'] >= .25)
    improvement = dict(
        below_time=new['after3']['below_0p10_time_s'] < base['after3']['below_0p10_time_s'],
        below_episodes=new['after3']['below_0p10_episodes'] < base['after3']['below_0p10_episodes'],
        command_below=new['command_after3']['below_0p10'] < base['command_after3']['below_0p10'])
    out = dict(scope='평가 전용 사후 판정. 판정선 완화·재판정 없음. 추월 기능 아님.',
               registration_sha256=hashlib.sha256((HERE/'registered.json').read_bytes()).hexdigest(),
               runtime=rt, actor_motion=dict(baseline=base_act, trial=act), baseline=base, trial=new,
               verdict=dict(valid=valid, runtime_option_confirmed=runtime_ok, gates_contact_surface=gates,
                            actor_kept_moving=actor_moving, improvement=improvement,
                            accepted=bool(valid and runtime_ok and gates and actor_moving and all(improvement.values()))))
    text = json.dumps(out, indent=1, ensure_ascii=False)+'\n'
    if argv[:1] == ['--output']:
        Path(argv[1]).write_text(text, encoding='utf-8', newline='\n')
    print(json.dumps(out['verdict'], ensure_ascii=False))


if __name__ == '__main__':
    main(sys.argv[1:])
