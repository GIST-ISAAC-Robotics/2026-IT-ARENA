"""대표 155초 교통 각본과 신호는 평가기에만 연결한다."""
import json
from pathlib import Path
from prelap_signal_eval import ROOT, fixture_inputs, sequence_evidence, digest

BASE=ROOT/'artifacts/validation/2026-11-01/mixed_course_v1'
ENTRY=BASE/'course.launch.py'
CONFIG=BASE/'course.json'
PRIOR=ROOT/'artifacts/validation/2026-10-31/g1_continuous_prefix_course_v1'
OVERLAY=PRIOR/'actor_overlay.json'
PARENT=ROOT/'config/tests/g1_traffic_full_course_v1.json'
ROLLING=BASE.parent/'rolling_lead_v1'


def entry_for_output(output):
    """두 등록 교통 시험에서만 신호를 사용한다. 짧은 코너 시험은 영상 녹화기를 띄우지 않는다."""
    parent=Path(output).resolve().parent
    if parent==BASE:return ENTRY
    if parent==ROLLING:
        from prelap_signal_eval import ENTRY as signal_entry
        return signal_entry
    raise ValueError('등록되지 않은 2×5 교통 시험 출력 경로')


def _rolling_inputs(args,config,case):
    import importlib.util
    source=ROLLING/'course.py'
    spec=importlib.util.spec_from_file_location('registered_rolling_lead',source)
    registered=importlib.util.module_from_spec(spec);spec.loader.exec_module(registered)
    if (args.output.resolve().parent!=ROLLING or args.mode!='g1g2' or args.actor_overlay is not None
            or args.seed!=0 or not args.record_follow_decisions or args.case!='first_curve'
            or config!=registered.expected() or case!=config['cases'][0]):
        raise ValueError('움직이는 앞차 시험은 기존 출구·간격·17초 각본과 소스 기록을 유지해야 한다')
    inputs=fixture_inputs()
    for p in (source,registered.CONFIG,registered.PARENT,Path(__file__),entry_for_output(args.output)):
        inputs[p.relative_to(ROOT).as_posix()]=digest(p)
    return inputs


def inputs(args,config,case):
    if args.config.resolve()==ROLLING/'course.json':
        return _rolling_inputs(args,config,case)
    if (args.config.resolve()!=CONFIG or args.actor_overlay is None or args.actor_overlay.resolve()!=OVERLAY
            or args.mode!='g1g2' or args.seed!=0 or not args.record_follow_decisions
            or args.output.resolve().parent!=BASE or args.case!='stop_restart_full_course'):
        raise ValueError('대표 교통 시험에는 지정 코스·옆차·판단 기록이 필요하다')
    parent=json.loads(PARENT.read_text(encoding='utf-8'))
    old=json.loads((PRIOR/'traffic_config.json').read_text(encoding='utf-8'))
    expected=next(x for x in parent['cases'] if x['name']==args.case)
    if (old['cases']!=[expected] or config['cases']!=[expected] or config['acceptance']!=parent['acceptance']
            or config['actor']!=old['actor'] or config['seeds']!=old['seeds']
            or case['duration_s']!=155. or len(case['additional_actors'])!=1
            or config['parent_config']['sha256']!=digest(PARENT)):
        raise ValueError('대표 각본·상대·판정 기준이 원본과 다름')
    result=fixture_inputs()
    paths=[CONFIG,OVERLAY,PARENT,PRIOR/'record_chase.py',PRIOR/'traffic_config.json',ENTRY,Path(__file__)]
    result.update({p.relative_to(ROOT).as_posix():digest(p) for p in paths})
    return result


def signal(output,labels,trace):
    return sequence_evidence(output,labels,[dict(time=r['t'], autonomy=r['autonomy']) for r in trace])
