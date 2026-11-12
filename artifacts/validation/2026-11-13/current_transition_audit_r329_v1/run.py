"""r325의 양수 계획 전이 거부 사유만 기록한다. 정책·시한·구동을 바꾸지 않는다."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
BASE = A.parent/'current_deadline_r325_v1'
sys.path.insert(0, str(BASE))
spec = importlib.util.spec_from_file_location('r325_timing', BASE/'timed_replay.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
import priority

original = priority.may_finish_current
rows = []


def summary(plan):
    if plan is None:
        return None
    normal = plan['output'][2].get('normal', {})
    return dict(generation=plan['generation'], sequence=plan['sequence'],
        reference_s=plan['input']['reference_s'], speed=plan['output'][0], steering=plan['output'][1],
        state=plan['output'][2].get('handoff_state'), reason=plan['output'][2].get('reason'),
        path_source_s=normal.get('path_source_s'),
        normal={k: normal.get(k) for k in ('mode', 'preferred_wall', 'path_wall', 'selector_mode', 'selector_choice')})


def audited(old, new):
    accepted = original(old, new)
    a, b = summary(old), summary(new)
    causes = []
    if a is None or b is None:
        causes.append('no_active_or_new_plan')
    else:
        if a['generation'] != b['generation']:
            causes.append('generation')
        if a['speed'] > b['speed']:
            causes.append('new_speed_lower')
        if a['speed'] <= 0 or b['speed'] <= 0:
            causes.append('nonpositive_speed')
        for key in a['normal']:
            if not isinstance(a['normal'][key], str) or a['normal'][key] != b['normal'][key]:
                causes.append('normal.'+key)
        if a['normal']['selector_mode'] != 'SOLO' or a['normal']['selector_choice'] not in ('baseline', 'parallel'):
            causes.append('not_solo_baseline_or_parallel')
        if any(x['state'] != 'FOLLOW' or x['reason'] != 'handoff_follow' for x in (a, b)):
            causes.append('handoff_state_or_reason')
        if old['input']['epoch'] != new['input']['epoch'] or old['input']['side'] != new['input']['side']:
            causes.append('motion_epoch_or_side')
        if runner.source_identity(old['input'])[0] != runner.source_identity(new['input'])[0]:
            causes.append('sensor_epoch')
    rows.append(dict(accepted=accepted, causes=causes, old=a, new=b))
    return accepted


def main():
    priority.may_finish_current = audited
    sys.argv = [str(BASE/'timed_replay.py'), '--policy', 'deadline', '--mode', 'delay20',
        '--submission', 'latest', '--duration', '3', '--plan-ms', '0', '--current-ms', '0',
        '--trace', str(BASE/'selected_trace.jsonl'), '--out', str(A/'jetson_diagnostic')]
    try:
        runner.main()
    finally:
        priority.may_finish_current = original
        (A/'transitions.json').write_text(json.dumps(rows, indent=2)+'\n')


if __name__ == '__main__':
    main()
