"""r325와 같은 저장 시간표/실제 계산 두 프로세스. 실물 포트나 구동은 사용하지 않는다."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
BASE = A.parent/'current_deadline_r325_v1'
sys.path.insert(0, str(BASE))
sys.path.insert(0, str(A.parent/'current_deceleration_r334_v1'))
spec = importlib.util.spec_from_file_location('r325_timing', BASE/'timed_replay.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
from deceleration import CurrentDecelerationProcess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('variant', choices=('baseline', 'candidate'))
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    caps = []
    plans = []
    class Audited(CurrentDecelerationProcess):
        def collect(self):
            rows = super().collect()
            for row in rows:
                if row['stage'] == 'plan':
                    normal = row['output'][2].get('normal', {})
                    plans.append(dict(reference_s=row['input']['reference_s'], speed=row['output'][0],
                        mode=normal.get('selector_mode'), state=row['output'][2].get('handoff_state'),
                        deferred=row.get('deferred_positive_transition', False)))
            return rows
        def prepared(self, **state):
            result = super().prepared(**state)
            if result and result['kind'] == 'candidate':
                row = result['result']
                cap = row['output'].get('new_plan_deceleration_cap')
                if cap:
                    caps.append(dict(**cap, current_sequence=row['sequence'],
                        final_speed_mps=row['output']['speed_mps'],
                        current_reference_s=row['output']['reference_s'], now_s=state['now_s']))
            return result
    runner.CurrentDeadlineProcess = Audited
    sys.argv = [str(BASE/'timed_replay.py'), '--policy', 'deadline' if args.variant == 'candidate' else 'new_only',
        '--mode', 'delay20', '--submission', 'latest', '--duration', '31', '--plan-ms', '0', '--current-ms', '0',
        '--trace', str(A/'selected_trace.jsonl'), '--out', args.out]
    try:
        runner.main()
    finally:
        destination = Path(args.out)
        if destination.is_dir():
            (destination/'caps.json').write_text(json.dumps(caps, indent=2)+'\n')
            (destination/'plans.json').write_text(json.dumps(plans, indent=2)+'\n')


if __name__ == '__main__':
    main()
