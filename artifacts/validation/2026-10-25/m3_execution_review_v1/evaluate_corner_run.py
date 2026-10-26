"""등록된 40 s 코너 연결 시험의 동일 구간 비교. 원본 판정/제어 입력은 바꾸지 않는다."""
import argparse
import hashlib
import json
from pathlib import Path
from corner_window_eval import stop_restart_window, after_window


def evaluate(run):
    names = ('report.json', 'trace.json', 'ego_poses.jsonl', 'actor_poses.jsonl', 'follow_decisions.jsonl')
    before = {name: hashlib.sha256((run/name).read_bytes()).hexdigest() for name in names}
    report, metrics, gates = stop_restart_window(run, 'config/tests/g1_corner_after_threat_v1.json',
                                                'stop_restart_first_curve', 15.)
    assert report['case'] == 'stop_restart_first_curve'
    t = report['evaluation']['t_green']
    result = {'original_passed': report['passed'], 'original_run_valid': report['run_valid'],
              'stop_restart_15s': {'metrics': metrics, 'gates': gates},
              'after_15s': after_window(run, t+15., t+40., report), 'input_sha256': before}
    result['inputs_unchanged'] = all(hashlib.sha256((run/name).read_bytes()).hexdigest() == h for name, h in before.items())
    assert result['inputs_unchanged']
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = evaluate(args.run)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps(result, ensure_ascii=False))
