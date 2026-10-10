#!/usr/bin/env python3
"""등록 순서대로 고정 요청 probe를 순차 실행하고, 0이 아닌 종료나 잔존 프로세스에서 즉시 멈춘다.

- 실행 전 시험 잔존 프로세스가 있으면 시작하지 않는다(강제 종료하지 않음).
- 각 사례의 종료코드·report 요약·실행 후 잔존 여부를 status JSON에 즉시 기록한다.
- 기대 실패 사례도 배치를 멈춘다. 기대 여부 판정은 audit_schedule_gate.py가 별도로 한다.
- 기존 출력 이름은 재사용하지 않는다. 재시도는 하지 않는다.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[1]
RESIDUAL_PATTERN = ('gz sim', 'probe_braking_contact', 'validate_g1_traffic', 'parameter_bridge', 'rotating_lidar',
                    'actuation_ros', 'local_pursuit', 'lidar_safety', 'gz topic -e')
REPORT_KEYS = ('passed', 'input_valid', 'stopped', 'shutdown_clean', 'stop_acknowledged', 'source_unchanged', 'error',
               'wall_elapsed_s')
RESIDUAL_EXIT = 95


def residual_processes():
    """시험 관련 잔존 프로세스 목록(자기 자신 제외)."""
    lines = subprocess.run(['ps', '-eo', 'pid,args'], capture_output=True, text=True, check=True).stdout.splitlines()[1:]
    own = {str(Path(__file__).name)}
    return [line.strip() for line in lines
            if any(p in line for p in RESIDUAL_PATTERN) and not any(o in line for o in own) and 'ps -eo' not in line]


def load_plan(path):
    plan = json.loads(Path(path).read_text(encoding='utf-8'))
    names = [case['name'] for case in plan['cases']]
    if not names or len(set(names)) != len(names):
        raise ValueError('plan needs unique case names')
    for case in plan['cases']:
        if not case['name'].replace('_', '').isalnum() or case.get('expect') not in ('pass', 'expected_failure'):
            raise ValueError(f"bad case {case['name']}")
        if any(a == '--output' or a.startswith('--output=') for a in case['args']):
            raise ValueError('output is derived from base/name')
    return plan


def summarize(report_path):
    if not report_path.is_file():
        return {'report': 'missing'}
    report = json.loads(report_path.read_text(encoding='utf-8'))
    audit = report.get('request_audit') or {}
    return {**{k: report.get(k) for k in REPORT_KEYS}, 'request_audit_checks': audit.get('checks'),
            'all_ticks_checks': (report.get('request_audit_all_ticks') or {}).get('checks'),
            'max_lateness_ns': (audit.get('lateness_ns') or {}).get('max'),
            'fault_injection_applied': (report.get('evaluation_fault_injection') or {}).get('applied'),
            'plugin_trace_valid': (report.get('plugin_command_trace') or {}).get('valid')}


def run_plan(plan, status_path, command=None, residual=residual_processes, repo=REPO):
    """사례를 순서대로 실행한다. command는 시험용 대체 실행기(기본: 실제 probe)."""
    command = command or [sys.executable, '-B', 'scripts/probe_braking_contact.py']
    base = (repo/plan['base']).resolve()
    status = {'plan': plan, 'started_wall_s': time.time(), 'cases': [], 'stopped_at': None, 'not_started': []}

    def write():
        status_path.write_text(json.dumps(status, indent=2, ensure_ascii=False))
    write()
    exit_code = 0
    for index, case in enumerate(plan['cases']):
        output, log = base/case['name'], base/f"{case['name']}.runner.log"
        entry = {'name': case['name'], 'expect': case['expect'], 'args': case['args']}
        status['cases'].append(entry)
        before = residual()
        if before or output.exists() or log.exists():
            entry.update(started=False, reason='residual_before' if before else 'output_exists', residual_before=before)
            exit_code = RESIDUAL_EXIT if before else 2
        else:
            entry['started'] = True
            with log.open('w') as stream:
                rc = subprocess.run([*command, *case['args'], '--output', str(output)], cwd=repo,
                                    stdout=stream, stderr=subprocess.STDOUT).returncode
            after = residual()
            entry.update(rc=rc, residual_after=after, summary=summarize(output/'report.json'))
            exit_code = rc if rc else (RESIDUAL_EXIT if after else 0)
        write()
        if exit_code:
            status['stopped_at'] = case['name']
            status['not_started'] = [c['name'] for c in plan['cases'][index+1:]]
            break
    status['exit_code'] = exit_code
    status['finished_wall_s'] = time.time()
    write()
    return exit_code


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--status', type=Path, required=True)
    args = parser.parse_args()
    if args.status.exists():
        raise SystemExit('status file exists; use a new name')
    raise SystemExit(run_plan(load_plan(args.plan), args.status))
