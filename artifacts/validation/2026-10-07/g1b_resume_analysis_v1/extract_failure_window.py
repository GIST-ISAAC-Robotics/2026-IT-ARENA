#!/usr/bin/env python3
"""등속 seed 0 보존 실패·재개 실행에서 회귀 시험용 소형 발췌를 만든다(원본 파일 불변).

실제 원시 scan과 가상 MCU 평균 바퀴 속도만 옮긴다. 자이로는 기록되지 않았으므로
시험은 0 자이로를 사용한다(직선 출발 구간). 평가기 자세는 제어 입력에 넣지 않고
출처 확인용 요약값만 남긴다.
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
WINDOWS = {
    # 최초 실패: 기억 지원 허용치 안 기둥 모서리 → TTL 만료, 정지 뒤 무기억 재획득 불가.
    'failure_window.json': ('g1b_pilot_v3_slow_s0', [7.202, 7.302, 7.402, 7.502, 7.602, 7.702, 7.802, 7.902],
                            [9.802, 9.902, 10.002], (6.9, 8.1)),
    # 1차 수정 실행의 8.72초 일시 상실: 기둥이 직전 경로 시작점 뒤(기억 범위 밖)에 있음.
    'transient_window.json': ('g1b_resume_v1_slow_s0', [8.202, 8.302, 8.402, 8.502, 8.602], [], (7.9, 8.75)),
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extract(name, run_name, moving, stationary, feedback_window):
    run, out = ROOT/'artifacts/validation/2026-10-07'/run_name, Path(__file__).resolve().parent/name
    scans = [json.loads(line) for line in (run/'scans_received.jsonl').read_text().splitlines()]
    pick = {round(s, 3) for s in moving+stationary}
    rows = [{k: row[k] for k in ('stamp_s', 'time_increment', 'angle_min', 'angle_increment', 'ranges')}
            for row in scans if round(row['stamp_s'], 3) in pick]
    assert len(rows) == len(pick), 'missing preserved scan'
    trace = json.loads((run/'actuation_trace.json').read_text())
    feedback = sorted({row['feedback_capture_us']: row['mean_wheel_mps'] for row in trace
                       if row.get('feedback_valid') and row.get('feedback_capture_us') is not None
                       and feedback_window[0]*1e6 <= row['feedback_capture_us'] <= feedback_window[1]*1e6}.items())
    report = json.loads((run/'report.json').read_text())
    out.write_text(json.dumps({
        'scope': ('excerpt of a preserved G1-b slow_lead seed 0 run for regression tests: raw C1 scans and '
                  'virtual-MCU mean wheel feedback only; gyro was not recorded (tests use zero gyro on the start straight)'),
        'source_run': str(run.relative_to(ROOT)), 'source_report_sha256': sha(run/'report.json'),
        'scans_received_sha256': sha(run/'scans_received.jsonl'), 'actuation_trace_sha256': sha(run/'actuation_trace.json'),
        'report_scans_received_sha256': report.get('scans_received_sha256'),
        'moving_scan_stamps_s': moving, 'stationary_scan_stamps_s': stationary,
        'scans': rows, 'feedback_capture_s_mean_wheel_mps': [[us*1e-6, v] for us, v in feedback]},
        separators=(',', ':')), encoding='utf-8')
    print(out, out.stat().st_size, len(rows), len(feedback))


def main():
    for name, args in WINDOWS.items():
        extract(name, *args)


if __name__ == '__main__':
    main()
