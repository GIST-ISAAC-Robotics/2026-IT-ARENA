"""LiDAR 시험의 실행 부모·자식 종료를 분리해 검증하는 순수 보조 함수."""

import os
from pathlib import Path
import re
import signal
import subprocess
import time


ERROR_TOKENS = ("[ERROR]", "[Err]", "Traceback", "KeyboardInterrupt", "Segmentation fault")


def group_processes(pgid):
    """우리 launch 그룹의 살아 있는 프로세스와 재사용 방지용 시작 시각만 읽습니다."""
    rows = {}
    for root in Path('/proc').iterdir():
        if not root.name.isdigit():
            continue
        try:
            # comm에는 공백/괄호가 들어갈 수 있습니다.
            fields = (root / 'stat').read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == pgid and fields[0] != 'Z':
                rows[int(root.name)] = {'ppid': int(fields[1]), 'start': fields[19]}
        except (FileNotFoundError, ProcessLookupError):
            continue
    return rows


def stop_gazebo_service(process, log_text, partition, timeout_s=20.):
    """본 검사 전용 transport 파티션에서 정상 종료 요청 후 실제 서버 소멸을 기다립니다.

    서비스 응답만으로 성공 처리하지 않습니다. 실패 시 호출자가 기존 launch
    종료/그룹 정리를 수행하되, 그 결과로 이 서비스 실패를 덮어쓰지 않습니다.
    """
    result = {'method': 'server_control', 'partition': partition, 'verified': False}
    expected = f'arena_autonomy_test_{os.getpid()}'
    if partition != expected or os.environ.get('GZ_PARTITION') != expected:
        return dict(result, error='test_partition_not_verified')
    if process is None or process.poll() is not None:
        return dict(result, error='launch_not_running')
    rows = group_processes(process.pid)
    roots = {int(pid) for pid in re.findall(
        r'\[gz-\d+\]: process started with pid \[(\d+)\]', log_text)}
    roots &= rows.keys()
    if len(roots) != 1:
        return dict(result, error='owned_gazebo_process_not_unique')
    owned = set(roots)
    while True:
        descendants = {pid for pid, row in rows.items() if row['ppid'] in owned}
        if descendants <= owned:
            break
        owned |= descendants
    identities = {pid: rows[pid]['start'] for pid in owned}
    result['server_processes_before'] = sorted(identities)
    command = ['gz', 'service', '-s', '/server_control',
               '--reqtype', 'gz.msgs.ServerControl', '--reptype', 'gz.msgs.Boolean',
               '--timeout', '5000', '--req', 'stop: true']
    began = time.monotonic()
    try:
        response = subprocess.run(command, capture_output=True, text=True, timeout=8)
        result.update(return_code=response.returncode, stdout=response.stdout, stderr=response.stderr)
        result['acknowledged'] = response.returncode == 0 and bool(
            re.search(r'\bdata:\s*true\b', response.stdout))
    except (subprocess.TimeoutExpired, OSError) as error:
        return dict(result, error=str(error), elapsed_s=time.monotonic() - began)
    while True:
        current = group_processes(process.pid)
        remaining = {pid for pid, start in identities.items()
                     if pid in current and current[pid]['start'] == start}
        # 요청 직후 fork된 서버 자식도 정리 감사에 포함합니다.
        for pid, row in current.items():
            if row['ppid'] in remaining:
                identities[pid] = row['start']
                remaining.add(pid)
        if not remaining or not result['acknowledged'] or time.monotonic() - began >= timeout_s:
            break
        time.sleep(.1)
    result.update(remaining_server_pids=sorted(remaining), elapsed_s=time.monotonic() - began,
                  verified=result['acknowledged'] and not remaining)
    return result


def audit_shutdown(log_text, return_code, forced_cleanup=None):
    lines = log_text.splitlines()
    shutdown_index = next((index for index, line in enumerate(lines)
                           if "user interrupted with ctrl-c" in line), len(lines))
    errors = [{"line": index + 1, "after_shutdown_request": index >= shutdown_index, "text": line}
              for index, line in enumerate(lines) if any(token in line for token in ERROR_TOKENS)]
    started = set(re.findall(r"process started with pid \[(\d+)\]", log_text))
    finished = set(re.findall(r"process has finished cleanly \[pid (\d+)\]", log_text))
    died = dict(re.findall(r"process has died \[pid (\d+), exit code (-?\d+)", log_text))
    missing = sorted(started - finished - set(died), key=int)
    clean = return_code == 0 and not forced_cleanup and not errors and bool(started) and not missing and not died
    return {"clean": clean, "launch_return_code": return_code, "forced_cleanup": forced_cleanup,
            "started_child_count": len(started), "clean_child_count": len(started & finished),
            "failed_children": {pid: int(code) for pid, code in died.items()},
            "unaccounted_child_pids": missing, "errors": errors,
            "errors_before_shutdown_count": sum(not row["after_shutdown_request"] for row in errors),
            "errors_after_shutdown_count": sum(row["after_shutdown_request"] for row in errors)}


def stop_launch(process):
    """SIGINT는 launch에 한 번만 보냅니다. launch가 자식에게 전달하게 합니다."""
    if process is None or process.poll() is not None:
        return {}
    result = {}
    for sig, timeout in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            if sig == signal.SIGINT:
                process.send_signal(sig)
            else:
                # 정상 종료가 실패한 경우에만 본 시험이 만든 그룹을 정리합니다.
                result["forced_cleanup"] = sig.name
                os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return result
        except ProcessLookupError:
            process.poll()
            return result
        except subprocess.TimeoutExpired:
            continue
    result["cleanup_timeout"] = True
    return result
