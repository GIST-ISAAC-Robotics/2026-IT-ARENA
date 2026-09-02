"""LiDAR 시험의 실행 부모·자식 종료를 분리해 검증하는 순수 보조 함수."""

import os
import re
import signal
import subprocess


ERROR_TOKENS = ("[ERROR]", "[Err]", "Traceback", "KeyboardInterrupt", "Segmentation fault")


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
