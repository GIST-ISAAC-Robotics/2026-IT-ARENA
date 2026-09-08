"""지정한 진단 실행의 Gazebo만 종료 직후 스택을 수집합니다 (root 조회용)."""
import re
import subprocess
import time
from pathlib import Path

root = Path(__file__).resolve().parent
log = root / "gpu_diagnostic" / "official_low_load_30_smoke.log"
deadline = time.monotonic() + 180
while time.monotonic() < deadline:
    text = log.read_text(errors="replace") if log.exists() else ""
    if "user interrupted with ctrl-c" in text:
        match = re.search(r"\[gz-1\]: process started with pid \[(\d+)\]", text)
        if not match:
            raise RuntimeError("Gazebo PID not found")
        time.sleep(2)
        with (root / "gpu_shutdown_stack.txt").open("w") as output:
            subprocess.run(["gdb", "-batch", "-nx", "-ex", "set debuginfod enabled off",
                            "-ex", "set pagination off", "-ex", "thread apply all bt 12",
                            "-p", match.group(1)], stdout=output, stderr=subprocess.STDOUT, timeout=10)
        break
    time.sleep(.2)
else:
    raise RuntimeError("No shutdown observed")
