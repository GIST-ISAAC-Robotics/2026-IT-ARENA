"""지정한 이번 GPU 실행의 종료 직후 스택만 읽습니다."""
from pathlib import Path
import re, subprocess, time
root = Path(__file__).resolve().parent
log = root / "gpu_bullet_no_scene" / "official_low_load_30_smoke.log"
deadline = time.monotonic() + 180
while time.monotonic() < deadline:
    text = log.read_text(errors="replace") if log.exists() else ""
    if "user interrupted with ctrl-c" in text:
        match = re.search(r"\[gz-1\]: process started with pid \[(\d+)\]", text)
        if not match: raise RuntimeError("PID not found")
        time.sleep(2)
        with (root / "gpu_shutdown_stack.txt").open("w") as output:
            subprocess.run(["gdb", "-batch", "-nx", "-ex", "set debuginfod enabled off",
                            "-ex", "thread apply all bt 15", "-ex", "info sharedlibrary",
                            "-p", match.group(1)], stdout=output, stderr=subprocess.STDOUT, timeout=10)
        break
    time.sleep(.2)
