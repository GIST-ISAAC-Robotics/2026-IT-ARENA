from pathlib import Path
import json, os, re, subprocess, time

root = Path(__file__).resolve().parent
repo = root.parents[3]
exe = repo / "build/gpu_final/arena_sensor_profile"
log = Path.home() / ".gz/rendering/ogre2.log"
rows = []
# 서로 다른 계측을 동시에 실행하지 않는다. 반복은 역순으로 실행한다.
for backend, mode, repeat in [
    (b, m, r) for r in (1, 2) for m in ("rgbd", "lidar500", "tof6")
    for b in (("cpu", "gpu") if r == 1 else ("gpu", "cpu"))
] + [("gpu_egl", "rgbd", 1)]:
    name = f"{backend}_{mode}_{repeat}"
    output = root / f"{name}.log"
    if output.exists():
        raise FileExistsError(output)
    env = os.environ.copy()
    env.update(GALLIUM_DRIVER="llvmpipe" if backend == "cpu" else "d3d12",
               LIBGL_ALWAYS_SOFTWARE="true" if backend == "cpu" else "false",
               MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA")
    env["LD_LIBRARY_PATH"] = str(repo / "install/arena_gazebo/lib") + ":" + env.get("LD_LIBRARY_PATH", "")
    if backend != "cpu":
        env["LD_PRELOAD"] = "libarena-wsl-d3d12-lifetime.so"
    start = time.monotonic()
    try:
        result = subprocess.run([str(exe), mode, "1" if backend == "gpu_egl" else "0"],
                                env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=90, text=True)
        content, code = result.stdout, result.returncode
    except subprocess.TimeoutExpired as exc:
        content = (exc.stdout or b"").decode(errors="replace")
        code = "timeout90"
    output.write_text(content)
    renderer_log = log.read_text(errors="replace") if log.exists() else ""
    (root / f"{name}.ogre.log").write_text(renderer_log)
    profile = re.search(r"PROFILE (\{[^\n]+\})", content)
    renderer = re.findall(r"GL_RENDERER\s*=\s*(.+)", renderer_log)
    row = {"name": name, "returncode": code, "total_wall_s": time.monotonic()-start,
           "renderer": renderer[-1] if renderer else None,
           "profile": json.loads(profile[1]) if profile else None,
           "clean_exit": code == 0 and "CLEAN_EXIT" in content}
    rows.append(row)
    (root / "results.json").write_text(json.dumps(rows, indent=2))
    print(json.dumps(row), flush=True)
