import importlib.util
import sys

script = "artifacts/validation/2026-11-05/right_angle_course_v1/diag_replay.py"
spec = importlib.util.spec_from_file_location("diag_replay_loaded", script)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
print("DIAG_IMPORTED", flush=True)
module.main(*sys.argv[1:])
