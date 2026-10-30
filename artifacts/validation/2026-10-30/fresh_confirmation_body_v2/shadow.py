import sys, importlib.util
from pathlib import Path
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import candidate_state
spec=importlib.util.spec_from_file_location('_shadow_v1', BASE.parent/'fresh_confirmation_memory_v1/shadow_saved.py')
mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
raise SystemExit(mod.main(Path(sys.argv[1]).resolve(),Path(sys.argv[2]).resolve()))
