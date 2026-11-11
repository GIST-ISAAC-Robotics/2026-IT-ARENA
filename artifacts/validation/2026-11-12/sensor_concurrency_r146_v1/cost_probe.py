"""r144 입력/구현을 변경하지 않고 별도 결과 폴더에서 1회 계산한다."""
import importlib.util
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent
FROZEN = Path('/home/q/it_arena_guard_current_r144_20261112')
A = FROZEN/'artifacts/validation/2026-11-12/guard_current_cost_r144_v1'


def main():
    out = ROOT/'capture/cost'
    out.mkdir(exist_ok=False)
    for name in ('samples.jsonl', 'bundle_manifest.json'):
        shutil.copy2(A/name, out/name)
    spec = importlib.util.spec_from_file_location('frozen_r144_measure', A/'measure.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # ROOT와 모든 코드/설정은 동결본이다. 입력 사본과 새 결과 위치만 변경한다.
    module.A = out
    module.main()


if __name__ == '__main__':
    main()
