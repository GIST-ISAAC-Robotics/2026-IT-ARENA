"""저장 렌더 프레임을 수정 전/후 검출기에 같은 순서로 넣어 손익을 비교한다. 정지 프레임 반복이며 주행이 아니다."""
import importlib.util
import json
from pathlib import Path
import sys
import zipfile

import cv2

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RENDER = HERE.parent/'official_facility_v1/render_v1'
sys.path[:0] = [str(ROOT/'src/arena_autonomy')]
from arena_autonomy.race_start_local import LocalOfficialStartSignal as New  # noqa: E402
from arena_autonomy.race_start import OfficialStartSignal as NewBase  # noqa: E402
import arena_autonomy.race_start as new_module  # noqa: E402


def old_detectors():
    """수정 전 소스를 별도 이름으로 불러온다."""
    scratch = HERE/'_before'
    scratch.mkdir(exist_ok=True)
    with zipfile.ZipFile(HERE/'before.zip') as archive:
        base = archive.read('src/arena_autonomy/arena_autonomy/race_start.py')
        local = archive.read('src/arena_autonomy/arena_autonomy/race_start_local.py')
    (scratch/'old_race_start.py').write_bytes(base)
    (scratch/'old_race_start_local.py').write_bytes(local.replace(b'arena_autonomy.race_start ', b'old_race_start '))
    sys.path.insert(0, str(scratch))
    spec = importlib.util.spec_from_file_location('old_race_start_local', scratch/'old_race_start_local.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    import old_race_start
    return old_race_start.OfficialStartSignal, module.LocalOfficialStartSignal


def rgb(view, stage):
    return cv2.cvtColor(cv2.imread(str(RENDER/f'{view}_{stage}.png')), cv2.COLOR_BGR2RGB)


def run(detector, view):
    signal = detector()
    early = [signal.update(rgb(view, 'green_probe'), i*.1) for i in range(5)]
    signal.reset()
    sequence = ['off']*3+['green_probe']*3+['off']*3+['red4']*3+['green']*4
    states = []
    for i, stage in enumerate(sequence):
        signal.update(rgb(view, stage), i*.1)
        states.append(signal.observed)
    return dict(green_only_start=any(early), started=bool(signal.started),
                last_red=states[11], last_green=states[-1])


def main():
    OldBase, OldLocal = old_detectors()
    views = sorted(path.stem[:-5] for path in RENDER.glob('*_red4.png'))
    rows = []
    for view in views:
        rows.append(dict(view=view, before=run(OldLocal, view), after=run(New, view),
                         base_before=run(OldBase, view)['started'], base_after=run(NewBase, view)['started']))
    bank = {}
    for view in ('slot2_up15', 'slot3_up15', 'slot5_up15'):
        signal = New()
        for i in range(3):
            signal.update(rgb(view, 'red4'), i*.1)
        bank[view] = signal.bank
    result = dict(scope='저장 정지 프레임 반복. 시험 초록·소등을 거친 뒤 빨강 4→초록 순서. 주행·실물 아님.', rows=rows, bank=bank)
    (HERE/'replay_saved.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    for row in rows:
        print(row['view'], row['before'], '->', row['after'])
    print(bank)
    assert new_module.__file__


if __name__ == '__main__':
    main()
