"""촬영한 정지 영상으로 기존 검출기를 대조한다. 실제 점등 시각 재현은 아니다."""
import hashlib
import json
from pathlib import Path
import sys
import cv2

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'src/arena_autonomy'))
from arena_autonomy.race_start import OfficialStartSignal,_components,_red_bank
from arena_autonomy.core import red_signal_mask


def main():
    out=BASE/'pixel_check.json'
    if out.exists():raise FileExistsError(out)
    sources=[Path(__file__),ROOT/'src/arena_autonomy/arena_autonomy/race_start.py',ROOT/'src/arena_autonomy/arena_autonomy/core.py']
    rows=[]
    for index in range(6):
        paths=[BASE/f'render_{stage}/capture/slot{index}.png' for stage in ('red4','green')]
        sources.extend(paths)
        red,green=[cv2.cvtColor(cv2.imread(str(p)),cv2.COLOR_BGR2RGB) for p in paths]
        signal=OfficialStartSignal();green_only=[]
        for i in range(5):green_only.append(signal.update(green,i*.1))
        signal.reset();red_states=[]
        for i in range(3):
            signal.update(red,i*.1);red_states.append(signal.observed)
        green_states=[]
        for i in range(3):
            signal.update(green,.3+i*.1);green_states.append(signal.observed)
        mask=red_signal_mask(cv2.cvtColor(red,cv2.COLOR_RGB2HSV));mask[int(red.shape[0]*.58):]=0
        components,unclassified=_components(mask)
        rows.append(dict(grid=index,red_states=red_states,green_states=green_states,green_only_start=any(green_only),
                         paired_bank=_red_bank(components),components=components,unclassified_red=unclassified,
                         passed=not any(green_only) and red_states[-1]=='red4_confirmed' and signal.started))
    result=dict(passed=all(r['passed'] for r in rows),scope='independent static frames repeated at assigned .1s, not live transition/drive proof',
                rows=rows,source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
