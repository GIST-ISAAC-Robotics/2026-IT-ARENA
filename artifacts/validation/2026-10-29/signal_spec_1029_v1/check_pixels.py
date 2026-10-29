"""정지 촬영 프레임을 기존/지역 신호 검출기에 같은 순서로 넣는다. 실제 점등 전환·주행 증거가 아니다."""
import hashlib
import json
from pathlib import Path
import sys

import cv2

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'src/arena_autonomy'))
from arena_autonomy.race_start import OfficialStartSignal
from arena_autonomy.race_start_local import LocalOfficialStartSignal


def main():
    out=BASE/'pixel_check.json'
    if out.exists():raise FileExistsError(out)
    sources=[Path(__file__),ROOT/'src/arena_autonomy/arena_autonomy/race_start.py',
             ROOT/'src/arena_autonomy/arena_autonomy/race_start_local.py',ROOT/'src/arena_autonomy/arena_autonomy/core.py']
    rows=[]
    for variant in ('official','no_id45'):
        for index in range(6):
            paths=[BASE/f'render_{variant}_{stage}/capture/slot{index}.png' for stage in ('red4','green')]
            sources.extend(paths)
            red,green=[cv2.cvtColor(cv2.imread(str(p)),cv2.COLOR_BGR2RGB) for p in paths]
            for detector in (OfficialStartSignal,LocalOfficialStartSignal):
                signal=detector()
                early=[signal.update(green,i*.1) for i in range(5)]
                signal.reset();states=[]
                for i,rgb in enumerate([red]*3+[green]*3):
                    signal.update(rgb,i*.1);states.append(signal.observed)
                rows.append(dict(variant=variant,grid=index,detector=detector.__name__,states=states,
                                 green_only_start=any(early),started=bool(signal.started),
                                 expected=bool(signal.started) and not any(early)))
    result=dict(scope='static frames repeated at assigned .1 s; no live transition, exposure change or driving',
                rows=rows,summary={f"{v}/{d}":[r['grid'] for r in rows if r['variant']==v and r['detector']==d and r['expected']]
                                   for v in ('official','no_id45') for d in ('OfficialStartSignal','LocalOfficialStartSignal')},
                source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    with out.open('w',encoding='utf-8',newline='\n') as stream:stream.write(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result['summary'],ensure_ascii=False))
    for r in rows:print(r['variant'],r['grid'],r['detector'][:5],r['states'],r['green_only_start'],r['started'])
    return 0


if __name__=='__main__':raise SystemExit(main())
