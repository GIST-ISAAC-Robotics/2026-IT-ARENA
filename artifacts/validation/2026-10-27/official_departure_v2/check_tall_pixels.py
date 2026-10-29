"""높은 상대차 장면의 실제 픽셀 진단. 검출되지 않는 배치도 결과로 보존한다."""
import hashlib
import json
from pathlib import Path
import sys
import cv2

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'src/arena_autonomy'))
from arena_autonomy.race_start import OfficialStartSignal, _components, _red_bank
from arena_autonomy.core import red_signal_mask


def main():
    output=BASE/'tall_pixels.json'
    if output.exists():raise FileExistsError(output)
    heights=json.loads((BASE/'tall_red4/report.json').read_text(encoding='utf-8'))['camera_heights_m']
    sources=[Path(__file__),ROOT/'src/arena_autonomy/arena_autonomy/race_start.py',ROOT/'src/arena_autonomy/arena_autonomy/core.py']
    rows=[]
    for name,height in heights.items():
        images=[]
        for stage in ('red4','green'):
            path=BASE/f'tall_{stage}/capture/{name}.png';sources.append(path)
            image=cv2.imread(str(path))
            if image is None:raise ValueError('missing pixel evidence: '+str(path))
            images.append(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
        red,green=images;signal=OfficialStartSignal()
        green_only=[signal.update(green,i*.1) for i in range(5)]
        signal.reset();states=[]
        for i,image in enumerate([red]*3+[green]*3):
            signal.update(image,i*.1);states.append(signal.observed)
        mask=red_signal_mask(cv2.cvtColor(red,cv2.COLOR_RGB2HSV));mask[int(red.shape[0]*.58):]=0
        components,unclassified=_components(mask)
        rows.append(dict(camera_height_m=height,name=name,states=states,
            green_only_start=any(green_only),normal_sequence_started=signal.started,
            red_components=components,unclassified_red=unclassified,bank=_red_bank(components)))
    result=dict(scope='static rendered frames repeated at assigned timestamps; no live signal/drive proof',
        rows=rows,no_green_only_false_start=not any(r['green_only_start'] for r in rows),
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))
    return 0


if __name__=='__main__':raise SystemExit(main())
