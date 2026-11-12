"""정지 RGB 자체의 검출 가능성과 시험 수집 간격의 영향을 분리한다.

각 단계의 마지막 저장 사진을 10Hz 입력으로 반복한 정적 패턴 검사다.
실제 연속 영상·출발 시각·실차 인식 성공을 증명하지 않는다.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import cv2

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('audit',HERE/'audit.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('directory')
args=parser.parse_args();directory=HERE/args.directory
path=directory/'report.json'
if not path.exists():path=directory/'report.partial.json'
raw=json.loads(path.read_text())
results=[]
for case in raw['cases']:
    frames=[]
    for stage in ('off','red4','green'):
        rgb=cv2.cvtColor(cv2.imread(str(directory/(case['name']+'_'+stage+'.png'))),cv2.COLOR_BGR2RGB)
        for _ in range(5):frames.append(dict(stage=stage,stamp=1.+len(frames)*.1,rgb=rgb))
    detection=a.detect(frames)
    result=dict(name=case['name'],height_m=case['height_m'],slot=case['slot'],**detection)
    result['marker_threshold_sensitivity']={str(rate):sorted(set(int(x[0]) for x in a.marker_observations(rgb,rate))) for rate in (.05,.02,.01,.005)}
    result['max_capture_gap_s']=max(b['stamp']-c['stamp'] for c,b in zip(case['frames'],case['frames'][1:]))
    results.append(result)
report=dict(scope='saved static image pattern replay at assumed 10Hz; not an independent temporal or operational start test',
            raw_report=str(path.relative_to(HERE)),cases=results)
a.save(directory/'static_patterns.json',report)
for r in results:
    print(dict(name=r['name'],normal=r['local_started'],late=r['green_only_started'],ids=r['frames'][-1]['ids'],marker_rates=r['marker_threshold_sensitivity']))
