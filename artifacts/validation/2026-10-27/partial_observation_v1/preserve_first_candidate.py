"""run2에 기록된 해시로 첫 실패 후보의 바이트 복원 여부를 확인한다."""
import hashlib
import json
from pathlib import Path

OUT=Path(__file__).parent
ROOT=OUT.parents[3]
report=json.loads((OUT/'run2_saved_input/report.json').read_text())
source=(ROOT/'src/arena_autonomy/arena_autonomy/partial_observation.py').read_text()
start=source.index('def near_sweep(')
end=source.index('class PartialObservationPolicy:')
source=source[:start]+source[end:]
source=source.replace('def evaluate(self, scan, points, history, now, speed, steering=0., previous_steering=0.,\n                 wheelbase=.135, length=.20, width=.15):',
                      'def evaluate(self, scan, points, history, now, speed):')
start=source.index('        if speed > 1.2:')
end=source.index('        step = float(scan.angle_increment)',start)
source=source[:start]+source[end:]
start=source.index("        report['history_distance_m']")
end=source.index('        for run in runs:',start)
source=source[:start]+"        report['required_neighbor_range_m'] = required\n"+source[end:]
source=source.replace(' < near_boundary:', ' < required:')
start=source.index('            relevant = near_sweep(')
end=source.index('                self.latched = True',start)
source=source[:start]+"            if np.any((along >= 0) & (along <= required) & (across <= spread)):\n"+source[end:]
matched=None
for newline in ('\n','\r\n'):
    candidate=source.replace('\n',newline).encode()
    if hashlib.sha256(candidate).hexdigest()==report['source_sha256']['partial_observation.py']:
        matched=candidate
if matched is None:
    raise RuntimeError('복원 바이트가 기존 실행 해시와 다릅니다. 과거 소스로 저장하지 않습니다.')
destination=OUT/'run2_saved_input/partial_observation_failed_source.py'
with destination.open('xb') as stream:stream.write(matched)
print(json.dumps({'reconstructed_and_hash_matched':True,'sha256':hashlib.sha256(matched).hexdigest()}))
