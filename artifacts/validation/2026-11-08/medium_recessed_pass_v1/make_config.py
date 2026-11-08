"""r15 등록 설정을 복사하고 actor.visual_outline='recessed_sides' 한 키만 더한다. 다른 값 변화가 있으면 거부한다."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent/'medium_pass_connected_v1/new_track_course.json'
TARGET = HERE/'new_track_course.json'

prior = json.loads(SOURCE.read_text(encoding='utf-8'))
current = json.loads(SOURCE.read_text(encoding='utf-8'))
assert 'visual_outline' not in current['actor']
current['actor']['visual_outline'] = 'recessed_sides'
check = json.loads(json.dumps(current))
check['actor'].pop('visual_outline')
assert check == prior
if TARGET.exists():
    raise SystemExit('refused: target exists')
TARGET.write_text(json.dumps(current, ensure_ascii=False, indent=2)+'\n', encoding='utf-8', newline='\n')
print('only actor.visual_outline added')
