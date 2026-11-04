"""판단 기록의 시간 구간 필드 추출. 평가 전용. 인자: 실행, 시작, 끝(녹색 기준 초)."""
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
name, lo, hi = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
green = json.loads((HERE/name/'report.json').read_bytes())['evaluation']['t_green']
SKIP = ('mode', 'source_', 'reference_stamp', 'occupancy_', 'occupied', 'relative_velocity', 'passing_enabled', 'path_source',
        'partial_observation_policy', 'unfiltered', 'measured_speed', 'boundary_age')
last = None
for line in (HERE/name/'follow_decisions.jsonl').open(encoding='utf-8'):
    e = json.loads(line)
    if e['kind'] != 'call':
        continue
    i, o = e['data']['input'], e['data']['output']
    t = i['reference_s']-green
    if not lo <= t <= hi:
        continue
    d = o[2]
    key = (round(o[0], 1), d.get('reason'), d.get('behavior'), d.get('wall_fallback'), d.get('gate'), d.get('state'))
    if key != last:
        last = key
        print(round(t, 2), round(o[0], 3), {k: (round(v, 3) if isinstance(v, float) else v) for k, v in d.items()
                                            if not k.startswith(SKIP) and isinstance(v, (bool, str, int, float, type(None)))})
