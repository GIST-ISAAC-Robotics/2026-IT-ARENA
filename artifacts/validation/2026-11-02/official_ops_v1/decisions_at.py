"""판단 기록의 특정 시각대 요약. 인자: 파일 시작s 끝s [간격s]."""
import json
import sys

path, start, end = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
every = float(sys.argv[4]) if len(sys.argv) > 4 else .2
last = None
DROP = ('path_points', 'target_xy', 'partial_observation', 'recent_observation')


def compact(detail):
    result = {}
    for key, value in detail.items():
        if key in DROP or isinstance(value, list):
            continue
        if isinstance(value, dict):
            value = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in value.items()
                     if not isinstance(v, (list, dict))}
        result[key] = round(value, 3) if isinstance(value, float) else value
    return result


with open(path, encoding='utf-8') as stream:
    for line in stream:
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if event.get('kind') != 'call':
            continue
        source = event['data']['input']
        reference = source.get('reference_s')
        if type(reference) not in (int, float) or not start <= reference <= end:
            continue
        if last is not None and reference-last < every:
            continue
        last = reference
        output = event['data']['output']
        print(round(reference, 3), 'side', source['side'], 'speed', round(source['speed_mps'], 3), 'out', output[0], output[1])
        print('   ', json.dumps(compact(output[2]), ensure_ascii=False)[:1500])
        partial = output[2].get('partial_observation')
        if isinstance(partial, dict):
            print('    partial', {k: v for k, v in partial.items() if k in ('allowed', 'reason', 'speed_cap_mps')})
