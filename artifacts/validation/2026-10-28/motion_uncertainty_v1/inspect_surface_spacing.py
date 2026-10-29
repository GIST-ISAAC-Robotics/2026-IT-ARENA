"""저장된 최초 소실 근거만 분석한다. 운영 대응 거리나 명령을 변경하지 않는다."""
import hashlib
import json
import math
from pathlib import Path

BASE = Path(__file__).resolve().parent
SOURCE = BASE / 'angle_diagnosis_v1/decisions.jsonl'
OUTPUT = BASE / 'surface_spacing_v1.json'


def subtract(a, b):
    return [a[0] - b[0], a[1] - b[1]]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def observed_surface_candidates(point, points, ids, maximum_chord):
    """기존과 같은 3반환/각도/수직거리 조건에서 현 길이만 대조한다."""
    matches = []
    for i in range(len(points) - 2):
        if ids[i + 1] != ids[i] + 1 or ids[i + 2] != ids[i + 1] + 1:
            continue
        vectors = [subtract(points[j + 1], points[j]) for j in (i, i + 1)]
        lengths = [math.hypot(*v) for v in vectors]
        if not all(1e-6 < length <= maximum_chord for length in lengths):
            continue
        cosine = dot(*vectors) / math.prod(lengths)
        if cosine < math.cos(math.radians(30)):
            continue
        for offset, (vector, length) in enumerate(zip(vectors, lengths)):
            delta = subtract(point, points[i + offset])
            fraction = dot(delta, vector) / (length * length)
            if not 0 <= fraction <= 1:
                continue
            normal = math.hypot(*(delta[k] - fraction * vector[k] for k in (0, 1)))
            if normal <= .01:
                matches.append(dict(source_indices=ids[i:i + 3],
                    current_points=points[i:i + 3], segment_lengths_m=lengths,
                    direction_change_deg=math.degrees(math.acos(max(-1., min(1., cosine)))),
                    segment_index=offset, projection_fraction=fraction,
                    normal_distance_m=normal))
    return matches


def main():
    digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    first = None
    with SOURCE.open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if abs(row['t'] - 133.7) < 1e-6:
                first = row['candidate'][2]['partial_observation']['latch_evidence']
                break
    if first is None:
        raise ValueError('registered latch not found')
    context = first['correspondence_context']
    point = first['witnesses'][0]['previous_point_now_xy']
    points = context['current_return_points_xy']
    ids = context['current_return_source_indices']
    baseline = observed_surface_candidates(point, points, ids, .06)
    longer = observed_surface_candidates(point, points, ids, .12)
    result = dict(scope='stored local geometry only; no policy change or driving claim',
        source_sha256=digest, reference_s=first['reference_s'],
        previous_point_now_xy=point, nearest_current_distance_m=min(math.dist(point, p) for p in points),
        current_6cm_matches=baseline, diagnostic_12cm_matches=longer,
        limitations=['12cm is a comparison, not a new accepted operating threshold',
            'adjacent returns do not prove a continuously occupied surface or object identity',
            'a thin independent object and nearby wall can be confused; counterexamples required'])
    if hashlib.sha256(SOURCE.read_bytes()).hexdigest() != digest:
        raise RuntimeError('source changed')
    with OUTPUT.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
