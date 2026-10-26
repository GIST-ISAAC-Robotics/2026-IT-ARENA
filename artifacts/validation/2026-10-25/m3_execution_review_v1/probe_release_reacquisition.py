"""기록 만료와 재관측의 상태 전이 진단. 주행·연속 물체 추적 검증이 아니며 제어기에 정답을 넣지 않는다."""
import hashlib
import json
from pathlib import Path
import sys

import arena_autonomy.m3_execution_trial as mx
from test_lateral_intent import GRASS, car
from test_local_follow import history
from test_m3_execution_trial import padding
from test_m3_observed_follow import timed_scene
from test_m3_baseline_release import span_dropout


def scenario(kind):
    trial = mx.ExecutionLateralTrial(GRASS, mx.ExecutionConfig(release_to_baseline=True))
    motion = history(end=5.)
    rows = []
    for k in range(36):
        t = round(1.+k*.1, 3)
        hidden_x = max(0., .3-.3*(t-1.))
        hidden = car(hidden_x, .13, -.09)
        visible = car(.3+.45*max(0., t-2.3), .13, -.09)
        if t == 1.:
            objects, dropout = [hidden], None
        elif t < 2.3 or kind == 'no_reappearance':
            objects, dropout = [hidden], span_dropout(hidden)
        elif kind == 'fresh_visible_track':
            # 상태 갱신 분기를 검사하는 새 시퀀스. 동일 물체라는 연관을 전제하지 않는다.
            objects, dropout = [visible], None
        else:
            # 옛 물체는 계속 존재하지만 그 방위 반환만 누락, 새 물체는 같은 측면 앞쪽에서 보임.
            objects, dropout = [hidden, visible], span_dropout(hidden)
        scene, path = timed_scene(t, -.09, extra=objects, dropout=dropout)
        out = trial.step(scene, [], path, .35, padding(scene), history=motion)
        record = trial.records.get('left')
        rows.append({'t': t, 'mode': out['mode'], 'released': bool(out.get('released_to_baseline')),
                     'release_reason': (out.get('release_gate') or {}).get('reason'),
                     'left_present': (out.get('threats') or {}).get('left', {}).get('present'),
                     'record_scene_s': record.scene.t_ref_s if record is not None else None,
                     'record_detected_s': record.detected_at_s if record is not None else None,
                     'previous_object_present_in_synthetic_scene': kind != 'fresh_visible_track' or t < 2.3,
                     'possible_cells': (out.get('release_gate') or {}).get('possible_cells')})
    return {'rows': rows, 'release_times': [r['t'] for r in rows if r['released']]}


if __name__ == '__main__':
    output = Path(sys.argv[1])
    if output.exists():
        raise FileExistsError(output)
    source = Path(mx.__file__)
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    result = {'scope': 'synthetic scan/state sequence only; no actor association or physically continuous trajectory proof',
              'cases': {name: scenario(name) for name in
                        ('no_reappearance', 'fresh_visible_track', 'old_hidden_plus_new_same_side')}}
    result['source_sha256'] = before
    result['source_unchanged'] = before == hashlib.sha256(source.read_bytes()).hexdigest()
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({name: case['release_times'] for name, case in result['cases'].items()}))
