"""완료된 저장 입력 대조를 보고서에 반영한다."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
EDITS = {
    'artifacts/validation/2026-11-01/course_compare_v1/README.md': [
        ('- **코드 경로:** `low_motion_reserve`·`moving_pace_floor`는 FOLLOW 분기에서만 쓰인다. 우회 진입 뒤에는 닿지 않는다. 진입 측방 위치도 −0.267 vs −0.263 m로 거의 같다.\n',
         '- **코드 경로:** `low_motion_reserve`·`moving_pace_floor`는 FOLLOW 분기에서만 쓰인다. 우회 진입 뒤에는 닿지 않는다. 진입 측방 위치도 −0.267 vs −0.263 m로 거의 같다.\n'
         '- **같은 입력 대조:** [run_paced의 7,785회 입력을 `closing` 설정에 넣으면](closing_on_paced_inputs.json) 같은 호출(4.833초)에서 BYPASS에 들어가고, 진입 뒤 7,559회의 속도·조향·상태 차이가 **0**이다. 0명령 6,435회도 같다. 진입 전 226회 중 속도만 4회 다르다. 같은 관측이 주어지면 `closing`도 똑같이 멈춘다. 다만 `closing`으로 달렸을 때 같은 관측이 나온다는 뜻은 아니다.\n'),
        ('- **간접 영향은 배제하지 못한다:** 진입이 0.094초 빨랐고 궤적이 달라 12초대 정지가 일어나지 않았다. 후보가 이 위상을 만들었는지, 우연인지는 1회 실행으로 가를 수 없다.\n',
         '- **간접 영향은 배제하지 못한다:** 진입이 run1보다 0.094초 빨랐고 궤적이 달라 12초대 정지가 일어나지 않았다. 후보가 이 위상을 만들었는지, 우연인지는 1회 실행으로 가를 수 없다.\n'),
        ('- **미완료 대조:** `replay_closing.sh`가 run_paced 입력을 `closing` 설정에 넣어 비교한다(약 17분). 결과 `closing_on_paced_inputs.json`, 로그 `replay_closing.log`. 끝나지 않았으면 그대로 다시 실행하면 된다. 진입 뒤 판단이 같으면 위 코드 경로 판단이 확인된다. 첫 시도는 numpy 정수 직렬화 오류로 결과 없이 끝났고 도구를 고쳤다.\n',
         '- **저장 입력 대조:** 완료했다([결과](closing_on_paced_inputs.json), 종료코드 0). 첫 시도는 numpy 정수 직렬화 오류로 결과 없이 끝나 도구를 고쳐 다시 실행했다. 남은 프로세스는 없다.\n'),
    ],
    'docs/PROJECT_CONTEXT.md': [
        ('남은 저장 입력 대조 하나는 [인계 상태](../artifacts/validation/2026-11-01/course_compare_v1/README.md#인계-상태)에 있다. ',
         '[인계 상태](../artifacts/validation/2026-11-01/course_compare_v1/README.md#인계-상태)를 남겼다. '),
        ('이 교착은 `closing`에도 잠재하며 run1 완주는 우연한 0.86초 정지에 기댔다.',
         '같은 입력을 `closing`에 넣으면 진입 뒤 7,559회 판단이 모두 같아 교착은 후보 논리에서 오지 않는다. run1 완주는 우연한 0.86초 정지에 기댔다.'),
    ],
}
pending = {}
for name, pairs in EDITS.items():
    path = ROOT / name
    data = path.read_bytes()
    newline = '\r\n' if data.count(b'\r\n') > data.count(b'\n') / 2 else '\n'
    for old, new in pairs:
        old_bytes = old.replace('\n', newline).encode('utf-8')
        new_bytes = new.replace('\n', newline).encode('utf-8')
        if data.count(old_bytes) != 1:
            raise ValueError(f'{name}: 기대 원문이 한 번이 아님: {old[:60]!r}')
        data = data.replace(old_bytes, new_bytes)
    pending[path] = data
for path, data in pending.items():
    path.write_bytes(data)
print('updated', len(pending))
