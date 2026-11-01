"""대표 코스 비교 결과와 인계 상태를 현황·계획·활동에 반영한다. 기대 원문이 없으면 쓰지 않는다."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
LINK = '../artifacts/validation/2026-11-01/course_compare_v1/README.md'
EDITS = {
    'artifacts/validation/2026-11-01/course_compare_v1/README.md': [
        ('[`mixed_course_v1/run1`](../mixed_course_v1/report.json)', '[`mixed_course_v1/run1`](../mixed_course_v1/run1/report.json)'),
    ],
    'artifacts/validation/2026-11-01/follow_continuity_v1/README.md': [
        ('다음은 대표 155초 코스를 `closing`과 `paced`로 비교해 채택을 정하는 것이다. 첫 발견의 1.4 → 0.95 m/s 명령 계단은 남은 큰 감속이다. 필요하면 그때 다룬다.',
         '**후속:** [대표 155초 코스 비교](../course_compare_v1/README.md)에서 `paced`는 BYPASS 복귀 교착으로 미완주했다. 이 교착은 `closing`에도 잠재한다. 첫 발견의 1.4 → 0.95 m/s 명령 계단은 남은 큰 감속이다.'),
    ],
    'docs/PROJECT_CONTEXT.md': [
        ('갱신: **2026-11-01 사용자 재개 지시로 개발 지속 중. 앞차 추종 지속성 묶음은 구현·실주행을 마치고 GPT 최종 검토 대기.** 10/31',
         f'갱신: **2026-11-01 사용자 재개 지시로 개발 지속 중. 5시간 사용량 경고로 Clarisse에서 GPT로 인계. [대표 코스 비교]({LINK})에서 BYPASS 복귀 교착을 찾았고 소스는 동결 상태다.** 10/31'),
        ('**기본값은 바꾸지 않았고 대표 코스 비교가 남았다.**',
         f'**기본값은 바꾸지 않았다.** [대표 155초 코스]({LINK})에서 `paced`는 진행 8.639 m에서 BYPASS로 멈춰 미완주했다(최장 정체 128초, 접촉·벌점 0). 원인은 양보 예산 약 4.9초가 복귀에 필요한 앞선 길이 약 0.78 m에 모자라는 교착이다. 이 교착은 `closing`에도 잠재하며 run1 완주는 우연한 0.86초 정지에 기댔다. 대표 코스의 앞차는 0.35 m/s라 `paced`의 이득을 재지 못한다.'),
        ('3. **다음 우선 작업:** GPT 최종 검토 뒤 대표 155초 코스를 `closing`과 `paced`로 비교해 채택을 정한다. 그 전에 5랩·고속 프로필로 넓히지 않는다. 첫 발견의 1.4→0.95 m/s 명령 계단과 BYPASS 안의 짧은 `no_entry_command_candidate` 정체가 남은 후보다. 단독의 추가 증속에는 위 코너 진입 속도와 감속 응답을 먼저 해석한다.',
         f'3. **다음 우선 작업:** [BYPASS 복귀 교착]({LINK}#권고)을 먼저 푼다. 후보는 양보 지속 조건(`maximum_continuation_s` 5초)이고, 저장 입력으로 BYPASS 탈출 경로도 확인한다. 짧은 정지 각본 뒤 대표 코스를 `closing`·`paced` 각 한 번 달린다. 그 전에 `paced`를 기본 활성화하지 않고 5랩·고속 프로필로 넓히지 않는다. 첫 발견의 1.4→0.95 m/s 명령 계단은 남은 감속이다. 단독의 추가 증속에는 위 코너 진입 속도와 감속 응답을 먼저 해석한다.'),
        ('5. **작업 상태:** 시험 프로세스는 모두 종료했고 새 실행은 유효·소스 불변·정상 종료·명시 STOP을 확인했다. 전체 회귀의 실패 16건은 수정 전 복원본에서도 같은 항목이며 앞선 미커밋 변경의 해시·의존성 사슬 검사와 C++ 시험 1건이다. 커밋·푸시는 하지 않았다.',
         f'5. **작업 상태:** 시뮬레이션 프로세스는 없고 새 실행은 유효·소스 불변·정상 종료·명시 STOP을 확인했다. 대표 코스의 `run_paced`는 재실행하지 않는다. 남은 저장 입력 대조 하나는 [인계 상태]({LINK}#인계-상태)에 있다. 전체 회귀의 실패 16건은 수정 전 복원본에서도 같으며 [분류와 정리 방향](../artifacts/validation/2026-11-01/follow_continuity_v1/README.md#기존-실패-16건-분류)을 기록했다. 커밋·푸시는 하지 않았다.'),
    ],
    'docs/DEVELOPMENT_PLAN.md': [
        ('**11/1 상태:** [선택형 `paced`](../artifacts/validation/2026-11-01/follow_continuity_v1/README.md)가 짧은 세 각본을 마쳤고 대표 코스 비교가 남았다. |',
         f'**11/1 상태:** [선택형 `paced`](../artifacts/validation/2026-11-01/follow_continuity_v1/README.md)는 짧은 세 각본을 마쳤으나 [대표 코스]({LINK})에서 BYPASS 복귀 교착으로 미완주했다. 교착을 푼 뒤 다시 비교한다. |'),
        ('앞차 속도 가정 0.5 m/s를 쓰는 `paced`를 대표 155초 코스의 비교 후보로 둔다. 기본값은 그대로다.',
         f'앞차 진행 가정 0.5 m/s를 쓰는 `paced`를 [대표 155초 코스]({LINK})에서 달렸으나 진행 8.639 m에서 BYPASS로 멈췄다. 양보 예산이 복귀에 필요한 앞선 길이에 모자라는 교착이며 `closing`에도 잠재한다. `paced`는 시험 후보로 유지하고 이 교착을 먼저 푼다.'),
        ('- **뒤로 미룬 일:**',
         f'- **기초 부채:** 전체 회귀의 기존 실패 16건은 이번 주행에 직접 영향이 없어 새 관문으로 삼지 않았다. [계약 사슬·전이 import·manifest·legacy C++ 분류](../artifacts/validation/2026-11-01/follow_continuity_v1/README.md#기존-실패-16건-분류)대로 경기 구성 고정 전에 한 묶음으로 정리한다.\n- **뒤로 미룬 일:**'),
    ],
    'docs/activity/2026-11-01.md': [
        ('여유를 완전히 푼 후보는 코너 12.558 m·1.078 m로 전체 판정을 통과했지만 급정지 뒤 우회를 잃어 기각했다.',
         '여유를 완전히 푼 후보는 코너 12.558 m·1.078 m로 전체 판정을 통과했지만 급정지 뒤 우회를 잃어 기각했다. 이어 입력 동등성과 명령 재현을 근거로 기준 재실행 없이 대표 155초 코스를 `paced`로 한 번 달렸고, **진행 8.639 m에서 BYPASS로 멈춰 미완주**했다(기준 run1은 137.35초에 한 랩). 접촉·세 바퀴 벌점은 0이었다.'),
        ('`paced`는 기본값으로 올리지 않았고 코너 최종 간격 1.386 m는 옛 1.20 m 비교선을 넘는다.',
         '`paced`는 기본값으로 올리지 않았고 코너 최종 간격 1.386 m는 옛 1.20 m 비교선을 넘는다. 대표 코스 미완주의 원인은 양보 예산이 복귀에 필요한 앞선 길이에 모자라는 BYPASS 교착이며 `closing`에도 잠재한다. 대표 코스의 0.35 m/s 앞차로는 `paced`의 이득을 재지 못한다.'),
        ('다음은 대표 155초 코스의 `closing`·`paced` 비교다. 전체 회귀의 실패 16건은 수정 전 복원본에서도 같다. 시뮬레이션 프로세스는 종료했다.',
         '다음은 복귀 교착을 푼 뒤 같은 코스를 다시 비교하는 일이다. 전체 회귀의 실패 16건은 수정 전 복원본에서도 같고 분류를 남겼다. 5시간 사용량 경고로 GPT에 인계했다. 시뮬레이션 프로세스는 종료했다.'),
        ('[앞차 추종과 우회 준비 분리](../../artifacts/validation/2026-11-01/follow_continuity_v1/README.md), [미실행 5랩 준비]',
         '[앞차 추종과 우회 준비 분리](../../artifacts/validation/2026-11-01/follow_continuity_v1/README.md), [대표 코스 비교](../../artifacts/validation/2026-11-01/course_compare_v1/README.md), [미실행 5랩 준비]'),
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
