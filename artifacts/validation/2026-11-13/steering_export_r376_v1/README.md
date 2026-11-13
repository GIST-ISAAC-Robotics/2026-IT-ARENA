# 실장 영점 프로필의 C++ 전달 r376

2026-11-13. 조립 뒤 기록할 영점·회전 방향·허용 count 구간을 Python JSON에서 MCU용 C++ `ZeroProfile`로 내보내는 경로를 추가했다. **현재 실장 교정값은 없으며 이번 값은 합성 시험용이다.** 실제 구동 펌웨어를 설치하거나 출력 HAL을 활성화하지 않았다.

## 사용 경로

기존 [`steering_zero_profile.py`](../../../../scripts/steering_zero_profile.py)의 `record`→`confirm-mount`→`goal` 뒤 다음 명령을 사용할 수 있다.

```text
python scripts/steering_zero_profile.py export-cpp <확인된 프로필.json> --assembly-id <같은 장착 판본> --output <새 헤더.hpp>
```

같은 장착 판본, 방향/끝점/위치 모드 확인, CAD 전체 요구 범위가 허용 count 안에 있는지 검사한다. 기존 파일은 덮어쓰지 않는다. 헤더에는 사용자 메모를 삽입하지 않고 정규화한 JSON 해시와 CAD 표 해시를 넣는다. 이 해시는 원본 파일의 공백·줄바꿈까지 포함한 해시와 다르다. 헤더를 만드는 것만으로 MCU가 연결되거나 모터·서보가 움직이지 않는다.

생성된 `arena_steering_mount::Profile()`은 기존 `ResolveCadServoGoal`에 전달할 값이다. **운영 빌드의 프로필 선택·실제 출력 HAL·실물 장착 확인은 남아 있다.** 합성 예시 count2137을 실제 중립으로 복사하지 않는다. 현재 확인 전 기본값을 자동 승인하거나 EEPROM을 바꾸는 기능도 없다.

## 확인 결과

[`verify.py`](verify.py)로 기존 Python15회귀와 C++17 엄격 컴파일(`-Wall -Wextra -Werror`)을 수행했다. 합성 중립2137·허용1600~2700에서 방향±1을 각각 내보내고 중앙 조향 명령201개씩, 총402개를 비교했다. 목표 count는 모두 같고 좌우/중앙각 최대 차이는1.11×10⁻¹⁶rad였다. CAD 양 끝의 한 count 안쪽 반올림, 범위 밖 요청 거절, 잘못된 장착 ID/미완료 확인/전체 범위 부족/기존 출력 파일 덮어쓰기 거절을 확인했다.

- [결과·검사 소스 해시](native_result/result.json)
- [Python 회귀 로그](native_result/unit.log)
- [시험 당시 소스](native_result/tested_sources.zip)

검사 중 소스는 불변이며 결과는 파일 전달과 수치 동등성에 한정한다. 실물 조향각·유격·간섭·전류와 동적 서보 속도는 확인하지 않았다.
