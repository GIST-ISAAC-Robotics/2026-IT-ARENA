# r497 남은 느린 plan 3호출의 함수 비용

원 r493의 느린 호출은 **부분 관측 평가와 여러 경계 후보 생성이 겹치는 구간**입니다. 두 대표에서는 측방 `threat_view`도 한 plan 안에서 두 번 계산됩니다. 가장 작은 후속 후보는 **같은 호출의 scene/path/config가 실제로 같은 경우에만 threat_view 결과를 재사용하는 것**입니다. 입력 동일성까지 계측한 것은 아니므로 지금 중복 제거가 검증됐다고 주장하지 않습니다. 이번에는 최적화·공통 변경을 하지 않았고, r496/실물 이관을 우선하며 추가 희귀 사례 조사는 중단합니다.

## 측정 범위

r493 동결239파일·원 입력·기존 ARM native를 `/home/q/it-arena-r493-a`에서 그대로 읽었습니다. 508개 plan과 generation356 초기 reset을 원 순서대로 한 번 실행했고 과거 상태를 행마다 주입하지 않았습니다. 원 r493에서 가장 느린 `(356,1304)`, `(356,717)`, `(356,1311)` 세 호출에만 cProfile을 켰습니다. 전체35개의100ms 초과 원인을 전부 분류한 것은 아닙니다.

| sequence / 기준시각 | 원 r493, ms | 이번 profile 포함, ms | 부분 관측 evaluate, 누적 ms | local_path 횟수 / 누적 ms | threat_view 횟수 / 누적 ms |
|---|---:|---:|---:|---:|---:|
| 1304 / 51.32 s | 158.650 | 233.997 | 88.304 | 8 / 38.288 | 2 / 25.180 |
| 717 / 31.62 s | 134.206 | 123.151 | 20.120 | 4 / 13.897 | 1 / 8.669 |
| 1311 / 51.72 s | 130.002 | 204.966 | 56.529 | 7 / 33.780 | 2 / 24.744 |

**profile 포함 수치는 새 처리율이 아닙니다. 누적 시간끼리는 중첩되므로 합산하지 않습니다.** seq717은 profiler를 켠 이번 값이 원 단일 측정값보다 작아, 원 호출 간 차이를 코드 작업량만으로 모두 설명할 수 없습니다. 반복 측정이나 새100ms 충족률 계산은 하지 않았습니다.

## 좁은 원인과 다음 수정 가치

- seq1304의 `partial_surface_alignment.surface_translation` 1회가 누적26.588ms를 차지했습니다. 부분 관측 evaluate 전체는88.304ms이며 seq1311에서도56.529ms가 남았습니다. 원 `local_follow.py:686`은 current route를 선택한 뒤에도 기존 관측 기억 갱신을 유지합니다. 이 호출을 통째로 생략하는 최적화는 상태 보존 범위가 아니므로 권하지 않습니다.
- seq1304/1311의 `threat_view`는 `traffic_solo_gap.eligible`과 `pass_release_trial._select`에서 각각 한 번입니다. 각각 약12ms(profile 포함)이며 내부 `ReturnGrid`/`compact_return_mask` 계산을 다시 수행합니다. **후속 소규모 후보 1순위**는 실제 인자가 같은 호출에 한정한 결과 전달/재사용입니다. TTL 확대·이전 scan 캐시·조건 생략은 필요하지 않습니다. 100ms 이하를 보장할 규모의 증거는 아닙니다.
- `terminal_plan.py`로 표시된 `observed_path`는 동결 `local_follow._baseline_command`를 compile한 경로입니다. seq1304는 선호 벽1회, 이력 support3회, 반대 벽1회로 local_path를 여러 번 만들었습니다. support/refined cloud가 달라질 수 있어 횟수만으로 동일 계산이라 단정하거나 경계 검사를 제거하지 않습니다.
- `native_geometry._geometry`는 세 대표 모두 누적 약19.2–19.5ms입니다. **약1,000이라는 호출 수는 generator resume 횟수이며, 1,000번 전체 scan 검증을 했다는 뜻이 아닙니다.** 점군용·scene용 두 투영 경로가 합쳐진 수치입니다. 무마스크 점군과 마스크 scene 의미가 다르므로 즉시 합치는 변경은 이번 최소 후보보다 범위가 큽니다.

## 동일성·종료

r493 ARM 결과와 명령508/508 동일, 전체 출력507/508 동일입니다. 유일한 차이는 seq1304의 `normal.partial_observation.surface_correspondence.processing_ms`가25.316105→26.577415로 바뀐 것입니다. profiler 영향을 받는 진단 시간이며 원 strict 비교507/508은 그대로 남겼습니다. 이를 임의로508/508 통과로 덮지 않았습니다.

child8219는 최대240초 제한에서 exit0·강제종료 없음으로 끝났습니다. 소유 process group 및 관련 작업/센서 포트 잔류0, r493 source/manifest/입력·출력/native 모두 불변입니다. 센서·ROS·구동·설치·전력·네트워크 설정 변경은0이고 **Jetson은 반환했습니다**. 결과 ZIP SHA256은 `fb3856a8563ca5254d3906138f362cbf4b1cd9a2cf0857ce7daf61caaca58f3a`입니다.

근거는 `analysis.json`, `output_difference.json`, `remote_result/execution_receipt.json` 및 `remote_result/results/profile_356_*.json/.pstats`입니다. 새 처리율·전체10Hz·실물 안정성·최신 공통 전체 성능의 증명으로 확대하지 않습니다.
