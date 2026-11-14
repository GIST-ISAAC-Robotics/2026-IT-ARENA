# r540 — r538 guard helper 비용과 동일각도 중복

**r538의 추가 연산은 확인됐지만, 현재 자료만으로 실시간 병목이라고 판정할 근거는 없다.** Windows에서 네 대표 크기의 swept 부분은 약0.1–0.2 ms 중앙값이었다. 전체 guard/ARM 비용은 미측정이다. 정확히 같은 현재·직전 각도의 중복만 제거하는 작은 stage는 출력·기억 상태 동등성을 확인했으므로 이관을 제안한다. 공통 및 실행 중 r539 소스는 수정하지 않았다.

고정 r538 SHA `dc81494c5162d0e07851e835bee7dfc96fac303a857fe0c922da50959647b26d`와 `before.py` SHA `0002afdceab8fe45675123d41371730fd99309e6efc5527a192ff00c16ae7e0a`를 사용했다. r530 `guard_event_extract` 네 사건과 r538의 저장670개 비교 결과를 읽었으며 원시670 callback을 다시 실행하지 않았다. [집계·원자료 SHA](cost_result.json), [분석 코드](analyze_cost.py).

| 저장583개 sweep 실행 helper의 지표 | r538 이전 | r538 | 동일각도 재사용 stage |
|---|---:|---:|---:|
| `swept_limit` 호출 수 | 2,076 | 2,076 | **1,449** |
| 점×자세 평가 수 | 4,270,004 | 6,927,014 | **4,824,263** |
| horizon 중앙 / 최대 | 0.200 / 0.731 m | 0.511 / 0.731 m | 동일 |

r538은 호출 수를 늘리지 않고 요청 속도에 맞춰 horizon을 늘린다. 점×자세 평가는 저장 horizon·clearance와 기존4 cm/8행 batch로 산출한 연산량이며 CPU cycle이 아니다. 이 대표에서는 저장 clearance가 모두 horizon까지 열려 있어 조기 종료 감소가 없다. 연산량은 이전 대비 **62.2% 증가**, 중복 제거 뒤는 r538 대비 **30.4% 감소**하고 이전 대비13.0% 증가다.

raw는465–480점, retained 최대113점이다. **113점 구간의 horizon은0.20 m**이고 최대0.731 m 구간은 retained0이다. 각 최대를 동시에 곱해 실제 최악 부하로 취급하지 않았다. 기억 운반·재관측·광선 검사는 r538 전후 및 최적화에서 그대로다. 같은 scan이어도 자차 좌표·명령이 달라질 수 있으므로 callback 사이 결과 cache는 제안하지 않는다.

## 최소 stage와 동등성

현재/직전 각도가 **정확히 같은 호출352/583개(60.4%)**에서 `angles=(steering,)`로 계산한 clearance를 두 필드에 사용한다. 근사 동일각 허용오차를 추가하지 않는다. 각도가 다르면 원 두 계산, 기억이 비었으면 raw만 계산한다. 순서·horizon·검사·기억/원본/래치·STOP은 동일하다.

[stage/guard_memory_command.py](stage/guard_memory_command.py), [최소 patch](same_angle.patch), [SHA](manifest.json). 후보 SHA는 `0bc97fe6ec5e1fa596fa0f536f9706b85c09b4fd920c78b3165df163bc957742`이다.

[동등성 검사](equivalence.json): **9 fixture/10 helper 호출**의 반환 dict 전체와 `memory.__dict__` 전체를 비교했다. 배열은 bytes·dtype·shape·writeability까지, 래치 증거·원 scan ranges·운동 표본도 일치한다. 113점 동일각4→2회, 다른 각도/1e−14 차이4→4회, 빈 기억2→1회, STOP0→0회다. 다음 callback에서 조향을 바꾸면 기존처럼 다시 검사하여 관련성/상한이 달라지고 두 구현은 동일했다. [검사 코드](verify_equivalence.py). 저장670개 전체 상태 재생 검증으로 확대하지 않는다.

## 제한된 Windows 비용

저장 사건의 점 개수·각도·horizon을 사용하고 좌표는 간섭 없는 합성점으로 둔 **swept 코드 부분만** 각9회 측정했다. 실제 saved geometry나 전체 helper 비용 측정이 아니다. 코드·제동 horizon·4 cm 표본열은 동결 원문을 사용했다.

| 대표 | retained / raw | r538 중앙 | stage 중앙 |
|---|---:|---:|---:|
| 최대 기억, horizon0.20 m | 113 / 471 | 0.1124 ms | **0.0557 ms** |
| 기억 해소, horizon0.386 m | 0 / 470 | 0.1145 ms | **0.0574 ms** |
| 후반 옛 기억, horizon0.448 m | 3 / 476 | 0.2004 ms | **0.1013 ms** |

후반 대표의 변경 전 horizon0.20 m에서는0.1039 ms였다. 서로 다른 두 각도 대표는 계산 수가4회로 같으며 관찰된 짧은 시간 차이를 개선으로 해석하지 않는다. Windows Python/NumPy fallback·소수 표본·동시 PC 작업의 영향이 있는 수치다. **ARM 환산·20 ms 마감·전체 guard 지연·주행 성능을 주장하지 않는다.** Jetson/WSL/ROS 실행0, 새 대규모 행렬이나 추가 매개변수 탐색 없이 마쳤다.

## 공통 채택

r539의 종료·WSL 반환 뒤 root가 정확 동일각도 계산만 재사용하는 생산1파일을 채택했다. r541 native 묶음의 관련110회귀에 guard memory/독립 관련성 검사도 포함돼 통과했다. r539는 변경 전r538 SHA로 수행한 결과이며 새최적화로 소급하지 않는다. `common_transfer.json`에이관SHA를보존했다.
