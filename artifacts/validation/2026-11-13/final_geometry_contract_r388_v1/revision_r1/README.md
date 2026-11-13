# r388 r1 — 저장 송신 한계와 물리 조향 한계 구별

상태: **준비만 완료, 미실행**. 원r0 번들·하네스·회수 ZIP·실패 결과는 그대로 보존한다. 원r0 pipeline은 계속false다. 공통 소스/장치/WSL 실행 변경은 없다.

## r0 실패 원인

원r0은 native 빌드와39검사, frozen/legacy 입력 계산을 마친 뒤 explicit의 선행 assert에서 멈췄다. 두 완료 arm은 원r377 기대 출력과 정확히 일치하며, 서로의 전체 출력과 NPZ 내부 질의 배열 바이트도 같다. 원파일543개의 변경은0이었다. 감사는 `../failure_audit_v1/audit.json`에 있다.

| 값 | 수치/단위 |
|---|---:|
| 저장/YAML 축거 | 양쪽0.128m, exact동일 |
| YAML 바퀴 조향한계 | 0.5235987755982988rad(30°) |
| 같은 CAD 원식의 중앙 물리한계 | 0.457843626096037rad |
| 원 launch의 안쪽 float32 송신한계 | 0.4578436017036438rad |
| 저장 FollowConfig의 조향한계 | 0.4578436017036438rad, 송신한계와 exact동일 |
| 물리한계−저장값 | 2.4392393216388086e-08rad ≈0.000001397581° |

물리한계 수치는 보존 CAD 원문의 **Windows 순수 산술 재계산**이다. 원격 explicit 로그는 raw물리한계 값을 찍기 전에 assert로 끝났으므로 원격 raw double의 마지막 비트까지 관측했다고 주장하지 않는다. 저장값은 원 launch가 Ackermann float32 직렬화로 물리/MCU 상한을 넘지 않게 적용한 `_float32_limit_not_above`의 결과와 정확히 같다. 축거나 CAD 선택이 달라진 근거가 없으며, 하네스가 이 변환을 빠뜨린 것이 직접 원인이다.

## 최소 변경

저장 대조의 `compare_last.py`는 원 launch의 float32 함수 원문을 `original_float32_limit.py`로 보존하여 물리한계→송신한계를 계산한 뒤 저장값과 exact비교한다. tolerance를 넣지 않는다. 실제 저장 `cfg.wheelbase_m`와 `cfg.max_steering_rad`는 수정하지 않으며 원 목표·현재/이전 조향·시각·이력·후보를 그대로 두고 윤거/바퀴 반경/폭3필드만 `replace`한다. 실패 전에도 `geometry_precondition.json`에 양쪽 수치를 남긴다. aggregate는 기존 출력/질의 완전동일과 함께 축거/송신한계 불변을 검사한다.

별도 staged 보완은 `vehicle_geometry.py`에 같은 float32 함수 본문을 옮기고 simulation의 기존 helper는 이를 위임한다. 명시 vehicle YAML을 받은 preview도 물리한계→송신한계 변환을 적용한다. 생략 시 기존 preview 설정은 그대로다. 이 preview/launch 경로는 저장 대조에서 실행되지 않으므로 기하3치수 인과대조에 새 조향한계를 섞지 않는다.

원본/하네스 사본/공유함수 본문이 AST로 같은지 번들 준비 단계에서 검사한다. 순수 테스트에 유효 기존 launch 송신 출력 exact동일, ideal/CAD preview 송신한계≤물리한계, 저장 header의 송신값 exact동일을 추가했다. 기존39검사는 그대로 포함하고 추가10검사로 총49개를 예상하지만 **미실행이므로 통과 수로 표기하지 않는다.**

## 보존과 실행 경계

`staged/`9파일과 `base/`8원문, 새 full patch/manifest를 r1에 따로 보관한다. 번들은 원r0의 동결r377 소스256개에서만 재구성한다. 원후보/모델기하/보호선/속도/시간/물성/CAD 표는 변경하지 않는다. frozen/geometry arm의 소스차이는 지정9파일뿐이다. r386은 계속 미포함/미실행이다.

지정 실행자는 fresh `$HOME/it-arena-r388-r1-*` 루트에서 동일 native1회·순수회귀1회·frozen/legacy/explicit 각1회·aggregate1회를 실행한다. 단계별120s 제한, 기존 결과/루트 덮어쓰기 금지, 소유 자식만 종료한다. `HANDOFF.md`에 고정 명령과 해시를 기록한다. 이번 준비에서는 AST/바이트/해시/ZIP검증만 수행했고 제어 호출·빌드·시험 실행은0이다.

마지막 저장1입력 대조가 성공해도 원r0 실행 성공으로 소급하지 않는다. 명시기하의 정지/통과 양쪽 모두 전체 결과와 실제 질의 차이를 보존한다. 새 주행, 정체 해소, 전체랩, 실차 증명은 아니다.
