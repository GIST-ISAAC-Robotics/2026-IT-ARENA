# r526 fresh current 결과 전달 stage

**100 ms 고정 대기 대신, 이미 완료된 current 결과의 조향 변화가 실제 출력 간격의 `3 rad/s` 예산 안에 있으면 일찍 전달하는 후보 하나를 준비했다.** 새 계산·새 stamp·TTL 연장·오래된 점유 허가 재사용은 없다. 공통/WSL/Jetson을 변경하지 않았으며 실시간 빈도·물리 성능은 미검증이다.

## 현재 계약과 병목

- `runtime_node_backend.control()`은 이미 `decision_output_dt_s = min(.1, now-last_request_time_s)`를 control 시작에 계산한다.
- `g1_g2_node.command()`는 이 실제 dt를 current 재검사 snapshot에 넣는다. 일반 plan snapshot은 여전히 `.1`이다.
- current 계산은 그 dt로 요청 조향을 제한하고 실제 점유를 검사한다. 발행하는 `WallFollow.control()`도 같은 control 시작 dt로 최종 `3 rad/s` clamp를 적용한다.
- 그런데 `RuntimeDualProcess.application_reason()`은 조향 변화가 작아도 양수 적용 사이 **100 ms**를 반드시 기다린다. 원 command reference는 그동안 그대로다.
- 선택형 runtime은 이미 50 Hz control을 요구한다(`g1_g2_node.py`, r523 profile 모두 확인). 후보도 **20 ms보다 빠른 반복 발행을 허용하지 않는다.** timer/계산 주기를 새로 올리지 않는다.

동일각일 때만 예외를 주는 안은 제외했다. r523의 양수 `/drive` 176개 조향이 모두 달라 연속 동일각은 0개였다.

## 저장 r523 근거

[`saved_timing.json`](saved_timing.json), [`analyze_saved.py`](analyze_saved.py)는 원 source stamp·control callback 구간·계산 회수 영수증만 사용한다. 원 파일 hash와 바이트 불변을 확인했다.

| 지표 | 저장 기록 결과 |
|---|---:|
| 양수 `/drive` / current 계산과 source stamp 정확 대응 | 176 / 176 |
| current 회수 182개 중 전달 대응 없음 | 6 |
| current 회수 → command 관찰자 수신 | 중앙 58.113 ms / p95 79.400 ms / 최대 104.534 ms |
| 직전 command 관찰 후 100 ms 전에 current 회수 | 169 / 176 |
| 실제 연속 조향 변화에 필요한 `3 rad/s` 시간 | 중앙 4.951 ms / p95 22.661 ms / 최대 77.882 ms |
| 조향 변화가 20 ms 예산 안인 양수 명령 | 162 / 176 |
| 도착한 명령의 남은 200 ms 수명 | 중앙 101.390 ms / 최소 59.363 ms |

current 제출 시각을 포함하는 **유일한 controller callback**의 reference와 `/drive` 원 stamp를 대응시켰다. 회수→관찰자 수신에는 callback 나머지 작업·발행·구독자 스케줄링도 들어가므로 전부 100 ms gate 때문이라고 주장하지 않는다. 조향 차이는 관찰된 float32 값의 진단이며 이를 원 checked float64 명령 대신 사용하지 않는다.

r523은 상세 trace OFF이므로 대기 중인 ready 전체 출력·당시 실제 출력 dt·모든 pending 전이가 없다. 따라서 17.95%의 0 목표 시간을 얼마 줄이는지 정확한 정책 재생/반사실 수치로 만들 수 없다. 이번 후보가 실제 대표의 current 전달 경로에 해당한다는 근거와, 최소 계약 시험을 분리한다.

## 최소 후보

[`stage.py`](stage.py)의 `FreshCurrentProcess`/`FreshCurrentDelivery`는 기존 구현을 재사용한다. current 결과만 대상으로 한다.

1. 원 `application_reason()`의 source/command/geometry TTL, generation/revision, epoch, side, 직전 요청각 동일성, 현재 proof 검사를 먼저 그대로 실행한다.
2. **유일한 거절 이유가 `positive_application_interval`일 때만** 아래를 본다.
   - 현재 reference가 직전 실제 발행 기록보다 새로워야 한다.
   - 직전 발행 이후 최소 20 ms가 지나야 한다. 기존 시간 비교 허용오차 `1e-9 s`만 유지한다.
   - caller가 control 시작 때 계산한 실제 `output_dt_s`가 유한하며 `0 < dt <= min(.1, elapsed)`여야 한다.
   - 원 checked angle이 `previous ± 3*dt` 안이어야 한다. 별도 각도 허용오차·보간·각도 수정은 없다.
3. 위 조건이면 동일 output/원 reference를 전달한다. 이후 기존 clamp가 각도를 바꾸지 않으므로 기존 점유 검사 각도와 실제 발행각이 일치한다.
4. 기존 pending → publication ack → commit/consume를 그대로 사용한다. 동일 checked 결과는 한 번만 소비하고, STOP/reset은 pending/ready를 취소한다. next command는 새 current 계산 결과가 있어야 한다.

`nonretained` plan의 100 ms 제한과 plan `.1` snapshot은 **변경하지 않는다**. stage가 새 검사를 요청하거나 동일 관측을 중복 계산하여 stamp를 갱신하는 기능도 없다. 단, 슬롯이 더 일찍 비면 기존 scheduler의 실제 작업 시작 순서/계산량은 달라질 수 있어 실시간 부하는 재측정 대상이다.

### 왜 실제 dt를 전달하는가

poll 직전 elapsed만 보고 허용하면, control 시작 시 이미 계산해 둔 `decision_output_dt_s`보다 큰 조향 예산을 잘못 허용할 수 있다. 그러면 발행 직전 기존 clamp가 checked angle을 다시 잘라 점유 검사 각도와 달라진다. 후보는 **기존 최종 clamp가 사용할 더 작은 dt**까지 확인한다. ack에도 같은 dt를 전달하여 판단을 일관되게 유지한다.

### 남는 유효기간

명령의 절대 만료시각은 항상 **`current_reference + .2 s`**다. 일찍 전달해도 이 시각은 늘지 않는다. 해당 조기 분기에서는 `current_reference > last_publication`이고 적용 간격이 100 ms 미만이므로 **poll 시점의 명령 잔여 수명은 100 ms보다 크다**. 실제 발행/guard 도착까지 추가 지연이 있으면 그만큼 줄어든다. 다음 계산 완료·전송 지연에 대한 최악 상한이 없으므로 무중단 TTL 유지나 일정한 실효 Hz를 보장할 수 없다.

계약 예: 마지막 발행1.08 s, fresh current reference1.10 s, 결과 준비1.12 s, 실제 출력dt35 ms, 조향 .10→.15 rad. 원 정책은1.18 s까지 대기하지만 새 후보는40 ms 간격에서 그대로 전달 가능하다. `.05 rad <= 3*.035`이며 command 잔여 수명은180 ms다. 원 정책의1.18 s 전달에서는120 ms이며 둘의 절대 만료는같은1.30 s다.

## 검증과 최소 공통 연결

[`focused.log`](focused.log)/[`focused.xml`](focused.xml): **Windows 순수 계약15개 통과(0.20 s)**. 원 checked angle/stamp/proof 무변경, 조향 예산 초과 대기, 20 ms 하한, 알 수 없는 dt의 기존 동작, 0 즉시 전달, source 만료·epoch·side·previous-angle 거절, retained proof 거절, STOP 취소, 중복 ack 방지, nonretained 불변, 기존100 ms 정상 결과 완전 동일을 확인했다. 실제 ROS 발행이나 guard 물리 시험은 아니다.

통합 선택 시 수정 범위는 작다.

- `RuntimeDualDelivery.poll(..., output_dt_s=None)`: current의 `state`에 실제 dt를 추가하여 기존 pending/ack까지 전달.
- `RuntimeDualProcess.application_reason(..., output_dt_s=None)`: 마지막100 ms 거절만 위 helper로 대체. dt가 없으면 기존100 ms 정책 유지.
- `G1G2Node.command()`의 **runtime dual 분기만** `output_dt_s=self.decision_output_dt_s`를 전달. 다른 delivery 호출에는 새 인자를 넘기지 않음.

공통 구현 검토·통합은 root 소유다. Jetson 전원 OFF 상태에서 추가 실행하지 않았다. 최초 실패는 zero fixture의 `reason` 누락 하나였으며 [`focused_initial_fixture_failure.log`](focused_initial_fixture_failure.log)에 보존했다.
