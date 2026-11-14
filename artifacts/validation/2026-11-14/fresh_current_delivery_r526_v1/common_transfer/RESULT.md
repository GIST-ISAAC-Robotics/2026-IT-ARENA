# r526 공통 통합 결과

현재 관측으로 재검사한 `current` 결과에 한하여, 실제 출력 `dt`의 기존 조향 변화량 제한을 만족하면 고정 100 ms 발행 대기를 앞당깁니다. 공통 구현 통합과 Windows 계약 검사를 완료했습니다. Jetson의 실제 전달 주기·정지 비율 개선은 아직 검증하지 않았습니다.

## 변경 범위

- `runtime_dual_state.py`: 선택적 `output_dt_s`가 주어진 경우만 조기 전달을 허용합니다. 직전 발행 후 20 ms 이상, 그 이후 원천 시각의 새 current 결과, 유한한 실제 `dt`, 동일한 `previous ± 3 × dt` 범위의 검사된 조향을 요구합니다. 기존 원천·명령·경로 만료, proof, epoch, side, 이전 조향 검사는 그대로 먼저 실행됩니다.
- `runtime_dual_delivery.py`: 선택적 실제 `dt`를 준비·발행 확인·소비까지 전달합니다. 인자를 생략한 기존 호출과 nonretained의 100 ms 분기는 유지합니다.
- `g1_g2_node.py`: 선택된 runtime-dual의 `poll`에만 이미 계산한 `decision_output_dt_s`를 전달합니다. 기존 제어 콜백의 조향 clamp와 같은 `dt`입니다.
- `tests/test_fresh_current_delivery.py`: stage subclass 없이 실제 공통 클래스와 실제 노드 `command` 메서드를 검사하는 17개 계약 검사입니다.

원 source stamp를 갱신하거나 TTL을 늘리지 않습니다. 기존 `pending`/`ack`/`commit` 소유권으로 같은 검사 결과는 한 번만 소비하며, STOP/reset은 미발행 후보를 무효화합니다. 기본 100 ms 이상 간격의 기존 결과와 optional 인자 생략 동작은 유지합니다.

## 보존 및 검증

통합 직전 현재 소스는 `before/`, 통합 후 소스는 `after/`, 이 둘의 변경만 포함한 패치는 `r526_common.patch`에 보존했습니다. 파일별 SHA-256은 `manifest.json`에 있습니다. `g1_g2_node.py` 전체 대조에서 변경은 `poll`의 선택적 인자 연결뿐이며, root가 먼저 통합한 r523의 6 ms drain과 r524의 compact trace는 그대로입니다. nonretained 분기도 통합 전후 동일합니다.

Windows에서 신규 17개와 기존 dual contract/slots/node backend 61개, 합계 **78개 통과, 0.55 s**입니다. `regression.log`와 `regression.xml`에 결과를 보존했습니다. 소스 구문 컴파일과 `git diff --check`도 통과했습니다.

```powershell
$env:PYTHONPATH='src/arena_autonomy;src/arena_vehicle_interface;tests;scripts;artifacts/validation/2026-11-13/operator_panel_r429_v1/pydeps'
$env:PYTHONIOENCODING='utf-8'
python -m pytest tests/test_fresh_current_delivery.py tests/test_runtime_dual_contract.py tests/test_runtime_dual_slots.py tests/test_runtime_node_backend.py -q --tb=short -p no:cacheprovider --junitxml=artifacts/validation/2026-11-14/fresh_current_delivery_r526_v1/common_transfer/regression.xml
```

대표 검증은 40 ms 경과·실제 `dt=35 ms`에서 변화한 조향의 조기 전달, 과한 조향 변화·알 수 없는 `dt`·20 ms 미만 간격의 기존 대기, 즉시 0 전달, 만료·이전 조향·epoch·side·proof 거절, STOP 후 발행 확인 거절, 소비 후 재검사 요구, 기존 100 ms 결과 동일성, legacy 노드 호출 인자 불변입니다.

## 실행 한계와 인계

WSL·Jetson·ROS·실차에는 접근하지 않았습니다. 저장 자료 분석의 관찰 결과는 상위 `RESULT.md`와 `saved_timing.json`을 유지합니다. 이번 계약 검사는 실제 콜백 빈도나 통신 지연을 재현하지 않으므로 `/drive/safe`의 짧은 0이 사라진다는 실시간 성능 증거는 아닙니다. 원천 시각부터 200 ms인 만료 시각도 그대로여서 발행 이후의 처리·통신 지연은 여전히 남은 유효 시간을 소모합니다.

위 네 파일의 소유권을 root에 반환합니다. 추가 공통 변경·반복 검사는 수행하지 않습니다.
