# r393 같은 호스트 ROS 원천시각의 단조 구간 변환 준비

상태: **57순수+설치된 ROS CDR1검사 통과 후 공통 이관 완료**. 아래 준비 절은 당시 작업 범위로 보존한다. 실물 시계 교정이나 장치 구동을 검증한 것은 아니며 실제 host 연결·STOP·세션 재시작은 r394에 이어진다.

`staged/scripts/same_host_command_clock.py`는 같은 Jetson의 물리 시스템시각 기반 `/drive/safe` 원 header를 `HostTimeWindow`로 변환하는 좁은 helper다. MCU↔host offset fit과 별개이며 명령/통신/clock교정을 만들지 않는다. 기존 `clock_pair_audit.ClockPair/capture_pair`, `clock_freshness.HostTimeWindow`를 직접 재사용한다. `host_clock_pair.sample_clocks`와 같은 monotonic-before→ROS→monotonic-after bracket을 쓰되, 여러 읽기 중 최소폭 midpoint만 고르면 연속 변화 검사를 빠뜨릴 수 있으므로 기존 `capture_pair`의 모든 단일표본을 유지한다. 테스트에서 두 helper의 단일 raw bracket 의미를 대조한다.

## API와 호출 경계

```python
bounds = SameHostClockBounds(
    source_ttl_ns=source_ttl_ns,
    max_lookback_ns=max_lookback_ns,
    max_sample_gap_ns=max_sample_gap_ns,
    max_read_width_ns=max_read_width_ns,
    ros_timestamp_error_ns=ros_error_ns,
    host_timestamp_error_ns=host_error_ns,
    max_rate_error_ppm=rate_error_ppm,
    assume_no_unsampled_clock_step=True,
    policy_note=assumption_note,
)
clock = SameHostCommandClock(host_epoch=host_epoch, ros_epoch=ros_epoch, bounds=bounds)
pair = clock.capture(monotonic_ns=monotonic_ns, ros_now_ns=ros_now_ns, read_mode=read_mode)
# 최소2개 연속 정상 pair 확보. command receipt와 processing 시각을 원 stamp 대신 넣지 않는다.
converted = clock.map_source(original_header_ns, source_ros_epoch=ros_epoch, now=now_window)
# 상위가 검토한 뒤 r391에 source=converted.source, now=now_window로 전달한다.
```

`observe(ClockPair)`로 이미 bracket을 기록한 동일호스트 표본도 받을 수 있다. 첫sequence0, 이후연속+1이어야 한다. `capture`는 기존 함수 순서대로 mode→mono→ROS→mono→mode를 읽는다. `read_mode`는 `('SYSTEM_TIME',False)` 또는 override가 꺼진 `('ROS_TIME',False)`여야 한다. simulated ROS time/steady clock, 읽기중 및 표본간 mode변경은 거절한다. host/ROS epoch 변경, sequence 누락, ROS0/역행/정지, host역행/겹침, rate경계 위반은 latch하는 clock fault다.

`CommandClockError.reason`으로 거절 이유를 구분한다. `.clock_fault=True`인 시계고장은 객체에 latch되고 새 pair로 자동 복구하지 않는다. `.fault`로 현재고장을 조회할 수 있다. 재시작은 새 객체/상위 세션에서 수행하며 이 helper에 reset·STOP·출력기능은 없다. 잘못되거나 만료된 command source는 변환만 거절하며 같은 clock객체의 정상 표본을 버리지는 않는다.

**넓은 읽기·장기공백·오래된 latest pair는 scheduling 지연일 수 있으므로 영구 clock fault로 단정하지 않는다.** 이 경우 `.clock_fault=False, .transient=True`로 거절하고 현재 변환 segment만 버린다. 이후 좁고 정상인 pair2개를 받으면 자동으로 `.ready=True`가 된다. 넓은 pair는 재수립에 세지 않고, 장기공백 뒤의 좁은 pair는 새 첫표본으로 센다. **재수립된 두 번째 정상pair 시각을 새 source 하한으로 고정**하므로 그 이전/복구중 source를 새anchor로 재활용하지 않는다. 오차가 있으면 그 이후 순서가 확실한 source와 후속anchor가 필요하다.

최소 상태는 `.ready`, `.sample_count`(현segment), `.observed_count`(전체sequence), `.reanchors`, `.transient_reason`이다. 새source가 없으면 이전명령의 MCUexpiry는 그대로 만료되며 상위 r391 watermark/ARM/세대는 이 helper가 초기화하지 않는다. 마지막 관측pair와 마지막 now는 transient에도 남겨 이후 역행/모드·epoch/rate불일치를 계속 검사한다. 겹친 읽기나 불일치가 실제로 생기면 그때는 latch다.

`now`는 호출자가 같은 monotonic에서 읽은 `HostTimeWindow`다. 그 자체의 읽기오차는 이미 구간에 포함해야 한다. latest pair 이후여야 하며 이전 map의 now와 역행/겹침도 거절한다. 다음 pair 역시 마지막 map now 이후여야 한다. 단일 소유자 순차 호출을 전제한다. 중복 source 소비/ARM barrier/세대/STOP는 기존 r391 watermark와 세션이 담당한다.

## 구간식과 판정

`r = dROS/dMONOTONIC ∈ [1−ρ,1+ρ]`, `ρ=ppm/10⁶`. 최신표본의 host bracket을 `[B,A]`, ROS측정값을`R`, 원source를`S`, per-ROS-stamp 오차를`eR`, per-host-read 오차를`eH`로 둔다. 같은 오차경계를 시스템ROS읽기와 원header stamp 모두에 적용한다.

```text
Δ = R − S
elapsed_lower = floor((Δ−2eR)/(1+ρ))
elapsed_upper = ceil ((Δ+2eR)/(1−ρ))
source_host = [B−eH−elapsed_upper, A+eH−elapsed_lower]
age = [now.lower−source_host.upper, now.upper−source_host.lower]
```

모든 계산은 정수ns의 곱/나눗셈이며 큰 epoch offset을 float로 바꾸지 않는다. floor/ceil은 구간 바깥쪽이다. `Δ≥2eR`, `S−firstROS≥2eR`로 source가 최초관측 이후·최신anchor 이전이라는 순서를 확정해야 한다. 최신 pair가 source보다 앞이면 새 pair가 필요하며 앞으로 외삽하지 않는다. 처음관측보다 오래된 source를 임의 offset으로 살리지 않는다.0source도 거절한다.

연속 pair의 실제 host경과는 `[max(0,B₂−A₁−2eH), A₂−B₁+2eH]`, ROS경과는 `[R₂−R₁−2eR,R₂−R₁+2eR]`에 있다. 두 구간이 선언 rate범위와 양립하지 않으면 `clock_step_or_rate_bound_exceeded`다. 이를 NTP/드리프트/step의 물리원인 중 하나로 확정하지 않는다. 샘플폭과 최악 sample gap도 명시상한 내여야 한다.

source의 최악 lookback이 명시 `max_lookback_ns` 이내, latest pair의 최악 나이가 `max_sample_gap_ns` 이내여야 한다. `age.lower≥0` 및 `age.upper<source_ttl_ns`일 때만 변환 결과를 반환한다. TTL 경계는 strict이며 오차를TTL에서 빼거나100ms를 자동 늘리지 않는다. 현재/수신시각은 age검사에만 쓰고 원source를 그 시각으로 재작성하지 않는다. 반환에 원 `source_ros_ns`, source/now구간, age구간, anchor/first sequence를 남긴다.

## 가정과 한계

한 호스트의 kernel MONOTONIC과 physical ROS 시스템시각 사이에 표본간 불연속이 없고 상대rate·읽기오차가 선언값 이내라는 전제다. 이 조건을 **표본으로 완벽히 입증하지 않는다**. 표본 사이 jump후복귀나 오차안에 숨은 jump는 검출하지 못할 수 있다. 들쑥날쑥 offset을 constantoffset fit으로 맞춰 입증했다고 하지 않는다. `hardware_assumptions_verified=False`, `unsampled_clock_steps_excluded_by_observation=False`를 결과에 남긴다.

ROS Header 자체에는 boot/host/ROS epoch와 생성기 provenance가 없다. 상위는 같은호스트 생성기·동일물리시계·epoch를 보장해야 하고, 연결/재시작/clock mode변화를 알아차리면 새epoch로 세션을 닫아야 한다. 이 helper에 잘못 선언한 epoch를 진짜로 검증하는 기능은 없다. 모드가 바뀌었다가 표본전에 돌아온 경우도 표본만으로는 보장할 수 없다. 원시 system stamp와 kernel MONOTONIC 관계의 가정은 MCU rate가정과 따로 설정해야 한다.

같은 건강한 segment에서 동일 source를 새anchor로 다시 변환하면 불확실도구간은 달라질 수 있지만, 원 source값은 그대로이고 age가0으로 새로 시작하지 않는다. 상위는 이미 소비한 source를 재발급하지 않아야 한다. transient 재수립을 넘는 이전source는 helper도 거절한다. STOP·세션재시작은 상위에 맡기며 scheduling지연 자체 때문에 영구 재시작을 요구하지 않는다.

## 검증 준비

순수 tests는 원 timestamp나이, arbitrary큰epoch offset, 수신/처리지연,rate양끝, 정수floor/ceil, 읽기폭/오차, 두 기존표본 helper의 raw bracket, 최초관측/미래/0/이전epoch/역행·step·simtime·공백·만료/불확실도, 동일source 재변환, clock fault latch와 scheduling transient 재수립/이전source 거절, unsampled jump 한계를 다룬다. 외부 환경·device를 열지 않고 합성 callback만 쓴다.

지정 담당자의 고정 순수검사 명령(기존 공통 dependency만 읽음):

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -B -m pytest -q 'artifacts/validation/2026-11-13/same_host_command_clock_r393_v1/tests/test_same_host_command_clock.py'
```

준비자는 AST/원문/해시만 확인했다. 아직 pytest/import/ROS/WSL/Jetson 실행으로 통과를 확인하지 않았다. `source_manifest.json`은 staged/helper/tests와 읽는 기존dependency 해시를 보존한다. 통과해도 실물 rate/error범위 보증이나 구동허가가 아니다.

## 검증·공통 이관

`verify_v1`에서 시간 경계57개와 설치된 ROS CDR 검사1개, 총58개가 통과했다. 동결 입력 전후 불변, skip0이며 실제 시계 교정/장치 실행은 아니다. 구현과 artifact 경로 의존을 제거한 검사를 공통에 이관했다. sample gap/넓은 읽기는 정상 두 표본으로 재수립하고, 실제 역행·모드/세대 변경·rate 불일치와 구분한다.
