# 동일 판단 안 native 투영 공유 후보 r389

2026-11-13. **정확성·비용 실행 완료, 공통 이관은 미채택이다.** Windows에서 소스 추적·AST 구문 검사·diff/해시 준비만 수행했다. 공통 `src/`, r388 담당 파일, WSL, Jetson, 장치는 수정하거나 실행하지 않았다. 채택·실시간 달성 결과가 아니다.

## 중복 경로와 줄이는 계산

계획 자식의 `follow_trace.replay_call`은 `motion_scene → measurement_motion_scene → native_motion_scene`으로 장면을 만든다. 같은 호출의 `LocalFollow._partial_command → KnownStructurePolicy.evaluate → RecoveryPolicy._evaluate → TimedPolicy.evaluate`는 `project_scan → measurement_motion_points → native_motion_points`로 같은 원천 입력의 점군을 다시 만들고, 순서/좌표를 대조하여 취득 인덱스를 연결한다. `RecoveryPolicy._evaluate`의 특정 래치 회복 분기에는 세 번째 `project_scan`도 있다. 현재 공통 구현에는 이 기하 공유가 없다.

정확한 연결은 다음 파일의 해당 함수다.

- [`follow_trace.replay_call`](../../../../src/arena_autonomy/arena_autonomy/follow_trace.py): 스캔/운동 이력을 복원하고 장면 생성 후 같은 `scan`, `history`, 기준 시각을 제어기에 전달한다.
- [`LocalFollow._partial_command`](../../../../src/arena_autonomy/arena_autonomy/local_follow.py): 원 스캔과 장면 반환 인덱스를 대조하고 장면 점군을 부분관측 정책에 전달한다.
- [`TimedPolicy.evaluate`, `RecoveryPolicy._evaluate`](../../../../src/arena_autonomy/arena_autonomy/traffic_recovery_trial.py): 같은 점군을 재투영하여 배열 순서/좌표를 대조한다. 이 대조와 취득 인덱스 연결은 유지한다.
- [`_measurement_transforms`](../../../../src/arena_autonomy/arena_autonomy/scene_motion.py): 시각 정책/센서 시각 신뢰/프레임/미래·만료/운동 이력과 합성 epoch 검사를 수행한다. 후보에서도 points 요청마다 이 함수가 실행된다.
- [`native_geometry._geometry`](../../../../src/arena_autonomy/arena_autonomy/native_geometry.py): 원점·방향·끝점의 삼각함수/유한성 계산이 중복되는 실제 위치다.

저장 [r359 계획 프로파일](../latest_plan_profile_r359_v1/runtime/runtime_result/plan_profile.json)의 index2는 `_geometry` 호출1018회, 누적19.555ms다. 508광선의 두 순회는 각508회 yield와 종료1회씩이므로1018회와 일치한다. 같은 표본의 `native_motion_scene`은15.581ms, `project_scan`은9.515ms다. 나머지 고정5표본도1018/1026회, 누적20.185–21.241ms다. 이는 cProfile 비용을 포함한 관측이며 **19–21ms 전부를 절감한다는 뜻이 아니다.** points의 반복 삼각함수·기하 검사 일부를 줄일 여지가 있다는 근거다. 현재 시점의 비계측 절감량은 미측정이다.

장면에는 마스크와 세대 의미가 있다. scene의 epoch은 센서+motion epoch 합성이고, `measurement_motion_points`는 motion epoch0을 사용한다. 이 차이를 같다고 취급하지 않는다. 현재 보수적 후보는 양쪽 epoch까지 같은 경우에만 공유한다. 대표 r353399입력은 저장 JSON을 확인한 결과 source/motion epoch이 모두0이며, 실제 공유 빈도는 실행 보고서에서 따로 계수한다. 다른 시각의 현재 재검사와 이전 계획을 공유하지 않는다. 부모 ROS의 points→scene 순서 최적화도 이번 범위에 포함하지 않는다.

## 후보와 운영 연결의 구분

[`staged/native_projection.py`](staged/native_projection.py)는 원 `native_motion_scene`을 그대로 호출한 뒤 성공한 장면 하나를 가진다. points가 요청될 때 이미 검증된 원점·방향과 반환 hit를 사용한다. `masked`인 유효 거리도 raw points에는 원 산식으로 복원한다. 장면의 `masked/unknown`은 계속 `hit=None`이며 자유 공간이나 반환으로 바꾸지 않는다. 반환 배열·인덱스는 매번 새로 만들므로 호출자의 배열 변경이 장면/후속 반환으로 전파되지 않는다.

**공유 시 생략하는 것은 두 번째 원점·방향 삼각함수 계산뿐이다.** points 쪽의 시간/이력/미래/만료 검증, `_measurement_transforms`, 상위 점 순서/좌표 대조는 유지한다. 원 scene 생성 중에는 masked 끝점 overflow를 검사하지 않으므로 후보도 raw points 요청 전에는 이를 계산/거절하지 않는다. points 호출에서 원래와 같은 `ValueError('hit must be finite numeric')`를 발생시킨다. source metadata, 취득 시각, 광선 수와 순서는 원 scene 함수가 그대로 만든다.

[`candidate.py`](candidate.py)는 **오프라인 직렬 한 `replay_call`에만** 이 공유를 연결한다. 한 스캔 객체·동일 config 객체/장착값·기준 시각·epoch·transforms dtype/shape/바이트가 같을 때만 사용하고 나머지는 원 함수로 돌아간다. `MeasurementScan`/배열/config의 비표준 형식, 일회용 마스크는 원 경로를 유지한다. context 종료나 예외 시 함수 참조를 복원하며 공유 객체를 버린다. 스캔/시각/프로세스 사이 전역 캐시는 없다. **monkeypatch와 배열 바이트 대조는 효과 검증용이며 운영 설치 API가 아니다.**

운영 소스에 그대로 이식하기 전에 비용 결과를 검토한다. 효과가 있으면 `replay_call`의 새로 복원한 입력을 소유하는 **명시적인 한 호출 문맥**에서 장면/원점·방향을 보관하고, 같은 문맥의 부분관측 points 요청으로 전달하는 연결을 제안한다. 해당 문맥 안에서 검증한 입력을 명시적으로 전달하여 전역 함수 교체나 반복 대량 바이트 비교를 운영에 넣지 않는다. 시간/이력 검증 순서를 유지하고, scene과 points의 epoch이 다른 경우는 기존 경로를 유지해야 한다. 이 API 전달이 여러 제어기 래퍼 변경으로 커지면 이번 결과만 남기고 통합을 보류한다. `local_follow`, `local_pursuit` 등 r388 소유 파일은 현재 patch에 포함하지 않았다.

| 선택 | 계산/상태 비용 | 원 동작 보존 부담 | 판단 |
|---|---|---|---|
| 성공한 scene 기하에서 요청 시 raw points 생성 | 추가 원점/방향 배열 없음. 유효점 순회와 fresh 배열 생성. offline 대조용 transforms 사본 약12KiB/508광선 | masked 끝점은 실제 points 요청 시 검사하므로 원 실패 순서 유지가 쉽다 | 이번 단일 후보 |
| scene 생성 때 native points도 한 번 계산해 저장 | 점/인덱스 배열을 미리 만들고 저장. points를 요청하지 않아도 계산. 다시 원 `native_motion_points`를 부르면 중복 삼각함수가 그대로 남는다 | 순회를 합쳐도 masked overflow를 먼저 던지면 원래의 성공한 scene/그 뒤 상태 변화가 달라진다. 지연 예외·추가 저장 관리가 필요하다 | 현재는 변경이 더 크므로 준비하지 않음 |

`proposed.patch`는 staged 후보 모듈을 새 파일로 놓는 검토용 diff다. 운영 호출 연결이나 기본 활성화를 포함하지 않으며 지금 적용하라는 지시가 아니다.

## 준비한 검증과 고정 명령

[`test_candidate.py`](test_candidate.py)는424/508/524광선, CW/CCW·비균일 각도, 혼합 NaN/Inf/범위 밖 반환, rear/거리 제한 구조 마스크에서 장면 전체와 raw points dtype·shape·바이트를 비교한다. masked 유효점 보존/overflow 검사 시점, 빈 점군, 배열 aliasing, 다른 스캔·now·epoch·config·transforms·signed zero·dtype, 비표준 입력/일회용 mask, 시간/운동 이력 재검사와 context 복구를 포함한다. 모두 합성/offline 검사이며 실물 시간 정확성 검증이 아니다.

[`compare.py`](compare.py)는 r353399입력을 두 독립 현재 제어기에 같은 순서로 적용하며 r278 원 경계 정책도 양쪽에 동일 설치한다. 전체 판단과 **부분관측 정책의 전체 상태·partial_epoch·선택기 전체 상태**를 대조한다. 부분관측의 현재/과거점, 취득 인덱스·시각, 래치/구조 기억과 재관측 상태를 빠뜨리지 않는다. tuple 키, set/frozenset, ndarray dtype/shape/바이트를 보존한다. 출력의 기존 `comparable_decision` 정규화와 같은 표면 정합 `surface_cache[2].processing_ms` 저장 사본만0으로 정규화한다. 다른 시각/진단 필드는 제거하지 않는다. 원 r353 출력과 현재 baseline의 차이는 별도로 남긴다.

`--verify-points`는 공유된 매 points 요청을 원 함수와 정확 대조한다. 이 실행은 비용 결과가 아니다. 다음 별도 비용 실행은 profiler/원 points 재계산 없이 baseline/candidate 순서를 교대하며 전체 및 RELEASED의 합계/p50/p95, 전체 호출 중 공유가 발생한 호출 수와 points 공유/원 경로 횟수를 남긴다. 함수 설치/복구와 상태 직렬화는 타이머 밖이며, 공유 객체 구성·바이트 대조·배열 생성은 타이머 안이다. 준비 당시 해시는 `manifest.json`, 실행 입력 전체 해시와 종료 불변 검사는 각 출력 폴더에 보존한다.

프로젝트 루트/정해진 WSL 환경에서 실행 담당자가 아래 순서로 수행한다. 기존 폴더가 있으면 덮어쓰지 않고 중단한다. correctness 결과를 확인하기 전에 cost를 실행하지 않는다.

```sh
python3 -m pytest -q artifacts/validation/2026-11-13/native_projection_share_r389_v1/test_candidate.py tests/test_native_geometry.py
python3 artifacts/validation/2026-11-13/native_projection_share_r389_v1/compare.py --backend native_v1 --verify-points --out artifacts/validation/2026-11-13/native_projection_share_r389_v1/verify_native
```

399건 전체 판단/상태 동일·공유 points 정확 동일·원 소스 불변을 확인한 후, 자원 독점 상태에서 다음 비용 비교 **1회**를 수행한다.

```sh
python3 artifacts/validation/2026-11-13/native_projection_share_r389_v1/compare.py --backend native_v1 --out artifacts/validation/2026-11-13/native_projection_share_r389_v1/cost_native
```

기하 공유는 Python 함수 단계이므로 별도 C++ 커널에 의존하지 않으며 `--backend python`도 지원한다. 기존 native 수치 커널과 공유되는 판단의 대표 비교는 위 native1묶음으로 제한한다. 새 Python 전체399비용·Jetson·프로파일 실행은 현재 배정하지 않았다.

배열/장면/전체 출력/상태 중 차이1건, 실패 종류·시점 변화, 입력/소스 변경,399건 미완료, 실제 공유0회면 실패를 보존하고 중단한다. 정확성 통과 뒤에도 비용 효과가 미미하거나 p95가 악화하면 미채택으로 종료한다. 반복 미세 최적화·TTL/보호 기준 완화·다른 입력 재사용으로 이어가지 않는다. 이 비교는 현재 검사63–67ms, ROS/통신, 실물 장착,10Hz deadline 성립을 검증하지 않는다.


## 실행 결과와 채택 판단

`receipt_v1/`의 고정 명령은 67개 검사 통과, `verify_native/`는399건 판단·전체 상태·raw points 정확 일치와 소스 불변을 확인했다. 이어진 `cost_native/` 비계측 비교도399건 일치다. RELEASED343건 합계12495.809→12272.128ms(1.790% 감소), 중앙39.248→38.691ms, p95 43.210→42.633ms이고 전체399건 합계 개선은1.179%다. 저장 원본과 현재 baseline의 기존 출력 차이30건은 별도 목록으로 유지했다.

GPT 최종 검토: 약0.6ms 중앙 개선을 얻기 위해 여러 래퍼에 새 문맥을 전달하는 부담이 커서 **이번 후보는 공통 코드에 채택하지 않는다.** 단일 직렬 재생의 작은 효과를 Jetson10Hz·실제 갱신 공백 해소로 확대하지 않는다. 새 원인을 찾지 않은 반복 비용 시험은 하지 않으며 최종 기하 전달과 실제 실행 연결을 우선한다.
