# 교통 한 랩 정지의 standing_wall_reacquire 저장 입력 비교 (11/8)

**결론: 옵션 ON으로도 해결되지 않습니다.** [교통 한 랩 run1](../traffic_lap_newcourse_v1/README.md)의 저장 판단 입력을 0–38초 prefix로 재생했습니다. 기본값(OFF)은 기록과 **전체 판단 출력이 일치**했습니다. `standing_wall_reacquire=True`는 38초까지 **기본값과 한 번도 달라지지 않았습니다**. 정지 구간(28.12–38초)에서 양수 속도 명령은 0회였습니다. 원인은 선호 벽(왼쪽, 코너 안쪽) 후보가 짧다는 데 있습니다. 해당 옵션 분기는 이 상태에서 **코드상 도달하지 않습니다**. 앞차 후면을 벽으로 혼동한 증거는 없습니다.

범위: 저장된 판단 단계 입력만 재생했습니다. Gazebo 실행·빌드·소스/설정 변경은 하지 않았습니다. 양수 명령이 나왔더라도 실제 재출발 증명은 아닙니다. 이번에는 양수 명령도 없었습니다.

## 입력과 실행

| 항목 | 값 |
|---|---|
| 원본 | `traffic_lap_newcourse_v1/two_actor_course_lap/run1/follow_decisions.jsonl` sha256 `aebdc4e701359ccbc35539794f7e456506cc37dd38108a762e1e8700e83d8585`(재생 전후 동일, footer capture_errors/dropped/failed/pending 0) |
| 제어기 | 헤더 `follow_config`/`execution_config` 그대로 `BypassHandoffFollow(..., allow_simulation_trial=True)`. ON은 `dataclasses.replace(cfg, standing_wall_reacquire=True)` 한 항목만 변경 |
| 재생 범위 | seq 순서 그대로, reset 379회(SENSOR_STOP 2, WAIT_GREEN 377)를 두 제어기에 동일 적용, `reference_s ≤ 38.0` call 1518회 |
| 비교 | `follow_trace.comparable_decision` + `equal_output`(표면 정합 `processing_ms` 벽시계 비용만 제외) |
| 실행 | `wsl.exe -e bash run.sh replay` → `replay_v1.exit.txt`: `EXIT=0 WALL_S=52`; `run.sh probe` → `probe_v1.exit.txt`: `EXIT=0 WALL_S=28`. 환경 `2026-11-01/turn_preview_v1/environment.sh`. 실행은 Claude Haiku 5.5 보조가 담당(자체 보고 `claude-haiku-5-5`) |
| 소스 | 로드된 저장소 소스 60개 sha256을 `replay_result.json.loaded_source_sha256`에, 압축본을 `loaded_source_snapshot.zip`에 보존. 실행 당시 `run1/source_snapshot.zip`과 59개 동일, 1개(`arena_vehicle_interface/__init__.py`)는 스냅샷에 없음 |

## 결과

| 판정 항목 | 결과 |
|---|---|
| 기본값 전체 출력 일치 | **1518/1518 일치**(`baseline_mismatch_count=0`). 속도·조향뿐 아니라 판단 상세 전체 |
| ON 최초 차이 | **없음**(`on_first_difference=null`, 0–38초) |
| 정지 구간 28.12–38초 (495 call) | 기록/OFF/ON 모두 양수 속도 0, 비0 조향 0, 사유 `handoff_follow`/`follow_boundary_unconfirmed` 495회 |
| 32–38초 정지 상태 (301 call) | ON도 `follow_boundary_unconfirmed`/HOLD 301회. 측정 속도 최대 0.0. 새 스캔 후보 사유 `wall_segment_short` 60회 |
| 계측 재생 | ON 1318 call(≤34초) 모두 기록과 동일. 계측이 출력을 바꾸지 않았음 |

## 진단 (probe, 27.8–29.3초·33–34초 새 스캔 25개)

`probe.py`는 이 프로세스 안에서만 `local_follow.local_path`를 감쌌습니다. 또한 `sys.settrace`로 `LocalFollow._baseline_command`의 분기 행 지역 변수를 기록했습니다. 세그먼트 분해는 `local_path` 연결·앵커 규칙을 진단용으로 복제한 것입니다. 모든 호출에서 실제 `local_path` 사유와 일치했습니다(`mirror_all_consistent=true`). 상대 위치는 평가용 정답이며 제어기에 넣지 않았습니다.

1. **선호(왼쪽) 벽 후보 부족.** 모든 표본에서 왼쪽 전체 점 `local_path` 사유는 `wall_segment_short`였습니다. 실제 왼쪽 벽 구간은 앵커 (0.00, 0.50 m)에 정확히 붙어 있었습니다(앵커 점수 0.001–0.005 m). 그러나 앵커 앞 관측 길이는 0.216(27.82초) → 0.125(28.12초) → **0.088 m**(28.22초 이후 고정)였습니다. 이는 `local_path`의 0.28 m(`local_path.py:110`)과 `sufficient()`의 `boundary_min_ahead_m=0.65`에 모두 미달합니다. 벽은 x≈0.09–0.2 m에서 끝납니다. 그 뒤는 단일 점 조각뿐이며, 앞 0.6–1.4 m에는 왼쪽으로 꺾이는 다음 구간이 있습니다. 정리하면, 차량이 왼쪽 코너 안쪽 끝 바로 옆에 정지해 선호 벽 근거가 사라진 상태입니다.
2. **반대(오른쪽) 벽은 충분.** 매 스캔 `local_path` 사유는 ahead 1.8 m, wall_distance 0.51 m입니다. 그러나 이를 쓰는 경로가 모두 막혔습니다.
   - `bilateral_confirmed_alternative`(`local_follow.py:702`): `preferred is not None`이 필요하나 preferred가 None
   - `bilateral_reference_support`(`:713–725`): 오른쪽 경로를 기준으로 고른 왼쪽 부분집합(약 301점)도 `wall_segment_short`
   - `corner_distance_candidate`(`:785–801`): 27.82/27.92초에는 `observed_corner_distance_support`(reference_ahead 0.686/0.654 m)로 채택. 28.02초부터 `corner_reference_short`(0.638→0.634→**0.628 m**). 오른쪽 벽 오프셋 경로의 자차 앞 길이가 `boundary_min_ahead_m=0.65`(`corner_distance.py:38`) 아래로 내려갔음
   - 회랑 후보: `not_consistent_inward_corner`, positive=false
3. **정지 확정.** 28.02초는 기억(carried) 보조로 1.0 명령을 냈습니다. 28.12초에는 기억 TTL 0.3초가 지나 `carried=None`이 됐고, `follow_boundary_unconfirmed`로 정지했습니다(`:824–826`). 이후 속도 0이라 관측이 바뀌지 않아 같은 거부가 반복됐습니다.
4. **옵션이 적용되지 않는 이유(코드 행).** `standing_wall_reacquire` 분기(`local_follow.py:747`)는 `if eligible and carried is None and not bilateral:`(`:729`) 블록 안에 있습니다. 즉 선호 벽 후보가 이미 `sufficient`한데 양측 일치만 실패한 경우만 다룹니다. 분기 자체도 `sufficient(candidate, info)`를 다시 요구합니다. 이번에는 `:729` 진입 시 `eligible=False`였습니다. 그래서 `:747`·`:751` 행 이벤트가 25개 표본 모두에서 0회였습니다. 옵션이 겨냥하는 것은 "선호 벽은 길고 반대쪽이 열린" 상황입니다. 이번은 그 반대인 "선호 벽이 짧고 반대 벽이 긴" 상황입니다. 정지 상태 조향(`:881–887`)은 `clearance` 검사 단계 분기여서 경계 미확정 정지보다 뒤에 있으므로 역시 도달하지 않습니다.
5. **벽/후면 혼동 여부.** 왼쪽 정렬에서 x 0.58–0.75, y 0.16–0.48 m의 26–28점 구간이 보였습니다. 이는 평가용 앞차 상대 위치(0.66, 0.41 m, 상대 yaw +1.51 rad)의 차체와 겹칩니다. 그러나 앵커 영역(|x|<0.35 m)의 점이 0개라 후보로 쓰이지 않았습니다. 33–34초에는 앞차가 y≈2.85–3.29 m로 멀어져 이 구간이 사라졌는데도 같은 거부가 이어졌습니다. 따라서 이번 정지는 후면 혼동이 아니라 **안쪽 벽 후보 부족**과 **반대 벽 코너 근거의 길이 미달(0.628 < 0.65 m)**이 겹친 결과입니다. 앞차는 원인이 아닙니다.

## 한계

- 저장 입력 재생이며 폐루프 결과가 아닙니다. 새 시험은 열지 않았습니다.
- 세그먼트 분해는 진단 복제입니다. 앵커 앞 길이는 복제 계산값이며, 실제 함수의 사유 일치만 확인했습니다.
- 해결책 설계나 임계값 변경은 범위 밖입니다. 위 수치는 현재 조건이 어디서 막히는지를 보여 줄 뿐 조정 제안이 아닙니다.
- 벽시계 처리시간(OFF 중앙 14.7 ms/call)은 이 PC의 재생 비용이며 Jetson 성능이 아닙니다.

## 파일

`replay.py`, `probe.py`, `run.sh`(LF, 명령·종료 코드 기록), `replay_result.json`(요약·해시), `replay_rows.jsonl`(0–38초 call별 기록/OFF/ON 요약과 reset), `probe_result.json`(표본별 `local_path` 호출·분기 지역값·평가용 상대 위치), `*_v1.log`, `*_v1.exit.txt`, `loaded_source_snapshot.zip`.
