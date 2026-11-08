# v2026.11.06 빌더·별도 실행 월드·명시 선택·신호 연결 (이관 순서 1)

작업: 2026-11-07, 구현 Claude Opus 5.5, 짧은 시뮬 실행·회수 Claude Haiku 5.5(별칭 `haiku`, 보조 자체 보고 `claude-haiku-5-5`). 범위는 [이관 순서](../../../../docs/track/V2026_11_06_MIGRATION_2026_11_07.md) 1번이며 **제어기·카메라·주행 상수는 바꾸지 않았고 차량 주행은 하지 않았다.** 근거 감사는 [원본 감사](../../../../docs/track/OFFICIAL_V2026_11_06_AUDIT.md).

## 결과 요약

| 항목 | 결과 | 근거 |
|---|---|---|
| 빌더 등록 | `RELEASES`를 릴리스별 (SHA-256, 크기, 파일 수, 본선 폭, 지름길 폭)으로 일반화. v2026.11.06 = `d1f3fbc1…25fa`, 691,524 B, **25파일, 0.61 m / 0.20·0.20 m**(공식 design·scene 값). 이전 3릴리스는 23파일·0.45 m 그대로 검사 | `scripts/build_official_track.py`, 새 테스트 |
| 새 프로필 | `config/tracks/official_v2026.11.06.yaml`(커밋 `45691aec…`), 기존 2×5 파생 정책(기둥 제외·소등 시작·결승 표시) 재사용 | |
| 새 실행 월드 | `src/arena_gazebo/worlds/it_arena_official_v2026_11_06/`, `world.sdf` `4c2136c7…851b3b`. 차체·조향 외곽 정적 통과 검사 통과, `--check` 정상 | [build 로그](checks/build_v2026_11_06.log), [check](checks/check_v2026_11_06.log) |
| 공식 대비 전수 diff | 공식 출력에 같은 실행 변환만 적용한 기준과 정규화 XML 대조: **제거 `tl_post` 1, 추가 `finish_line_paint` 1, 변경은 광원 10개 재질뿐**(pose·형상 동일, emissive `0 0 0 1`). 월드 수준·병합 도로/잔디/벽/그리드·가로대·패널·마커 4개 링크·PNG·CSV·지도 파일 동일. 충돌 874→873 | [world_diff_audit.json](checks/world_diff_audit.json) |
| 옛 월드 보존 | `it_arena_official` 18파일 트리 해시 편집 전후 동일, `world.sdf` `e4fe4d0d…e048` | [전](checks/old_world_tree_sha256_before.txt), [해시](checks/old_world_sha256.txt) |
| 옛 월드 `--check` | 빌더가 바뀌어 기록된 builder 해시가 달라짐. 현재 빌더로 v2026.11.01을 임시 경로에 재생성해 **provenance의 builder 해시 외 17파일 바이트 동일**을 확인하고, 그 근거 파일을 `REPRODUCED_BUILDER_SHA256`에 등록(근거의 현재 빌더 해시가 맞을 때만 허용). 옛 월드는 다시 쓰지 않음 | [rebuild_v2026_11_01.json](checks/rebuild_v2026_11_01.json), [check](checks/check_v2026_11_01.log) |
| 교차 덮어쓰기 방지 | `RUNTIME_DESTINATIONS`로 릴리스별 경로 등록. 다른 릴리스에 등록된 월드로의 build/check는 거부. `--destination` 생략 시 프로필 릴리스 경로 | 새 테스트 |
| 실행 선택 | `simulation.launch.py` `track:=official_v2026_11_06`(기본 `official` 유지), `demo.launch.py` `track` 인자(기본 `official`), `validate_basic_autonomy.py --track`(기본 `official`; 지정 시 launch 인자와 중심선·scene·충돌 다각형을 같은 월드에서 읽음) | 새 테스트 |
| 실제 launch 1회 | `track:=official_v2026_11_06 traffic_light:=true`(자율주행 없음). 실행 사본 `runtime_world.sdf` = 새 월드 해시, 로그 `main width: 0.61 m`. 기존 `traffic_light_controller`가 green_probe→off→red1–4→green을 모두 10광원 회신으로 적용(점등 수 2/0/2/4/6/8/2). 47 s, launch exit 0, 잔여 프로세스 0 | [summary](launch_probe_v1/summary.txt), [순서 기록](launch_probe_v1/evidence/signal_sequence.jsonl) |
| 실제 렌더 1회 | 11/2 도구 사본으로 같은 서비스 재질 전환. **첫 off는 요청 없이 월드 초기 상태**: 10개 시점 모두 빨강 화소 0. red1→red4에서 빨강 화소 단조 증가, 두 검출기 모두 red4 확정 뒤 **green 단계에서만 출발 판정**. 정상 종료·소스 불변 | [report.json](render_v2/report.json), [초기 소등](render_v2/overview_0_off.png), [red4](render_v2/overview_6_red4.png), [초록](render_v2/overview_7_green.png) |

렌더 첫 시도 `render_v1`은 도구 사본에서 내가 넣은 변수명이 함수 인자(`target`)를 가려 **Gazebo 시작 전** 실패했다([로그](render_v1.log), [종료 코드](render_v1.exit)). 이름만 고쳐 `render_v2`로 한 번 다시 실행했다.

## 테스트

영향 범위(launch·빌더·출처 계약·공식 월드를 참조하는 55파일 + 새 파일, [목록](checks/affected_test_files.txt))를 WSL ROS 환경에서 실행했다. 시뮬레이터는 띄우지 않는다.

| 시점 | 결과 |
|---|---|
| 편집 전 | 876 통과·11 실패 [로그](checks/pytest_baseline.log), [실패 목록](checks/pytest_baseline_failed.txt) |
| 편집 후, 계약 전 | 843 통과·53 실패(출처 사슬 42건 증가) [로그](checks/pytest_after1.log) |
| 후속 계약 등록 후 | **886 통과·11 실패, 실패 목록이 편집 전과 동일** [로그](checks/pytest_after2.log), [실패 목록](checks/pytest_after2_failed.txt) |

후속 계약 `config/tests/track_migration_v2026_11_06_r1_sha256.json`(부모 r16)은 [prepare_contract.py](prepare_contract.py)로 만들었고 변경 전 바이트는 [before_r1.zip](before_r1.zip)에 있다(`git cat-file --filters`로 복원, [편집 전 기록](before/sha256_before.txt)과 일치). 기존 11건은 이번 변경과 무관한 이전 실패다. 저장소 전체 회귀는 실행하지 않았다.

## 판단·가정

- 0.61/0.20 m는 공식 design·scene과 생성기 결과가 같아야 통과하는 원본 확인값이다. 제어 상수 0.425/0.225 m와 평가·교통 시나리오 위치는 바꾸지 않았다(설계 분류는 GPT 담당).
- 결승 체크무늬는 기존 9×2 설정을 그대로 써서 새 폭에서 칸 너비가 약 6.8 cm로 커졌다. 장식이며 충돌체가 없다.
- 렌더·launch의 카메라는 현행 B0183 설정이다. B0390 적용·검증이 아니다. 렌더는 정지 시점 고정 카메라와 재질 서비스 전환이며 주행 중 판독이 아니다.
- 뒷칸 index 3에서는 ID0이 모든 단계에서 검출되지 않고 ID45만 검출됐다. 원본 감사의 같은 관찰과 일치한다.
- 차량 제어기는 이번 실행에 없었다. 제어기 소스가 신호 상태 토픽·UDP·정답 자세를 읽지 않는다는 기존 정적 테스트는 통과했다.

## 남은 일

- 새 월드에서 실제 주행 0회. 단독 기준 주행은 B0390 이관 뒤 GPT 설계 판단에 따른다.
- 옛 경로를 고정한 평가/시나리오 도구(`validate_g1_traffic.py`, `official_traffic_course.py`, `race_ops_eval.py`, `summarize_motion_lap.py`)와 `--team-signal-prelap` fixture는 v2026.11.01 전용으로 남겼다. 새 코스 교통 재등록(이관 순서 4) 때 같은 `TRACK_WORLDS` 방식으로 바꿔야 한다.
- 다음 확인 명령 예: `python3 scripts/validate_basic_autonomy.py --track official_v2026_11_06 --autonomy-mode local_pursuit ...`(등록 프로필·인자는 설계 후 결정).
