# 공식 v2026.11.01 시설 실행 연결

상태: **실행 월드 갱신·2×5 점등·RGB 출발·단독 대표 1랩 완료.** 새 파생 월드에서 11/1 선정 구성이 **35.606초·최고 2.373 m/s(8.54 km/h)** 로 완주했고 접촉·노면 이탈 표본 0, 명시 STOP·정상 종료를 확인했다. 단독 1회다. 전체 본선·실차 검증이 아니며 [11/1 35.281초](../../2026-11-01/solo_corner_handoff_v1/README.md)와 다른 월드라 그 결과를 이 월드로 옮기지 않는다.

**남은 결함 두 가지:** 선두 2칸(index 2·5)은 현 임시 수평 카메라의 수직 화각 밖이라 신호를 볼 수 없다. 신호 지점에 통상 상한 1.0 m/s가 걸리며 공식 `tl_post`가 왼쪽 벽 경로를 끊는 것으로 추정한다(스캔 재현 전).

## 원본·파생본·실행의 구분

| 층 | 내용 |
|---|---|
| 공식 원본 | [v2026.11.01 ZIP](../../../../assets/track/official/v2026.11.01/it_arena_track_v2026.11.01.zip), SHA-256 `ef50e0f1…2cf5`. 읽기만 했다. 이전 ZIP·[11/1 시험 fixture](../../2026-11-01/signal_marker_offset_v1/README.md)는 수정하지 않았다 |
| 그대로 쓴 공식 항목 | 도로·잔디·벽·중심선, 검정 그리드, 마커 네 개(ID45 왼쪽·ID0 오른쪽, 신호 중심에서 ±145 mm), `tl_post`·`tl_beam`·`tl_panel` 전체, 광원 10개의 pose·구 형상. 생성 때 링크 단위로 대조한다. 옛 ID45 오른쪽 0.20 m 시험값은 쓰지 않았다 |
| 팀 파생 | [프로필](../../../../config/tracks/official_v2026.11.01.yaml)·[생성기](../../../../scripts/official_2x5_facilities.py). 광원 재질만 소등으로 시작한다(원본은 10개 상시 발광). 결승 체크무늬는 기존 팀 표시다. 방지턱 없음 |
| 실행 확인 | 아래 렌더·주행. 공식 `traffic_light.py`의 UDP와 무작위 유지 시간은 쓰지 않았다. 빨강 3까지 1 s, 빨강 4는 `red_duration_s`로 고정했다 |

## 점등 제어

[`traffic_light_controller.py`](../../../../src/arena_gazebo/scripts/traffic_light_controller.py)를 공식 10개 링크로 바꾸고 옛 단일열 이름·노랑을 지웠다. 시험 초록→소등→빨강 1–4→초록 순서이며, 광원별 `gz service` 프로세스 대신 지속 연결 [`visual_material_client`](../../../../src/arena_gazebo/src/visual_material_client.cpp)를 쓴다. 끄는 광원을 먼저 적용해 빨강과 초록이 함께 켜진 표본을 피한다. 상태 토픽은 평가기 전용이며 차량은 RGB만 읽는다. 공식 월드의 `local_pursuit`는 `official_4red_local_candidate`로 판독한다.

`experimental` 월드에서 이 제어기를 켜면 launch가 거부한다. 기존 `--team-signal-prelap`/`--team-signal-course`는 11/1 fixture 재현용으로 그대로 남아 있다.

## 실제 점등·RGB 판독

[`live_signal_render.py`](live_signal_render.py)로 재생성한 월드를 실제 Gazebo(WSL GPU)에 띄워 재질을 바꾸고 현 임시 장착 위치의 프레임을 순서대로 검출기에 넣었다. 주행 없는 정지 시점이다. [결과](render_v1/report.json), 10개 회신 모두 적용. [후열 빨강](render_v1/slot3_red4.png), [후열 초록 전경](render_v1/overview_green.png), [선두](render_v1/slot2_red4.png).

| 칸(앞→뒤) | 신호 판독 | 시험 초록 | 0·45 동시 검출 |
|---|---|---|---|
| 2·5 (선두) | **불가**: 광원·마커가 영상 밖 | 출발 없음 | 없음 |
| 1·4·0·3 | 빨강 4 확인 → 초록 확인 후 출발 판정 | 출발 없음 | 네 칸 모두 검출 |

[칸별 기하](camera_pitch_table.json): 수직 반화각 23.3°, 광원 올려본각은 index 2가 35.0–37.5°, 5가 24.5–26.3°다. index 2는 상향 15° 이상, 5는 5° 이상이 필요하다. 당시 검출기는 15°에서 index 0·3이 상단 58% 판독 영역 밖으로 내려갔다. 렌더의 진단 시점(15°)에서 index 5는 출발 판정까지 됐고, index 2는 빨강 4만 확인했으며 초록은 `green_roi_outside`로 판정되지 않았다. index 3은 영상 하단으로 내려가 판독되지 않았다. 차량 설정은 바꾸지 않았다. 수평 시점에서 광원이 영상 밖인 문제는 소프트웨어로 복원할 수 없다. 뒤의 검출 영역 제한은 별개 문제였고 [후속 수정](../official_start_roi_v1/README.md)에서 분리·해소했다. 위 15° 결과는 수정 전 검출기의 기록이다.

높은 앞차 가림, 실제 조명·노출은 확인하지 않았다.

## 기준 벽 선택

주행 [요약](run1_summary.json) 기준 그리드와 랩 끝 신호 접근에서 0·45가 함께 검출됐고(226표본), 그동안 기준 벽은 계속 왼쪽이었다. ID0·20·45가 왼쪽, ID30이 오른쪽을 고르는 기존 동작이 유지됐고 ID30에서 오른쪽으로 전환됐다. 두 ID의 동시 검출이 출발이나 벽 선택을 방해한 사례는 없었다.

## 단독 대표 1랩

[실행 입력](run.py)은 11/1 [등록 인자](../../2026-11-01/solo_corner_handoff_v1/registered.json)에서 `--team-signal-prelap`만 빼고 `--red-duration-s 1.6`과 출력만 지정했다. 프로필 `g1_open_10kmh_solo_gap05_arc_preview_bridge`, 그리드 3, C1 순차 10 Hz, 8.970588:1, 28카운트, 65 mm, 영상. 실행은 한 번이며 등록 [입력](run1_registered.json) 해시가 실행 뒤에도 같았다. 실행과 회수는 Claude Haiku 5.5 보조가 맡았다.

| 항목 | 새 공식 월드 run1 | 11/1 (팀 파생 v10.24) |
|---|---:|---:|
| 첫 녹색 표본부터 1랩 거리 | **35.606초** | 35.281초 |
| 차체 최고 / 1초 평균 최고 | 2.373 / 2.301 m/s | 2.440 / 2.327 m/s |
| 10 km/h의 95% 이상 | 0초 | 0초 |
| 출발 뒤 0명령 합계 | 0.14초 (43.6 m 한 구간) | 0.26초 |
| 속력 0.05 m/s 이하 누적 | 0.614초 | 0.465초 |
| 접촉 / 바퀴-벽 / 최대 잔디 바퀴 | 0 / 0 / 0 | 0 / 0 / 0 |
| 명시 STOP / 정상 종료 / 소스 불변 | 통과 | 통과 |

- **신호:** 실제 초록 적용 8.050초, 첫 출발 허가 8.284초(0.234초 뒤). 시험 초록에 출발하지 않았고 일곱 단계 모두 10개 광원 회신을 받았다. [순서 기록](run1/signal_sequence.jsonl), [보고](run1/report.json).
- **정지:** 실제 정지 판정 통과, 최종 속력 약 0, 명령 정지 뒤 이동 0.342 m, 자식 15개 정상 종료.
- **노면:** 22,950개 평가 표본에서 차체 이탈·바퀴-벽 표본 0. 실물 접촉 증명이 아니다. [노면](run1_surface.json), [모터](run1_motor_audit.json), [속도](run1_metrics.json).
- **영상:** [3인칭 영상](run1/lidar_third_person_simtime_1x.mp4) 735프레임·36.75초 전 프레임 디코딩. [검사](run1_video_review/video_inspection.json). 차량 시점 [초록 프레임](run1/signal_green.png)을 직접 봤다.

## 공식 기둥에 의한 감속

[같은 위치 비교](start_compare.json) 기준 0–4 m가 11/1보다 0.75초 느리며 그중 1–2 m만 0.45초다. 11/1에는 2.5 m/s 명령이던 이 구간이 **1.0 m/s 상한**으로 내려갔다. 판단 기록(`run1/follow_decisions.jsonl`, 로컬 보존)과 11/1 같은 위치의 대조로 판단한 원인은 다음과 같다. 스캔별 재현으로 입증한 것은 아니다.

1. C1 스캔 높이(95 mm)를 지나는 `tl_post`가 왼쪽 벽 경계를 끊어 왼쪽 경로 길이가 약 0.65–1.0 m로 줄었다.
2. 이후 오른쪽 벽 대안 경로(`wall_fallback`)를 쓰는 동안 `open_route`가 꺼져 통상 상한 1.0 m/s가 된다.
3. 랩 끝의 같은 지점(47.6–48.2 m)에서도 같은 분기가 나타났다. 다만 그 직전 구간이 이미 저속이라 손실은 출발 때보다 작을 것으로 추측한다. 측정하지 않았다.

정지·접촉·경로 이탈은 없어 시설 사용을 막는 결함은 아니다. 실제 지지대 형상이 사진으로 확정되지 않았고 교통 조건의 영향도 함께 봐야 해서 이번 묶음에서 주행 판단 코드는 고치지 않았다. 랩 시간 차이 전체를 이 원인에 돌리지는 않는다. 다른 위치의 차이는 한 번의 실행 분산일 수 있고 나누어 재지 않았다. 나머지 구간 비교는 [요약](run1_summary.json)에 있다.

## 회귀와 범위 밖

- [전체 회귀](tests_full2.log): 7,334 통과·13 실패·19 건너뜀(종료 코드 1). 남은 실패는 [변경 전 HEAD 사본](baseline_failed.log)에서도 같은 13건이며 소스 계약 사슬·import 목록 등 기존 항목이다. 이번 변경으로 기존 시험의 기대값을 바꾼 것은 공식 월드 전제(`test_official_track`, 충돌체 수 3,710, 왼쪽 잔디 장면의 기준 월드)다.
- [소스 계약](../../../../config/tests/official_facility_r1_sha256.json), [수정 전 소스](before.zip).
- [등록 교통 장면 정적 대조](traffic_scene_audit.json): 16개 장면 중 `side_left` 1건을 경로 검사가 거부한다. 왼쪽 잔디 상대가 공식 기둥 자리와 겹치기 때문이다. E3 잔디 쪽 이동 상대도 같은 이유로 거부되며 등록 판정은 이전 월드 기준으로 시험에 남겼다. 이 장면들은 새 월드로 옮기지 않았다.
- 교통 주행은 새 월드에서 실행하지 않았다. `--team-signal-course` 등록 시험은 아직 11/1 fixture를 쓴다. `lidar`/`stereo` 기초 데모는 임시 3등 검출기를 쓰므로 새 월드 출발이 미검증이다.
- 모든 조명·배치 전수, 지지대 미관, 실물 밝기·노출은 범위 밖이다.

## 재현

```bash
# WSL, 저장소 루트
python3 -B scripts/build_official_track.py && python3 -B scripts/build_official_track.py --check
colcon build --packages-select arena_gazebo arena_bringup arena_autonomy --symlink-install
bash artifacts/validation/2026-11-02/official_facility_v1/render.sh render_v2
bash artifacts/validation/2026-11-02/official_facility_v1/run_once.sh run2
```
