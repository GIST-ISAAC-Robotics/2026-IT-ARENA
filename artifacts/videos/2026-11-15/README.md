# 경기 종료 후 기록 영상 — 2026-11-15

IT ARENA 경기(11/14)가 끝난 뒤, **이미 성공으로 기록된 Gazebo 시뮬레이션 실행에서 선정한 영상 8편**입니다. 원 실행은 7개이며, r475 한 실행에서 완주(C)와 신호 출발(E1)을 따로 잘랐습니다. 모든 영상은 원 실행이 남긴 차량·상대차 자세 기록을 렌더 전용 Gazebo 월드에 재생해 새 카메라로 찍은 **저장 자세 재렌더**입니다. 새 주행·제어기 재실행·실차 촬영이 아닙니다. 실제 경기에서는 MCU 부재와 Jetson 장치 미연결로 차량이 움직이지 못했습니다([경기 결과](../../../docs/reports/COMPETITION_RESULT_2026_11_14.md)).

"성공"은 각 영상이 보여 주는 기능의 사전 기준(완주, 추월·복귀, 접촉·노면, STOP·정상 종료 등)을 원 보고서가 통과로 기록했다는 뜻입니다. 같은 실행에 남아 있는 원 평가기의 다른 실패 항목은 아래 표와 영상 설명에 그대로 적었습니다.

**최종 ESP32 묶음과의 관계:** [대회용 최종 ESP32 소스](../../../docs/releases/FINAL_ESP32_2026_11_14.md)(r533 `selected_v2`)는 별도 보존본입니다. 아래 영상의 원 실행 소스는 모두 그 묶음과 다릅니다. 확보한 기록에서는 최종 묶음과 같은 소스로 성공한 완주를 확인하지 못했습니다. 대조 가능한 공통 `src/`·`config/` 287경로가 최종 묶음과 바이트 단위로 일치한 실행은 r544 attempt2 하나입니다. 이 실행은 19.56 m에서 정체한 실패여서 영상화하지 않았고, 원 근거는 [해당 결과](../../validation/2026-11-14/measured_steering_lap_r544_attempt2_v1/RESULT.md)에 있습니다. 출처·선정·검증의 자세한 내용은 [기술 보고서](../../../docs/reports/SIMULATION_RECORDINGS_2026_11_15.md)에 있습니다.

## 영상 목록

모두 1920×1080·30 fps·H.264(High, yuv420p, faststart)·무음이며 **1.0배속**입니다(영상 1초 = 원 시뮬레이션 1초). 포스터는 1280×720 JPEG입니다.

| # | 영상 | 포스터 | 길이 | 원 실행 | 무엇을 보여 주나 |
|---|---|---|---:|---|---|
| A | [a_curved_course_r15_solo_lap.mp4](a_curved_course_r15_solo_lap.mp4) | [포스터](posters/a_curved_course_r15_solo_lap_poster.jpg) | 36.03 s | r15 · 11/5 · 곡선 코스 v2026.11.01 | 코스 개편 전 최고 성능 단독 한 바퀴. 첫 초록 표본→결승 31.850 sim s, 최고 2.854 m/s |
| B | [b_right_angle_course_r25_solo_lap.mp4](b_right_angle_course_r25_solo_lap.mp4) | [포스터](posters/b_right_angle_course_r25_solo_lap_poster.jpg) | 32.83 s | r25 · 11/8 · 직각 코스 v2026.11.06 | 변경 코스 최고 성능 단독 한 바퀴. RGB 출발 기록→목표 도달 29.228 sim s, 최고 2.862 m/s |
| C | [c_right_angle_late_dev_r475_solo_lap.mp4](c_right_angle_late_dev_r475_solo_lap.mp4) | [포스터](posters/c_right_angle_late_dev_r475_solo_lap_poster.jpg) | 53.17 s | r475 · 11/14 · 직각 코스+시설 지주 | 더 현실적인 시설·센서 표본 조건의 느린 완주. GREEN→랩 검출 49.882 sim s, 최고 1.539 m/s |
| D1 | [d1_overtake_medium_lead_r15.mp4](d1_overtake_medium_lead_r15.mp4) | [포스터](posters/d1_overtake_medium_lead_r15_poster.jpg) | 12.03 s | medium_v1 r15 · 11/8 | 0.50 m/s 앞차 추월과 본선 복귀 |
| D2 | [d2_bypass_stopped_car_r18.mp4](d2_bypass_stopped_car_r18.mp4) | [포스터](posters/d2_bypass_stopped_car_r18_poster.jpg) | 11.03 s | r18 · 11/8 | 멈춰 있는 앞차 우회와 본선 복귀 |
| E1 | [e1_signal_red_green_start_r475.mp4](e1_signal_red_green_start_r475.mp4) | [포스터](posters/e1_signal_red_green_start_r475_poster.jpg) | 9.63 s | r475 · 11/14 (C와 같은 실행) | 적색 1→4 점등 대기, 초록 뒤 출발(신호등 근접 보조 화면 포함) |
| E2 | [e2_stop_restart_r40.mp4](e2_stop_restart_r40.mp4) | [포스터](posters/e2_stop_restart_r40_poster.jpg) | 18.57 s | r40 · 11/8 | 운영 STOP → RESET 없이 같은 경기 재출발 → 최종 STOP |
| F | [f_sensor_noise_delay_traffic_r38.mp4](f_sensor_noise_delay_traffic_r38.mp4) | [포스터](posters/f_sensor_noise_delay_traffic_r38_poster.jpg) | 63.07 s | r38 · 11/8 | C1 거리 오차 ±3 cm·추가 지연 32–70 ms를 주입한 조건에서 0.8 m/s 앞차 2대 뒤 60 s 동안 48.09 m 진행 |

SHA-256·바이트·sim 구간·포스터 해시는 [recordings_index.json](recordings_index.json)에 있습니다. 8개 MP4의 합계는 46,178,243 bytes(약 46.2 MB)이고 가장 큰 파일은 F(11.5 MB)입니다. 파일 이름의 `r15`는 날짜별 실행 번호입니다. A(11/5 곡선 코스)와 D1(11/8 `medium_v1`)은 서로 다른 실행입니다.

### 조건이 달라 직접 순위를 매기지 않습니다

A·B·C·F는 코스(곡선 v2026.11.01 / 직각 v2026.11.06), 차량·센서 구성, 시설 지주 유무, 상대 차량, 월드 시계 설정(RTF 1.0 / 0.25), 측정 기준점(첫 초록 표본 / RGB 출발 기록 / GREEN 관측 / 60 s 진행 거리)이 서로 다릅니다. **값을 한 줄로 놓고 빠름·느림 순위로 읽지 마십시오.** 각 값은 해당 실행의 "확인된 대표 기록"입니다.

- 최고 성능 대표: A(곡선 코스), B(직각 코스).
- 더 의미 있는 조건을 반영해 느려진 성공 기록: C(시설 지주·C1 실측 각도 표본), F(C1 거리 오차·추가 지연 주입, 앞차 2대).
- F의 지연은 시뮬레이션 설정으로 넣은 값(50±20 ms)이며 Jetson에서 측정한 지연이 아닙니다. 어느 영상도 Jetson에서 실행하거나 Jetson 계산 지연을 재현한 것이 아닙니다. 실측 조향 상한(r541/r542)을 적용한 성공 완주도 없습니다.

### 영상별 조건과 남은 한계

| # | 조건 | 원 보고서에 보존된 한계 |
|---|---|---|
| A | 기둥 없는 팀 실행 월드, `--straight-budget` ON, B0183·C1 10 Hz 운동 보정 | 43.61 m에서 실제 지면 정지 0.138 s 1회. [원 보고](../../validation/2026-11-05/solo_straight_closed_loop_v1/README.md) |
| B | 신호/마커 지주 시설 추가 전 월드 | 42.75 m 부근 저속 표본 3개, 명시 STOP 뒤 변위 0.945 m. [원 보고](../../validation/2026-11-08/solo_gap_envelope_connected_v3/README.md) |
| C | old610 신호·마커 지주 3개, C1 실측 각도 배열 표본(하드웨어 시각 미검증), 현재 경로 우선 판정, 상대 0 | 최종 ESP32 묶음과 src/config 21개 파일이 다릅니다(실측 조향 상한 미적용). [원 보고](../../validation/2026-11-14/solo_full_lap_r475_v1/RESULT.md) |
| D1 | 오른쪽 12 cm·등속 0.50 m/s 평가용 상대 1대, 명시 `--moving-pass-profile medium_v1`, 명시 평탄 잔디 파생본 | 사전 등록 추월 보충 판정(진입→옆→앞→복귀→2.008 m 전진, 접촉 0, 최대 두 바퀴 잔디, STOP·정상 종료)은 모두 통과. 원 검사기의 최종 간격·차체 전체 도로 안·끝까지 앞차 뒤 판정은 실패로 보존. [원 보고](../../validation/2026-11-08/medium_pass_connected_v1/README.md) |
| D2 | 평가용 각본 정지 상대 1대 | 실물 운동의 상계를 증명한 것은 아닙니다. [원 보고](../../validation/2026-11-08/rear_face_motion_connected_v1/README.md) |
| E1 | C와 같은 실행의 출발 부분, 공식 2×5 신호 모형 | 실제 신호 촬영이나 HSV 교정이 아닙니다. 신호 단계는 `signal_sequence.jsonl` 기록값입니다. |
| E2 | 앞에 멈춘 평가용 상대 1대, 운영 STOP/START | 원 `report.passed=false`는 공용 실행기가 갱신하지 않는 필드이고, 전용 assessment 21조건은 통과했습니다(원 보고 설명). Wi-Fi/패널 STOP은 하드웨어 비상정지가 아닙니다. [원 보고](../../validation/2026-11-08/entry_width_transition_v1/README.md) |
| F | 0.8 m/s 앞차 2대(seed 23, 60 s), C1 ±3 cm·추가 지연 50±20 ms 주입, 선택형 `traffic_recovery_trial` 감속 재검사 후보 | 보충 평가 게이트(거리 한 랩, 접촉·겹침·노면 0, 주입 범위, STOP·정상 종료, 입력 불변 등)는 모두 통과. 원 평가의 최종 간격 판정은 실패(HARNESS_EXIT=1)로 보존. 한 seed·한 조건이며 실물 오차 전체를 검증한 것이 아닙니다. [원 보고](../../validation/2026-11-08/masked_motion_speed_v1/README.md) |

## 화면 읽기

- 왼쪽 위: 영상 제목과 코스·실행·날짜. 오른쪽 위: `Gazebo 시뮬레이션 · 저장 자세 재렌더 · 1.0×`, `실차 영상 아님`.
- 왼쪽 아래: 해당 프레임의 원 시뮬레이션 시각 `sim t`와 GREEN 기준 경과. 출발 장면에는 `신호 기록:` 줄로 그 시각에 적용된 신호 단계를 표시합니다.
- 가운데 아래 자막: 원 보고서·원시 기록에서 옮긴 사건 시각(예: 추월 시작, 나란히, 복귀, STOP 입력, 재개 허가). 판정은 각 원 보고서를 따릅니다.
- 오른쪽 아래: 파랑 = 자차, 빨강 = 상대차(평가용 각본).
- A·B·C·E1·F의 출발 구간에는 같은 시각을 신호등 정면에서 찍은 **근접 보조 화면**을 오른쪽 위에 넣었습니다. 주 화면에서는 점등 구가 작게 보이기 때문입니다.

## 만든 방법

1. **장면 생성**([tools/build_scene.py](tools/build_scene.py)): 원 실행의 `runtime_world.sdf`·`batched_*.obj`·`runtime_vehicle.sdf`(+상대 모형, F는 `additional_actors.json`의 두 번째 앞차 포함)를 읽고, 물리·제어·접촉 시스템을 뺀 렌더 전용 월드를 만듭니다. 자차(약 500 Hz)·상대(100 Hz) 기록 자세는 선형 보간만 합니다. 신호등은 `signal_sequence.jsonl`의 실제 적용 시각에 맞춰 점등 구 모형을 켜고 끕니다.
2. **관찰 카메라**: 실제 차량 센서와 무관한 촬영 전용 카메라입니다. 평활한 자차 진행 방향(σ 0.45 s) 뒤 1.3 m·높이 0.8 m에서 약 19–28° 내려다보며, 시선은 자차와 1.5 m 앞 기록 경로 사이를 향합니다. 벽 높이가 0.4 m라서, 코너에서는 카메라가 바깥쪽으로 돌아 벽 위에서 다음 코너를 봅니다. 상대차가 앞·옆 3.2 m 안에 있을 때만 두 차가 모두 들어오도록 최대 1.8배까지 물러납니다. 기록 자세는 카메라 위치 계산에만 쓰며 주행 판단에는 쓰지 않습니다.
3. **렌더**([tools/render_clip.sh](tools/render_clip.sh), [tools/render_all.sh](tools/render_all.sh)): WSL Gazebo(ogre2, headless)에서 클립마다 소유 `gz sim` 1개를 띄웁니다. 1920×1080 카메라를 약 30 Hz(sim 1 ms 단계)로 저장한 뒤 그 PID만 종료합니다. 재생 플러그인은 [tools/render_plugin](tools/render_plugin/)입니다.
4. **합성**([tools/compose_clip.py](tools/compose_clip.py)): 출력 프레임 k에는 원 시각 `t0 + k/30 s`에 가장 가까운 렌더 프레임을 씁니다(오차 최대 16.3 ms, 보간·배속 없음). 오버레이를 그린 뒤 libx264 crf 20으로 인코딩합니다.
5. **검사**([tools/qa_clip.py](tools/qa_clip.py)): 완성 MP4를 끝까지 디코드해 프레임 수·PTS·검은 화면·정지 화면·faststart를 확인하고 contact sheet를 만듭니다. 결과는 [검증 폴더](../../validation/2026-11-15/video_recording_v1/)에 있습니다.

클립 구간·자막·사실 문구는 [tools/clips.json](tools/clips.json)에 있습니다. 교체된 이전 D1(r13 저속 앞차) 정의는 같은 파일의 `unselected` 항목에 남겼습니다.

### Git 저장소만으로는 다시 만들 수 없는 범위

렌더 입력 가운데 원시 자세(`ego_poses.jsonl`, `actor_poses.jsonl`, `additional_actors.json`), 실행 월드(`runtime_world.sdf`), 지형 OBJ(`batched_*.obj`), 그리고 r475 `run1/` 전체는 기존 Git 제외 정책에 따라 **로컬에만** 있습니다. 따라서 Git clone만으로는 재렌더를 완료할 수 없습니다. 같은 입력인지는 각 [편집 JSON](../../validation/2026-11-15/video_recording_v1/edit/)의 `source_run`·`render_inputs_sha256`, [render_logs/&lt;clip&gt;/meta.json](../../validation/2026-11-15/video_recording_v1/render_logs/)의 `input_sha256`과 파일별 SHA-256으로 확인할 수 있습니다. 이번 작업에서 원시 로그를 새로 공개하지는 않았습니다. 로컬 입력이 있으면 `tools/build_plugin.sh`(WSL) → `tools/render_all.sh`(WSL) → `compose_clip.py --clip <key>`·`qa_clip.py --clip <key>`(Windows 미디어 venv) → `make_index.py` 순서로 다시 만듭니다. GPU 렌더의 바이트 단위 동일성은 확인하지 않았습니다.

## 재렌더의 한계

- 바퀴 회전·앞바퀴 조향·차체 롤/피치는 재현하지 않았습니다(차체 x·y·yaw만). 높이는 고정값입니다.
- 상대차 자세는 50 Hz 순간이동을 100 Hz로 기록한 계단 값이라 미세한 끊김이 보일 수 있습니다.
- 신호 갠트리 아래를 지날 때 구조물이 화면 아래쪽을 잠깐 가립니다.
- 렌더 시간(wall)은 원 실행의 처리율·Jetson 성능·실차 시간과 무관합니다. 원 실행의 월드 시계 설정도 실제 처리율과 다를 수 있습니다.
- 중간 프레임은 JPEG(q95)로 옮긴 뒤 H.264로 다시 압축했습니다.
