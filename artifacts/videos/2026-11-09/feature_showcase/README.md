# 기능 소개 영상 — 2026-11-09

새 공식 코스 v2026.11.06에서 **이미 확인한 주행 결과를 눈으로 볼 수 있도록** 만든 컷편집 영상입니다. 음악·효과·표지 없이 장면을 이어 붙였습니다. 모든 장면은 **Gazebo 시뮬레이션**이며 실차 영상은 없습니다. 새 주행 시험, 제어기·시뮬레이터 수정, 지름길 r51 실행은 하지 않았습니다. 영상에 넣지 않은 성과와 아직 끝나지 않은 기능은 아래 [주요 성과](#주요-성과-영상-밖-요약)와 [미완료·계획](#미완료계획-기능-영상에-성과로-넣지-않음)에 따로 정리했습니다.

## 결과물

| 파일 | 내용 |
|---|---|
| [feature_showcase.mp4](feature_showcase.mp4) | 최종 영상. **87.20 s**, 1,744프레임, 1280×720, 20 fps, H.264 High·yuv420p, faststart, 무음. SHA-256 `aee4ca76fac24bae6908b49a68c8c943e30da32d078021fa620325818991cd73` |
| [edit_table.md](edit_table.md) · [edit_table.json](edit_table.json) | 장면별 영상 시각 ↔ 원 시뮬레이션 시각, 원자료, 기록에서 계산한 사건 시각 |
| [contact_sheet.png](contact_sheet.png) | 최종 MP4를 디코딩해 뽑은 장면 시작·주요 사건·끝 프레임(영상 시각/원 시뮬 시각 표기) |
| [qa_report.json](qa_report.json) | 최종 MP4 전체 디코딩 검사 결과 |
| [provenance.json](provenance.json) | 입력 원시 기록·도구의 SHA-256과 실행 환경 |
| [render_meta/](render_meta/) | 재렌더 장면별 시간 대응·카메라 설정·프레임 수 |
| [tools/](tools/) | 재렌더 플러그인·장면 생성·편집·검사 스크립트 |

r25 단독 한 랩의 **편집 전 원본 3인칭 영상**은 [원 실행 폴더의 local_pursuit_third_person_drive.mp4](../../../validation/2026-11-08/solo_gap_envelope_connected_v3/run1/local_pursuit_third_person_drive.mp4)(960×540, 586프레임, 29.30 s)입니다.

## 장면 구성

영상 시각은 최종 MP4 기준, 원 시뮬 시각은 해당 실행의 Gazebo 시뮬레이션 시각입니다. **모든 장면은 1.0배속**(프레임 1장 = 시뮬레이션 0.05 s)이며 장면 안에서 자르거나 빠르게 돌린 곳은 없습니다.

| # | 영상 시각 | 장면 | 원자료(실행) | 원 시뮬 시각 | 방식 | 화면에서 볼 점 |
|---|---|---|---|---|---|---|
| ① | 0.00–29.30 s | 단독 주행 · 거리 한 랩 | [r25](../../../validation/2026-11-08/solo_gap_envelope_connected_v3/README.md) | 8.25–37.50 s | 원본 3인칭 녹화 | 출발(8.295 s)부터 한 랩 끝 직전까지 연속 주행. 후반 26.9 s 부근 짧은 저속 |
| ② | 29.30–40.05 s | 저속 0.30 m/s 앞차 추월 | [r13](../../../validation/2026-11-07/traffic_piecewise_v1/README.md) | 7.70–18.40 s | 기록 자세 재렌더 | 뒤 → 왼쪽 옆(34.9 s 나란히) → 앞(35.5 s 완전히 앞섬) → 본선 복귀(39.0 s) |
| ③ | 40.05–49.10 s | 중속 0.50 m/s 비정형 앞차 추월 | [오목 외형 1회](../../../validation/2026-11-08/medium_recessed_pass_v1/README.md) | 8.40–17.40 s | 기록 자세 재렌더 | 옆면이 오목한 상대. 나란히 45.0 s, 완전히 앞섬 45.5 s, 복귀 48.5 s |
| ④ | 49.10–57.55 s | 앞차 가속 → 양보 후 뒤로 복귀 | [r9](../../../validation/2026-11-07/return_guide_front_v1/README.md) | 9.00–17.40 s | 기록 자세 재렌더 | 추월 시작(50.2 s) 뒤 상대가 0.30→0.58 m/s로 가속(51.6 s 부근), 자차는 나란히 서기 전에 물러나 뒤 추종으로 복귀(56.3 s) |
| ⑤ | 57.55–68.10 s | 정지차 우회 후 본선 복귀 | [r18](../../../validation/2026-11-08/rear_face_motion_connected_v1/README.md) | 8.70–19.20 s | 기록 자세 재렌더 | 멈춰 있는 상대의 오른쪽으로 우회, 완전히 앞섬 62.7 s, 추종 복귀 65.7 s |
| ⑥ | 68.10–81.35 s | 앞차 정지·재출발 → 뒤로 합류 | [r18 재출발](../../../validation/2026-11-08/rear_face_restart_v1/README.md) | 10.20–23.40 s | 기록 자세 재렌더 | 앞차 정지 뒤 자차가 오른쪽으로 우회를 시작하고, 옆에 붙을 무렵 앞차가 재출발하자 뒤로 합류(79.6 s) |
| ⑦ | 81.35–87.20 s | 빠른 뒤차에게 추월당함 | [r24](../../../validation/2026-11-08/rear_overtake_measured_v1/README.md) | 8.60–14.40 s | 기록 자세 재렌더 | 왼쪽 뒤 1.8 m/s급 상대가 옆(85.0 s 나란히)을 지나 앞으로 나감. 자차는 계속 전진 |

장면별 사건 시각(제어 상태 전이, 종방향 겹침 시작/끝, 상대 정지·재출발·가속)은 [편집표](edit_table.md)에 원시 기록 기준으로 정리했습니다. 상대 속도 사건은 50 Hz 순간이동·100 Hz 계단 기록이므로 **중심 1초 창 이동량**으로 계산했으며, 실제 변화보다 최대 약 0.5 s 이르게 표시될 수 있습니다(예: ⑥ 재출발 표시 15.07 s, 보고서상 재출발 약 15.49 s).

### 화면 표기

- 왼쪽 위: 장면 이름(①만 원본 정보 패널 바로 아래, 화면 왼쪽 위쪽). 오른쪽 위: `Gazebo 시뮬레이션 원본 녹화` 또는 `Gazebo 시뮬레이션 · 기록 자세 재렌더`, 실행 이름, `1.0×`.
- 왼쪽 아래: 원 시뮬레이션 시각 `sim t`. 오른쪽 아래: **파랑 = 자차, 빨강 = 상대차(평가용 각본)**.
- ①의 상단 검은 정보 패널(`progress`, `actual speed` 등)은 **원 녹화에 새겨진 값**이며 이번 편집에서 그린 것이 아닙니다. 장면 이름은 이 패널과 겹치지 않도록 패널 바로 아래(왼쪽 위쪽)에 두었습니다. 원본 960×540을 1280×720으로 확대(PIL LANCZOS)했습니다.

## 영상 제작 방식

**① r25:** 원 실행이 녹화한 3인칭 카메라 영상을 그대로 썼습니다. `video_timestamps.json`의 586개 시각이 8.25–37.50 s에서 모두 0.05 s 간격이고 디코딩 프레임 수와 같아, 20 fps 재생이 정확히 1배속입니다. 마지막 프레임 37.50 s는 목표 도달 표본 37.523 s보다 0.023 s 앞서며, 명시 STOP 이후 구간은 원 녹화에 없습니다.

**②–⑦ 교통 장면:** 새 코스 교통 실행에는 녹화 영상이 없어 **같은 실행의 Gazebo 기록 자세로 재렌더**했습니다(새 주행 아님).

- 월드: 해당 실행의 `runtime_world.sdf`와 같은 실행의 `batched_*.obj`(노면·잔디·벽, 명시 평탄 잔디 파생본 포함)를 그대로 사용했습니다. 물리·접촉·자세 발행 시스템은 빼고 렌더 센서(ogre2)만 남겼습니다.
- 차체: 자차는 실행의 `runtime_vehicle.sdf` 시각 요소 48개(차체·센서·ID49 표식·바퀴), 상대차는 `evaluation_actor.sdf`의 시각 요소(③은 오목 외형 3개)를 정적 모델로 옮겼습니다. 크기·색은 원 실행과 같습니다.
- 자세: 자차 `ego_poses.jsonl`(약 500 Hz), 상대 `actor_poses.jsonl`(100 Hz)의 **기록된 x·y·yaw를 선형 보간만** 해서 매 시뮬레이션 스텝에 적용했습니다([pose_replay_system.cpp](tools/render_plugin/pose_replay_system.cpp)). 각본 목표 경로나 계획 경로는 쓰지 않았습니다.
- 신호등: `signal_sequence.jsonl`에서 장면 시작 전에 실제 적용된 단계(모든 장면 `green`, 7.4 s 적용)를 고정 표시했습니다. 장면 안에서 신호가 바뀌면 생성을 거부하도록 했습니다.
- 카메라: 관찰 전용입니다. 평활한 자차 진행 방향의 뒤·위(약 42° 내려다봄)에서, 2.6 m 안의 상대차와 자차가 화면 반각의 78% 안에 들어오는 최소 거리로 따라가고, 상대가 멀면 자차와 자차 앞 0.7 m를 담습니다([build_replay_scene.py](tools/build_replay_scene.py)).
- 카메라 이미지는 시뮬레이션 시각 0.05 s마다 1장이며, 파일 이름의 시각 + `render_tau_to_sim_offset_s`로 원 시각에 대응합니다([render_meta/](render_meta/)). 편집 시 시간 간격이 0.05 s에서 1 ms 이상 벗어나면 중단하도록 했습니다.

**재렌더에서 원 실행과 다른 점(한계):** 바퀴 회전과 앞바퀴 조향은 재현하지 않았습니다(차체 자세만). 높이·롤·피치는 기록이 없어 자차 z = 0.003 m(노면 OBJ 최고 높이), 상대 z = 0.06 m(원 pose)로 고정했습니다. 그림자·조명은 같은 월드 설정이지만 렌더 해상도와 카메라 위치가 원 실행 카메라와 다릅니다. 상대 자세는 50 Hz 순간이동을 100 Hz로 기록한 계단 값이라 미세한 끊김이 그대로 보일 수 있습니다. 신호 갠트리 근처에서는 실제 구조물이 차를 잠시 가릴 수 있습니다(②·④·⑥의 갠트리 통과 순간).

## 품질 확인

- 최종 MP4를 PyAV로 **끝까지 디코딩**: 1,744/1,744프레임, 87.20 s, PTS 간격 0.05 s 일정, `h264` High·`yuv420p`·1280×720·20/1, 오디오 없음. 최상위 상자 순서 `ftyp, moov, free, mdat`(faststart 확인). [qa_report.json](qa_report.json)
- 검은 화면(평균 RGB < 15) 0프레임. 직전 프레임과 거의 같은 프레임 최장 연속 0.30 s(① 출발 직전 정지 구간)이며 긴 정지 화면은 없습니다.
- [contact_sheet.png](contact_sheet.png)와 전 해상도 프레임을 직접 보고 글자 잘림·겹침을 확인했습니다. 첫 편집본에서 ①의 장면 이름이 원 녹화 정보 패널과 겹쳐 아래로 옮겼고, 추월 뒤 화면이 지나치게 멀어지는 구도를 고쳐 다시 렌더했습니다.
- 사람이 볼 때: 파랑 자차와 빨강 상대가 색·형태로 분명히 구분되고, ②③⑤에서 뒤→옆→앞→복귀, ④⑥에서 뒤로 돌아오는 장면, ⑦에서 뒤차가 앞질러 가는 순서를 화면에서 확인할 수 있습니다. 다만 추월 뒤 두 차가 멀어지는 구간은 카메라가 물러나 차가 작게 보입니다. 이는 제작자의 육안 확인이며 별도 시청자 평가는 하지 않았습니다.

## 장면별 근거 수치(원 보고서 기준)

영상은 아래 결과를 보여 주는 용도이며, 판정은 각 보고서와 원시 기록을 따릅니다. 각 장면은 **한 번의 시뮬레이션 실행**이고 반복 통계가 아닙니다.

| # | 원 보고서에서 확인한 결과 | 보존된 한계·실패 |
|---|---|---|
| ① | 원 노면 46.67 m·**29.228 s**·최고 **2.862 m/s(10.30 km/h)**, 접촉·노면 이탈 0, STOP 안정·정상 종료 | 후반 0.138 s 저속, STOP 정착 1.446 s·**변위 0.945 m**(영상 밖), 단독 1회 |
| ② | 14 s에 10.06 m, 초록 뒤 9.819 s 추월 완료, 복귀 뒤 3.488 m, 최장 정체 0.101 s, 접촉 0·간격 하계 0.212 m | 최대 같은 쪽 두 바퀴 잔디(화면에서도 보임), 명시 평탄 잔디 파생본, 원 추종 판정 3항목 실패 보존 |
| ③ | 초록 뒤 9.118 s 추월·복귀, 복귀 뒤 1.969 m, 최소 간격 0.2109 m, 접촉 0, 오목 중앙면 C1 반환 522개 확인 | 원상자 r15(2.008 m)와 단일 변수 비교 아님, 원 판정 3항목 실패 보존 |
| ④ | 초록 뒤 8.719 s 양보 후 추종 복귀, 이후 0.916 m, 최소 swept 간격 0.569 m, 접촉 0 | 원 평가 최종 간격 1.262 m·차체 전체 도로 안 실패 보존 |
| ⑤ | 20 s에 12.809 m, 최장 저속 0.607 s, 초록 뒤 6.444 s 완전 통과, 최소 간격 0.128 m, 접촉 0 | 실물 운동 상계 증명 아님 |
| ⑥ | 재출발 뒤 3.693 m, 재출발 뒤 최장 저속 0.203 s, 최종 FOLLOW, 최소 간격 0.126 m, 접촉 0 | 전체 최장 저속 1.112 s(앞차 정지 직후 자차 정지, 자세 차분 0.05 m/s 미만 10.82–11.96 s = 영상 약 68.7–69.9 s), 뒤로 합류 중 0.15–0.18 s 저속 3회(같은 기준, 원 시각 20.6–21.9 s), 원 HOLD·15 cm 판정 실패 보존 |
| ⑦ | 통과 뒤 1.304 m 전진·총 6.342 m, 간격 하계 0.1304 m, 피추월 구간 0명령 10→0, 접촉·노면 이탈 0 | 측방 0.35 m/s 목표 상한 남음, 원 15 cm·앞차 뒤 유지 판정 실패 보존 |

## 주요 성과 (영상 밖 요약)

현재 현황([PROJECT_CONTEXT](../../../../docs/PROJECT_CONTEXT.md)) 기준으로 새 코스에서 **대표 1회씩 확인한** 기능입니다. 영상에 없는 항목은 링크된 보고서에 근거가 있습니다.

- **단독 속도:** r25 거리 한 랩 29.228 s·최고 10.30 km/h(영상 ①).
- **이동차 추월·양보:** 0.30 m/s(②)·0.50 m/s 원상자 [r15](../../../validation/2026-11-08/medium_pass_connected_v1/README.md)·비정형(③) 추월과 복귀, 상대 가속 시 양보(④).
- **정지차·재출발:** 정지차 우회(⑤), 앞차 정지·재출발 대응(⑥).
- **측방·피추월:** 빠른 뒤차 피추월(⑦), [오른쪽 앞 차량](../../../validation/2026-11-08/right_side_newcourse_v1/README.md) 옆 통과(실제 나란히 1.010 s, 영상 미수록).
- **교통 거리:** [r23 교통 한 랩](../../../validation/2026-11-08/traffic_lap_transition_v1/README.md) 96 s·48.29 m, 두 0.8 m/s 앞차와 [경기 거리 5랩](../../../validation/2026-11-08/traffic_race_distance_v1/README.md) 240.14 m(접촉·노면 이탈 0). 각본 상대이며 독립 5대가 아닙니다.
- **센서 오차:** [r38](../../../validation/2026-11-08/masked_motion_speed_v1/README.md) C1 ±3 cm·추가 지연 32–70 ms에서 거리 한 랩 48.09 m.
- **신호·출발:** [r12](../../../validation/2026-11-07/late_start_marker_v1/README.md) 가림 해제 뒤 0.278 s 첫 출발, [공식 5대 그리드](../../../validation/2026-11-08/grid_pack_newcourse_v1/README.md) 출발, [시차 출발](../../../validation/2026-11-09/grid_staggered_start_v1/README.md) 4.49 m.
- **운영:** [r26 재배치](../../../validation/2026-11-08/ops_relocation_link_v2/README.md) START 0.383 s 허가, 교통 속 STOP/START [r40](../../../validation/2026-11-08/entry_width_transition_v1/README.md) 1.200 m.
- **이관 준비:** [r48 Jetson 묶음](../../../validation/2026-11-09/jetson_r48_bundle_v1/README.md) 96파일, PC 격리 재생 2,452판단 일치(기기 실행은 아님).

## 미완료·계획 기능 (영상에 성과로 넣지 않음)

- **0.70 m/s 이상 빠른 앞차 추월:** [r31](../../../validation/2026-11-08/moving_pass_faster_v1/README.md)·[r32](../../../validation/2026-11-08/moving_pass_wall_mask_v1/README.md) 실패, [r44](../../../validation/2026-11-08/faster_entry_consistency_r44_v1/README.md)는 불리한 진입을 포기하고 추종한 대표일 뿐 추월 성공이 아닙니다.
- **지속 코너 병렬:** [r48](../../../validation/2026-11-08/observed_corner_continuity_r48_v1/README.md) 코너 정체는 개선됐으나 실제 병렬 0.301 s로 0.5 s 목표 미달.
- **고속 제동:** r25 STOP 변위 0.945 m 미해결, 실제 제동거리 미측정.
- **지름길 P21/P22:** r49 입구 통과만 확인, [r50](../../../validation/2026-11-09/p22_route_connected_v1/README.md) 본선 복귀 실패, [r51](../../../validation/2026-11-09/p22_exit_alignment_connected_v1/README.md)은 연결·저장 재생까지만 하고 실제 주행 미실행. 합류·시간 이득 미확인이며 본선 유지가 기본입니다.
- **다중 차량·운영 실환경:** 독립 5대 경기, 공용 Wi-Fi·실제 MCU 정지, 동적 상대 속 재개 미검증.
- **연산·실물:** 최신 Jetson 실시간 성능 미측정(20 ms 마감 미달 기록 있음), 실차 조립·배선·전원·센서 시야·실제 주행 전부 미검증.
- **원 판정 실패:** 위 표의 원 평가 실패 항목은 보조 판정 통과로 덮어쓰지 않았습니다.

## 재현 방법

전제: WSL2 Ubuntu-24.04·ROS 2 Jazzy(gz-sim 8), 저장소 `install/arena_gazebo/lib/libarena-wsl-d3d12-lifetime.so`(읽기만 함), Windows Python 3.14.

```bash
# 1) 편집용 Windows venv(저장소 밖)
python -m venv C:/Users/Jinhyeong/arena_media_venv
C:/Users/Jinhyeong/arena_media_venv/Scripts/python -m pip install imageio-ffmpeg av numpy pillow opencv-python-headless

# 2) 재렌더 플러그인 빌드(WSL ~/arena_scratch/feature_showcase/build)
MSYS_NO_PATHCONV=1 wsl.exe -e bash -lc 'bash "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/videos/2026-11-09/feature_showcase/tools/build_plugin.sh"'

# 3) 여섯 장면 재렌더 → C:/Users/Jinhyeong/arena_media_work/clips/<장면>/f_<ns>.png (장면당 약 15–20 s)
MSYS_NO_PATHCONV=1 wsl.exe -e bash -lc 'bash "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/videos/2026-11-09/feature_showcase/tools/render_all.sh"'

# 4) 편집·인코딩, 5) 전체 디코딩 검사와 contact sheet (feature_showcase 폴더에서)
C:/Users/Jinhyeong/arena_media_venv/Scripts/python tools/compose_showcase.py --work C:/Users/Jinhyeong/arena_media_work --out .
C:/Users/Jinhyeong/arena_media_venv/Scripts/python tools/qa_showcase.py --dir .
```

장면 구간은 [render_all.sh](tools/render_all.sh)(렌더 여유 포함)와 [compose_showcase.py](tools/compose_showcase.py)의 `cut`(최종 사용 구간)에 있습니다. 같은 입력과 카메라 설정이면 같은 자세가 렌더되지만, GPU 렌더 결과의 바이트 단위 동일성은 확인하지 않았습니다. [scene_timeline.py](tools/scene_timeline.py)는 구간 선정에 쓴 읽기 전용 요약 도구입니다.

## 작업 범위와 기록

- 기존 증거·시뮬레이터·제어기·설정은 수정하지 않았습니다(작업 후 `git status`에서 이 영상 폴더와 당일 활동 기록 `docs/activity/2026-11-09.md`만 변경). 새 Gazebo 주행·지름길 r51·Jetson/전원 작업·커밋/푸시/외부 업로드는 하지 않았습니다.
- 재렌더는 자세 재생만 하는 Gazebo 서버를 장면당 한 번씩 띄웠습니다. 1초 시험 렌더 1회 뒤, 카메라 구도 수정으로 여섯 장면 일괄 렌더를 4회 반복했으며(입력 기록·장면 구간은 동일, 최종본은 4회차), 매회 종료 후 잔여 `gz sim` 0개를 확인했습니다. 일괄 렌더 실행·대기는 Claude Haiku 5.5 보조(별칭 `haiku`, 자체 보고 `claude-haiku-5-5`)가 맡았고, 도구 작성·구간 선정·편집·검사·문서는 Claude Opus 5.5가 맡았습니다.
- 중간 PNG(약 103 MB)는 저장소 밖 `C:/Users/Jinhyeong/arena_media_work/clips/`에 두었고, WSL 원시 PPM 프레임은 PNG 변환 뒤 삭제했습니다.
