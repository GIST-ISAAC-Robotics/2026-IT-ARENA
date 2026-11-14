# r508 — Jetson 입력 연속성 원인 분리와 한정 후보

**세 문제를 분리했다.** 원본 입력 공백은 없었다. 큰 wheel 전달 적체는 실행별 SHM 16 MiB 후보에서 줄었고, 이미 수신 큐에 있는 gyro의 소비 순서는 제어 직전 bounded drain으로 개선됐다. 이어 현재 경로 재검사의 `current_route_proof` 전달 누락을 수정한 독립 stage에서 해당 폐기 99건이 0건이 됐다. **연속 주행 완전 성공은 아니다.** 최종 active 구간에 controller `motion_stale` 3회와 최장 양수 명령 공백 1.326초가 남는다.

공통 이관용 [4파일 패치](common_transfer/common_transfer.patch)와 [원본/후보 SHA](common_transfer/manifest.json)를 제공했다. 이 담당은 공통 파일과 r500 설치본을 직접 수정하지 않았다. 이후 통합 담당이 4파일 반영과 고유 196검사 통과를 전달했다. guard 검사 최초 실패는 fake MotionInput에 새 메서드가 없던 fixture 문제였으며, fixture 보완 후 통과했고 원 실패 로그는 통합 담당이 보존한다. 최신 공통 설치/trace OFF 실행은 후속이다.

## 범위와 입력

- 고정 설치: `/home/q/it-arena-field-r500-20261114-a`, 원 소스 ZIP SHA-256 `73bebdcdc176eef04b641dc78c986b76ec32431b194c787cba0f351ad6f95965`. 매 실행 전후 원 소스 721파일과 설치 Python/공유라이브러리 234파일 불변을 확인했다.
- [r506](../jetson_integrated_ready_r506_v1/RESULT.md)의 저장 입력과 원 `simulation_acquisition` 시각을 유지했다. 원 입력 SHA `909c107fe700cf751e0a8c09b154bec09bff99140cd78b6b449014f8dfa829ac`. wheel 2,191개/100 Hz/최대 간격 10 ms, gyro 4,381개/200 Hz/최대 간격 5 ms, native scan 200개이다. [입력 감사](saved_input_audit.json).
- 선택 구간은 `[7.62,27.62)`의 20초이며 운동 이력 prefix는 5.614초부터다. 마지막 원 gyro/wheel은 27.515/27.514초, 마지막 scan 발행 참조는 27.52초다. **단계 비교는 입력이 계속 존재하는 `[7.62,27.52)`의 19.9초**로 통일했다. 그 뒤 입력 소진·EOF 정지를 성능 실패와 합치지 않는다.
- 설치 G1G2 runtime 두 계산 process, 독립 guard, 읽기 전용 RaceOps, 실제 detector를 거치는 합성 RGB 848×480/20 Hz를 함께 실행했다. ROS domain 158, `/replay_r508`, simulation clock, retained SingleThreadedExecutor이다. 모든 입력/출력 토픽의 publisher·consumer와 실제 node profile의 frame/time policy를 발행 전 대조하고, 두 노드 모두 gyro/drive accepted≥1을 첫 scan 전에 확인했다.
- 원시 CDR 재생·실제 B0390 영상·실제 encoder 위치·물리 LiDAR/IMU·host·MCU·물리 포트/출력·B3 wait/start는 없다. legacy 시작 시험이므로 RaceOps binding stale은 기존 시험 한계다. 합성 입력은 실차 센서 또는 hardware backend 증거가 아니다.
- buffered TimingProbe와 제어 직전 원 stamp/ROS clock 계측을 추가했다. timing buffer drop은 모든 단계 0이다. 발행 시각은 publish 호출 직전, callback 시각은 rclpy 변환 뒤이다. DDS 내부 도착/ACK/NACK 시각은 측정하지 않았다.

## 동일 입력에서 분리한 효과

각 단계는 원인 또는 후보를 바꾼 뒤 1회씩 수행했다. 서로 다른 동적 실행 경로와 trace 양이 발생하므로 계획 계산시간의 통계적 우열을 주장하지 않는다. 원시 결과와 [집계 코드](summarize_stages.py), [기계 집계](stage_comparison.json)를 보존한다.

| 지표 | 고정 설치+계측 | SHM16만 | +bounded drain | +현재 proof 전달 |
|---|---:|---:|---:|---:|
| controller wheel 발행→callback 최대(ms) | 841.40 | 67.68 | 70.42 | 100.42 |
| guard wheel 발행→callback 최대(ms) | 641.98 | 49.23 | 45.10 | 44.78 |
| guard gyro 발행→callback p95(ms) | 37.84 | 38.41 | 16.53 | 17.01 |
| active controller motion_stale 호출 | 295 | 105 | 1 | 3 |
| active guard motion_stale 호출 | 195 | 182 | 0 | 0 |
| active guard stale_input 호출 | 57 | 62 | 484 | 128 |
| new_current_route_proof_required 폐기 | 0 | 2 | 99 | 0 |
| active 양수 /drive 발행 | 7 | 4 | 83 | 161 |
| active 양수 /drive/safe 발행 | 8 | 4 | 469 | 765 |
| 양수 /drive 간격 p95(ms) | 4330.58 | 5072.86 | 574.59 | 131.90 |

`stale_input`은 이 비교 구간에서 command stale에 따른 거절이다. controller의 입력 거절이 줄자 실제 계획/현재 검사가 진행되어 그 뒤의 계약 누락이 드러났으므로, drain 단계의 command stale 증가를 입력 처리가 악화된 것으로 해석하지 않는다. 호출 수는 독립 정지 사건 수가 아니다. 최종 양수 명령 나이는 p95 **114.23 ms**, 최대 **161.77 ms**, 200 ms 초과 **0**이다. `/drive/safe`는 새 guard 발행 시각이므로 그 header 나이를 원 명령의 종단 지연으로 사용하지 않았다.

### 1. 수백 ms wheel 적체

baseline controller의 wheel callback이 835.70 ms 멈춘 구간에 IMU 170회·control 43회·scan 9회·image 17회가 실행됐다. 계측 callback 시간 합계는 약 135.12 ms이고 단일 최대는 7.70 ms였다. 재개 직후 원 stamp 19.814초의 다음 순서 표본을 받았으며 원 입력 자체의 공백은 없다. 별도 guard와 controller가 같은 표본을 약 642 ms 늦게 받는 구간도 있다. 전체 CPU 정지나 단일 계획 계산으로 이 특정 스트림 지연을 설명할 수 없다.

기존 [A3 DDS 기록](../../../../docs/reports/RECORD_REPLAY_JETSON_HANDOFF_2026_09_29.md)의 유사 실패와 후보를 재사용했다. 기존 `scripts/replay_transport.py`와 ARM probe SHA `b235a5df0f8abe0c0c014209e687578d1558bdd9a587c61fb5355c982982adbf`로 **4개 실제 participant 각각 SHM 단독, segment_size=16777216**을 검사했다. SHM16 단계에서 양쪽 wheel 100 ms 초과 공백이 0이 됐다. 전송 경로/용량과의 연관은 확인했으나 DDS 내부 손실의 근본 원인을 확정하지 않는다. SHM16은 같은 호스트 전용의 실행별 선택이고 OS/global 환경 변경이나 원격 ROS 기본값이 아니다.

### 2. 이미 준비된 gyro보다 제어 timer가 먼저 실행됨

설치 Humble executor는 ready timer를 subscription보다 먼저 내보내고, 각 순회에서 subscription 표본을 하나씩 소비한다. 저장 순서에서 guard가 19.11 ms, 15.87 ms의 제어를 연속 수행하는 동안 gyro가 밀려, 다음 제어 진입 시 마지막 반영 표본이 37.57 ms 전이었다. 이 `motion_stale` 계산이 1.8 ms로 끝난 직후 밀린 gyro 9개가 약 6 ms 안에 소비됐다.

후보는 제어 직전 준비된 wheel/gyro만 **기존 callback과 검증을 그대로 거쳐** 소비한다. 원 stamp·순서·history·frame/future/slew 검증·30 ms 외삽 제한은 유지한다. 입력을 기다리거나 버리지 않으며 2 ms/64개에 도달하면 남긴다. 시간 상한은 메시지 사이에서 검사하므로 마지막 callback만큼 초과할 수 있다. 제어의 실제 `now`는 drain 뒤 기존 control 함수가 읽는다. `source_before_drain`, 소비 수/시간/상한 도달, 이후 available source와 실제 나이를 보존했다.

전제는 control과 운동 subscriptions의 같은 기본 mutually-exclusive callback group이다. 계측 실행은 한 SingleThreadedExecutor였다. 별도 병렬 direct callback 호출에 확대하지 않는다. baseline controller gyro 3개 누락은 보존하며 SHM16 이후 세 단계에서는 두 노드 모두 gyro 4,381개·wheel 2,191개 전량, rejected 0이다.

### 3. 현재 경로 proof 전달 누락

`current_route_priority.rebase`는 `_geometry.rebase`의 현재 경로 이동·epoch/시각·점유 sweep과 전방 관측을 검사한 뒤 양수를 반환한다. 그러나 `current_route_proof`를 만들지 않고 `current_region.rebase`가 즉시 반환하여, `RuntimeDualProcess.application_reason`이 retained 경로 또는 원 계획 300/200 ms 경계 이후 그 결과를 거절했다. drain 단계의 폐기 99건은 전부 이 사유였다. 이 중 48건은 검사 시작과 적용 사이 원 scan의 300 ms 경계를 넘었다.

후보는 **기존 현재 검사에서 양수인 경우에만** 소비자가 요구하는 proof 필드를 전달한다. `original_path_source_s`, `current_evidence_first_s`, `current_reference_s`는 기존 검사 출력의 값이고 새 now로 바꾸지 않는다. 과거 점유 허가를 재사용하지 않으며, 0출력/전방 관측 소실에는 accepted proof가 없다. application의 시간·epoch·직전 조향 검사는 수정하지 않았다. 독립 stage에서 해당 폐기 99→0을 확인했다.

## 남은 실패와 계측 한계

최종 최장 양수 명령 공백 **24.782→26.109초, 1.326초**는 active 구간이다. 직전 controller callback **105.57 ms** 뒤 gyro가 밀려 2 ms drain 뒤에도 실제 나이 약 82/61 ms가 남았고 `motion_stale` 두 번과 generation reset이 발생했다. 이후 `follow_boundary_unconfirmed`/`follow_release_wait` 재확인이 이어졌다. 별도 active stale 한 번 직전 callback도 **73.65 ms**였다. [남은 공백 근거](remaining_gap_analysis.json).

이번 진단은 `g1_trace_path`를 명시해 최종 **40,539,104 bytes**의 상세 trace를 기록한다. `FollowTrace.emit`의 `freeze(data)`는 제어 callback에서 동기로 수행되며 저장 writer는 별도다. r508 당시에는 상세 기록을 분리한 부하와 대조하지 않았고 긴 callback 중 trace 비용 비율을 계측하지 않았다. **현재 선택 구성은 trace 경로가 필수여서 단순 빈 경로로 실행할 수 없다.** 따라서 운영 기본값이 이미 가볍다고 가정하지 않는다. 후속 [r522 기록 분리 대조](../jetson_traceoff_continuity_r522_v1/RESULT.md)는 기존 생성 검증을 유지한 test-only sink OFF 1회이며, 원 r508 판정을 대체하거나 실운영 완전 검증으로 확대하지 않는다. TTL 증가는 해법이 아니다.

## 이관·검사·종료

- drain과 proof는 각각 다른 실패를 고치므로 함께 이관하는 편이 타당하다. SHM16은 별도 실행 설정으로 유지한다. 검토용 공통 패치 SHA `0bc5abad7c9fe9bf7d9c69b18bff2fe82e30fd8c6458b5edfda3b40e9b713f86`; 통합 전 `git apply --check --ignore-space-change` 통과. CRLF 혼재를 처리한 LF 패치이며 공통 통합 회귀 결과는 위 통합 담당 전달을 따른다.
- [drain 회귀](test_motion_ingress_candidate.py) 5개: 원 순서/시각 보존, 수량 상한, 시간 상한, 빈 큐 무대기, 기존 검증 예외 유지. [proof 계약 회귀](candidate_shm16_drain_proof/test_current_route_proof_candidate.py) 6개: 기존 실패 재현→proof 통과, 6개 identity/renewal 변조 거절, 0출력 유지, 전방 소실 정지, 시간/epoch/조향 경계, 입력 불변. ARM에서 모두 통과했다.
- 첫 r508 시도는 harness 이름 치환으로 IMU frame 불일치가 생겼다. 원 profile의 `r506_reconstructed_sim_imu`를 producer가 그대로 읽게 수정하고 전체 frame/topic/time 계약 및 첫 accepted 관문을 추가했다. 실패 자료는 `remote_result/attempt1_frame_mismatch`에 보존하며 성능 비교에서 제외했다.
- 모든 단계 STOP service 성공, 자식 exit 0, 강제 종료 0, 잔존 소유 process 0, 원본/설치 불변, 물리 포트/출력 0. 최종 [자원 반환](final_resource_return.json)에서 소유 PID 27개 모두 없음, 모든 r508 작업 폴더의 잔존 process 0, 소스 721/설치 234파일 불변을 다시 확인하고 Jetson 독점권을 반환했다. 장치·권한·네트워크·패키지·전력 설정 변경과 commit/push는 없다.

회수 ZIP SHA-256(원격/로컬 일치): baseline `4e7d075290001bf741f79f4c29070e832222599a5fd44282606329f3a4fff0d1`, SHM16 `c89a22e6640dcf40dc4ec4492b6aab2561f7c23fe344b884a4f739a70f30ad6f`, drain `85cadaf28d2eddea92e935225c44ce9f8efa1449041030c724f40259eef32efb`, proof `eafaddf0a2c5f85bf4e76bab08ef2c339133cfdc656474d28baaa58881a197c8`. 원시 trace·수신 로그·ZIP은 Git 제외를 유지한다.
