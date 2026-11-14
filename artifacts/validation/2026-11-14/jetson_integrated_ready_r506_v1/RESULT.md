# r506 설치된 Jetson ROS 동시 재생

2026-11-14. **r500 설치본의 시뮬레이션 시간 backend·독립 보호층·운영 모니터를 실제 Jetson ROS에서 함께 실행했다. 연속 명령 유지와 전체 10Hz는 미달이며 실물 운용 검증은 아니다.** 유효 20초 재생 1회와 시험기 결함으로 무효인 20초 재생 1회를 구분한다. 공통 소스·r500 설치·TTL·정책·전력/네트워크 설정은 변경하지 않았다.

## 입력·격리·측정 범위

- r500 `/home/q/it-arena-field-r500-20261114-a/activate_r500_v2.bash`와 설치된 두 패키지를 사용했다. 원ZIP SHA `73bebdcdc176eef04b641dc78c986b76ec32431b194c787cba0f351ad6f95965`→내장 manifest→721파일을 확인했다. 실행 전후 원소스721개와 설치 `.py/.so`234개는 불변이다. r498 후속 공통 변경은 이 설치본에 넣지 않았다.
- 입력은 r475 저장 시뮬레이션에서 추출한 r493 `inputs.json`, SHA `909c107fe700cf751e0a8c09b154bec09bff99140cd78b6b449014f8dfa829ac`다. `reference_s` 7.62≤t<27.62의 scan 200개와 중복 제거한 gyro 4,381개/평균 구동속도 2,191개를 1× 재생했다. 운동이력 prefix 2.006초와 EOF 후 1초가 별도로 있다.
- 원 광선 시각·frame·`simulation_acquisition`을 유지했다. 스캔 발행 시각은 저장 evaluator의 `reference_s`이며 원 DDS 도착시각의 재현이 아니다. 구동속도는 모터축 각속도로 역변환했고 위치 0은 파서용 placeholder다. 원시 encoder/CDR 재생이나 실측값이 아니다.
- RGB는 기존 `light_image`의 848×480 합성 off→red1..4→green 픽셀을 20Hz로 보냈다. 실제 detector를 거쳐 출발했으며 신호상태 직접 주입은 없다. 합성 RGB 441개, 영상 221개 제출/완료·오류 0을 확인했다. 실제 B0390 영상/인식/처리율 증거는 아니다.
- `ROS_DOMAIN_ID=156`, `ROS_LOCALHOST_ONLY=1`, `/replay_r506`에서 모든 센서·clock·명령·보호·운영 토픽을 연결했다. `/drive`와 `/drive/safe`의 유일 발행자는 각각 설치된 `local_pursuit`와 `lidar_safety`였다. 루트 `/drive`·`/drive/safe` 발행자0. host·MCU·serial·실센서·Gazebo를 시작하지 않았다.
- `G1G2Node`의 실제 두 계산 자식과 별도 guard/RaceOps 프로세스를 사용했다. 시간/기하/계획 설정은 저장header와 같고 구조sector JSON은 공백만 다르다. `legacy` 출발을 사용했으므로 B3 START/재개/원격구동 확인은 범위 밖이다.

## 유효 재생 결과

| 측정 | 결과 |
|---|---:|
| 계획 완료 회수 표본 |99개, 계산 중앙 75.60 / p95 95.63 / 최대 125.16ms |
| 같은 계획의 제출→부모 회수 |p95 107.07 / 최대 132.57ms |
| 현재검사 완료 회수 표본 |22개, 계산 중앙 15.31 / p95 16.34 / 최대 19.79ms |
| 같은 현재검사의 제출→회수 |p95 34.31 / 최대 34.70ms |
| `/drive` 명령·양수 명령 |504개·12개 |
| `/drive` 양수 명령 원시각 나이 |p95 80.90 / 최대 96.74ms, 200ms 초과 0 |
| 양수 명령의 다른 원시각 간 최대 수신 간격 |3.513초 |
| 보호 명령·양수 명령 |1,164개·14개 |
| controller 구동/gyro 수신 |2,191/2,191·4,378/4,381 |
| guard 구동/gyro 수신 |2,191/2,191·4,379/4,381 |

delivery 내부 카운터는 submitted 129, applied 84, discarded 15, invalidated 417이다. 이 applied를 실제 양수 발행 횟수로 쓰지 않는다. **계산 통계 99/22개는 `worker.collect()`가 반환한 표본**이다. 이전 generation에서 폐기된 계산과 종료 때 남은 작업을 포함한 전체 계산비용으로 확대하지 않는다. 회수된 계획 4개는 100ms를 넘었다.

대표 20초 내부에서 controller `MOTION_STOP` 52상태는 모두 `motion_stale`였다. trace의 `MOTION_STOP` reset은 262회, `decision_source_expired` reset은 6회다. guard 50Hz의 1,000행은 `clear` 602·`motion_stale` 207·`stale_input` 190·`missing_input` 1이며 `command_stale=true` 210행을 기록했다. 행 수를 독립 고장 사건 수로 바꾸지 않는다. 시뮬레이션 운동은 재생 명령에 반응하지 않으므로 실제 차량의 정체시간/완주 결과로 해석하지 않는다.

대표 20초의 발행일정 p95/max는 scan 2.07/4.36ms, gyro 2.94/8.83ms, 구동 2.37/8.08ms다. 전체 prefix 포함 gyro 최대 76.37ms는 최초 5.615초 표본이었다. 수정 후에도 관측된 운동표본 누락·`motion_stale`·명령공백은 남았으며 원인을 순수 계획계산 비용 하나로 확정하지 않았다. 각 노드의 원천수신/콜백나이를 보존한 실제입력 통합이 후속 확인 대상이다. TTL 연장·주기 하향은 하지 않았다.

`/drive/safe`의 header는 guard가 자신의 현재시각으로 생성한다. 그 header 나이 최대 28.09ms를 **원계획의 종단나이로 쓰지 않는다.** 원계획 나이는 위 `/drive` 표본과 guard의 명령만료 진단을 구분해 읽는다.

RaceOps HTTP 46회는 p95 5.08/최대 7.76ms였고 모든 응답에서 계획속도·보호속도·보호상태가 수신됐다. 반면 상태 binding은 46/46 `status_stale`다. 설치 `race_ops_node.py:on_status`는 `race` dict를 요구하고 legacy 상태에는 그 필드가 없으므로 설계상 입력을 채택하지 않는다. 따라서 **B3 운영상태통합 성공으로 판정하지 않는다.** 이 확인을 위해 공통 수신조건을 완화하지 않았다.

## 실패 보존·종료

1. attempt1은 노드 시작 전 manifest 종류 혼동으로 중단됐다. r500 보고서의 외부 manifest `ee450…`와 ZIP 내장 `64062a…`를 혼동했다. ZIP 원본→내장 manifest byte 동일→721파일 SHA를 검증하여 시험기를 고쳤다.
2. attempt2는 출력 endpoint 발견 대기 누락으로 입력 발행 전에 중단됐다. 주노드/자식 모두 정상 종료·원본 불변이다. 양쪽 endpoint의 발견을 기다리도록 고쳤다.
3. attempt3의 20초 재생은 무효다. 합성 green에 red4도 켜 계획 0이었고, 매 RGB의 bytes→ROS array 변환이 발행기 부하를 만들었다(RGB 104개, 발행 p95 약 203ms). 같은 thread 관측도 밀려 최대 16초의 가짜 명령나이를 만들었다. 이를 런타임 성능으로 사용하지 않는다. green 픽셀과 사전 생성 array·전용 관측 executor thread만 수정했다.
4. attempt4가 위의 유효 재생이다. `/autonomy/enable=false` 응답·0명령과 모든 주노드/계산/영상 자식 exit 0, 강제 0·잔존 0을 확인했다. 소유 PID 8845/8846/8847/8908/8914/8901은 종료됐으며 Jetson을 주 담당에게 반환했다. **물리 STOP은 미검증**이다.

원로그·실패·프로필·graph·시각행은 로컬 제외 경로 `remote_result/`와 회수 ZIP에 보존했다. ZIP SHA `efe702f901ffaa90fd5d93645c642c8d357274087828c51444ab695813865a5a`는 양쪽 일치한다. 유효 재생 이후 추가 재생·공통 수정·커밋·푸시는 하지 않았다.

근거: [실행 등록](manifest.json), [대표구간·만료·운영 분석](analysis.json), [최종 집계](summary.json), [자원 반환](resource_return.json), [시험기](run_replay.py), [실제 설치 노드 연결](node_runner.py). 원실패 실행기는 최초 bundle와 revision ZIP/원격 attempt 보존본에 남아 있다.
