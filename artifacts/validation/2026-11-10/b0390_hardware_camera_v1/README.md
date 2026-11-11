# B0390 실물 720p60 연속 취득 결과

2026-11-10. 실제 Jetson/IMX219 시험이며 시뮬레이션·합성 영상이 아니다. 상세 설정·변경·한계는 [실물 기준 보고서](../../../../docs/sensors/B0390_HARDWARE_2026_11_10.md)에 둔다. 사용자 요청에 따라 워커 중단 후 GPT가 직접 수행했다.

## 최종 판정

**기본 취득 기준 통과.** L4T R36.4.4의 기본 IMX219 driver/Argus mode 4에서 센서 active 1280×720, 요청 60/1 fps로 취득하고 BGR까지 변환했다. 워밍업 5초를 제외한 300.014563초에서 18,016프레임을 수신했다. 취득 중 timeout·오류·경고·재시작·PTS 역전·연속 동일 CRC32·DISCONT는 0이며 EOS와 NULL 정상 종료를 확인했다.

| 지표 | 실측 |
|---|---:|
| Gst source PTS 기준 평균 | 60.047025 fps |
| Jetson BGR 수신 단조 시각 기준 평균 | 60.047085 fps |
| PTS 간격 중앙 / p95 / p99 / 최대 | 16.641507 / 18.258843 / 18.905569 / 27.401936 ms |
| BGR 수신 간격 중앙 / p95 / p99 / 최대 | 16.622624 / 18.436808 / 19.475797 / 26.577298 ms |
| PTS / 수신 간격 25 ms 초과 | 각각 3회 |
| PTS / 수신 간격 33.333333 ms 초과 | 각각 0회 |
| source와 sink의 측정창 PTS열 | 18,016개 전부 순서 일치 |
| source pad→BGR 수신 중앙 / p99 / 최대 | 7.470857 / 8.667829 / 12.500982 ms |
| BGR shape / stride | 720×1280×3 / 3840 B |
| CPU / GPU 온도 최대 | 49.5 / 48.781 °C |
| 전체 6코어 평균 사용률 / 순간 최대 평균 | 30.81 / 44.17 % |
| 보드 VDD_IN 평균 / 최대 | 6.629 / 7.612 W |

25 ms 초과 순간 간격 3회를 숨기지 않는다. 평균 60 fps 유지와 완전히 일정한 프레임 간격은 다른 주장이다. 사용자 방향 변경(바닥→벽)과 실내 조명 조건이 포함됐으며 마지막 벽·근접 케이블 영상을 눈으로 확인했다. 색상표·원거리 초점 시험은 아니다.

source `offset`은 전체에서 한 값뿐이므로 capture ID/센서 순번으로 사용할 수 없다. `do-timestamp=true`인 Gst PTS와 source/sink 대조는 생산자 이후 관측이며 센서 노출 시각·센서 내부 프레임 무손실을 독립 보증하지 않는다. 파이프라인에 `videorate`나 의도적 복제/폐기는 없고, 연속 BGR 버퍼 CRC32 동일 0을 확인했다. source→sink 수치는 노출→인식 지연이 아니다. 스로틀 전용 카운터는 확인하지 않았으며 온도와 kernel log 관측만으로 스로틀 부재를 단정하지 않는다. CPU/전력은 GUI와 계측기까지 포함한 시스템 값이다.

## 증거와 재현

- [`summary.json`](summary.json): Jetson이 생성한 원본 집계, 파이프라인·caps·온도 표본·대표 프레임 해시.
- [`telemetry_summary.json`](telemetry_summary.json): tegrastats 305표본의 온도·CPU·입력 전력 요약.
- [`verification.json`](verification.json): PC에서 원시 CSV를 독립 재집계한 값의 일치, 전송 해시·stride·source PTS열 확인.
- [`first-capture.txt`](first-capture.txt), [`actual-mode4.txt`](actual-mode4.txt), [`after-capture.txt`](after-capture.txt): 실제 모드·초기 수신·종료 후 kernel과 해시 확인.
- [`failed-v1.json`](failed-v1.json), [`failed-v2.json`](failed-v2.json): 온도 sysfs 읽기 예외로 측정기만 중단한 원 실패. v3에서 미제공 온도 처리 보완 후 짧은 5초와 최종 5분 시험을 통과했다.
- 계측 코드 [`scripts/probe_jetson_camera.py`](../../../../scripts/probe_jetson_camera.py), SHA-256 `c07f63a34c728aa60157b5429ce033159b0473b9ae2bfa54c07df96d5c5e2308`. 업로드한 코드와 로컬 해시 일치.

Jetson에서 GI/GStreamer·numpy·cv2가 설치된 시스템 Python으로 다음과 같이 실행한다. 기존 출력 경로는 덮어쓰지 않는다.

```bash
python3 probe_jetson_camera.py <새 비공개 결과 경로> \
  --sensor-id 0 --sensor-mode 4 --duration 300 --warmup 5
```

실내 PNG·전체 프레임 CSV·source CSV·tegrastats·상세 실행 로그는 Git 제외 `build/b0390_hardware_2026_11_10/`에 회수했다. 공개 문서에는 사진을 넣지 않았다. 5분 원본은 Jetson `/tmp/it_arena_b0390_long_20261110/`에 보존했고, 임시 probe v1/v2/v3와 짧은 실패/성공 산출물도 보존했다. 시험 후 관련 Python capture·gst-launch·tegrastats 프로세스가 남지 않았음을 확인했다. `nvargus-daemon`은 기존 서비스로 유지했다.

Python 문법 검사, 원시 CSV 재집계, 전송 SHA-256, 문서 링크·diff 검사를 수행했다. ROS 발행·노출/capture ID·인식·실물 주행·다른 센서/액추에이터는 미검증이다.
