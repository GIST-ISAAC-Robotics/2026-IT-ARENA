# 코스 물리 비용·GPU 종료 분리

- `summary.json`: 유효한 네이티브 계측과 실제 ROS 보고서/자식 종료 재감사. `summarize.py`로 재생성합니다.
- `native/`, `prepare.py`, `run_static.sh`: 원본을 보존한 정적 월드 비교. 실제 엔티티·누적 1,100회 갱신을 확인합니다.
- `detector_*`: 같은 DART 엔진에서 충돌 검출 방식만 비교. dart는 90초 내 미완료.
- `baseline/`, `bullet/`, `bullet_no_scene/`: CPU에서 동일 RGB-D/ToF6·영상 90개 계측과 주행·정지. 모두 정상 종료.
- `gpu_bullet_no_scene/`: GPU 추가 비교. 영상 계측 후 종료 실패. `gpu_shutdown_stack.txt`는 종료 시각 스택입니다.
- `gpu_grace/`, `gpu_grace_repeat/`: SIGINT 유예 60초 진단. 1회 정상·1회 강제 종료. 디버거 미사용.
- `tests.txt`: Python 58개 통과.
- `.log`: 초기 빈 경로 실패 기록. 성능 근거에서 제외합니다.

코스 원본이나 운영 월드를 교체하지 않았습니다. SDF 파일은 진단 사본이며 리소스 절대 경로는 이 PC 기준입니다.
최종 보고서: [성능 분석](../../../../docs/simulation/PERFORMANCE_2026_09_08.md).
