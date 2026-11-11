# C1 비균일 측정 입력 선행 수정

2026-11-11, GPT의 코드 수정·단위 검사·저장된 실물 CSV 투영 검사다. 새 시뮬/ROS 운전/하드웨어 구동은 하지 않았다. 설계와 한계는 [C1 기준 문서](../../../../docs/sensors/C1_HARDWARE_2026_11_10.md#1111-선행-수정-비균일-원천-각도-입력)에 둔다.

- [native10.json](native10.json):102스캔·51,924점,500/508/524점(최소/중앙/최대).
- [native12.json](native12.json):242스캔·102,716점,412/424/440점. 두 검사 모두 원천 배열 순서·각도·개수·유효 거리 보존. 시각은 합성 상대200μs이며 `timing_verified=false`다.
- [verification.json](verification.json): 검사 목록·결과·환경 실패와 최종 소스 해시. 상세 실내 포인트클라우드는 Git 제외 원래 경로에 유지한다.

실행은 WSL Ubuntu-24.04, Python3.12, ROS Jazzy의 Python 패키지 경로를 사용했다. 처음 단위 검사201통과/3실패 및3개 환경 재시도 실패를 보존한다. 설치된 xacro 경로가 환경변수에 들어가도록 명시한 뒤3개가 통과해 관련204개 모두 통과했다. 500점 기존 입력의 이전 계산과 정확히 같은 장면/격자를 만드는 비교도 포함한다. 주행 판단 전체 재생이나 성능 계측으로 확대하지 않는다.

재현 예시(출력은 새로운 경로 사용):

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts:/opt/ros/jazzy/lib/python3.12/site-packages
python3 -m pytest -q tests/test_measurement_scene.py tests/test_local_scene.py tests/test_grid_cost_r43.py tests/test_lidar_mount_trial.py tests/test_scan_topology.py
python3 scripts/audit_c1_measurement_scene.py \
  --scans build/c1_hardware_2026_11_10/remote/raw_startup_checked-scans.csv \
  --points build/c1_hardware_2026_11_10/remote/raw_startup_checked-points.csv \
  --output build/c1_measurement_audit_new/native10.json
```

12 Hz는 `build/c1_hardware_2026_11_10/speed_remote/speed_retry/delayed12-{scans,points}.csv`를 같은 방식으로 사용한다. `source_valid` 등 운영 가정은 아직 그대로이며, 이 검사는720개의 ROS 표시 칸을 원천 측정으로 승인하지 않는다.
