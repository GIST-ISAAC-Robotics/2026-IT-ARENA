# r552 취득 후 운동 반경의 원형 팽창

2026-11-14. 주 담당이 지정한 `local_path.py`의 `swept_limit`과 신규 `tests/test_swept_radial_motion.py`만 변경했다. caller/helper·독립 보호층 연결은 주 담당의 별도 범위이며 이번 작업에서는 변경하지 않았다.

## 공통 구현

`swept_limit(..., *, radial_margin=None)`을 추가했다. `None`이면 기존 축별 `margin` 판정·4cm 표본·8개 묶음·첫 간섭에서4cm 차감·반환을 유지한다. 선택 인자가 없을 때 radial 입력 변환/검증/거리 계산은 실행하지 않는다.

`radial_margin`을 주면 차체와 기존 고정 `margin`의 사각형은 유지하고 각 점의 축별 초과량을 계산한다.

```text
outside_x = max(0, abs(local_x) - (length / 2 + margin))
outside_y = max(0, abs(local_y) - (width / 2 + margin))
hit = hypot(outside_x, outside_y) <= radial_margin
```

이는 사각형과 원판의 합이다. 취득 후 물체의 운동거리 상계만 원형으로 적용하고 고정 여유를 원형으로 줄이지 않는다. 점 삭제·광선 정렬·경로 생성은 바꾸지 않는다. scalar 또는 점수와 길이가 같은1D 유한 비음수 numeric 입력만 허용한다. bool과 혼합 bool 목록, 음수, NaN/Inf, 잘못된 shape는 `ValueError`다. NumPy 수치 배열은 벡터 검증을 사용하며 목록의 bool은 float 변환으로 사라지지 않게 별도 확인한다.

## 검증

Windows **35/35 통과,0.44초**, 오류·실패·skip0이다. 원본 None 경로는 선택값 분기를 제거했을 때 수치 연산 AST가 정확히 같으며, 함수 밖 AST도 동일하다.

- 직진·거의 직진·좌/우 선회, 빈 점군·실제 차체 점·경계·일반 점군과 여러 horizon에서 기본 호출/명시None/scalar0/vector0의 결과가 기존 순차 기하 대조와 정확히 같다.
- 측면·정면·실제 차체 내부 점과 원판의 내부/접선 점에 대한0거리 판정을 유지한다. 접선은 정확한 이진수3:4:5 좌표로 확인한다.
- 기존 고정 사각형(.125,.100)의 모서리에서 각축 .04m 떨어진 점(.165,.140)은 각축 .05m 팽창에 겹치지만 실제 원판 .05m 밖이다. horizon .03m에서 기존 사각 팽창은0, 원형 운동 반경은.03m를 반환한다. 이후4cm 전진의 실제 원판 간섭은 그대로0을 반환한다.
- ray별 반경을 뒤집으면 해당 점의 간섭 결과가 바뀌고 점/반경을 함께 뒤집으면 동일하다. 실제 차체 내부 점을 추가하면 다시0이다. 원시 점은 바꾸지 않는다.
- 첫 간섭 표본과4cm 차감, 입력 오류를 별도 확인했다.

```powershell
$env:PYTHONPATH='src/arena_autonomy;artifacts/validation/2026-11-13/operator_panel_r429_v1/pydeps'
$env:PYTHONIOENCODING='utf-8'
python -m pytest tests/test_swept_radial_motion.py -q --tb=short -p no:cacheprovider --junitxml=artifacts/validation/2026-11-14/radial_motion_reacquire_r552_v1/pytest.xml
```

원본·변경본은 `local_path.py.before`/`local_path.py.after`, SHA·AST 보존 근거는 `manifest.json`, diff는 `r552_swept_radial.patch`, 실행 결과는 `pytest.log`/`pytest.xml`에 있다.

WSL/ROS/Gazebo/Jetson/물리 장치 실행0이다. 이 단위 기하 검사는 실차 안전·운동 상계의 타당성·전체 제어 연결·주행 성능의 증명이 아니다. 두 공통 파일의 소유권을 주 담당에게 반환한다.
