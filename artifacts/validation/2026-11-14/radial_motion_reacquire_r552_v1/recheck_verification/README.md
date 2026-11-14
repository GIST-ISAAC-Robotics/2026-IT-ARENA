# r552 재획득 radial 재검사 연결

새 정식 파일 `tests/test_reacquire_radial_recheck.py`만 작성했다. 공통 제어 코드와 기존 테스트는 수정하지 않았다. 기존 `test_current_wall_reacquire.sensor/history/CFG`를 재사용하며 외부 artifact를 읽지 않는다. 생성 raw를 `MeasurementScan`으로 만들어 실제 `motion_scene`·`snapshot_call`을 사용한다.

검사 6종을 current/retained 모듈 각각에 적용하여 총 12개 pytest 항목이다.

1. 실제 helper 양수 출력→normal 포장→실제 rebase 양수, 현재/직전 조향 두 sweep, 입력/원시 ray 불변.
2. 새로운 실제 전방 장애물 관측에서 두 rebase가 0 반환.
3. 같은 취득 기록의 reference가 .1 s 늦어지면 새 scene에서 모든 이동 반경이 .5×.1=.05 m 증가.
4. 일반 후보·shape 누락/불일치·원 속도 .100001 m/s는 radial 인자를 받지 않고 기존 축별 margin 사용.
5. path source 만료, 제어/scan epoch, side, timing, 시간 역행, scene reference, source index 불일치 거절. 점유 검사 이전에 반환하며 입력 불변.
6. 원 속도 0 또는 STOP은 radial 메타데이터가 있어도 재허가하지 않음.

monkeypatch는 각 모듈의 sweep 호출 인자·결과 기록에만 쓴다. 원 `swept_limit`을 그대로 호출하여 반환값을 조작하지 않는다. helper/경로/pursue/운동 함수는 실제 함수다.

요청 범위대로 Windows에서는 `compile(source, path, 'exec')` 문법 검사만 통과했다. pytest/WSL/장치 실행은 하지 않았다. native 통합 실행은 root 소유다.

```text
python -m pytest -q tests/test_reacquire_radial_recheck.py
```
