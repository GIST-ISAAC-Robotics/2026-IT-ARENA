# r561 희소 actor 반환 사후 평가 도구 — 사용자 중단으로 미검증 보존

**2026-11-14 사용자 개선 작업 중단 지시에 따라 여기서 종료했다. 새 개발·검사·실행을 시작하지 않는다.**

현재 산출물은 `audit_returns.py`의 첫 작성 초안이다. **구문 검사·단위 검사·실제 자료 평가 모두 미실행**이며 준비 완료 도구로 취급하지 않는다. 실제 r560 자료는 읽지 않았고 r560 평가도 실행하지 않았다. 공통 소스·기존 원본/로그 변경0, WSL/ROS/Gazebo/Jetson/장치 접근0이다.

## 확인한 입력 형식

보존 r539의 첫 native scan과 소수 ego/actor pose 행, 기록된 header/선택 SDF 및 기존 r546 actor fixture만 읽었다. r539 scan은 `native_measurements_v1`의 `acquired_at_s`·`angles_rad`·`ranges`, `simulation_acquisition`, `laser_frame` 형식이다. 자세는 `[time,x,y,yaw]`이며 동적 z/pitch/roll이 없다. r555/r560 실행 형식은 기존 준비 레시피 연결에서 확인했을 뿐, 실행 중인 r560 파일을 조회한 것은 아니다.

기존 `scripts/audit_actor_outline_returns.py`는 follow trace의 옛 균일 LaserScan 형식과 다른 actor 형상에 결합되어 있어 그대로 적용하지 않았다. 새 초안은 controller/ROS를 import하지 않고 NumPy와 표준 라이브러리만 사용한다.

## 작성한 초안의 의도

- 실제 저장 `runtime_vehicle.sdf`의 LiDAR extrinsic/range와 `follow_decisions.jsonl` header의 mount를 대조한다. 기존 선택 mount는 CAD 추정이며 이번 작업으로 실측 보정을 주장하지 않는다.
- ray별 취득 시각으로 ego/actor 평면 pose를 보간하고 반환점을 actor frame으로 옮긴다. pose 외삽/큰 보간 공백은 집계에서 분리한다.
- 저장 `evaluation_actor.sdf`의200×150mm 보수 collision box 근접 반환과 명목 LiDAR 높이에 걸리는 sparse visual 지주 근접 subset을 따로 집계한다.
- 출력 명칭은 **추정 actor 연관 반환**이다. 전체 유효반환수와 혼용하지 않으며 근처 벽/가림물 반환의 오분류 가능성을 명시한다. renderer object ID나 확정 감지 수가 아니다.
- scan별 수·거리·연관 반환0 구간을 기록하고 누락된 pose/scan을0으로 채우지 않는다. 기본15mm 연관 여유와50ms pose 보간 공백은 평가 가정이며 센서 정확도·제어 안전 기준이 아니다.
- 정상 실행 시 새 출력 폴더에만 `per_scan.jsonl`·`summary.json`을 쓰도록 작성했다. 기존 입력/원 판정은 바꾸지 않는다. 아직 이 출력들은 생성하지 않았다.

## 재개 시 남은 사항 — 지금은 수행하지 않음

사용자가 다시 개선 작업을 허용한 뒤, 초안의 입력 검증·geometry frame·경계 시각·연관0 구간·출력/원본 보존을 먼저 검토해야 한다. 작은 합성 대표(움직이는 actor/회전한 ego/주변 벽 오분류/누락 pose)와 보존 표본 검사도 아직 없다. 실제 r560 평가 여부와 실행 시점은 root가 종료·회수 상태를 확인하고 정한다.

특히 평면 pose의 높이 가정, sparse visual subset과 실제 광선 첫 교차의 차이, 불완전 scan의 수치 표기, 누락 구간을 연속 미관측으로 해석하지 않는지 검토가 남는다. 현재 초안의 JSON 필드나 결과가 검증됐다고 인용하지 않는다. 이 단계에서는 새 실행 명령을 인계하지 않는다.
